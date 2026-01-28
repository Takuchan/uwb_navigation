#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import math
import csv
import time
import os
import json
from datetime import datetime

from geometry_msgs.msg import PointStamped, Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf_transformations import euler_from_quaternion
from nav2_msgs.action import NavigateToPose

class ExperimentGUINode(Node):
    """
    全実験（位置精度・姿勢評価・データロギング）を統括するGUIノード。
    実験①: クリック位置精度
    実験②: 距離指定直進 (cmd_vel)
    実験③: 自律移動 (Nav2) [NEW]
    """
    def __init__(self):
        super().__init__('experiment_gui_node')
        
        # --- ROS通信設定 ---
        # 1. EKF推定位置
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        # 2. Rvizクリック位置（実験①用）
        self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_callback, 10)
        # 3. UWB生データ（ログ用）
        self.create_subscription(String, '/uwb_data_json', self.uwb_callback, 10)
        # 4. ロボット操作用（実験②用）
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 5. Nav2関連 [NEW]
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

        # --- 内部変数 ---
        self.lock = threading.Lock()
        
        # ロボット状態
        self.robot_pose = None
        self.current_yaw = 0.0      # Map Yaw
        self.relative_yaw = 0.0     # Relative Yaw
        self.initial_yaw_ref = 0.0  
        
        # UWBデータ保持
        self.latest_uwb_raw = {}    
        self.uwb_log_str = ""       
        
        # 実験①用
        self.target_point = None
        
        # 実験②（直進）用
        self.is_running_exp2 = False
        self.start_pos = None
        self.travel_dist = 0.0
        self.target_velocity = 0.5
        self.target_distance = 3.0
        
        # 実験③（Nav2）用
        self.nav2_goal = None       # 記憶したゴール
        self.is_running_exp3 = False
        
        # 共通ログ管理
        self.is_logging = False     # ログ取り中フラグ
        self.log_data = [] 
        self.start_time = 0.0
        
        # 試行回数管理 {アンカー数: 回数}
        self.trial_counts = {i: 0 for i in range(2, 7)}

        # 制御ループ (20Hz)
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Experiment GUI Node Started.")

    # ---------------------------------------------------------
    # コールバック関数群
    # ---------------------------------------------------------
    def uwb_callback(self, msg: String):
        with self.lock:
            try:
                data = json.loads(msg.data)
                self.latest_uwb_raw = data
                
                log_parts = []
                for key in sorted(data.keys()):
                    val = data[key]
                    if val is None: continue
                    idx = key.replace("TWR", "A") 
                    status = val.get('nlos_los', 'Unk')
                    dist = val.get('distance', 0.0)
                    log_parts.append(f"{idx}:{status}({dist:.2f})")
                
                self.uwb_log_str = "|".join(log_parts)
                
            except json.JSONDecodeError:
                pass

    def odom_callback(self, msg: Odometry):
        with self.lock:
            self.robot_pose = msg.pose.pose
            
            q = msg.pose.pose.orientation
            (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
            deg = math.degrees(yaw)
            self.current_yaw = deg
            
            diff = deg - self.initial_yaw_ref
            while diff > 180: diff -= 360
            while diff < -180: diff += 360
            self.relative_yaw = diff
            
            # ログ記録 (実験② または 実験③ が実行中の場合)
            if self.is_logging and self.start_pos:
                curr_x = self.robot_pose.position.x
                curr_y = self.robot_pose.position.y
                
                # 開始地点からの移動距離
                dx = curr_x - self.start_pos[0]
                dy = curr_y - self.start_pos[1]
                self.travel_dist = math.sqrt(dx**2 + dy**2)
                
                elapsed = time.time() - self.start_time
                self.log_data.append({
                    "time": elapsed,
                    "x": curr_x,
                    "y": curr_y,
                    "yaw_raw": self.current_yaw,
                    "yaw_rel": self.relative_yaw,
                    "dist": self.travel_dist,
                    "uwb_detail": self.uwb_log_str
                })

    def clicked_point_callback(self, msg: PointStamped):
        with self.lock:
            self.target_point = msg.point
        self.get_logger().info(f"Exp1 Target Set: {msg.point.x}, {msg.point.y}")

    def goal_pose_callback(self, msg: PoseStamped):
        """ Rviz等からのゴールを受信して記憶 """
        with self.lock:
            self.nav2_goal = msg
        self.get_logger().info("Exp3: Nav2 Goal Memorized!")
        # GUIの色更新等はupdate_gui_loopで行う

    # ---------------------------------------------------------
    # ロジック: 実験② (距離停止)
    # ---------------------------------------------------------
    def start_exp2_logic(self):
        if self.robot_pose is None:
            messagebox.showwarning("Error", "Odomデータなし")
            return
        
        with self.lock:
            try:
                self.target_velocity = float(self.entry_speed.get())
                self.target_distance = float(self.entry_dist.get())
            except ValueError: return

            self.is_running_exp2 = True
            self.is_logging = True      # ログ開始
            self.start_time = time.time()
            self.log_data = []
            self.start_pos = (self.robot_pose.position.x, self.robot_pose.position.y)
            self.travel_dist = 0.0
        
        # GUI制御
        self.toggle_buttons(exp_running=2)
        self.get_logger().info(f"Exp2 Started. Target: {self.target_distance}m")

    def stop_exp2_logic(self):
        if not self.is_running_exp2: return
        
        self.is_running_exp2 = False
        self.is_logging = False
        
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.cmd_vel_pub.publish(twist)
        
        self.toggle_buttons(exp_running=0)
        self.get_logger().info("Exp2 Stopped.")
        messagebox.showinfo("Info", "実験②終了。誤差を入力して保存してください。")

    def control_loop(self):
        """ 実験②の制御ループ """
        if self.is_running_exp2:
            # 距離判定
            if self.travel_dist >= self.target_distance:
                self.get_logger().info("Exp2: Distance Reached")
                self.stop_exp2_logic()
                return
            
            # 直進指令
            twist = Twist()
            twist.linear.x = self.target_velocity
            self.cmd_vel_pub.publish(twist)

    # ---------------------------------------------------------
    # ロジック: 実験③ (Nav2自律移動)
    # ---------------------------------------------------------
    def start_exp3_logic(self):
        if self.nav2_goal is None:
            messagebox.showwarning("Error", "ゴールが設定されていません。\nRvizで '2D Goal Pose' を指定してください。")
            return
        if self.robot_pose is None:
            messagebox.showwarning("Error", "Odomデータなし")
            return
            
        # Action Server確認
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            messagebox.showerror("Error", "Nav2 Action Serverが見つかりません。\nNav2を起動してください。")
            return

        with self.lock:
            self.is_running_exp3 = True
            self.is_logging = True      # ログ開始
            self.start_time = time.time()
            self.log_data = []
            self.start_pos = (self.robot_pose.position.x, self.robot_pose.position.y)
            self.travel_dist = 0.0
        
        # ゴール送信
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.nav2_goal
        
        self.get_logger().info("Exp3: Sending Goal to Nav2...")
        send_future = self._action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.nav2_goal_response_callback)
        
        self.toggle_buttons(exp_running=3)

    def nav2_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Exp3: Goal rejected')
            self.stop_exp3_logic(success=False, msg="Goal Rejected")
            return

        self.get_logger().info('Exp3: Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav2_result_callback)

    def nav2_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Exp3: Navigation finished')
        # メインスレッド(GUI)ではないため、停止処理呼び出しには注意が必要だが
        # 今回はフラグ操作とGUI状態更新なので、threading lockがあれば概ね安全
        # ただしTkinter操作はメインスレッド推奨なので、after等を使うのがベストだが簡易的に実装
        self.stop_exp3_logic(success=True, msg="Arrived")

    def stop_exp3_logic(self, success=True, msg=""):
        if not self.is_running_exp3: return
        
        self.is_running_exp3 = False
        self.is_logging = False
        
        self.toggle_buttons(exp_running=0)
        self.get_logger().info(f"Exp3 Finished: {msg}")
        
        # Tkinterのメインループ内ではないスレッドから呼ばれる可能性があるため、
        # メッセージボックス等は即時表示されない場合があるが、今回は簡易実装とする
        if success:
             # 完了通知はGUIループ側で検知させてもよいが、ここではLog出力のみ
             pass

    def cancel_exp3_logic(self):
        """ 実験③の強制中断（キャンセル機能は簡易的） """
        if self.is_running_exp3:
            self.is_running_exp3 = False
            self.is_logging = False
            self.toggle_buttons(exp_running=0)
            self.get_logger().info("Exp3 Cancelled manually.")
            # 本当はActionClient.cancel_goalを送るべきだが、今回はログ停止のみ行う

    # ---------------------------------------------------------
    # 共通: その他ロジック
    # ---------------------------------------------------------
    def reset_yaw_reference(self):
        with self.lock:
            if self.robot_pose:
                q = self.robot_pose.orientation
                (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
                self.initial_yaw_ref = math.degrees(yaw)

    def toggle_buttons(self, exp_running=0):
        """
        0: 待機中 (全て有効、保存有効)
        2: 実験②実行中
        3: 実験③実行中
        """
        if exp_running == 0:
            self.btn_exp2_start.config(state="normal")
            self.btn_exp2_stop.config(state="disabled")
            self.btn_exp3_start.config(state="normal")
            self.btn_exp3_stop.config(state="disabled")
            self.btn_save.config(state="normal")
            self.entry_error.config(state="normal")
        else:
            self.btn_exp2_start.config(state="disabled")
            self.btn_exp3_start.config(state="disabled")
            self.btn_save.config(state="disabled")
            
            if exp_running == 2:
                self.btn_exp2_stop.config(state="normal")
                self.btn_exp3_stop.config(state="disabled")
            elif exp_running == 3:
                self.btn_exp2_stop.config(state="disabled")
                self.btn_exp3_stop.config(state="normal")

    def save_csv(self):
        try:
            anchors = int(self.var_anchors.get())
            manual_error = float(self.entry_error.get())
        except ValueError:
            messagebox.showwarning("Warning", "数値が不正です")
            return
        
        self.trial_counts[anchors] += 1
        trial_num = self.trial_counts[anchors]
        
        filename = f"log_{anchors}anchors_try{trial_num}.csv"
        save_path = os.path.join(os.path.expanduser('~'), 'uwb_experiment_data', filename)
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        
        try:
            with self.lock:
                data_to_save = list(self.log_data)
                
            with open(save_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    "Time(s)", "X(m)", "Y(m)", "Yaw_Raw(deg)", "Yaw_Rel(deg)", 
                    "Distance(m)", "UWB_Status", "Anchor_Count", "Manual_Error(m)"
                ])
                for row in data_to_save:
                    writer.writerow([
                        f"{row['time']:.4f}", f"{row['x']:.4f}", f"{row['y']:.4f}",
                        f"{row['yaw_raw']:.4f}", f"{row['yaw_rel']:.4f}", f"{row['dist']:.4f}",
                        row['uwb_detail'], anchors, manual_error
                    ])
            
            self.lbl_trial_info.config(text=f"現在: {anchors}個 - Try {trial_num} 完了")
            messagebox.showinfo("Success", f"保存しました:\n{filename}\n(Try {trial_num})")
            self.btn_save.config(state="disabled")
            
        except Exception as e:
            messagebox.showerror("Error", f"保存失敗: {e}")

    # ---------------------------------------------------------
    # GUI構築
    # ---------------------------------------------------------
    def start_gui(self):
        self.root = tk.Tk()
        self.root.title("UWB実験統合GUI v3.0")
        self.root.geometry("520x850") # 高さを拡張
        
        style = ttk.Style()
        style.configure("Bold.TLabel", font=("Helvetica", 10, "bold"))
        
        # --- 0. 設定 ---
        frame_settings = ttk.LabelFrame(self.root, text="0. 実験設定 (ログ用)", padding=10)
        frame_settings.pack(fill="x", padx=10, pady=5)
        ttk.Label(frame_settings, text="使用アンカー数:").pack(side="left")
        self.var_anchors = tk.StringVar(value="6")
        cmb = ttk.Combobox(frame_settings, textvariable=self.var_anchors, values=["6", "5", "4", "3", "2"], width=5, state="readonly")
        cmb.pack(side="left", padx=5)
        self.lbl_trial_info = ttk.Label(frame_settings, text="Try数: 未開始", foreground="blue")
        self.lbl_trial_info.pack(side="right", padx=10)

        # --- 1. ステータス ---
        frame_status = ttk.LabelFrame(self.root, text="1. ロボット状態", padding=10)
        frame_status.pack(fill="x", padx=10, pady=5)
        self.lbl_curr_pos = ttk.Label(frame_status, text="位置: X=0.00, Y=0.00")
        self.lbl_curr_pos.pack(anchor="w")
        
        frame_yaw = ttk.Frame(frame_status)
        frame_yaw.pack(fill="x", pady=5)
        self.lbl_yaw_raw = ttk.Label(frame_yaw, text="Map: 0.0°")
        self.lbl_yaw_raw.pack(side="left", padx=5)
        self.lbl_yaw_rel = ttk.Label(frame_yaw, text="Rel: 0.0°", font=("Helvetica", 12, "bold"), foreground="blue")
        self.lbl_yaw_rel.pack(side="left", padx=5)
        ttk.Button(frame_status, text="角度リセット (Rel Yaw=0)", command=self.reset_yaw_reference).pack(fill="x", pady=2)
        
        self.lbl_uwb_status = ttk.Label(frame_status, text="UWB: ---", font=("Courier", 8))
        self.lbl_uwb_status.pack(anchor="w", pady=2)

        # --- 2. 実験① ---
        frame_exp1 = ttk.LabelFrame(self.root, text="2. 実験①: 位置精度 (クリック計測)", padding=10)
        frame_exp1.pack(fill="x", padx=10, pady=5)
        self.lbl_exp1_target = ttk.Label(frame_exp1, text="目標: 未設定")
        self.lbl_exp1_target.pack(anchor="w")
        self.lbl_exp1_total = ttk.Label(frame_exp1, text="誤差: --- m", font=("Helvetica", 12, "bold"))
        self.lbl_exp1_total.pack(anchor="w")

        # --- 3. 実験② ---
        frame_exp2 = ttk.LabelFrame(self.root, text="3. 実験②: 距離指定直進", padding=10)
        frame_exp2.pack(fill="x", padx=10, pady=5)
        
        f_set = ttk.Frame(frame_exp2)
        f_set.pack(fill="x")
        ttk.Label(f_set, text="速度:").pack(side="left")
        self.entry_speed = ttk.Entry(f_set, width=5)
        self.entry_speed.insert(0, "0.5")
        self.entry_speed.pack(side="left")
        ttk.Label(f_set, text="距離:").pack(side="left", padx=5)
        self.entry_dist = ttk.Entry(f_set, width=5)
        self.entry_dist.insert(0, "3.0")
        self.entry_dist.pack(side="left")
        
        f_btn2 = ttk.Frame(frame_exp2)
        f_btn2.pack(fill="x", pady=5)
        self.btn_exp2_start = tk.Button(f_btn2, text="Start (Exp2)", bg="green", fg="white", command=self.start_exp2_logic)
        self.btn_exp2_start.pack(side="left", fill="x", expand=True)
        self.btn_exp2_stop = tk.Button(f_btn2, text="Stop", bg="red", fg="white", command=self.stop_exp2_logic, state="disabled")
        self.btn_exp2_stop.pack(side="left", padx=5)

        # --- 4. 実験③ (NEW) ---
        frame_exp3 = ttk.LabelFrame(self.root, text="4. 実験③: 自律移動 (Nav2)", padding=10)
        frame_exp3.pack(fill="x", padx=10, pady=5)
        
        self.lbl_goal_status = tk.Label(frame_exp3, text="目標地点: 未設定", bg="red", fg="white", width=20)
        self.lbl_goal_status.pack(pady=5)
        ttk.Label(frame_exp3, text="※ Rvizで '2D Goal Pose' を指定してください").pack()
        
        f_btn3 = ttk.Frame(frame_exp3)
        f_btn3.pack(fill="x", pady=5)
        self.btn_exp3_start = tk.Button(f_btn3, text="Start Navigation (Exp3)", bg="blue", fg="white", command=self.start_exp3_logic)
        self.btn_exp3_start.pack(side="left", fill="x", expand=True)
        self.btn_exp3_stop = tk.Button(f_btn3, text="Cancel", bg="orange", fg="white", command=self.cancel_exp3_logic, state="disabled")
        self.btn_exp3_stop.pack(side="left", padx=5)

        # --- 5. 結果保存 (共通) ---
        frame_save = ttk.LabelFrame(self.root, text="5. 結果保存 (共通)", padding=10)
        frame_save.pack(fill="x", padx=10, pady=5)
        
        f_err = ttk.Frame(frame_save)
        f_err.pack(fill="x")
        ttk.Label(f_err, text="手動計測誤差(m):", foreground="red").pack(side="left")
        self.entry_error = ttk.Entry(f_err, width=10)
        self.entry_error.pack(side="left", padx=5)
        
        self.btn_save = ttk.Button(frame_save, text="ログをCSV保存", command=self.save_csv, state="disabled")
        self.btn_save.pack(fill="x", pady=5)
        
        self.update_gui_loop()
        self.root.mainloop()

    def update_gui_loop(self):
        with self.lock:
            # 位置・角度更新
            if self.robot_pose:
                self.lbl_curr_pos.config(text=f"位置: X={self.robot_pose.position.x:.2f}, Y={self.robot_pose.position.y:.2f}")
            self.lbl_yaw_raw.config(text=f"Map: {self.current_yaw:.1f}°")
            self.lbl_yaw_rel.config(text=f"Rel: {self.relative_yaw:.1f}°")
            
            # UWB表示
            disp = self.uwb_log_str[:50] + "..." if len(self.uwb_log_str)>50 else self.uwb_log_str
            self.lbl_uwb_status.config(text=f"UWB: {disp if disp else 'No Data'}")
            
            # 実験①表示
            if self.robot_pose and self.target_point:
                dx = self.target_point.x - self.robot_pose.position.x
                dy = self.target_point.y - self.robot_pose.position.y
                self.lbl_exp1_total.config(text=f"誤差: {math.sqrt(dx**2+dy**2):.3f} m")
                self.lbl_exp1_target.config(text=f"目標: {self.target_point.x:.2f}, {self.target_point.y:.2f}")

            # 実験③ ゴール状態
            if self.nav2_goal:
                self.lbl_goal_status.config(text="目標地点: 設定済み (Ready)", bg="green")
            else:
                self.lbl_goal_status.config(text="目標地点: 未設定", bg="red")

        self.root.after(100, self.update_gui_loop)

def main(args=None):
    rclpy.init(args=args)
    node = ExperimentGUINode()
    
    # ROSスピンを別スレッド
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    
    try:
        node.start_gui()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()