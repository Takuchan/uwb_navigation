#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import math
import csv
import time
import os
import json
from datetime import datetime

from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf_transformations import euler_from_quaternion

class ExperimentGUINode(Node):
    """
    全実験（位置精度・姿勢評価・データロギング）を統括するGUIノード。
    距離指定停止機能とUWB詳細ログ機能を追加。
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
        # 4. ロボット操作用（実験②自動化用）
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- 内部変数 ---
        self.lock = threading.Lock()
        
        # ロボット状態
        self.robot_pose = None
        self.current_yaw = 0.0      # 生のYaw角 (Map座標系)
        self.relative_yaw = 0.0     # リセット基準からの相対Yaw
        self.initial_yaw_ref = 0.0  # 角度リセット時の基準値
        
        # UWBデータ保持用
        self.latest_uwb_raw = {}     # 最新のJSONデータ辞書
        self.uwb_log_str = ""        # CSV記録用に整形した文字列
        
        # 実験①用
        self.target_point = None
        
        # 実験②（自動走行）用
        self.is_running = False
        self.start_pos = None
        self.travel_dist = 0.0
        self.log_data = [] 
        self.start_time = 0.0
        
        # 設定パラメータ（GUIから取得）
        self.target_velocity = 0.5
        self.target_distance = 3.0   # 目標走行距離 (m)
        
        # --- GUI起動 ---
        self.gui_thread = threading.Thread(target=self.run_gui, daemon=True)
        self.gui_thread.start()
        
        # 制御ループ (20Hz) - 停止精度向上のため少し早める
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Experiment GUI Node Started (Distance Mode).")

    def uwb_callback(self, msg: String):
        """ /uwb_data_json を購読して、ログ用の文字列を生成する """
        with self.lock:
            try:
                data = json.loads(msg.data)
                self.latest_uwb_raw = data
                
                # CSV記録用に整形: "A0:LOS(1.2m)|A1:nLOS(3.5m)..."
                log_parts = []
                # キー(TWR0, TWR1...)でソートして順序を固定
                for key in sorted(data.keys()):
                    val = data[key]
                    if val is None: continue
                    
                    # マッピング TWR0 -> A0 (簡易変換)
                    idx = key.replace("TWR", "A") 
                    status = val.get('nlos_los', 'Unk')
                    dist = val.get('distance', 0.0)
                    
                    # 例: A0:LOS(3.45)
                    log_parts.append(f"{idx}:{status}({dist:.2f})")
                
                self.uwb_log_str = "|".join(log_parts)
                
            except json.JSONDecodeError:
                pass

    def odom_callback(self, msg: Odometry):
        with self.lock:
            self.robot_pose = msg.pose.pose
            
            # 姿勢(Quaternion) -> Yaw(Euler)
            q = msg.pose.pose.orientation
            (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
            deg = math.degrees(yaw)
            self.current_yaw = deg
            
            # 相対角度計算 (-180 ~ 180)
            diff = deg - self.initial_yaw_ref
            while diff > 180: diff -= 360
            while diff < -180: diff += 360
            self.relative_yaw = diff
            
            # 自動走行中の処理
            if self.is_running and self.start_pos:
                curr_x = self.robot_pose.position.x
                curr_y = self.robot_pose.position.y
                
                # 走行距離計算
                dx = curr_x - self.start_pos[0]
                dy = curr_y - self.start_pos[1]
                self.travel_dist = math.sqrt(dx**2 + dy**2)
                
                # ログ記録
                elapsed = time.time() - self.start_time
                self.log_data.append({
                    "time": elapsed,
                    "x": curr_x,
                    "y": curr_y,
                    "yaw_raw": self.current_yaw,
                    "yaw_rel": self.relative_yaw,
                    "dist": self.travel_dist,
                    "uwb_detail": self.uwb_log_str # その瞬間のUWB状況
                })

    def clicked_point_callback(self, msg: PointStamped):
        with self.lock:
            self.target_point = msg.point
        self.get_logger().info(f"Exp1 Target Set: {msg.point.x}, {msg.point.y}")

    def control_loop(self):
        """ 距離監視と自動停止 """
        if not self.is_running:
            return

        # 1. 停止判定: 目標距離を超えたら停止
        if self.travel_dist >= self.target_distance:
            self.get_logger().info(f"Target Distance Reached! ({self.travel_dist:.3f}m)")
            self.stop_experiment_logic()
            return

        # 2. 速度指令 (直進)
        twist = Twist()
        twist.linear.x = self.target_velocity
        twist.angular.z = 0.0 # 補正なしの純粋な直進指令
        self.cmd_vel_pub.publish(twist)

    # --- ロジック系 ---
    def reset_yaw_reference(self):
        with self.lock:
            if self.robot_pose:
                q = self.robot_pose.orientation
                (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
                self.initial_yaw_ref = math.degrees(yaw)
                self.get_logger().info(f"Yaw Reference Reset. 0 deg = {self.initial_yaw_ref:.2f} (Map)")

    def start_experiment_logic(self):
        if self.robot_pose is None:
            messagebox.showwarning("Error", "Odomデータが来ていません")
            return
            
        with self.lock:
            # パラメータ取得
            try:
                self.target_velocity = float(self.entry_speed.get())
                self.target_distance = float(self.entry_dist.get())
            except ValueError:
                messagebox.showerror("Error", "数値が不正です")
                return

            self.is_running = True
            self.start_time = time.time()
            self.log_data = []
            self.start_pos = (self.robot_pose.position.x, self.robot_pose.position.y)
            self.travel_dist = 0.0
            
        self.get_logger().info(f"Auto Run Started. Target: {self.target_distance}m")

    def stop_experiment_logic(self):
        """ 停止処理 """
        self.is_running = False
        # 停止コマンド送信 (念のため数回送る)
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Auto Run Stopped.")
        
        # 終了時の角度判定ログ
        final_yaw = self.relative_yaw
        judge = "OK (Parallel)" if abs(final_yaw) < 5.0 else "NG (Drifted)"
        self.get_logger().info(f"Final Yaw Error: {final_yaw:.2f} deg -> {judge}")

    def save_csv(self):
        """ データのCSV保存 (UWB詳細付き) """
        filename = self.entry_filename.get()
        if not filename:
            messagebox.showwarning("Warning", "ファイル名を入力してください")
            return
        
        if not filename.endswith(".csv"): filename += ".csv"
            
        save_path = os.path.join(os.path.expanduser('~'), 'uwb_experiment_data', filename)
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        
        try:
            with self.lock:
                data_to_save = list(self.log_data)
                
            with open(save_path, 'w', newline='') as f:
                writer = csv.writer(f)
                # ヘッダー: UWBの詳細列を追加
                writer.writerow(["Time(s)", "X(m)", "Y(m)", "Yaw_Raw(deg)", "Yaw_Rel(deg)", "Distance(m)", "UWB_Status"])
                
                for row in data_to_save:
                    writer.writerow([
                        f"{row['time']:.4f}",
                        f"{row['x']:.4f}",
                        f"{row['y']:.4f}",
                        f"{row['yaw_raw']:.4f}",
                        f"{row['yaw_rel']:.4f}",
                        f"{row['dist']:.4f}",
                        row['uwb_detail'] # 例: A0:LOS(2.5)|A1:nLOS(5.0)
                    ])
            
            messagebox.showinfo("Success", f"保存しました:\n{save_path}\nデータ数: {len(data_to_save)}")
        except Exception as e:
            messagebox.showerror("Error", f"保存失敗: {e}")

    # --- GUI構築 ---
    def run_gui(self):
        self.root = tk.Tk()
        self.root.title("UWB実験GUI (距離停止版)")
        self.root.geometry("500x700")
        
        style = ttk.Style()
        style.configure("Bold.TLabel", font=("Helvetica", 10, "bold"))
        
        # ============================
        # セクション1: リアルタイムステータス
        # ============================
        frame_status = ttk.LabelFrame(self.root, text="ロボット状態", padding=10)
        frame_status.pack(fill="x", padx=10, pady=5)
        
        self.lbl_curr_pos = ttk.Label(frame_status, text="位置: X=0.00, Y=0.00")
        self.lbl_curr_pos.pack(anchor="w")
        
        # 角度表示
        frame_yaw = ttk.Frame(frame_status)
        frame_yaw.pack(fill="x", pady=5)
        self.lbl_yaw_raw = ttk.Label(frame_yaw, text="Map Yaw: 0.00°", font=("Helvetica", 10))
        self.lbl_yaw_raw.pack(side="left", padx=5)
        self.lbl_yaw_rel = ttk.Label(frame_yaw, text="Rel Yaw: 0.00°", font=("Helvetica", 14, "bold"), foreground="blue")
        self.lbl_yaw_rel.pack(side="left", padx=5)

        # 平行判定インジケータ
        self.lbl_parallel_judge = tk.Label(frame_yaw, text="---", bg="gray", fg="white", width=10)
        self.lbl_parallel_judge.pack(side="right", padx=5)

        btn_reset_yaw = ttk.Button(frame_status, text="現在の向きを0度とする (Reset Rel Yaw)", command=self.reset_yaw_reference)
        btn_reset_yaw.pack(fill="x", pady=2)
        
        # UWB状態簡易表示
        self.lbl_uwb_status = ttk.Label(frame_status, text="UWB: ---", font=("Courier", 8))
        self.lbl_uwb_status.pack(anchor="w", pady=2)

        # ============================
        # セクション2: 実験① 位置精度
        # ============================
        frame_exp1 = ttk.LabelFrame(self.root, text="実験①: 位置精度 (壁クリック計測)", padding=10)
        frame_exp1.pack(fill="x", padx=10, pady=5)
        
        self.lbl_exp1_target = ttk.Label(frame_exp1, text="目標: 未設定")
        self.lbl_exp1_target.pack(anchor="w")
        self.lbl_exp1_total = ttk.Label(frame_exp1, text="直線距離誤差: --- m", font=("Helvetica", 12, "bold"))
        self.lbl_exp1_total.pack(anchor="w")

        # ============================
        # セクション3: 実験② 距離指定停止
        # ============================
        frame_exp2 = ttk.LabelFrame(self.root, text="実験②: 距離指定自動停止", padding=10)
        frame_exp2.pack(fill="x", padx=10, pady=5)
        
        # 設定エリア
        frame_set = ttk.Frame(frame_exp2)
        frame_set.pack(fill="x")
        
        ttk.Label(frame_set, text="速度(m/s):").grid(row=0, column=0, padx=5)
        self.entry_speed = ttk.Entry(frame_set, width=6)
        self.entry_speed.insert(0, "0.5")
        self.entry_speed.grid(row=0, column=1)
        
        ttk.Label(frame_set, text="停止距離(m):").grid(row=0, column=2, padx=5)
        self.entry_dist = ttk.Entry(frame_set, width=6)
        self.entry_dist.insert(0, "3.0") # 初期値3m
        self.entry_dist.grid(row=0, column=3)
        
        # プリセットボタン
        frame_preset = ttk.Frame(frame_exp2)
        frame_preset.pack(fill="x", pady=5)
        ttk.Label(frame_preset, text="プリセット:").pack(side="left")
        ttk.Button(frame_preset, text="3m", command=lambda: self.set_dist(3.0), width=4).pack(side="left", padx=2)
        ttk.Button(frame_preset, text="5m", command=lambda: self.set_dist(5.0), width=4).pack(side="left", padx=2)
        ttk.Button(frame_preset, text="7m", command=lambda: self.set_dist(7.0), width=4).pack(side="left", padx=2)

        # コントロール
        frame_ctrl = ttk.Frame(frame_exp2)
        frame_ctrl.pack(fill="x", pady=10)
        
        btn_start = tk.Button(frame_ctrl, text="計測開始 (GO)", bg="green", fg="white", font=("Helvetica", 12, "bold"), command=self.start_experiment_logic)
        btn_start.pack(side="left", fill="x", expand=True, padx=5)
        
        btn_stop = tk.Button(frame_ctrl, text="STOP", bg="red", fg="white", font=("Helvetica", 12, "bold"), command=self.stop_experiment_logic)
        btn_stop.pack(side="left", padx=5)
        
        # 進行状況
        self.lbl_exp2_progress = ttk.Label(frame_exp2, text="走行距離: 0.00 m / 目標: --- m")
        self.lbl_exp2_progress.pack(anchor="w")
        self.progress_bar = ttk.Progressbar(frame_exp2, orient="horizontal", length=100, mode="determinate")
        self.progress_bar.pack(fill="x", pady=5)

        # 保存エリア
        frame_save = ttk.Frame(frame_exp2)
        frame_save.pack(fill="x", pady=(10, 0))
        ttk.Label(frame_save, text="ファイル名:").pack(side="left")
        self.entry_filename = ttk.Entry(frame_save, width=15)
        self.entry_filename.insert(0, "run_3m_try1")
        self.entry_filename.pack(side="left", padx=5)
        ttk.Button(frame_save, text="CSV保存", command=self.save_csv).pack(side="left")
        
        self.update_gui_loop()
        self.root.mainloop()

    def set_dist(self, val):
        """ プリセットボタン用 """
        self.entry_dist.delete(0, tk.END)
        self.entry_dist.insert(0, str(val))

    def update_gui_loop(self):
        with self.lock:
            # ロボット位置
            if self.robot_pose:
                self.lbl_curr_pos.config(text=f"位置: X={self.robot_pose.position.x:.2f}, Y={self.robot_pose.position.y:.2f}")
            
            # 角度更新
            self.lbl_yaw_raw.config(text=f"Map: {self.current_yaw:.1f}°")
            self.lbl_yaw_rel.config(text=f"Rel: {self.relative_yaw:.1f}°")
            
            # 平行判定 (0度に近いか)
            if abs(self.relative_yaw) < 5.0:
                self.lbl_parallel_judge.config(text="OK", bg="green")
            else:
                self.lbl_parallel_judge.config(text="BAD", bg="red")
                
            # UWBステータス (長すぎるので切り詰め)
            disp_str = self.uwb_log_str if len(self.uwb_log_str) < 60 else self.uwb_log_str[:57] + "..."
            if not disp_str: disp_str = "No Data"
            self.lbl_uwb_status.config(text=f"UWB: {disp_str}")
            
            # 実験①
            if self.robot_pose and self.target_point:
                dx = self.target_point.x - self.robot_pose.position.x
                dy = self.target_point.y - self.robot_pose.position.y
                total = math.sqrt(dx**2 + dy**2)
                self.lbl_exp1_target.config(text=f"目標: X={self.target_point.x:.2f}, Y={self.target_point.y:.2f}")
                self.lbl_exp1_total.config(text=f"直線距離誤差: {total:.3f} m")
            
            # 実験② プログレスバー
            self.lbl_exp2_progress.config(text=f"走行距離: {self.travel_dist:.2f} m / 目標: {self.target_distance:.1f} m")
            if self.target_distance > 0:
                percent = (self.travel_dist / self.target_distance) * 100
                self.progress_bar['value'] = min(percent, 100)

        self.root.after(100, self.update_gui_loop)

def main(args=None):
    rclpy.init(args=args)
    node = ExperimentGUINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_experiment_logic()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()