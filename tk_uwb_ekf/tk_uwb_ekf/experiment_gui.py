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
from datetime import datetime
import numpy as np

from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class ExperimentGUINode(Node):
    """
    全実験（位置精度・姿勢収束・データロギング）を統括するGUIノード。
    """
    def __init__(self):
        super().__init__('experiment_gui_node')
        
        # --- ROS通信設定 ---
        # 1. EKF推定位置の購読
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        # 2. Rvizクリック位置の購読（実験①用）
        self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_callback, 10)
        # 3. ロボット操作用パブリッシャー（実験②自動化用）
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- 内部変数 ---
        self.lock = threading.Lock()
        
        # ロボット状態
        self.robot_pose = None
        self.current_yaw = 0.0
        self.initial_yaw_ref = 0.0 # 相対角度計算用の基準
        self.use_relative_yaw = False
        
        # 実験①用
        self.target_point = None
        
        # 実験②（自動走行）用
        self.is_running = False
        self.start_pos = None
        self.travel_dist = 0.0
        self.log_data = [] # [(time, x, y, yaw, dist), ...]
        self.start_time = 0.0
        
        # 収束判定用パラメータ（GUIから更新）
        self.target_velocity = 0.5
        self.yaw_tolerance = 5.0     # ±度
        self.min_run_dist = 1.0      # 最低でもこれだけ走らないと停止しない（即時停止防止）
        self.convergence_count = 0   # 連続で範囲内に入った回数
        self.required_stable_count = 10 # 10回連続（約0.5~1秒）安定したら停止

        # --- GUI起動 ---
        self.gui_thread = threading.Thread(target=self.run_gui, daemon=True)
        self.gui_thread.start()
        
        # 制御ループタイマー (10Hz)
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Experiment GUI Node Started.")

    def odom_callback(self, msg: Odometry):
        with self.lock:
            self.robot_pose = msg.pose.pose
            
            # 姿勢(Quaternion) -> Yaw(Euler)
            q = msg.pose.pose.orientation
            (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
            deg = math.degrees(yaw)
            
            # 相対角度モードなら補正
            if self.use_relative_yaw:
                deg -= self.initial_yaw_ref
                # -180~180正規化
                while deg > 180: deg -= 360
                while deg < -180: deg += 360
            
            self.current_yaw = deg
            
            # 自動走行中の距離計算 & ログ記録
            if self.is_running and self.start_pos:
                curr_x = self.robot_pose.position.x
                curr_y = self.robot_pose.position.y
                
                # 走行距離計算 (ユークリッド距離)
                dx = curr_x - self.start_pos[0]
                dy = curr_y - self.start_pos[1]
                self.travel_dist = math.sqrt(dx**2 + dy**2)
                
                # データログ追加
                elapsed = time.time() - self.start_time
                self.log_data.append({
                    "time": elapsed,
                    "x": curr_x,
                    "y": curr_y,
                    "yaw": self.current_yaw,
                    "dist": self.travel_dist
                })

    def clicked_point_callback(self, msg: PointStamped):
        with self.lock:
            self.target_point = msg.point
        self.get_logger().info(f"Exp1 Target Set: {msg.point.x}, {msg.point.y}")

    def control_loop(self):
        """ 自動走行と停止判定を行うループ """
        if not self.is_running:
            return

        # 1. 指令速度の送信
        twist = Twist()
        twist.linear.x = self.target_velocity
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        # 2. 収束判定 (最低走行距離を超えてからチェック)
        if self.travel_dist > self.min_run_dist:
            if abs(self.current_yaw) < self.yaw_tolerance:
                self.convergence_count += 1
            else:
                self.convergence_count = 0 # リセット
            
            # 判定：指定回数連続で範囲内なら「収束」とみなして停止
            if self.convergence_count >= self.required_stable_count:
                self.get_logger().info(f"Converged! Yaw={self.current_yaw:.2f}, Dist={self.travel_dist:.2f}")
                self.stop_experiment_logic()

    # --- ロジック系 ---
    def reset_yaw_reference(self):
        with self.lock:
            if self.robot_pose:
                q = self.robot_pose.orientation
                (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
                self.initial_yaw_ref = math.degrees(yaw)
                self.use_relative_yaw = True
                self.get_logger().info("Yaw Reference Reset to 0.")

    def start_experiment_logic(self):
        if self.robot_pose is None:
            messagebox.showwarning("Error", "Odomデータが来ていません")
            return
            
        with self.lock:
            self.is_running = True
            self.start_time = time.time()
            self.log_data = []
            self.start_pos = (self.robot_pose.position.x, self.robot_pose.position.y)
            self.travel_dist = 0.0
            self.convergence_count = 0
            
            # パラメータの反映
            try:
                self.target_velocity = np.float64(self.entry_speed.get())
                self.yaw_tolerance = np.float64(self.entry_tol.get())
                self.min_run_dist = np.float64(self.entry_min_dist.get())
            except ValueError:
                pass # デフォルト値維持

        self.get_logger().info("Auto Run Started.")

    def stop_experiment_logic(self):
        """ 停止処理 """
        self.is_running = False
        # 停止コマンド送信
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Auto Run Stopped.")

    def save_csv(self):
        """ データのCSV保存 """
        filename = self.entry_filename.get()
        if not filename:
            messagebox.showwarning("Warning", "ファイル名を入力してください")
            return
        
        # 拡張子補完
        if not filename.endswith(".csv"):
            filename += ".csv"
            
        # 保存パス (ホームディレクトリ直下など)
        save_path = os.path.join(os.path.expanduser('~'), 'uwb_experiment_data', filename)
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        
        try:
            with self.lock:
                data_to_save = list(self.log_data) # コピー
                
            with open(save_path, 'w', newline='') as f:
                writer = csv.writer(f)
                # ヘッダー
                writer.writerow(["Time(s)", "X(m)", "Y(m)", "Yaw(deg)", "Distance(m)"])
                # データ
                for row in data_to_save:
                    writer.writerow([
                        f"{row['time']:.4f}",
                        f"{row['x']:.4f}",
                        f"{row['y']:.4f}",
                        f"{row['yaw']:.4f}",
                        f"{row['dist']:.4f}"
                    ])
            
            messagebox.showinfo("Success", f"保存しました:\n{save_path}")
        except Exception as e:
            messagebox.showerror("Error", f"保存失敗: {e}")

    # --- GUI構築 ---
    def run_gui(self):
        self.root = tk.Tk()
        self.root.title("UWBロボット実験統合GUI")
        self.root.geometry("450x650")
        
        style = ttk.Style()
        style.configure("Bold.TLabel", font=("Helvetica", 10, "bold"))
        
        # ============================
        # セクション1: 共通ステータス
        # ============================
        frame_status = ttk.LabelFrame(self.root, text="ロボット状態 (Realtime)", padding=10)
        frame_status.pack(fill="x", padx=10, pady=5)
        
        self.lbl_curr_pos = ttk.Label(frame_status, text="位置: X=0.00, Y=0.00")
        self.lbl_curr_pos.pack(anchor="w")
        
        self.lbl_curr_yaw = ttk.Label(frame_status, text="向き(Yaw): 0.00 deg", font=("Helvetica", 14, "bold"), foreground="blue")
        self.lbl_curr_yaw.pack(anchor="w")
        
        btn_reset_yaw = ttk.Button(frame_status, text="現在の向きを0度とする (Reset Yaw)", command=self.reset_yaw_reference)
        btn_reset_yaw.pack(fill="x", pady=2)

        # ============================
        # セクション2: 実験① 位置精度検証
        # ============================
        frame_exp1 = ttk.LabelFrame(self.root, text="実験①: 位置精度検証", padding=10)
        frame_exp1.pack(fill="x", padx=10, pady=5)
        
        self.lbl_exp1_target = ttk.Label(frame_exp1, text="目標(Rviz Click): 未設定")
        self.lbl_exp1_target.pack(anchor="w")
        
        self.lbl_exp1_diff = ttk.Label(frame_exp1, text="誤差: X=---, Y=---")
        self.lbl_exp1_diff.pack(anchor="w")
        
        self.lbl_exp1_total = ttk.Label(frame_exp1, text="直線距離誤差: --- m", font=("Helvetica", 12, "bold"))
        self.lbl_exp1_total.pack(anchor="w")

        # ============================
        # セクション3: 実験② 姿勢収束オートメーション
        # ============================
        frame_exp2 = ttk.LabelFrame(self.root, text="実験②: 姿勢収束自動計測", padding=10)
        frame_exp2.pack(fill="x", padx=10, pady=5)
        
        # 設定入力エリア
        frame_settings = ttk.Frame(frame_exp2)
        frame_settings.pack(fill="x")
        
        ttk.Label(frame_settings, text="速度(m/s):").grid(row=0, column=0, padx=5)
        self.entry_speed = ttk.Entry(frame_settings, width=5)
        self.entry_speed.insert(0, "0.5")
        self.entry_speed.grid(row=0, column=1)
        
        ttk.Label(frame_settings, text="許容誤差(±deg):").grid(row=0, column=2, padx=5)
        self.entry_tol = ttk.Entry(frame_settings, width=5)
        self.entry_tol.insert(0, "5.0")
        self.entry_tol.grid(row=0, column=3)
        
        ttk.Label(frame_settings, text="最低走行(m):").grid(row=1, column=0, padx=5, pady=5)
        self.entry_min_dist = ttk.Entry(frame_settings, width=5)
        self.entry_min_dist.insert(0, "1.0")
        self.entry_min_dist.grid(row=1, column=1, pady=5)
        
        # 実行ボタンエリア
        frame_ctrl = ttk.Frame(frame_exp2)
        frame_ctrl.pack(fill="x", pady=10)
        
        btn_start = tk.Button(frame_ctrl, text="計測開始 (自動走行)", bg="green", fg="white", font=("Helvetica", 10, "bold"), command=self.start_experiment_logic)
        btn_start.pack(side="left", fill="x", expand=True, padx=2)
        
        btn_stop = tk.Button(frame_ctrl, text="緊急停止", bg="red", fg="white", font=("Helvetica", 10, "bold"), command=self.stop_experiment_logic)
        btn_stop.pack(side="left", fill="x", expand=True, padx=2)
        
        # ステータス表示
        self.lbl_exp2_status = ttk.Label(frame_exp2, text="状態: 待機中", font=("Helvetica", 11))
        self.lbl_exp2_status.pack(anchor="w")
        
        self.lbl_exp2_dist = ttk.Label(frame_exp2, text="走行距離: 0.00 m")
        self.lbl_exp2_dist.pack(anchor="w")

        # データ保存エリア
        frame_save = ttk.Frame(frame_exp2)
        frame_save.pack(fill="x", pady=(10, 0))
        
        ttk.Label(frame_save, text="保存ファイル名:").pack(side="left")
        self.entry_filename = ttk.Entry(frame_save, width=15)
        self.entry_filename.insert(0, "exp2_run1")
        self.entry_filename.pack(side="left", padx=5)
        
        btn_save = ttk.Button(frame_save, text="CSV保存", command=self.save_csv)
        btn_save.pack(side="left")
        
        # 定期更新
        self.update_gui_loop()
        self.root.mainloop()

    def update_gui_loop(self):
        # ROSデータの反映
        with self.lock:
            # 共通ステータス
            if self.robot_pose:
                self.lbl_curr_pos.config(text=f"位置: X={self.robot_pose.position.x:.2f}, Y={self.robot_pose.position.y:.2f}")
            self.lbl_curr_yaw.config(text=f"向き(Yaw): {self.current_yaw:.2f} deg")
            
            # 実験①
            if self.robot_pose and self.target_point:
                dx = abs(self.target_point.x - self.robot_pose.position.x)
                dy = abs(self.target_point.y - self.robot_pose.position.y)
                total = math.sqrt(dx**2 + dy**2)
                
                self.lbl_exp1_target.config(text=f"目標: X={self.target_point.x:.2f}, Y={self.target_point.y:.2f}")
                self.lbl_exp1_diff.config(text=f"誤差: X={dx:.3f}m, Y={dy:.3f}m")
                self.lbl_exp1_total.config(text=f"直線距離誤差: {total:.3f} m")
            
            # 実験②
            status_txt = "走行中 (データ記録中)..." if self.is_running else "停止 (データ保持中)"
            if not self.is_running and len(self.log_data) == 0:
                status_txt = "待機中"
            
            self.lbl_exp2_status.config(text=f"状態: {status_txt}", foreground="green" if self.is_running else "black")
            self.lbl_exp2_dist.config(text=f"走行距離: {self.travel_dist:.3f} m")

        self.root.after(100, self.update_gui_loop)

def main(args=None):
    rclpy.init(args=args)
    node = ExperimentGUINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_experiment_logic() # 安全のため停止
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()