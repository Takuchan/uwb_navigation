#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import sys

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

from filterpy.kalman import ExtendedKalmanFilter

from .serial_filter import SerialFilter

class UwbSlamEkfNode(Node):
    def __init__(self):
        super().__init__('uwb_slam_ekf_node')

        # --- パラメータ宣言 ---
        self.declare_parameter('com_port', '/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 3000000)
        self.declare_parameter('uwb_timeout', 0.1)

        # アンカーの初期位置をパラメータから取得
        # これらはフィルターの初期推測値として使用される
        self.declare_parameter('anchor_a_pos', [0.0, 5.0])
        self.declare_parameter('anchor_b_pos', [5.0, 5.0])
        self.declare_parameter('anchor_c_pos', [2.5, 0.0])
        
        # --- パラメータ読み込み ---
        com_port = self.get_parameter('com_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.uwb_timeout = self.get_parameter('uwb_timeout').get_parameter_value().double_value
        
        initial_anchor_pos = [
            self.get_parameter('anchor_a_pos').get_parameter_value().double_array_value,
            self.get_parameter('anchor_b_pos').get_parameter_value().double_array_value,
            self.get_parameter('anchor_c_pos').get_parameter_value().double_array_value,
        ]
        self.get_logger().info(f"Initial anchor positions (guess): {initial_anchor_pos}")

        # --- EKFの状態空間モデル定義 ---
        self.ROBOT_STATE_DIM = 5  # x, y, theta, v, w
        self.ANCHOR_STATE_DIM = 2 # x, y
        self.NUM_ANCHORS = 3
        dim_x = self.ROBOT_STATE_DIM + self.NUM_ANCHORS * self.ANCHOR_STATE_DIM # 5 + 3*2 = 11

        self.ekf = ExtendedKalmanFilter(dim_x=dim_x, dim_z=1)

        # --- 状態ベクトル x の初期化 ---
        # [robot_x, robot_y, robot_theta, v, w, a_x, a_y, b_x, b_y, c_x, c_y]
        initial_robot_state = np.zeros(self.ROBOT_STATE_DIM)
        initial_anchor_states = np.array(initial_anchor_pos).flatten()
        self.ekf.x = np.concatenate([initial_robot_state, initial_anchor_states]).reshape(dim_x, 1)

        # --- 共分散行列 P の初期化 ---
        # ロボットの状態の不確かさは小さく設定
        robot_P = np.diag([0.1**2, 0.1**2, np.deg2rad(1.0)**2, 0.1**2, np.deg2rad(1.0)**2])
        # ★重要★ アンカーの初期位置の不確かさは非常に大きく設定
        # これによりフィルターはUWB測定に基づいてアンカー位置を積極的に調整する
        anchor_P = np.diag(np.tile([100.0**2, 100.0**2], self.NUM_ANCHORS))
        # ブロック行列としてPを組み立てる
        self.ekf.P = np.block([
            [robot_P, np.zeros((self.ROBOT_STATE_DIM, self.NUM_ANCHORS * self.ANCHOR_STATE_DIM))],
            [np.zeros((self.NUM_ANCHORS * self.ANCHOR_STATE_DIM, self.ROBOT_STATE_DIM)), anchor_P]
        ])

        # --- プロセスノイズ Q の設定 ---
        # ノイズはロボットの状態(特に速度)にのみ影響し、アンカーは静的と仮定
        self.ekf.Q = np.zeros((dim_x, dim_x))
        q_robot = np.diag([0.01**2, 0.01**2, np.deg2rad(0.1)**2, 0.1**2, np.deg2rad(1)**2])
        self.ekf.Q[0:self.ROBOT_STATE_DIM, 0:self.ROBOT_STATE_DIM] = q_robot

        # --- 測定ノイズ R の設定 ---
        self.R_uwb = np.diag([0.1**2]) # UWBの測定誤差分散 (標準偏差10cmを仮定)

        # --- シリアル通信のセットアップ ---
        self.uwb_reader = SerialFilter(com_port, baud_rate, self.NUM_ANCHORS)
        if not self.uwb_reader.connect_serial():
            self.get_logger().error("シリアルポートへの接続に失敗しました。ノードを終了します。")
            sys.exit(1)

        # --- ROS 2通信インターフェース ---
        self.last_time = self.get_clock().now()
        self.latest_odom = None
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.filtered_odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.5, self.timer_callback) # 20Hzでメインループを実行

    def destroy_node(self):
        self.get_logger().info("ノードをシャットダウンします。")
        self.uwb_reader.disconnect_serial()
        super().destroy_node()

    def odom_callback(self, msg):
        self.latest_odom = msg
        
    # --- U-SLAM EKFの計算モデル ---
    def state_transition_function(self, x, dt):
        """状態遷移関数 f(x) - アンカーの位置は変化しない"""
        x_new = np.copy(x)
        theta, v, w = x[2, 0], x[3, 0], x[4, 0]
        
        # ロボットの状態のみ更新
        x_new[0] = x[0, 0] + v * dt * np.cos(theta)
        x_new[1] = x[1, 0] + v * dt * np.sin(theta)
        x_new[2] = x[2, 0] + w * dt
        
        # アンカーの状態は静的なので変化しない (x_new[5:] = x[5:])
        return x_new

    def state_transition_jacobian(self, x, dt):
        """状態遷移ヤコビ行列 F - アンカーの状態遷移は単位行列"""
        F = np.eye(self.ekf.dim_x) # まずは単位行列で初期化
        theta, v = x[2, 0], x[3, 0]
        
        # ロボットに対応する部分のみを更新
        F[0, 2] = -v * dt * np.sin(theta)
        F[0, 3] = dt * np.cos(theta)
        F[1, 2] = v * dt * np.cos(theta)
        F[1, 3] = dt * np.sin(theta)
        F[2, 4] = dt
        return F

    def h_uslam(self, x, anchor_index):
        """測定関数 h(x) - ロボットと指定アンカー間の距離"""
        robot_pos = x[0:2]
        anchor_start_idx = self.ROBOT_STATE_DIM + anchor_index * self.ANCHOR_STATE_DIM
        anchor_pos = x[anchor_start_idx : anchor_start_idx + self.ANCHOR_STATE_DIM]
        
        return np.array([[np.linalg.norm(robot_pos - anchor_pos)]])

    def H_uslam(self, x, anchor_index):
        """測定ヤコビ行列 H - h(x)を状態ベクトルxで偏微分"""
        H = np.zeros((1, self.ekf.dim_x))
        robot_pos = x[0:2].flatten()
        anchor_start_idx = self.ROBOT_STATE_DIM + anchor_index * self.ANCHOR_STATE_DIM
        anchor_pos = x[anchor_start_idx : anchor_start_idx + self.ANCHOR_STATE_DIM].flatten()
        
        delta = robot_pos - anchor_pos
        dist = np.linalg.norm(delta)
        if dist < 1e-6: return H

        # ロボット位置(x, y)に関する偏微分
        H[0, 0] = delta[0] / dist
        H[0, 1] = delta[1] / dist
        
        # 対応するアンカー位置(x, y)に関する偏微分
        H[0, anchor_start_idx] = -delta[0] / dist
        H[0, anchor_start_idx + 1] = -delta[1] / dist
        
        return H

    # --- メインループ ---
    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        if dt <= 0: return

        # --- 予測(Predict)ステップ ---
        if self.latest_odom:
            self.ekf.x[3,0] = self.latest_odom.twist.twist.linear.x
            self.ekf.x[4,0] = self.latest_odom.twist.twist.angular.z
        
        F = self.state_transition_jacobian(self.ekf.x, dt)
        self.ekf.P = F @ self.ekf.P @ F.T + self.ekf.Q
        self.ekf.x = self.state_transition_function(self.ekf.x, dt)

        # --- 更新(Update)ステップ ---
        anchor_data = self.uwb_reader.read_anchor_data_snapshot(timeout=self.uwb_timeout)
        if anchor_data:
            for twr_id_str, data in anchor_data.items():
                if data and data['nlos_los'] == 'LOS':
                    twr_index = int(twr_id_str.replace('TWR', ''))
                    z = np.array([[data['distance']]])
                    
                    self.ekf.update(z, HJacobian=self.H_uslam, Hx=self.h_uslam, R=self.R_uwb,
                                    args=(twr_index,), hx_args=(twr_index,))

        # --- 結果の出力 ---
        self.publish_odometry(current_time)
        self.publish_tf(current_time)
        
    def publish_odometry(self, current_time):
        """EKFの結果（ロボットの状態のみ）をOdometryとして発行"""
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg(); odom_msg.header.frame_id = 'map'; odom_msg.child_frame_id = 'base_link'
        x, y, theta = self.ekf.x[0,0], self.ekf.x[1,0], self.ekf.x[2,0]
        odom_msg.pose.pose.position.x = x; odom_msg.pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, theta)
        odom_msg.pose.pose.orientation.x = q[0]; odom_msg.pose.pose.orientation.y = q[1]; odom_msg.pose.pose.orientation.z = q[2]; odom_msg.pose.pose.orientation.w = q[3]
        # 共分散もロボットの部分だけを抽出
        P_pose = self.ekf.P[0:3, 0:3]; indices = [0, 1, 5] # x, y, yaw in 6x6 covariance
        odom_msg.pose.covariance = np.zeros(36)
        for i, row in enumerate(indices):
            for j, col in enumerate(indices): odom_msg.pose.covariance[row*6 + col] = P_pose[i, j]
        odom_msg.twist.twist.linear.x = self.ekf.x[3, 0]; odom_msg.twist.twist.angular.z = self.ekf.x[4, 0]
        self.filtered_odom_pub.publish(odom_msg)
        
    def publish_tf(self, current_time):
        t = TransformStamped()
        t.header.stamp = current_time.to_msg(); t.header.frame_id = 'map'; t.child_frame_id = 'base_link'
        x, y, theta = self.ekf.x[0,0], self.ekf.x[1,0], self.ekf.x[2,0]
        t.transform.translation.x = x; t.transform.translation.y = y; t.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, theta)
        t.transform.rotation.x = q[0]; t.transform.rotation.y = q[1]; t.transform.rotation.z = q[2]; t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = UwbSlamEkfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

