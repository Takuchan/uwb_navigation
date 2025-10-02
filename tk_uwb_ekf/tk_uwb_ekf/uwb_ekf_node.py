#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import sys

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

from filterpy.kalman import ExtendedKalmanFilter
import math 

from .serialandFilter import SerialFilter

class UwbEkfNode(Node):
    def __init__(self):
        super().__init__('uwb_ekf_node')

        # ---変更点: シリアル通信用のパラメータを追加---
        self.declare_parameter('com_port', '/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 3000000)
        self.declare_parameter('num_anchors', 3)
        self.declare_parameter('uwb_timeout', 0.05) # UWBデータ読み取りタイムアウト

        self.declare_parameter('anchor_a_pos', [0.0, 5.0])
        self.declare_parameter('anchor_b_pos', [5.0, 5.0])
        self.declare_parameter('anchor_c_pos', [2.5, 0.0])
        
        # ---変更点: パラメータを取得---
        com_port = self.get_parameter('com_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        num_anchors = self.get_parameter('num_anchors').get_parameter_value().integer_value
        self.uwb_timeout = self.get_parameter('uwb_timeout').get_parameter_value().double_value
        
        # アンカーの座標をパラメータから取得
        # TWR0 -> a, TWR1 -> b, ... のようにマッピング
        self.anchor_positions = {
            'a': self.get_parameter('anchor_a_pos').get_parameter_value().double_array_value,
            'b': self.get_parameter('anchor_b_pos').get_parameter_value().double_array_value,
            'c': self.get_parameter('anchor_c_pos').get_parameter_value().double_array_value,
        }
        self.anchor_map = {0: 'a', 1: 'b', 2: 'c'}
        self.get_logger().info(f"Anchor positions: {self.anchor_positions}")

        # ---変更点: SerialFilterをインスタンス化し、接続---
        self.uwb_reader = SerialFilter(com_port, baud_rate, num_anchors)
        if not self.uwb_reader.connect_serial():
            self.get_logger().error("シリアルポートへの接続に失敗しました。ノードを終了します。")
            # rclpy.shutdown()を直接呼ぶのは推奨されないため、タイマーを止めるなどして終了を促す
            # ここではシンプルにsys.exit()を使用（本来はよりクリーンな終了処理が望ましい）
            sys.exit(1)

        # --- EKFの初期設定 (変更なし) ---
        self.ekf = ExtendedKalmanFilter(dim_x=5, dim_z=1)
        self.ekf.x = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]]).T
        self.ekf.P = np.diag([1.0, 1.0, np.pi, 1.0, 0.5])
        self.ekf.Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1])
        self.R_uwb = np.diag([0.2**2]) 

        self.last_time = self.get_clock().now()
        self.latest_odom = None
        self.latest_imu = None

        # Subscribers (UWB関連は削除)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        # Publishers
        self.filtered_odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # メインループタイマー (20Hz) - UWBの読み取り周期に合わせる
        self.timer = self.create_timer(0.05, self.timer_callback)

    # ---変更点: ノード終了時にシリアルを切断---
    def destroy_node(self):
        self.get_logger().info("ノードをシャットダウンします。")
        self.uwb_reader.disconnect_serial()
        super().destroy_node()

    # --- コールバック関数 ---
    def odom_callback(self, msg):
        self.latest_odom = msg

    def imu_callback(self, msg):
        self.latest_imu = msg
        
    # --- EKFの計算モデル (変更なし) ---
    def state_transition_function(self, x, dt):
        x_new = np.copy(x)
        theta, v, w = x[2, 0], x[3, 0], x[4, 0]
        x_new[0] = x[0, 0] + v * dt * np.cos(theta)
        x_new[1] = x[1, 0] + v * dt * np.sin(theta)
        x_new[2] = x[2, 0] + w * dt
        return x_new

    def state_transition_jacobian(self, x, dt):
        theta, v = x[2, 0], x[3, 0]
        F = np.eye(5)
        F[0, 2] = -v * dt * np.sin(theta)
        F[0, 3] = dt * np.cos(theta)
        F[1, 2] = v * dt * np.cos(theta)
        F[1, 3] = dt * np.sin(theta)
        F[2, 4] = dt
        return F

    def h_uwb(self, x, anchor_pos): #3平方の定理を行っている。
        dx = x[0, 0] - anchor_pos[0]
        dy = x[1, 0] - anchor_pos[1]
        return np.array([[np.sqrt(dx**2 + dy**2)]])

    def H_uwb(self, x, anchor_pos):
        dx = x[0, 0] - anchor_pos[0]
        dy = x[1, 0] - anchor_pos[1]
        dist = np.sqrt(dx**2 + dy**2)
        if dist < 1e-6: return np.zeros((1, 5))
        H = np.zeros((1, 5)); H[0, 0] = dx / dist; H[0, 1] = dy / dist
        return H

    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0:
            return

        if self.latest_odom:
            self.ekf.x[3,0] = self.latest_odom.twist.twist.linear.x
            self.ekf.x[4,0] = self.latest_odom.twist.twist.angular.z
        
        F = self.state_transition_jacobian(self.ekf.x, dt)
        
        self.ekf.P = F @ self.ekf.P @ F.T + self.ekf.Q
        self.ekf.x = self.state_transition_function(self.ekf.x, dt) # ホイールお度目取りの情報をもとに、「次はここにいるはずだ」という計算だけが行われる。

        anchor_data = self.uwb_reader.read_anchor_data_snapshot(timeout=self.uwb_timeout)
        
        # ここでAnchorData取得できたら！！！
        if anchor_data:
            for twr_id_str, data in anchor_data.items():
                if data: # データがNoneでないか確認
                    twr_index = int(twr_id_str.replace('TWR', '')) 
                    
                    if data['nlos_los'] == 'LOS':
                        anchor_id = self.anchor_map.get(twr_index)
                        if anchor_id:
                            anchor_pos = self.anchor_positions[anchor_id]
                            z = np.array([[data['distance']]])
                            
                            self.ekf.update(z, HJacobian=self.H_uwb, Hx=self.h_uwb, R=self.R_uwb,
                                            args=(anchor_pos,), hx_args=(anchor_pos,))

        # --- 結果の出力 (変更なし) ---
        self.publish_odometry(current_time)
        self.publish_tf(current_time)
        
    def publish_odometry(self, current_time):
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg(); odom_msg.header.frame_id = 'map'; odom_msg.child_frame_id = 'base_footprint'
        x, y, theta = self.ekf.x[0,0], self.ekf.x[1,0], self.ekf.x[2,0]
        odom_msg.pose.pose.position.x = x; odom_msg.pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, theta - math.pi / 2)
        odom_msg.pose.pose.orientation.x = q[0]; odom_msg.pose.pose.orientation.y = q[1]; odom_msg.pose.pose.orientation.z = q[2]; odom_msg.pose.pose.orientation.w = q[3]
        odom_msg.pose.covariance = np.zeros(36)
        P_pose = self.ekf.P[0:3, 0:3]; indices = [0, 1, 5]
        for i, row in enumerate(indices):
            for j, col in enumerate(indices): odom_msg.pose.covariance[row*6 + col] = P_pose[i, j]
        odom_msg.twist.twist.linear.x = self.ekf.x[3, 0]; odom_msg.twist.twist.angular.z = self.ekf.x[4, 0]
        self.filtered_odom_pub.publish(odom_msg)
        
    def publish_tf(self, current_time):
        t = TransformStamped()
        t.header.stamp = current_time.to_msg(); t.header.frame_id = 'map'; t.child_frame_id = 'base_link'
        x, y, theta = self.ekf.x[0,0], self.ekf.x[1,0], self.ekf.x[2,0]
        t.transform.translation.x = x; t.transform.translation.y = y; t.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, theta - math.pi / 2)
        t.transform.rotation.x = q[0]; t.transform.rotation.y = q[1]; t.transform.rotation.z = q[2]; t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = UwbEkfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()