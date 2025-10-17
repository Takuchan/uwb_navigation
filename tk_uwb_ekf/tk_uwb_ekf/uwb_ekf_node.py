#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import sys
import json
from collections import deque # <<< 変更点: dequeを追加

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

from filterpy.kalman import ExtendedKalmanFilter
import math 

class UwbEkfNode(Node):
    def __init__(self):
        super().__init__('uwb_ekf_node')

        self.declare_parameter('com_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 3000000)
        self.declare_parameter('num_anchors', 3)
        self.declare_parameter('uwb_timeout', 0.05)

        self.declare_parameter('anchor_a_pos', [0.0, 5.0])
        self.declare_parameter('anchor_b_pos', [5.0, 5.0])
        self.declare_parameter('anchor_c_pos', [2.5, 0.0])

        ### <<< 変更点: nLOSデータフィルタリング用のパラメータを追加 >>>
        self.declare_parameter('nlos_history_size', 5) # 距離データの履歴を保持する数
        self.declare_parameter('nlos_variance_threshold', 0.05) # nLOSデータを使用するかの判断に使う分散のしきい値 (標準偏差で約0.22m)
        self.declare_parameter('R_nlos_multiplier', 4.0) # nLOSデータを使う際のR値の倍率 (信頼度を下げる)

        self.nlos_history_size = self.get_parameter('nlos_history_size').get_parameter_value().integer_value
        self.nlos_variance_threshold = self.get_parameter('nlos_variance_threshold').get_parameter_value().double_value
        self.R_nlos_multiplier = self.get_parameter('R_nlos_multiplier').get_parameter_value().double_value
        ### <<< ここまで >>>

        self.anchor_positions = {
            'a': self.get_parameter('anchor_a_pos').get_parameter_value().double_array_value,
            'b': self.get_parameter('anchor_b_pos').get_parameter_value().double_array_value,
            'c': self.get_parameter('anchor_c_pos').get_parameter_value().double_array_value,
        }
        self.anchor_map = {0: 'a', 1: 'b', 2: 'c'}
        self.get_logger().info(f"Anchor positions: {self.anchor_positions}")

        self.ekf = ExtendedKalmanFilter(dim_x=5, dim_z=1)
        self.ekf.x = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]]).T
        self.ekf.P = np.diag([1.0, 1.0, np.pi, 1.0, 0.5])
        self.ekf.Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1])
        self.R_uwb = np.diag([0.1**2]) 

        self.last_time = self.get_clock().now()
        self.latest_odom = None
        self.latest_imu = None
        self.latest_uwb_data = None

        ### <<< 変更点: アンカーごとの距離データ履歴を保持する辞書 >>>
        self.distance_history = {twr_id: deque(maxlen=self.nlos_history_size) for twr_id in self.anchor_map.keys()}
        ### <<< ここまで >>>

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.uwb_sub = self.create_subscription(String,'/uwb_data_json',self.uwb_callback,10)

        self.filtered_odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.timer_callback)

    def destroy_node(self):
        self.get_logger().info("ノードをシャットダウンします。")
        super().destroy_node()

    def odom_callback(self, msg):
        self.latest_odom = msg

    def uwb_callback(self,msg):
        try:
            self.latest_uwb_data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"UWBデータのJSONのパースに失敗しました。: {e}")
        
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

    def h_uwb(self, x, anchor_pos):
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

        if dt <= 0: return

        if self.latest_odom:
            self.ekf.x[3,0] = self.latest_odom.twist.twist.linear.x
            self.ekf.x[4,0] = self.latest_odom.twist.twist.angular.z
        
        F = self.state_transition_jacobian(self.ekf.x, dt)
        
        self.ekf.P = F @ self.ekf.P @ F.T + self.ekf.Q
        self.ekf.x = self.state_transition_function(self.ekf.x, dt)

        anchor_data = self.latest_uwb_data
        
        if anchor_data:
            for twr_id_str, data in anchor_data.items():
                if not data: continue

                twr_index = int(twr_id_str.replace('TWR', ''))
                
                # --- 変更点: 距離データを履歴に追加 ---
                self.distance_history[twr_index].append(data['distance'])
                
                use_data = False
                # デフォルトはLOSと同じ信頼度
                current_R = self.R_uwb

                if data['nlos_los'] == 'LOS':
                    use_data = True # LOSは常に信用する
                
                elif data['nlos_los'] == 'nLOS':
                    # 履歴が十分に溜まっているか確認
                    if len(self.distance_history[twr_index]) == self.nlos_history_size:
                        # 分散を計算
                        variance = np.var(self.distance_history[twr_index])
                        
                        # 分散がしきい値以下ならデータを使用
                        if variance < self.nlos_variance_threshold:
                            use_data = True
                            # nLOSなので、信頼度を下げてRを大きくする
                            current_R = self.R_uwb * self.R_nlos_multiplier
                            self.get_logger().info(f"Using stable nLOS data from TWR{twr_index} (variance: {variance:.4f})")
                        else:
                            # データが不安定なので使用しない
                            self.get_logger().warning(f"Skipping unstable nLOS data from TWR{twr_index} (variance: {variance:.4f})")
                
                # データを使用すると判断された場合のみEKFを更新
                if use_data:
                    anchor_id = self.anchor_map.get(twr_index)
                    if anchor_id:
                        anchor_pos = self.anchor_positions[anchor_id]
                        z = np.array([[data['distance']]])
                        
                        self.ekf.update(z, HJacobian=self.H_uwb, Hx=self.h_uwb, R=current_R,
                                        args=(anchor_pos,), hx_args=(anchor_pos,))

        self.publish_odometry(current_time)
        self.publish_tf(current_time)
        
    def quaternion_to_euler(self, x, y, z, w):
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = 2.0 * (w * y - z * x)
        t2 = 1.0 if t2 > 1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return roll, pitch, yaw
    
    def publish_odometry(self, current_time):
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        x, y, theta = self.ekf.x[0,0], self.ekf.x[1,0], self.ekf.x[2,0]
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        odom_msg.pose.covariance = np.zeros(36)
        P_pose = self.ekf.P[0:3, 0:3]
        indices = [0, 1, 5]
        for i, row in enumerate(indices):
            for j, col in enumerate(indices):
                odom_msg.pose.covariance[row*6 + col] = P_pose[i, j]
        odom_msg.twist.twist.linear.x = self.ekf.x[3, 0]
        odom_msg.twist.twist.angular.z = self.ekf.x[4, 0]
        self.filtered_odom_pub.publish(odom_msg)
        
    def publish_tf(self, current_time):
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        
        map_x, map_y, map_theta = self.ekf.x[0,0], self.ekf.x[1,0], self.ekf.x[2,0]
        
        odom_x, odom_y, odom_theta = 0.0, 0.0, 0.0
        if self.latest_odom:
            odom_x = self.latest_odom.pose.pose.position.x
            odom_y = self.latest_odom.pose.pose.position.y
            q = self.latest_odom.pose.pose.orientation
            _, _, odom_theta = self.quaternion_to_euler(q.x, q.y, q.z, q.w)
        
        cos_map_theta = np.cos(map_theta)
        sin_map_theta = np.sin(map_theta)
        cos_odom_theta = np.cos(odom_theta)
        sin_odom_theta = np.sin(odom_theta)

        # map_T_base = pose of base_footprint in map frame (from EKF)
        # odom_T_base = pose of base_footprint in odom frame (from wheel odometry)
        # We want to find map_T_odom, which satisfies: map_T_base = map_T_odom * odom_T_base
        # Therefore, map_T_odom = map_T_base * inverse(odom_T_base)
        
        # Inverse of odom_T_base transformation
        inv_odom_x = -odom_x * cos_odom_theta - odom_y * sin_odom_theta
        inv_odom_y =  odom_x * sin_odom_theta - odom_y * cos_odom_theta
        inv_odom_theta = -odom_theta

        # Combine transformations: map_T_odom = map_T_base * inv_odom_T_base
        transform_x = map_x + (inv_odom_x * cos_map_theta - inv_odom_y * sin_map_theta)
        transform_y = map_y + (inv_odom_x * sin_map_theta + inv_odom_y * cos_map_theta)
        transform_theta = map_theta + inv_odom_theta
        
        t.transform.translation.x = transform_x
        t.transform.translation.y = transform_y
        t.transform.translation.z = 0.0
        
        q = quaternion_from_euler(0, 0, transform_theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = UwbEkfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # destroy_node()は自動で呼ばれる
        rclpy.shutdown()

if __name__ == '__main__':
    main()