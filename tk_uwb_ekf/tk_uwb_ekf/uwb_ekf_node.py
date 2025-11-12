#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import sys
import json
from collections import deque

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

from filterpy.kalman import ExtendedKalmanFilter
import math

# アンカー数の制限（アルファベットラベルの制限による）
MIN_ANCHORS = 3
MAX_ANCHORS = 26 

class UwbEkfNode(Node):
    def __init__(self):
        super().__init__('uwb_ekf_node')

        # --- パラメータ宣言 ---
        self.declare_parameter('num_anchors', 3)
        self.declare_parameter('anchor_height', 1.27)  # Anchor height from ground (z-coordinate)
        self.declare_parameter('tag_height', 0.6)  # Tag height from ground
        
        
        # UWBデータフィルタリング用パラメータ
        self.declare_parameter('history_size', 5)
        self.declare_parameter('variance_threshold', 0.05) # <<< 変更: nlos_を削除し、全データに適用
        self.declare_parameter('R_nlos_multiplier', 20.0)
        
        self.declare_parameter('stationary_velocity_threshold', 0.02) # 停止とみなす速度のしきい値 (m/s, rad/s)
        self.declare_parameter('R_stationary_multiplier', 20.0) # 停止時にRを何倍にするか

        self.num_anchors = self.get_parameter('num_anchors').get_parameter_value().integer_value
        
        # アンカー数の妥当性チェック
        if self.num_anchors < MIN_ANCHORS:
            self.get_logger().error(f"num_anchors must be at least {MIN_ANCHORS}, got {self.num_anchors}")
            raise ValueError(f"num_anchors must be at least {MIN_ANCHORS}")
        if self.num_anchors > MAX_ANCHORS:
            self.get_logger().error(f"num_anchors must be at most {MAX_ANCHORS}, got {self.num_anchors}")
            raise ValueError(f"num_anchors must be at most {MAX_ANCHORS} (limited by alphabet labels)")
        
        self.get_logger().info(f"アンカーのたかさは{self.get_parameter('anchor_height').get_parameter_value()}")
        self.get_logger().info(f"アンカーのたかさは{self.get_parameter('tag_height').get_parameter_value()}")
        self.anchor_height = self.get_parameter('anchor_height').get_parameter_value().double_value
        self.tag_height = self.get_parameter('tag_height').get_parameter_value().double_value
        self.history_size = self.get_parameter('history_size').get_parameter_value().integer_value
        self.variance_threshold = self.get_parameter('variance_threshold').get_parameter_value().double_value
        self.R_nlos_multiplier = self.get_parameter('R_nlos_multiplier').get_parameter_value().double_value
        self.stationary_velocity_threshold = self.get_parameter('stationary_velocity_threshold').get_parameter_value().double_value
        self.R_stationary_multiplier = self.get_parameter('R_stationary_multiplier').get_parameter_value().double_value
        
        # 動的にアンカー位置を読み込む
        self.anchor_positions = {}
        self.anchor_map = {}
        for i in range(self.num_anchors):
            # アルファベット順のラベルを生成 (a, b, c, d, ...)
            anchor_label = chr(97 + i)  # 97 = 'a'
            param_name = f'anchor_{anchor_label}_pos'
            self.declare_parameter(param_name, [0.0, 0.0])
            pos_2d = self.get_parameter(param_name).get_parameter_value().double_array_value
            # 3D座標に変換（2D座標の場合はアンカー高さを追加、3D座標はそのまま使用）
            self.anchor_positions[anchor_label] = self._convert_to_3d_position(list(pos_2d))
            self.anchor_map[i] = anchor_label
        
        self.get_logger().info(f"Number of anchors: {self.num_anchors}")
        self.get_logger().info(f"Anchor height: {self.anchor_height}m, Tag height: {self.tag_height}m")
        self.get_logger().info(f"Anchor positions: {self.anchor_positions}")

        # EKF設定
        self.ekf = ExtendedKalmanFilter(dim_x=5, dim_z=1)
        self.ekf.x = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]]).T
        self.ekf.P = np.diag([1.0, 1.0, np.pi, 1.0, 0.5])
        self.ekf.Q = np.diag([0.01, 0.01, 0.01, 0.05, 0.05])
        self.R_uwb = np.diag([0.3**2]) 

        # 内部変数
        self.last_time = self.get_clock().now()
        self.latest_odom = None
        self.latest_uwb_data = None
        self.distance_history = {twr_id: deque(maxlen=self.history_size) for twr_id in self.anchor_map.keys()}

        # ROS通信設定
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
    
    def _convert_to_3d_position(self, pos):
        """2D座標を3D座標に変換するヘルパーメソッド"""
        if len(pos) == 2:
            # 2D座標の場合、anchor_heightを追加
            return pos + [self.anchor_height]
        elif len(pos) >= 3:
            # 既に3D座標の場合、最初の3要素を取得し、有効性を確認
            pos_3d = pos[:3]
            # すべての座標が数値であることを確認
            if all(isinstance(x, (int, float)) for x in pos_3d):
                return pos_3d
            else:
                self.get_logger().warn(f"Invalid position values in {pos}, using default [0, 0, anchor_height]")
                return [0.0, 0.0, self.anchor_height]
        else:
            # 不正な座標の場合はデフォルト値を返す
            self.get_logger().warn(f"Invalid position format: {pos}, using default [0, 0, anchor_height]")
            return [0.0, 0.0, self.anchor_height]

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

        is_stationary = False
        if self.latest_odom:
            linear_vel = self.latest_odom.twist.twist.linear.x
            angular_vel = self.latest_odom.twist.twist.angular.z
            self.ekf.x[3,0] = linear_vel
            self.ekf.x[4,0] = angular_vel
            if abs(linear_vel) < self.stationary_velocity_threshold and \
               abs(angular_vel) < self.stationary_velocity_threshold:
                is_stationary = True
        
        # 1. 予測フェーズ
        F = self.state_transition_jacobian(self.ekf.x, dt)
        self.ekf.P = F @ self.ekf.P @ F.T + self.ekf.Q
        self.ekf.x = self.state_transition_function(self.ekf.x, dt)

        anchor_data = self.latest_uwb_data
        
        if anchor_data:
            for twr_id_str, data in anchor_data.items():
                if not data: continue

                twr_index = int(twr_id_str.replace('TWR', ''))
                
                self.distance_history[twr_index].append(data['distance'])
                
                # 履歴が溜まるまではフィルタしない
                if len(self.distance_history[twr_index]) < self.history_size:
                    continue 

                variance = np.var(self.distance_history[twr_index])
                if variance > self.variance_threshold:
                    self.get_logger().warning(f"❌データ不採用 [不安定] from TWR{twr_index} (variance: {variance:.4f})")
                    continue # 分散が大きすぎるので、このデータは使わない

                current_R = self.R_uwb

                # nLOSなら信頼度を下げる
                if data['nlos_los'] == 'nLOS':
                    current_R = self.R_uwb * self.R_nlos_multiplier

                if is_stationary:
                    current_R = current_R * self.R_stationary_multiplier

                anchor_id = self.anchor_map.get(twr_index)
                if anchor_id:
                    anchor_pos = self.anchor_positions[anchor_id]
                    self.get_logger().info(f"今これ処理してまーす{anchor_id}")
                    slant_range = data['distance']
                    height_diff = abs(self.tag_height - anchor_pos[2])
                    if (slant_range < height_diff):
                        self.get_logger().warn(
                            f"{anchor_id}:"
                            f"UWBからの距離データ{slant_range:.2f}m < 高さ差{height_diff:.2f}m"
                        )
                        continue

                    horizontal_distance = np.sqrt(slant_range**2 - height_diff**2)
                    z = np.array([[horizontal_distance]])

                    
                    self.ekf.update(z, HJacobian=self.H_uwb, Hx=self.h_uwb, R=current_R,
                                    args=(anchor_pos,), hx_args=(anchor_pos,))

        # 4. 出力フェーズ
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
        inv_odom_x = -odom_x * cos_odom_theta - odom_y * sin_odom_theta
        inv_odom_y =  odom_x * sin_odom_theta - odom_y * cos_odom_theta
        inv_odom_theta = -odom_theta
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
        rclpy.shutdown()

if __name__ == '__main__':
    main()