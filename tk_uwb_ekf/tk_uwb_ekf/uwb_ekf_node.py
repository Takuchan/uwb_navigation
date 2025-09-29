#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise


class UwbEkfNode(Node):
    def __init__(self):
        super().__init__('uwb_ekf_node')

        # パラメータ宣言
        self.declare_parameter('anchor_a_pos', [0.0, 5.0])
        self.declare_parameter('anchor_b_pos', [5.0, 5.0])
        self.declare_parameter('anchor_c_pos', [2.5, 0.0])

        # アンカーの座標をパラメータから取得
        self.anchor_positions = {
            'a': self.get_parameter('anchor_a_pos').get_parameter_value().double_array_value,
            'b': self.get_parameter('anchor_b_pos').get_parameter_value().double_array_value,
            'c': self.get_parameter('anchor_c_pos').get_parameter_value().double_array_value,
        }
        self.get_logger().info(f"Anchor positions: {self.anchor_positions}")

        # 状態ベクトル [x, y, theta, v, w] (位置x, y, 向き, 並進速度, 角速度)
        self.ekf = ExtendedKalmanFilter(dim_x=5, dim_z=1)

        # 初期状態
        self.ekf.x = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]]).T

        # 初期共分散行列 P: 不確かさを大きく設定
        self.ekf.P = np.diag([1.0, 1.0, np.pi, 1.0, 0.5])

        # プロセスノイズ Q
        # 状態遷移モデルの不確かさを表現
        # ここでは加速度と角加速度がノイズとして加わると仮定
        self.ekf.Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1])
        
        # UWB測定ノイズ R
        self.R_uwb = np.diag([0.1**2]) # 測定誤差10cmを仮定

        # 最後に処理した時間
        self.last_time = self.get_clock().now()
        
        # 最新のセンサーデータを保持する変数
        self.latest_odom = None
        self.latest_imu = None
        self.latest_uwb = {'a': None, 'b': None, 'c': None}

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.uwb_a_sub = self.create_subscription(UWBReading, '/uwb/anchor_a', lambda msg: self.uwb_callback(msg, 'a'), 10)
        self.uwb_b_sub = self.create_subscription(UWBReading, '/uwb/anchor_b', lambda msg: self.uwb_callback(msg, 'b'), 10)
        self.uwb_c_sub = self.create_subscription(UWBReading, '/uwb/anchor_c', lambda msg: self.uwb_callback(msg, 'c'), 10)

        # Publishers
        self.filtered_odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # メインループタイマー (50Hz)
        self.timer = self.create_timer(0.02, self.timer_callback)

    # --- コールバック関数 ---
    def odom_callback(self, msg):
        self.latest_odom = msg

    def imu_callback(self, msg):
        self.latest_imu = msg

    def uwb_callback(self, msg, anchor_id):
        self.latest_uwb[anchor_id] = msg
        
    # --- EKFの計算モデル ---
    def state_transition_function(self, x, dt):
        """状態遷移関数 f(x)"""
        x_new = np.copy(x)
        theta = x[2, 0]
        v = x[3, 0]
        w = x[4, 0]

        x_new[0] = x[0, 0] + v * dt * np.cos(theta)
        x_new[1] = x[1, 0] + v * dt * np.sin(theta)
        x_new[2] = x[2, 0] + w * dt
        x_new[3] = v
        x_new[4] = w
        return x_new

    def state_transition_jacobian(self, x, dt):
        """状態遷移ヤコビ行列 F"""
        theta = x[2, 0]
        v = x[3, 0]
        
        F = np.eye(5)
        F[0, 2] = -v * dt * np.sin(theta)
        F[0, 3] = dt * np.cos(theta)
        F[1, 2] = v * dt * np.cos(theta)
        F[1, 3] = dt * np.sin(theta)
        F[2, 4] = dt
        return F

    def h_uwb(self, x, anchor_pos):
        """UWBの測定関数 h(x)"""
        dx = x[0, 0] - anchor_pos[0]
        dy = x[1, 0] - anchor_pos[1]
        return np.array([[np.sqrt(dx**2 + dy**2)]])

    def H_uwb(self, x, anchor_pos):
        """UWBの測定ヤコビ行列 H"""
        dx = x[0, 0] - anchor_pos[0]
        dy = x[1, 0] - anchor_pos[1]
        dist = np.sqrt(dx**2 + dy**2)
        if dist < 1e-6: # ゼロ除算を避ける
            return np.zeros((1, 5))
            
        H = np.zeros((1, 5))
        H[0, 0] = dx / dist
        H[0, 1] = dy / dist
        return H

    # --- メインループ ---
    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # --- 予測(Predict)ステップ ---
        if self.latest_odom:
            # オドメトリから速度と角速度を取得
            self.ekf.x[3,0] = self.latest_odom.twist.twist.linear.x
            self.ekf.x[4,0] = self.latest_odom.twist.twist.angular.z

        # 状態遷移ヤコビ行列 F を計算
        F = self.state_transition_jacobian(self.ekf.x, dt)
        
        # 予測
        self.ekf.predict(F=F, u=None, B=None, f=self.state_transition_function, dt=dt)

        # --- 更新(Update)ステップ ---
        # 各UWBアンカーのデータで更新
        for anchor_id, uwb_data in self.latest_uwb.items():
            if uwb_data and not uwb_data.nlos: # データがあり、見通し内(LOS)の場合のみ
                anchor_pos = self.anchor_positions[anchor_id]
                z = np.array([[uwb_data.distance]]) # 測定値
                
                # 測定関数とヤコビ行列を、現在の状態とアンカー位置から計算
                self.ekf.update(z, HJacobian=self.H_uwb, Hx=self.h_uwb, R=self.R_uwb,
                                args=(anchor_pos,), hx_args=(anchor_pos,))
                
                # 一度使ったデータはクリア
                self.latest_uwb[anchor_id] = None

        # --- 結果の出力 ---
        self.publish_odometry(current_time)
        self.publish_tf(current_time)
        
    def publish_odometry(self, current_time):
        """EKFの結果をOdometryメッセージとして発行"""
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'map' # 推定結果はmapフレーム基準
        odom_msg.child_frame_id = 'base_link'

        # 位置と向き
        x, y, theta = self.ekf.x[0,0], self.ekf.x[1,0], self.ekf.x[2,0]
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # 共分散
        # 6x6のpose covarianceに変換
        odom_msg.pose.covariance = np.zeros(36)
        P_pose = self.ekf.P[0:3, 0:3] # x, y, thetaの共分散
        indices = [0, 1, 5] # ROSのOdometryのcovarianceにおけるx, y, yawの位置
        for i, row in enumerate(indices):
            for j, col in enumerate(indices):
                odom_msg.pose.covariance[row*6 + col] = P_pose[i, j]

        # 速度
        odom_msg.twist.twist.linear.x = self.ekf.x[3, 0]
        odom_msg.twist.twist.angular.z = self.ekf.x[4, 0]
        
        self.filtered_odom_pub.publish(odom_msg)
        
    def publish_tf(self, current_time):
        """map -> odom のTFを発行"""
        # このサンプルでは簡単化のためmap->base_linkを直接計算・発行
        # 実際にはmap->odomとodom->base_linkを分離することが多い
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        x, y, theta = self.ekf.x[0,0], self.ekf.x[1,0], self.ekf.x[2,0]
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        
        q = quaternion_from_euler(0, 0, theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = UwbEkfNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()