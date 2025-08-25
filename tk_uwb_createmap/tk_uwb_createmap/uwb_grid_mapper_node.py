# uwb_grid_mapper_node.py
import rclpy
from rclpy.node import Node
import numpy as np
import math
from collections import deque

# ROS 2 message types
from geometry_msgs.msg import PoseStamped, TransformStamped,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid

# TF
import tf2_ros
from tf_transformations import quaternion_from_euler, euler_from_quaternion

# 独自モジュール (同じディレクトリにあることを想定)
from .triliation import TrilaterationSolver
from .kalman_filter import ExtendedKalmanFilter
from .serialandFilter import SerialFilter
from .occupancy_grid_manager import OccupancyGridManager

class UWBGridMapperNode(Node):
    """
    UWBのLOS/nLOS情報を用いてOccupancy Grid Mapを生成するノード
    """
    
    def __init__(self):
        super().__init__('uwb_grid_mapper_node')
        
        # パラメータ宣言
        self.declare_parameters(
            namespace='',
            parameters=[
                ('d01', 6.12), ('d12', 6.05), ('d02', 6.65),
                ('com_port', '/dev/ttyUSB1'), ('baud_rate', 3000000), ('num_anchors', 3),
                ('ekf_dt', 0.1), ('fov_angle_deg', 120.0),
                ('map.resolution', 0.05), # 5cm/pixel
                ('map.width_m', 20.0),
                ('map.height_m', 20.0),
                ('map.origin_x', -10.0),
                ('map.origin_y', -10.0),
            ]
        )
        
        # パラメータ取得
        d01 = self.get_parameter('d01').value
        d12 = self.get_parameter('d12').value
        d02 = self.get_parameter('d02').value
        com_port = self.get_parameter('com_port').value
        baud_rate = self.get_parameter('baud_rate').value
        num_anchors = self.get_parameter('num_anchors').value
        ekf_dt = self.get_parameter('ekf_dt').value
        self.fov_angle = math.radians(self.get_parameter('fov_angle_deg').value)
        
        # 三辺測位ソルバー初期化
        try:
            self.trilateration_solver = TrilaterationSolver(d01, d12, d02)
            self.anchor_positions = [
                self.trilateration_solver.anchor0,
                self.trilateration_solver.anchor1,
                self.trilateration_solver.anchor2
            ]
        except ValueError as e:
            self.get_logger().error(f"アンカー配置エラー: {e}")
            rclpy.shutdown()
            return
            
        # シリアル通信初期化
        self.serial_filter = SerialFilter(com_port=com_port, baud_rate=baud_rate, num_anchors=num_anchors)
        if not self.serial_filter.ser or not self.serial_filter.ser.is_open:
            self.get_logger().error("シリアルポート接続に失敗。終了します。")
            rclpy.shutdown()
            return
        
        # EKF初期化
        self.ekf = ExtendedKalmanFilter(ekf_dt, np.array([0.0, 0.0]))
        
        # Occupancy Grid Manager初期化
        self.map_manager = OccupancyGridManager(
            resolution=self.get_parameter('map.resolution').value,
            width_m=self.get_parameter('map.width_m').value,
            height_m=self.get_parameter('map.height_m').value,
            origin_x=self.get_parameter('map.origin_x').value,
            origin_y=self.get_parameter('map.origin_y').value
        )
        
        # 状態変数
        self.robot_heading = 0.0
        self.last_valid_uwb_position = None
        self.path_positions = deque(maxlen=500)
        
        # ROS 2 サブスクライバー
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # ROS 2 パブリッシャー
        self.map_pub = self.create_publisher(OccupancyGrid, '/uwb_map', 10)
        self.path_pub = self.create_publisher(Path, '/uwb_path', 10)
        self.anchor_marker_pub = self.create_publisher(MarkerArray, '/anchor_markers', 10)
        self.connection_marker_pub = self.create_publisher(MarkerArray, '/anchor_connections', 10)
        self.uwb_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/uwb_pose', 10)

        
        # TFブロードキャスター
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # タイマー
        self.main_timer = self.create_timer(0.1, self.main_loop) # 10Hz for localization
        self.map_timer = self.create_timer(1.0, self.publish_map) # 1Hz for map publishing
        
        # 静的マーカーを発行
        self.publish_anchor_markers()
        self.get_logger().info("✅ UWB Grid Mapperノードが正常に起動しました。")
    
    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        _, _, self.robot_heading = euler_from_quaternion([q.x, q.y, q.z, q.w])
    
    def main_loop(self):
        # 1. UWBデータ取得
        uwb_data = self.serial_filter.read_anchor_data_snapshot(timeout=0.1)
        if not any(uwb_data.values()): return

        # 2. 三辺測位
        position = self.perform_trilateration(uwb_data)
        if position is None: return
        
        # 3. EKFによる平滑化
        self.ekf.predict()
        self.ekf.update(position)
        smoothed_pos = self.ekf.x[:2]
        self.last_valid_uwb_position = smoothed_pos
        self.path_positions.append(smoothed_pos.copy())
        # robot_localization用のメッセージを作成して配信
        uwb_pose_msg = PoseWithCovarianceStamped()
        uwb_pose_msg.header.stamp = self.get_clock().now().to_msg()
        uwb_pose_msg.header.frame_id = "map" # UWBはmap座標系での絶対位置を提供

        uwb_pose_msg.pose.pose.position.x = smoothed_pos[0]
        uwb_pose_msg.pose.pose.position.y = smoothed_pos[1]
        uwb_pose_msg.pose.pose.orientation.w = 1.0 # UWBは向きを提供しないので単位クォータニオン

        # 共分散を設定 (非常に重要)
        # 対角成分(x, y)の値を調整する。値が小さいほど、その測定値を信頼する。
        # UWBの精度が±10cm (0.1m) の場合、分散はその2乗の0.01程度に設定する。
        cov = np.zeros(36)
        cov[0] = 0.01  # xの分散
        cov[7] = 0.01  # yの分散
        cov[35] = 99999 # z軸回転の分散（使わないので非常に大きくする）
        uwb_pose_msg.pose.covariance = cov.tolist()

        self.uwb_pose_pub.publish(uwb_pose_msg)

        
        # 4. マップ更新
        self.update_occupancy_grid(uwb_data, smoothed_pos)
        
        # 5. 可視化情報のパブリッシュ
        self.publish_tf(smoothed_pos)
        self.publish_path()
        self.publish_connection_markers(uwb_data, smoothed_pos)

    def perform_trilateration(self, uwb_data):
            """
            三辺測位を実行する。
            uwb_data内の各アンカーデータがNoneである可能性を考慮する。
            """
            distances = []
            for i in range(3):
                # まず、'TWRi' に対応するデータを取得
                anchor_data = uwb_data.get(f"TWR{i}")

                # anchor_data が None でないことを確認してから 'distance' を取得する
                if anchor_data is not None:
                    distances.append(anchor_data.get("distance"))
                else:
                    distances.append(None)
            
            # 3つの有効な距離データが揃っているか確認
            if distances.count(None) > 0:
                # self.get_logger().warn("三辺測位に必要なデータが不足しています。") # 頻繁に出る場合はコメントアウト
                return None
            
            try:
                pos = self.trilateration_solver.calculate_position(*distances)
                return None if np.isnan(pos).any() else pos
            except Exception as e:
                self.get_logger().warn(f"三辺測位計算エラー: {e}")
                return None

    def update_occupancy_grid(self, uwb_data, robot_pos):
        for i, anchor_pos in enumerate(self.anchor_positions):
            twr_key = f"TWR{i}"
            if not uwb_data.get(twr_key): continue

            # ロボットからアンカーへの角度を計算
            to_anchor = anchor_pos - robot_pos
            anchor_angle = math.atan2(to_anchor[1], to_anchor[0])
            angle_diff = self.normalize_angle(anchor_angle - self.robot_heading)

            # 視野角外のデータはマップ更新に使用しない
            if abs(angle_diff) > self.fov_angle / 2:
                continue

            is_los = (uwb_data[twr_key]['nlos_los'] == 'LOS')
            self.map_manager.update_map(robot_pos, anchor_pos, is_los)

    def publish_map(self):
        """定期的にマップをパブリッシュする"""
        if rclpy.ok():
            map_msg = self.map_manager.get_ros_message(self.get_clock().now().to_msg())
            self.map_pub.publish(map_msg)

    # --- 以下は可視化のためのヘルパー関数群 (uwb_localizer_node.pyから流用・調整) ---

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    def publish_tf(self, position):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = 'uwb_pose'
        t.transform.translation.x = float(position[0])
        t.transform.translation.y = float(position[1])
        self.tf_broadcaster.sendTransform(t)

    def publish_path(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"
        path.poses = []
        for pos in self.path_positions:
            pose = PoseStamped()
            pose.pose.position.x = float(pos[0])
            pose.pose.position.y = float(pos[1])
            path.poses.append(pose)
        self.path_pub.publish(path)

    def publish_anchor_markers(self):
        marker_array = MarkerArray()
        for i, anchor_pos in enumerate(self.anchor_positions):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.ns = "anchors"; marker.id = i; marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(anchor_pos[0])
            marker.pose.position.y = float(anchor_pos[1])
            marker.scale.x = 0.2; marker.scale.y = 0.2; marker.scale.z = 0.2
            marker.color.r = 1.0; marker.color.g = 0.2; marker.color.b = 0.2; marker.color.a = 1.0
            marker.lifetime.sec = 0 # 永続
            marker_array.markers.append(marker)
        self.anchor_marker_pub.publish(marker_array)

    def publish_connection_markers(self, uwb_data, robot_pos):
        marker_array = MarkerArray()
        for i, anchor_pos in enumerate(self.anchor_positions):
            # ... (マーカーの色分けロジックは元のコードとほぼ同じなので省略可能だが、デバッグに役立つので残す)
            pass # 必要であれば元のコードからコピー＆ペースト
        self.connection_marker_pub.publish(marker_array)

    def destroy_node(self):
        self.serial_filter.disconnect_serial()
        self.get_logger().info("ノードをシャットダウンします。")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UWBGridMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()