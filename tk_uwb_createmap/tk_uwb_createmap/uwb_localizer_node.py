import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import numpy as np
import math
import time
from collections import deque
import pygame
import threading

# ROS 2 message types
from geometry_msgs.msg import PoseStamped, TransformStamped, Point
from nav_msgs.msg import Odometry, Path, OccupancyGrid, MapMetaData
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
import tf2_ros
from tf_transformations import quaternion_from_euler, euler_from_quaternion


# 独自モジュール
from .triliation import TrilaterationSolver
from .kalman_filter import ExtendedKalmanFilter
from .serialandFilter import SerialFilter

class UWBLocalizerNode(Node):
    """
    UWB三辺測位による自己位置推定ノード
    - /odomをサブスクライブしてロボットの向きを取得
    - 視野角制約（前方120°のみ信頼、後方240°は除外）
    - 拡張カルマンフィルタによる軌道平滑化
    - Pygameによるリアルタイム可視化
    """
    
    def __init__(self):
        super().__init__('uwb_localizer_node')
        
        # パラメータ宣言と取得
        self.declare_parameters(
            namespace='',
            parameters=[
                ('d01', 6.82),  # アンカー0-1間距離 [m]
                ('d12', 1.66),  # アンカー1-2間距離 [m] 
                ('d02', 7.12),  # アンカー0-2間距離 [m]
                ('com_port', '/dev/ttyUSB0'),
                ('baud_rate', 3000000),
                ('num_anchors', 3),
                ('ekf_dt', 0.1),  # EKFの時間間隔
                ('fov_angle', 120.0),  # 視野角 [度]
                ('map_frame', 'map'),
                ('odom_frame', 'odom'),
                ('base_link_frame', 'base_link'),
                ('uwb_tag_frame', 'uwb_tag'),
                ('publish_frequency', 10.0),
                ('enable_pygame', True)  # Pygame可視化の有効/無効
            ]
        )
        
        # パラメータ取得
        self.d01 = self.get_parameter('d01').value
        self.d12 = self.get_parameter('d12').value
        self.d02 = self.get_parameter('d02').value
        self.com_port = self.get_parameter('com_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.num_anchors = self.get_parameter('num_anchors').value
        self.ekf_dt = self.get_parameter('ekf_dt').value
        self.fov_angle = math.radians(self.get_parameter('fov_angle').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_link_frame = self.get_parameter('base_link_frame').value
        self.uwb_tag_frame = self.get_parameter('uwb_tag_frame').value
        self.publish_freq = self.get_parameter('publish_frequency').value
        self.enable_pygame = self.get_parameter('enable_pygame').value
        
        # 三辺測位ソルバー初期化
        try:
            self.trilateration_solver = TrilaterationSolver(self.d01, self.d12, self.d02)
            self.get_logger().info(f"アンカー間距離設定: d01={self.d01}m, d12={self.d12}m, d02={self.d02}m")
        except ValueError as e:
            self.get_logger().error(f"アンカー配置エラー: {e}")
            return
            
        # シリアル通信初期化
        self.serial_filter = SerialFilter(
            com_port=self.com_port,
            baud_rate=self.baud_rate,
            num_anchors=self.num_anchors
        )
        
        if not self.serial_filter.ser or not self.serial_filter.ser.is_open:
            self.get_logger().error("シリアルポート接続に失敗しました")
            return
        
        # EKF初期化（初期位置は原点）
        initial_pos = np.array([0.0, 0.0])
        self.ekf = ExtendedKalmanFilter(self.ekf_dt, initial_pos)
        
        # ロボットの状態
        self.robot_heading = 0.0  # ロボットの向き [rad]
        self.robot_position = np.array([0.0, 0.0])
        self.robot_velocity = np.array([0.0, 0.0])
        self.last_valid_uwb_position = None
        
        # 軌跡保存用
        self.path_positions = deque(maxlen=1000)
        self.uwb_positions = deque(maxlen=1000)  # UWB生データ用
        
        # ROS 2 サブスクライバー（/odomをサブスクライブ）
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        # ROS 2 パブリッシャー
        self.path_pub = self.create_publisher(Path, '/uwb_path', 10)
        self.anchor_marker_pub = self.create_publisher(MarkerArray, '/anchor_markers', 10)
        self.connection_marker_pub = self.create_publisher(MarkerArray, '/anchor_connections', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/uwb_map', 10)
        
        # TFブロードキャスター
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # タイマー（メインループ）
        self.timer = self.create_timer(1.0/self.publish_freq, self.main_loop)
        
        # アンカーマーカー発行（静的）
        self.publish_anchor_markers()
        
        self.get_logger().info("UWB自己位置推定ノードが開始されました")
    
    def odom_callback(self, msg: Odometry):
        """
        /odomをサブスクライブしてロボットの位置・向き・速度を取得
        """
        # 位置
        self.robot_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])
        
        # 向き（クォータニオンからヨー角を計算）
        q = msg.pose.pose.orientation
        _, _, self.robot_heading = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # 速度
        self.robot_velocity = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y
        ])
    
    def main_loop(self):
        """メインループ：UWBデータ取得→三辺測位→EKF→パブリッシュ"""
        
        # UWBデータ取得
        uwb_data = self.serial_filter.read_anchor_data_snapshot(timeout=0.05)
        if not uwb_data:
            return
        
        # 視野角制約を適用したデータフィルタリング
        filtered_data = self.apply_fov_constraint(uwb_data)
        self.get_logger().info(f"filterdataは{filtered_data}")
        
        # 三辺測位実行
        position = self.perform_trilateration(filtered_data)
        if position is not None:
            # EKFで平滑化
            self.ekf.predict()
            self.ekf.update(position)
            
            smoothed_pos = self.ekf.x[:2]  # [x, y]
            self.last_valid_uwb_position = smoothed_pos
            
            # 軌跡に追加
            self.path_positions.append(smoothed_pos.copy())
            self.uwb_positions.append(position.copy())  # 生データも保存
            
            # ROS 2メッセージパブリッシュ
            self.publish_path()
            self.publish_tf(smoothed_pos)
            self.publish_connection_markers(filtered_data, smoothed_pos)
            self.publish_map()
    
    def apply_fov_constraint(self, uwb_data):
        """
        視野角制約を適用：ロボットの前方±60°（合計120°）範囲外のアンカーは
        distanceデータは使用するが、nLOS情報は信頼せずLOSとして扱わない
        """
        filtered_data = {}
        
        # アンカー位置（ローカル座標系）
        anchor_positions = {
            0: self.trilateration_solver.anchor0,
            1: self.trilateration_solver.anchor1, 
            2: self.trilateration_solver.anchor2
        }
        
        if self.last_valid_uwb_position is None:
            # 初回は全データを使用
            return uwb_data
        
        robot_pos = self.last_valid_uwb_position
        
        for i, (twr_key, data) in enumerate(uwb_data.items()):
            if data is None:
                filtered_data[twr_key] = None
                continue
            
            # データをコピー
            filtered_data[twr_key] = data.copy()
            
            # アンカーへの方向ベクトル
            anchor_pos = anchor_positions[i]
            to_anchor = anchor_pos - robot_pos
            anchor_angle = math.atan2(to_anchor[1], to_anchor[0])
            
            # ロボット向きとの角度差
            angle_diff = self.normalize_angle(anchor_angle - self.robot_heading)
            
            # 視野角外の場合は、nLOS情報を信頼せずnLOSとして扱う
            if abs(angle_diff) > self.fov_angle/2:
                filtered_data[twr_key]['nlos_los'] = 'nLOS'  # 強制的にnLOSに設定
                self.get_logger().debug(f"アンカー{i}: 視野角外 - nLOS扱い")
            # 視野角内の場合は元のnLOS情報を保持
            
        return filtered_data
    
    def normalize_angle(self, angle):
        """角度を-π〜πの範囲に正規化"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def perform_trilateration(self, uwb_data):
        """三辺測位実行 - nLOSでも距離データは使用"""
        distances = []
        valid_count = 0
        
        for i in range(self.num_anchors):
            twr_key = f"TWR{i}"
            if twr_key in uwb_data and uwb_data[twr_key] is not None:
                distances.append(uwb_data[twr_key]['distance'])
                valid_count += 1
            else:
                distances.append(None)
        
        if valid_count < 3:
            self.get_logger().warn(f"有効なアンカーが不足: {valid_count}/3")
            return None
        
        # Noneを適当な値で補完（使わないが配列長を合わせるため）
        for i, dist in enumerate(distances):
            if dist is None:
                distances[i] = 1.0  # ダミー値
        
        try:
            position = self.trilateration_solver.calculate_position(
                distances[0], distances[1], distances[2]
            )
            
            if np.isnan(position).any():
                self.get_logger().warn("三辺測位の結果がNaNです")
                return None
            
            return position
            
        except Exception as e:
            self.get_logger().warn(f"三辺測位計算エラー: {e}")
            return None
    
    def publish_path(self):
        """UWB軌跡パスをパブリッシュ"""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.map_frame
        
        for pos in self.path_positions:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.map_frame
            pose_stamped.pose.position.x = float(pos[0])
            pose_stamped.pose.position.y = float(pos[1])
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            path.poses.append(pose_stamped)
        
        self.path_pub.publish(path)
    
    def publish_tf(self, position):
        """UWB位置のTFをブロードキャスト（uwb_frameとして）"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = 'uwb_position'
        
        t.transform.translation.x = float(position[0])
        t.transform.translation.y = float(position[1])
        t.transform.translation.z = 0.0
        
        q = quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_anchor_markers(self):
        """アンカー位置をマーカーでパブリッシュ（静的）"""
        marker_array = MarkerArray()
        
        anchors = [
            self.trilateration_solver.anchor0,
            self.trilateration_solver.anchor1,
            self.trilateration_solver.anchor2
        ]
        
        for i, anchor_pos in enumerate(anchors):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "anchors"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(anchor_pos[0])
            marker.pose.position.y = float(anchor_pos[1])
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.1
            
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            
            marker_array.markers.append(marker)
        
        self.anchor_marker_pub.publish(marker_array)
    
    def publish_connection_markers(self, uwb_data, robot_pos):
        """ロボット-アンカー間の接続線をLOS/nLOS/視野角外で色分けして表示"""
        marker_array = MarkerArray()
        
        anchors = [
            self.trilateration_solver.anchor0,
            self.trilateration_solver.anchor1,
            self.trilateration_solver.anchor2
        ]
        
        for i, anchor_pos in enumerate(anchors):
            twr_key = f"TWR{i}"
            
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "connections"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # ロボット位置
            robot_point = Point()
            robot_point.x = float(robot_pos[0])
            robot_point.y = float(robot_pos[1])
            robot_point.z = 0.0
            
            # アンカー位置
            anchor_point = Point()
            anchor_point.x = float(anchor_pos[0])
            anchor_point.y = float(anchor_pos[1])
            anchor_point.z = 0.0
            
            marker.points = [robot_point, anchor_point]
            marker.scale.x = 0.05
            
            # 視野角チェック
            to_anchor = anchor_pos - robot_pos
            anchor_angle = math.atan2(to_anchor[1], to_anchor[0])
            angle_diff = self.normalize_angle(anchor_angle - self.robot_heading)
            is_in_fov = abs(angle_diff) <= self.fov_angle/2
            
            # 色分け
            if twr_key in uwb_data and uwb_data[twr_key] is not None:
                nlos_status = uwb_data[twr_key].get('nlos_los', 'nLOS')
                
                if is_in_fov and nlos_status == 'LOS':
                    # 視野角内かつLOS: 緑
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                elif is_in_fov and nlos_status == 'nLOS':
                    # 視野角内だがnLOS: オレンジ
                    marker.color.r = 1.0
                    marker.color.g = 0.5
                    marker.color.b = 0.0
                else:
                    # 視野角外: 黄色
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
            else:
                # データなし: グレー
                marker.color.r = 0.5
                marker.color.g = 0.5
                marker.color.b = 0.5
            
            marker.color.a = 0.8
            marker_array.markers.append(marker)
        
        self.connection_marker_pub.publish(marker_array)
    
    def publish_map(self):
        """簡易的なマップをパブリッシュ"""
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.map_frame
        
        # マップメタデータ
        grid.info.resolution = 0.1  # 10cm/pixel
        grid.info.width = 200
        grid.info.height = 200
        grid.info.origin.position.x = -10.0
        grid.info.origin.position.y = -10.0
        grid.info.origin.orientation.w = 1.0
        
        # 空のマップ（未知領域）
        grid.data = [-1] * (grid.info.width * grid.info.height)
        
        self.map_pub.publish(grid)
    
    def pygame_visualization(self):
        """Pygameによるリアルタイム可視化（別スレッド）"""
        pygame.init()
        pygame.display.set_caption("UWB Real-Time Mapper")
        
        # 画面設定
        screen_width = 800
        screen_height = 800
        screen = pygame.display.set_mode((screen_width, screen_height))
        clock = pygame.time.Clock()
        font = pygame.font.Font(None, 24)
        
        # 座標変換設定
        padding_px = 50
        anchors = [
            self.trilateration_solver.anchor0,
            self.trilateration_solver.anchor1,
            self.trilateration_solver.anchor2
        ]
        all_anchor_coords = np.array(anchors)
        min_coords = np.min(all_anchor_coords, axis=0)
        max_coords = np.max(all_anchor_coords, axis=0)
        world_range = max_coords - min_coords
        
        scale_x = (screen_width - 2 * padding_px) / world_range[0] if world_range[0] > 0 else 1
        scale_y = (screen_height - 2 * padding_px) / world_range[1] if world_range[1] > 0 else 1
        scale = min(scale_x, scale_y) * 0.8  # 少し余裕を持たせる
        
        origin_px = np.array([
            screen_width/2 - (max_coords[0] + min_coords[0])/2 * scale,
            screen_height/2 + (max_coords[1] + min_coords[1])/2 * scale
        ])
        
        def world_to_screen(pos_m):
            screen_pos = origin_px + np.array([pos_m[0], -pos_m[1]]) * scale
            return int(screen_pos[0]), int(screen_pos[1])
        
        # 色定義
        COLOR_BACKGROUND = (20, 20, 20)
        COLOR_GRID = (40, 40, 40)
        COLOR_ANCHOR = (255, 0, 0)
        COLOR_LOS_LINE = (0, 255, 0)
        COLOR_NLOS_LINE = (255, 100, 0)
        COLOR_FOV_OUT_LINE = (255, 255, 0)
        COLOR_UWB_PATH = (0, 150, 255)
        COLOR_ROBOT = (0, 255, 0)
        COLOR_TEXT = (255, 255, 255)
        
        prev_uwb_pos_px = None
        
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            
            # 背景
            screen.fill(COLOR_BACKGROUND)
            
            # グリッド
            for x in range(0, screen_width, 50):
                pygame.draw.line(screen, COLOR_GRID, (x, 0), (x, screen_height))
            for y in range(0, screen_height, 50):
                pygame.draw.line(screen, COLOR_GRID, (0, y), (screen_width, y))
            
            # アンカー描画
            for i, anchor_pos in enumerate(anchors):
                pos_px = world_to_screen(anchor_pos)
                pygame.draw.circle(screen, COLOR_ANCHOR, pos_px, 8)
                text = font.render(f"A{i}", True, COLOR_TEXT)
                screen.blit(text, (pos_px[0] + 10, pos_px[1] - 10))
            
            # ロボット位置
            if self.robot_position is not None:
                robot_pos_px = world_to_screen(self.robot_position)
                pygame.draw.circle(screen, COLOR_ROBOT, robot_pos_px, 6)
                
                # ロボットの向き
                heading_end = self.robot_position + np.array([
                    math.cos(self.robot_heading) * 0.5,
                    math.sin(self.robot_heading) * 0.5
                ])
                heading_end_px = world_to_screen(heading_end)
                pygame.draw.line(screen, COLOR_ROBOT, robot_pos_px, heading_end_px, 2)
            
            # UWB軌跡
            if len(self.path_positions) > 1:
                path_points = [world_to_screen(pos) for pos in list(self.path_positions)]
                if len(path_points) > 1:
                    pygame.draw.lines(screen, COLOR_UWB_PATH, False, path_points, 2)
            
            # 現在のUWB位置
            if self.last_valid_uwb_position is not None:
                uwb_pos_px = world_to_screen(self.last_valid_uwb_position)
                pygame.draw.circle(screen, COLOR_UWB_PATH, uwb_pos_px, 4)
            
            # 情報表示
            info_text = [
                f"Robot: ({self.robot_position[0]:.2f}, {self.robot_position[1]:.2f})" if self.robot_position is not None else "Robot: N/A",
                f"Heading: {math.degrees(self.robot_heading):.1f}°",
                f"UWB: ({self.last_valid_uwb_position[0]:.2f}, {self.last_valid_uwb_position[1]:.2f})" if self.last_valid_uwb_position is not None else "UWB: N/A"
            ]
            
            for i, text in enumerate(info_text):
                text_surface = font.render(text, True, COLOR_TEXT)
                screen.blit(text_surface, (10, 10 + i * 25))
            
            pygame.display.flip()
            clock.tick(30)
        
        pygame.quit()
    
    def destroy_node(self):
        """ノード終了時の処理"""
        if hasattr(self, 'serial_filter'):
            self.serial_filter.disconnect_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = UWBLocalizerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("キーボード割り込みを検出しました")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()