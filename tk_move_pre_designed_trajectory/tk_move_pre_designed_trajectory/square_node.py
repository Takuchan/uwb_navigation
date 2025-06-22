import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String as StringMsg
# base_trajectory.py は同じディレクトリに存在する必要があります
from .base_trajectory import BaseTrajectoryNode
import math
import time

class SquareNode(BaseTrajectoryNode):
    def __init__(self):
        super().__init__('square_node')
        # パラメータの宣言
        self.declare_parameter('side_length', 1.0)
        self.declare_parameter('laps', 1.0)
        self.declare_parameter('turn_angular_velocity', 0.25) # rad/s
        self.declare_parameter('stop_duration', 1.0) # seconds

        # パラメータの取得
        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.side_length = self.get_parameter('side_length').value
        self.laps = self.get_parameter('laps').value
        self.turn_angular_velocity = self.get_parameter('turn_angular_velocity').value
        self.stop_duration = self.get_parameter('stop_duration').value

        # recorder制御用のPublisher
        self.recorder_control_pub = self.create_publisher(StringMsg, 'recorder/control', 10)

        # 旋回速度の安全チェック
        if not self.safety_check(self.turn_angular_velocity):
            self.get_logger().error(f"Turn angular velocity {self.turn_angular_velocity} rad/s exceeds the limit.")
            self.is_safe = False
            return
        
        # 辺の長さが正であるかチェック
        if self.side_length <= 0:
            self.get_logger().error("Side length must be a positive value.")
            self.is_safe = False
            return

        # 各動作にかかる時間を計算
        # 直進時間 = 距離 / 速度
        self.move_duration = self.side_length / self.linear_velocity+0.579 if self.linear_velocity > 0 else float('inf')
        # 旋回時間 = 角度 / 角速度 (90度 = pi/2 ラジアン)
        self.turn_duration = (math.pi / 2) / abs(self.turn_angular_velocity)+0.572 if self.turn_angular_velocity != 0 else float('inf')

        # 状態を管理する変数
        self.state = 'idle' # 'moving', 'stopping_before_turn', 'turning', 'stopping_after_turn', 'idle'
        self.current_side = 0
        self.lap_count = 1
        self.action_start_time = 0.0

        self.get_logger().info(f'SquareNode initialized with side_length: {self.side_length}m, laps: {self.laps}, linear_velocity: {self.linear_velocity}m/s, turn_angular_velocity: {self.turn_angular_velocity}rad/s, stop_duration: {self.stop_duration}s')

    def start_trajectory(self):
        """正方形軌道走行を開始する"""
        # rosbag記録開始のトピックを送信
        start_msg = StringMsg()
        start_msg.data = 'start'
        self.recorder_control_pub.publish(start_msg)
        self.get_logger().info('Published "start" to /recorder/control')

        self.state = 'moving'
        self.action_start_time = time.time()
        frequency = self.get_parameter('frequency').value
        self.timer = self.create_timer(1.0 / frequency, self.timer_callback)
        self.get_logger().info('Started square trajectory execution.')

    def stop_robot(self):
        """ロボットを停止し、rosbag記録停止のトピックを送信する"""
        # rosbag記録停止のトピックを送信
        stop_msg = StringMsg()
        stop_msg.data = 'stop'
        self.recorder_control_pub.publish(stop_msg)
        self.get_logger().info('Published "stop" to /recorder/control')

        # BaseTrajectoryNodeのstop_robotの機能を再現
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Robot stopped.')

    def timer_callback(self):
        """Twistメッセージを発行して正方形に走行させるコールバック関数"""
        if not self.is_safe:
            self.stop_robot()
            if self.timer:
                self.timer.cancel()
            return

        # 指定された周回数に達したら終了
        if self.lap_count > self.laps:
            self.get_logger().info(f'Completed {self.laps} laps.')
            self.stop_robot()
            if self.timer:
                self.timer.cancel()
            self.call_tts('正方形軌道走行を完了しました。')
            # ノード自身がシャットダウンをコールするのは一般的ではないためコメントアウト
            # rclpy.shutdown()
            return

        twist = Twist()
        elapsed_time = time.time() - self.action_start_time

        # --- ここからが修正・改善されたロジック ---

        # Step 1: 現在の状態に基づいて、発行すべきTwistメッセージを決定する
        if self.state == 'moving':
            twist.linear.x = self.linear_velocity
        elif self.state == 'turning':
            twist.angular.z = self.turn_angular_velocity
        # 'stopping_before_turn' と 'stopping_after_turn' の場合は、
        # twistはデフォルトのゼロのままで良いため、何もしない（= 停止）。

        # Step 2: 経過時間に基づいて、状態を遷移させるか判断する
        if self.state == 'moving':
            if elapsed_time >= self.move_duration:
                self.state = 'stopping_before_turn'
                self.action_start_time = time.time()
                self.get_logger().info('Moving -> Stopping before turn')
        
        elif self.state == 'stopping_before_turn':
            if elapsed_time >= self.stop_duration:
                self.state = 'turning'
                self.action_start_time = time.time()
                self.current_side += 1
                self.get_logger().info(f'Stopping before turn -> Turning (Side {self.current_side})')
        
        elif self.state == 'turning':
            if elapsed_time >= self.turn_duration:
                self.state = 'stopping_after_turn'
                self.action_start_time = time.time()
                self.get_logger().info('Turning -> Stopping after turn')
                if self.current_side >= 4:
                    self.get_logger().info(f'Lap {self.lap_count} complete.')
                    self.current_side = 0
                    self.lap_count += 1
            
        elif self.state == 'stopping_after_turn':
            if self.lap_count <= self.laps:
                if elapsed_time >= self.stop_duration:
                    self.state = 'moving'
                    self.action_start_time = time.time()
                    self.get_logger().info('Stopping after turn -> Moving')

        # 最終的に決定したTwistメッセージを発行
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        # Note: このコードは 'base_trajectory.py' 内の BaseTrajectoryNode に依存します
        node = SquareNode()
        # ユーザーの確認を待ってから軌道走行を開始
        node.execute_trajectory()
        
        if node.user_confirmed and node.is_safe:
            rclpy.spin(node)
            
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Keyboard interrupt received, shutting down.')
    finally:
        if node:
            # 終了時にロボットを停止させる
            node.stop_robot()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()