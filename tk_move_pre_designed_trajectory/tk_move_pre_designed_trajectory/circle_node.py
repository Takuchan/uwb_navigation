import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String as StringMsg
from .base_trajectory import BaseTrajectoryNode
import math
import time

class CircleNode(BaseTrajectoryNode):
    def __init__(self):
        super().__init__('circle_node')
        self.declare_parameter('radius', 1.0)
        self.declare_parameter('laps', 1.0)
        
        # Calculate angular velocity and perform safety check
        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.radius = self.get_parameter('radius').value
        self.laps = self.get_parameter('laps').value
        
        # recorder制御用のPublisher
        self.recorder_control_pub = self.create_publisher(StringMsg, 'recorder/control', 10)

        if self.radius <= 0:
            self.get_logger().error("Radius must be a positive value.")
            self.is_safe = False
            return

        self.angular_velocity = self.linear_velocity / self.radius
        
        if not self.safety_check(self.angular_velocity):
            self.get_logger().error(f"Calculated angular velocity {self.angular_velocity:.2f} rad/s exceeds the limit.")
            self.is_safe = False
        
        self.get_logger().info(f'CircleNode initialized with radius: {self.radius}m, linear_velocity: {self.linear_velocity}m/s, angular_velocity: {self.angular_velocity:.2f}rad/s, laps: {self.laps}')

        self.start_time = None
        self.total_duration = 0.0

    def start_trajectory(self):
        """Starts the circular trajectory."""
        # rosbag記録開始のトピックを送信
        start_msg = StringMsg()
        start_msg.data = 'start'
        self.recorder_control_pub.publish(start_msg)
        self.get_logger().info('Published "start" to /recorder/control')

        if self.laps > 0:
            # Avoid division by zero if linear velocity is zero
            if abs(self.angular_velocity) > 1e-6:
                self.total_duration = self.laps * 2 * math.pi / abs(self.angular_velocity)
            else:
                self.get_logger().warn("Angular velocity is zero. Cannot complete laps. Will run indefinitely.")
                self.laps = 0 # Treat as infinite laps

        self.start_time = time.time()
        frequency = self.get_parameter('frequency').value
        self.timer = self.create_timer(1.0 / frequency, self.timer_callback)
        self.get_logger().info('Started circular trajectory execution.')

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
        """Publishes Twist messages to move in a circle."""
        if not self.is_safe:
            self.stop_robot()
            if self.timer:
                self.timer.cancel()
            return

        if self.laps > 0 and self.start_time is not None:
            elapsed_time = time.time() - self.start_time
            if elapsed_time >= self.total_duration:
                self.get_logger().info(f'Completed {self.laps} laps in {elapsed_time:.2f} seconds.')
                self.stop_robot()
                if self.timer:
                    self.timer.cancel()
                self.call_tts('円軌道走行を完了しました。')
                rclpy.shutdown()
                return
            
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.angular.z = self.angular_velocity
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CircleNode()
        node.execute_trajectory()
        
        if node.user_confirmed and node.is_safe:
            rclpy.spin(node)
            
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Keyboard interrupt received, shutting down.')
    finally:
        if node:
            node.stop_robot()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
