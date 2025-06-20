import rclpy
from geometry_msgs.msg import Twist
import math
from .base_trajectory import BaseTrajectoryNode

class TanTrajectoryNode(BaseTrajectoryNode):
    def __init__(self):
        super().__init__('tan_trajectory_node')
        
        self.declare_parameter('num_segments', 3)
        self.declare_parameter('period_length', 10.0)
        self.declare_parameter('amplitude', 0.5)
        self.declare_parameter('safe_range_rad', 1.4)  # Avoid tan asymptotes at pi/2 ~ 1.57

        self.time_step = 0.0
        self.segment_count = 0
        
        self.check_tan_safety()

    def check_tan_safety(self):
        amplitude = self.get_parameter('amplitude').value
        period = self.get_parameter('period_length').value
        linear_vel = self.get_parameter('linear_velocity').value
        safe_range = self.get_parameter('safe_range_rad').value
        
        omega = (2 * safe_range) / period
        
        # Derivative of A*tan(omega*t) is A*omega*sec^2(omega*t)
        # This is an approximation for angular velocity
        max_angular_vel = abs(amplitude * omega / (math.cos(safe_range)**2) * linear_vel)

        if not self.safety_check(max_angular_vel):
            self.get_logger().error(f'Tan trajectory unsafe! Estimated max angular velocity: {max_angular_vel:.2f} rad/s')
            self.is_safe = False
        else:
            self.get_logger().info(f'Tan trajectory safe. Estimated max angular velocity: {max_angular_vel:.2f} rad/s')
        self.get_logger().warn('Tan軌道は非常に高い角速度を生成する可能性があり危険です。パラメータは慎重に設定してください。')

    def start_trajectory(self):
        frequency = self.get_parameter('frequency').value
        self.timer = self.create_timer(1.0/frequency, self.tan_callback)

    def tan_callback(self):
        num_segments = self.get_parameter('num_segments').value
        if self.segment_count >= num_segments:
            self.stop_robot()
            self.call_tts('Tan軌道が完了しました')
            self.timer.cancel()
            return

        period = self.get_parameter('period_length').value
        amplitude = self.get_parameter('amplitude').value
        linear_vel = self.get_parameter('linear_velocity').value
        safe_range = self.get_parameter('safe_range_rad').value
        
        omega = (2 * safe_range) / period
        
        # Map time_step from [0, period] to [-safe_range, safe_range]
        angle = -safe_range + omega * self.time_step
        
        twist = Twist()
        twist.linear.x = linear_vel
        
        # Angular velocity based on derivative of tan curve
        # This is a simplified model and can be inaccurate
        twist.angular.z = amplitude * omega / (math.cos(angle)**2)

        if not self.safety_check(twist.angular.z):
            self.stop_robot()
            self.call_tts('危険な角速度を検出しました。停止します')
            self.timer.cancel()
            return
            
        self.cmd_vel_pub.publish(twist)
        self.time_step += 1.0/self.get_parameter('frequency').value

        if self.time_step > period:
            self.time_step = 0
            self.segment_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = TanTrajectoryNode()
    try:
        node.execute_trajectory()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
