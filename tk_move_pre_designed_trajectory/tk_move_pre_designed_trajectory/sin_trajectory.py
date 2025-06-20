import rclpy
from geometry_msgs.msg import Twist
import math
from .base_trajectory import BaseTrajectoryNode

class SinTrajectoryNode(BaseTrajectoryNode):
    def __init__(self):
        super().__init__('sin_trajectory_node')
        
        # Sin-specific parameters
        self.declare_parameter('num_waves', 3)
        self.declare_parameter('period_length', 10.0)
        self.declare_parameter('amplitude', 1.0)
        
        self.time_step = 0.0
        
        # Safety check for sin trajectory
        self.check_sin_safety()

    def check_sin_safety(self):
        """Check if sin trajectory parameters are safe"""
        amplitude = self.get_parameter('amplitude').value
        period = self.get_parameter('period_length').value
        linear_vel = self.get_parameter('linear_velocity').value
        
        # Calculate maximum angular velocity
        max_angular_vel = 2 * math.pi * amplitude * linear_vel / period
        
        if not self.safety_check(max_angular_vel):
            self.get_logger().error(f'Sin trajectory unsafe! Max angular velocity: {max_angular_vel:.2f} rad/s')
            self.is_safe = False
        else:
            self.get_logger().info(f'Sin trajectory safe. Max angular velocity: {max_angular_vel:.2f} rad/s')

    def start_trajectory(self):
        """Start sin trajectory execution"""
        frequency = self.get_parameter('frequency').value
        self.timer = self.create_timer(1.0/frequency, self.sin_callback)

    def sin_callback(self):
        """Execute sin trajectory"""
        num_waves = self.get_parameter('num_waves').value
        period = self.get_parameter('period_length').value
        amplitude = self.get_parameter('amplitude').value
        linear_vel = self.get_parameter('linear_velocity').value
        
        # Calculate total time
        total_time = num_waves * period
        
        if self.time_step >= total_time:
            self.stop_robot()
            self.call_tts('Sin軌道が完了しました')
            self.timer.cancel()
            return
            
        # Calculate sin trajectory
        twist = Twist()
        twist.linear.x = linear_vel
        
        # Angular velocity for sin curve
        omega = 2 * math.pi / period
        twist.angular.z = amplitude * omega * math.cos(omega * self.time_step)
        
        # Safety check
        if not self.safety_check(twist.angular.z):
            self.stop_robot()
            self.call_tts('危険な角速度を検出しました。停止します')
            self.timer.cancel()
            return
            
        self.cmd_vel_pub.publish(twist)
        self.time_step += 1.0/self.get_parameter('frequency').value

def main(args=None):
    rclpy.init(args=args)
    node = SinTrajectoryNode()
    
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
