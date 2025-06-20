import rclpy
from geometry_msgs.msg import Twist
import math
from .base_trajectory import BaseTrajectoryNode

class CircleTrajectoryNode(BaseTrajectoryNode):
    def __init__(self):
        super().__init__('circle_trajectory_node')
        
        # Circle-specific parameters
        self.declare_parameter('radius', 2.0)
        self.declare_parameter('num_rotations', 2)
        
        self.time_step = 0.0
        self.check_circle_safety()

    def check_circle_safety(self):
        """Check if circle trajectory parameters are safe"""
        radius = self.get_parameter('radius').value
        linear_vel = self.get_parameter('linear_velocity').value
        
        # Angular velocity for circular motion
        angular_vel = linear_vel / radius
        
        if not self.safety_check(angular_vel):
            self.get_logger().error(f'Circle trajectory unsafe! Angular velocity: {angular_vel:.2f} rad/s')
            self.is_safe = False
        else:
            self.get_logger().info(f'Circle trajectory safe. Angular velocity: {angular_vel:.2f} rad/s')

    def start_trajectory(self):
        """Start circle trajectory execution"""
        frequency = self.get_parameter('frequency').value
        self.timer = self.create_timer(1.0/frequency, self.circle_callback)

    def circle_callback(self):
        """Execute circle trajectory"""
        radius = self.get_parameter('radius').value
        num_rotations = self.get_parameter('num_rotations').value
        linear_vel = self.get_parameter('linear_velocity').value
        
        # Calculate total time for rotations
        circumference = 2 * math.pi * radius
        total_time = (num_rotations * circumference) / linear_vel
        
        if self.time_step >= total_time:
            self.stop_robot()
            self.call_tts('円軌道が完了しました')
            self.timer.cancel()
            return
            
        # Calculate circle trajectory
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = linear_vel / radius
        
        self.cmd_vel_pub.publish(twist)
        self.time_step += 1.0/self.get_parameter('frequency').value

def main(args=None):
    rclpy.init(args=args)
    node = CircleTrajectoryNode()
    
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
