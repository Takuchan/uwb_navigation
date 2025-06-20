import rclpy
from geometry_msgs.msg import Twist
import math
from .base_trajectory import BaseTrajectoryNode

class EllipseTrajectoryNode(BaseTrajectoryNode):
    def __init__(self):
        super().__init__('ellipse_trajectory_node')
        
        self.declare_parameter('semi_major_axis', 2.0)
        self.declare_parameter('semi_minor_axis', 1.0)
        self.declare_parameter('num_rotations', 2)
        self.declare_parameter('parameter_speed', 0.5) # rad/s for parameter t
        self.declare_parameter('linear_velocity_limit', 1.0)

        self.time_step = 0.0
        self.check_ellipse_safety()

    def check_ellipse_safety(self):
        a = self.get_parameter('semi_major_axis').value
        b = self.get_parameter('semi_minor_axis').value
        omega = self.get_parameter('parameter_speed').value
        
        # Max linear vel: omega * max(a,b)
        max_linear_vel = omega * max(a, b)
        if max_linear_vel > self.get_parameter('linear_velocity_limit').value:
            self.get_logger().error(f'Ellipse unsafe! Max linear vel: {max_linear_vel:.2f} m/s')
            self.is_safe = False
        
        # Max angular vel: omega * a*b / min(b^2, a^2) = omega * max(a,b)/min(a,b)
        if min(a,b) > 0:
            max_angular_vel = omega * max(a,b) / min(a,b)
            if not self.safety_check(max_angular_vel):
                self.get_logger().error(f'Ellipse unsafe! Max angular vel: {max_angular_vel:.2f} rad/s')
                self.is_safe = False
        
        if self.is_safe:
            self.get_logger().info('Ellipse trajectory safe.')

    def start_trajectory(self):
        frequency = self.get_parameter('frequency').value
        self.timer = self.create_timer(1.0/frequency, self.ellipse_callback)

    def ellipse_callback(self):
        a = self.get_parameter('semi_major_axis').value
        b = self.get_parameter('semi_minor_axis').value
        num_rotations = self.get_parameter('num_rotations').value
        omega = self.get_parameter('parameter_speed').value
        
        total_angle = num_rotations * 2 * math.pi
        current_angle = self.time_step * omega

        if current_angle >= total_angle:
            self.stop_robot()
            self.call_tts('楕円軌道が完了しました')
            self.timer.cancel()
            return
            
        twist = Twist()
        angle = omega * self.time_step
        
        # Velocities in robot frame
        v_lin_sq_term = (a**2 * math.sin(angle)**2 + b**2 * math.cos(angle)**2)
        twist.linear.x = omega * math.sqrt(v_lin_sq_term)
        twist.angular.z = (a * b * omega) / v_lin_sq_term if v_lin_sq_term > 0 else 0.0
        
        self.cmd_vel_pub.publish(twist)
        self.time_step += 1.0/self.get_parameter('frequency').value

def main(args=None):
    rclpy.init(args=args)
    node = EllipseTrajectoryNode()
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
