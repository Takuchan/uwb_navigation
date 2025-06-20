import rclpy
from geometry_msgs.msg import Twist
import math
import time
from .base_trajectory import BaseTrajectoryNode

class RectangleTrajectoryNode(BaseTrajectoryNode):
    def __init__(self):
        super().__init__('rectangle_trajectory_node')
        
        self.declare_parameter('width', 2.0)
        self.declare_parameter('height', 1.0)
        self.declare_parameter('turn_velocity', 0.5)

        self.state = 'IDLE'
        self.side_index = 0
        self.action_start_time = 0.0
        
        self.check_rectangle_safety()

    def check_rectangle_safety(self):
        turn_vel = self.get_parameter('turn_velocity').value
        if not self.safety_check(turn_vel):
            self.get_logger().error(f'Rectangle unsafe! Turn velocity {turn_vel:.2f} is over the limit.')
            self.is_safe = False
        else:
            self.get_logger().info('Rectangle trajectory safe.')

    def start_trajectory(self):
        frequency = self.get_parameter('frequency').value
        self.state = 'MOVING'
        self.action_start_time = self.get_clock().now().nanoseconds / 1e9
        self.timer = self.create_timer(1.0/frequency, self.rectangle_callback)

    def rectangle_callback(self):
        if self.side_index >= 4:
            self.stop_robot()
            self.call_tts('長方形軌道が完了しました')
            self.timer.cancel()
            return

        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        linear_vel = self.get_parameter('linear_velocity').value
        turn_vel = self.get_parameter('turn_velocity').value
        
        side_lengths = [width, height, width, height]
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = current_time - self.action_start_time
        
        twist = Twist()

        if self.state == 'MOVING':
            move_duration = side_lengths[self.side_index] / linear_vel
            if elapsed_time < move_duration:
                twist.linear.x = linear_vel
            else:
                self.state = 'TURNING'
                self.action_start_time = current_time
                self.stop_robot()
                time.sleep(0.1)
        
        elif self.state == 'TURNING':
            turn_angle = math.pi / 2 # 90 degrees
            turn_duration = turn_angle / turn_vel
            if elapsed_time < turn_duration:
                twist.angular.z = turn_vel
            else:
                self.side_index += 1
                if self.side_index < 4:
                    self.state = 'MOVING'
                    self.action_start_time = current_time
                else:
                    self.state = 'IDLE'
                self.stop_robot()
                time.sleep(0.1)

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = RectangleTrajectoryNode()
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
