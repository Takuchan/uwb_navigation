import rclpy
from geometry_msgs.msg import Twist
import math
import time
from .base_trajectory import BaseTrajectoryNode

class PolygonTrajectoryNode(BaseTrajectoryNode):
    def __init__(self, node_name, num_sides):
        super().__init__(node_name)
        self.num_sides = num_sides
        
        self.declare_parameter('side_length', 1.0)
        self.declare_parameter('turn_velocity', 0.5) # rad/s

        self.state = 'IDLE' # IDLE, MOVING, TURNING
        self.side_count = 0
        self.action_start_time = 0.0
        
        self.check_polygon_safety()

    def check_polygon_safety(self):
        turn_vel = self.get_parameter('turn_velocity').value
        if not self.safety_check(turn_vel):
            self.get_logger().error(f'Polygon unsafe! Turn velocity {turn_vel:.2f} is over the limit.')
            self.is_safe = False
        else:
            self.get_logger().info('Polygon trajectory safe.')

    def start_trajectory(self):
        frequency = self.get_parameter('frequency').value
        self.state = 'MOVING'
        self.action_start_time = self.get_clock().now().nanoseconds / 1e9
        self.timer = self.create_timer(1.0/frequency, self.polygon_callback)

    def polygon_callback(self):
        if self.side_count >= self.num_sides:
            self.stop_robot()
            self.call_tts('多角形軌道が完了しました')
            self.timer.cancel()
            return

        side_length = self.get_parameter('side_length').value
        linear_vel = self.get_parameter('linear_velocity').value
        turn_vel = self.get_parameter('turn_velocity').value
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = current_time - self.action_start_time
        
        twist = Twist()

        if self.state == 'MOVING':
            move_duration = side_length / linear_vel
            if elapsed_time < move_duration:
                twist.linear.x = linear_vel
            else:
                self.state = 'TURNING'
                self.action_start_time = current_time
                self.stop_robot() # Stop before turning
                time.sleep(0.1) # Short pause
        
        elif self.state == 'TURNING':
            turn_angle = 2 * math.pi / self.num_sides
            turn_duration = turn_angle / turn_vel
            if elapsed_time < turn_duration:
                twist.angular.z = turn_vel
            else:
                self.side_count += 1
                if self.side_count < self.num_sides:
                    self.state = 'MOVING'
                    self.action_start_time = current_time
                else: # Completed
                    self.state = 'IDLE'
                self.stop_robot()
                time.sleep(0.1)

        self.cmd_vel_pub.publish(twist)
