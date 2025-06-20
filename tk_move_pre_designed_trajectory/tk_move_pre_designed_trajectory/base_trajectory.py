import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from oc_tts_interfaces.srv import TTS
import math
import time
import threading

class BaseTrajectoryNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        
        # Parameters
        self.declare_parameter('linear_velocity', 0.5)
        self.declare_parameter('angular_velocity_limit', 1.0)
        self.declare_parameter('frequency', 10.0)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # TTS Client
        self.tts_client = self.create_client(TTS, 'tts_service')
        
        # Safety flags
        self.is_safe = True
        self.user_confirmed = False
        
        # Timer
        self.timer = None
        
        self.get_logger().info(f'{node_name} initialized')

    def call_tts(self, text):
        """TTS service call"""
        if not self.tts_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('TTS service not available')
            return
            
        request = TTS.Request()
        request.text = text
        
        future = self.tts_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'TTS completed: {text}')
            else:
                self.get_logger().error(f'TTS failed: {response.message}')

    def safety_check(self, angular_velocity):
        """Check if angular velocity is safe"""
        limit = self.get_parameter('angular_velocity_limit').value
        if abs(angular_velocity) > limit:
            return False
        return True

    def wait_for_confirmation(self):
        """Wait for user confirmation"""
        def input_thread():
             self.user_confirmed = True
        thread = threading.Thread(target=input_thread)
        thread.daemon = True
        thread.start()
        
        # Wait for confirmation with timeout
        timeout = 30.0
        start_time = time.time()
        while not self.user_confirmed and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            
        return self.user_confirmed

    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Robot stopped')

    def start_trajectory(self):
        """Start trajectory execution - to be implemented by subclasses"""
        raise NotImplementedError("Subclasses must implement start_trajectory")

    def execute_trajectory(self):
        """Main execution flow"""
        # TTS announcement
        self.call_tts(f'{self.get_name()}軌道を開始します')
        
        # Wait for user confirmation
        if not self.wait_for_confirmation():
            self.call_tts('軌道実行がキャンセルされました')
            return
            
        # Safety check
        if not self.is_safe:
            self.call_tts('安全性チェックに失敗しました。実行を停止します')
            return
            
        # Start trajectory
        self.call_tts('軌道実行を開始します')
        self.start_trajectory()
