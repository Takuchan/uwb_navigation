import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np

class SimpleMapPublisher(Node):
    def __init__(self):
        super().__init__('Simple_map_publisher')

        self.publisher_ = self.create_publisher(OccupancyGrid,"/map",10)

        timer_period = 1.0
        self.timer = self.create_timer(timer_period_sec=timer_period,self.timer_callbck)

        self.get_logger().info("シンプルな地図を/mapトピックに配信開始します。")