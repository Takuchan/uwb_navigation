import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray
import serial
import threading

class TkUsbSerialNode(Node):
    def __init__(self):
        super().__init__('tk_usb_serial_node')
        self.publisher_ = self.create_publisher(UInt8MultiArray, 'serial_data', 10)
        self.subscription = self.create_subscription(
            String,
            'uwb_data',
            self.write_callback,
            10)
        self.subscription  # prevent unused variable warning

        # シリアルポートの設定
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.thread = threading.Thread(target=self.read_from_port)
        self.thread.daemon = True
        self.thread.start()

    def read_from_port(self):
        while rclpy.ok():
            if self.ser.in_waiting > 0:
                try:
                    data = self.ser.read(self.ser.in_waiting)
                    if data:
                        msg = UInt8MultiArray()
                        msg.data = data
                        self.publisher_.publish(msg)
                        self.get_logger().info(f'Publishing {len(data)} bytes to serial_data')
                except Exception as e:
                    self.get_logger().error(f"Error reading from serial port: {e}")
                    break

    def write_callback(self, msg):
        self.get_logger().info(f'Writing: "{msg.data}"')
        try:
            self.ser.write(msg.data.encode())
        except Exception as e:
            self.get_logger().error(f"Error writing to serial port: {e}")