import os
import threading
import time
from datetime import datetime

import rclpy
import rosbag2_py
import serial
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String as StringMsg


class SerialRecorderNode(Node):
    """
    Record serial data to a rosbag based on topic commands.

    Subscribes to '/recorder/control' for 'start'/'stop' commands.
    Data is saved to a rosbag with a timestamp and line count in the filename.
    """

    def __init__(self):
        """Initialize the node, parameters, and subscription."""
        super().__init__('serial_recorder_node')
        self.declare_parameter('com_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 3000000)
        self.com_port = self.get_parameter('com_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        self.ser = None
        self.is_recording = False
        self.recording_thread = None
        self.recorded_data = []
        self.data_lock = threading.Lock()

        self.control_subscription = self.create_subscription(
            StringMsg, 'recorder/control', self.control_callback, 10)

        self.get_logger().info(f"Serial Recorder started on {self.com_port}.")
        self.get_logger().info("Send 'start' or 'stop' to '/recorder/control'.")

    def control_callback(self, msg):
        """Handle start/stop commands."""
        command = msg.data.lower()
        if command == 'start' and not self.is_recording:
            self.start_recording()
        elif command == 'stop' and self.is_recording:
            self.stop_recording()

    def connect_serial(self):
        """Connect to the serial port."""
        try:
            self.ser = serial.Serial(self.com_port, self.baud_rate, timeout=0.1)
            self.get_logger().info(f'Connected to {self.com_port}.')
            time.sleep(2)
            self.ser.flushInput()
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to connect to serial port: {e}')
            return False

    def start_recording(self):
        """Start the recording process."""
        if not self.connect_serial():
            return
        self.is_recording = True
        with self.data_lock:
            self.recorded_data = []
        self.recording_thread = threading.Thread(target=self.serial_reader_thread)
        self.recording_thread.start()
        self.get_logger().info('Recording started.')

    def stop_recording(self):
        """Stop the recording process and save data."""
        self.is_recording = False
        if self.recording_thread:
            self.recording_thread.join()
        if self.ser:
            self.ser.close()
            self.ser = None
        self.get_logger().info('Recording stopped.')
        self.save_to_rosbag()

    def serial_reader_thread(self):
        """Read data from serial port in a separate thread."""
        while self.is_recording:
            try:
                if self.ser:
                    # Read multiple lines until timeout
                    lines = self.ser.readlines()
                    if lines:
                        timestamp = self.get_clock().now().nanoseconds
                        with self.data_lock:
                            for line_bytes in lines:
                                line = line_bytes.decode('utf-8', 'ignore').strip()
                                if line:
                                    self.recorded_data.append((timestamp, line))
            except Exception as e:
                self.get_logger().error(f'Error reading from serial: {e}')
                self.is_recording = False

    def save_to_rosbag(self):
        """Save the recorded data to a rosbag file."""
        with self.data_lock:
            if not self.recorded_data:
                self.get_logger().info('No data to save.')
                return
            line_count = len(self.recorded_data)
            filename = datetime.now().strftime('%Y%m%d_%H%M%S') + f'_{line_count}'
            self.get_logger().info(f'Saving {line_count} lines to rosbag: {filename}')
            try:
                writer = rosbag2_py.SequentialWriter()
                storage_options = rosbag2_py.StorageOptions(uri=filename, storage_id='sqlite3')
                converter_options = rosbag2_py.ConverterOptions(
                    input_serialization_format='cdr', output_serialization_format='cdr')
                writer.open(storage_options, converter_options)
                topic_info = rosbag2_py.TopicMetadata(
                    name='/serial_data', type='std_msgs/msg/String', serialization_format='cdr')
                writer.create_topic(topic_info)
                msg_type = get_message(topic_info.type)
                for timestamp, line in self.recorded_data:
                    msg = msg_type()
                    msg.data = line
                    writer.write(topic_info.name, serialize_message(msg), timestamp)
                self.get_logger().info(f'Saved rosbag to: {os.path.abspath(filename)}')
            except Exception as e:
                self.get_logger().error(f'Failed to write rosbag: {e}')
            finally:
                self.recorded_data = []


def main(args=None):
    """Run the serial recorder node."""
    rclpy.init(args=args)
    node = SerialRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
