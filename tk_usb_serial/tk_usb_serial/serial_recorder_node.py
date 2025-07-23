import os
import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String as StringMsg

class SerialPublisherNode(Node):
    """
    シリアルポートからデータを読み取り、ROS 2トピックとして配信するノード。
    """

    def __init__(self):
        """ノードの初期化、パラメータの宣言、シリアル接続、パブリッシャーとタイマーの作成を行う。"""
        super().__init__('serial_publisher_node')

        # パラメータの宣言（ポート、ボーレート、トピック名）
        self.declare_parameter('com_port', '/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 3000000)
        self.declare_parameter('topic_name', '/serial_data')

        # パラメータの取得
        self.com_port = self.get_parameter('com_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        self.get_logger().info(
            f"Trying to connect to '{self.com_port}' at {self.baud_rate} baud."
        )

        # パブリッシャーの作成
        self.publisher_ = self.create_publisher(StringMsg, self.topic_name, 10)

        # シリアルポートへの接続
        self.ser = None
        try:
            self.ser = serial.Serial(self.com_port, self.baud_rate, timeout=1.0)
            self.get_logger().info(f"Successfully connected to serial port '{self.com_port}'.")
            # 接続直後はデータが不安定な場合があるため、バッファをクリア
            self.ser.flushInput()
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port '{self.com_port}': {e}")
            # エラー発生時はノードを終了させる
            self.destroy_node()
            rclpy.shutdown()
            return

        # 定期的にシリアルデータを読み込むためのタイマーを設定 (0.001秒ごと)
        self.timer = self.create_timer(0.001, self.timer_callback)
        self.get_logger().info(f"Publishing data to '{self.topic_name}' topic.")


    def timer_callback(self):
        """タイマーによって定期的に呼び出され、シリアルデータの読み込みと配信を行う。"""
        if self.ser and self.ser.is_open:
            try:
                # 1行読み込む (改行コードまで)
                line_bytes = self.ser.readline()
                if line_bytes:
                    # bytesからstringに変換し、前後の空白や改行を削除
                    line_str = line_bytes.decode('utf-8', 'ignore').strip()
                    if line_str:
                        # Stringメッセージを作成してパブリッシュ
                        msg = StringMsg()
                        msg.data = line_str
                        self.publisher_.publish(msg)
                        # デバッグ用にコンソールにログを出力 (大量に出力されるためdebugレベルに)
                        self.get_logger().debug(f'Published: "{msg.data}"')

            except serial.SerialException as e:
                self.get_logger().error(f"Error reading from serial port: {e}")
                # エラーが発生した場合はタイマーを停止し、ポートを閉じる
                self.timer.cancel()
                self.ser.close()
            except Exception as e:
                self.get_logger().error(f"An unexpected error occurred: {e}")
                self.timer.cancel()
                if self.ser.is_open:
                    self.ser.close()

    def destroy_node(self):
        """ノードが破棄される際のクリーンアップ処理。"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()


def main(args=None):
    """メイン関数: ノードを初期化し、実行する。"""
    rclpy.init(args=args)
    serial_publisher_node = SerialPublisherNode()
    
    # ノードが正常に初期化された場合のみspin()を呼び出す
    # (シリアル接続失敗時にシャットダウンされるため)
    if rclpy.ok():
        try:
            rclpy.spin(serial_publisher_node)
        except KeyboardInterrupt:
            pass
        finally:
            # クリーンアップ
            serial_publisher_node.destroy_node()
            if not rclpy.is_shutdown():
                rclpy.shutdown()


if __name__ == '__main__':
    main()

    