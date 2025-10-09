from .serialandFilter import SerialFilter
from typing import Dict, Any, Optional

import json  # JSONライブラリをインポート

# ROS2関連のライブラリをインポート
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class UWBPublisherNode(Node):
    """
    UWBデータをシリアルポートから読み取り、ROS2トピックとして配信するノード。
    """
    def __init__(self):
        super().__init__('uwb_data_publisher')

        # パラメータの宣言（ROS2経由で値を変更可能にする）
        self.declare_parameter('com_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 3000000)
        self.declare_parameter('num_anchors', 3)
        self.declare_parameter('publish_rate', 20.0) # Hz

        # パラメータの取得
        com_port = self.get_parameter('com_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        num_anchors = self.get_parameter('num_anchors').get_parameter_value().integer_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.get_logger().info(f"COM Port: {com_port}, Baud Rate: {baud_rate}, Anchors: {num_anchors}")

        # SerialFilterクラスのインスタンスを作成
        self.uwb_filter = SerialFilter(
            com_port=com_port,
            baud_rate=baud_rate,
            num_anchors=num_anchors
        )
        
        # シリアルポートへの接続
        if not self.uwb_filter.connect_serial():
            self.get_logger().error("シリアルポートへの接続に失敗しました。ノードをシャットダウンします。")
            raise ConnectionError("Failed to connect to the serial port.")

        # パブリッシャーの作成
        # トピック名: 'uwb_data_json', メッセージ型: String, キューサイズ: 10
        self.publisher_ = self.create_publisher(String, 'uwb_data_json', 10)

        # タイマーの作成（指定した周期でコールバック関数を実行）
        timer_period = 1.0 / publish_rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """
        タイマーによって定期的に呼び出され、データの取得と配信を行う。
        """
        # データを1回分読み取る
        anchor_data = self.uwb_filter.read_anchor_data_snapshot(timeout=0.05)

        if anchor_data:
            # 辞書データをJSON形式の文字列に変換
            json_string = json.dumps(anchor_data)
            
            # Stringメッセージを作成してデータをセット
            msg = String()
            msg.data = json_string
            
            # トピックにメッセージを配信
            self.publisher_.publish(msg)
            
            # ログに出力（デバッグ用）
            # self.get_logger().info(f'Publishing: "{json_string}"')

    def on_shutdown(self):
        """ノード終了時にシリアルポートを切断する"""
        self.get_logger().info('ノードをシャットダウンします。')
        self.uwb_filter.disconnect_serial()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        # ノードの初期化と実行
        uwb_publisher_node = UWBPublisherNode()
        rclpy.spin(uwb_publisher_node)
    except (KeyboardInterrupt, ConnectionError) as e:
        # キーボード割り込みか接続エラーでここに来る
        if isinstance(e, ConnectionError):
            print(f"ノードの初期化中にエラーが発生しました: {e}")
        else:
            print("キーボード割り込みを検出しました。")
    finally:
        # ノードが正常に作成されていればクリーンアップ処理を行う
        if 'uwb_publisher_node' in locals() and rclpy.ok():
            uwb_publisher_node.on_shutdown()
            uwb_publisher_node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
