#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
import numpy as np
from collections import deque
from std_msgs.msg import String


class UwbDivergenceCorrection(Node):
    """
    UWBデータの揺れを補正するノード。
    生のUWBデータを受信し、分散ベースのフィルタリングを適用して、
    フィルタリングされたデータを配信する。
    """
    def __init__(self):
        super().__init__('uwb_divergence_correction')

        # パラメータ宣言
        self.declare_parameter('history_size', 5)
        self.declare_parameter('variance_threshold', 0.05)
        
        self.history_size = self.get_parameter('history_size').get_parameter_value().integer_value
        self.variance_threshold = self.get_parameter('variance_threshold').get_parameter_value().double_value
        
        self.get_logger().info(f"UWB Divergence Correction Node initialized")
        self.get_logger().info(f"History size: {self.history_size}, Variance threshold: {self.variance_threshold}")

        # 距離の履歴を保存する辞書 (twr_id -> deque)
        self.distance_history = {}

        # サブスクライバー: 生のUWBデータを受信
        self.uwb_sub = self.create_subscription(
            String,
            '/uwb_data_raw',
            self.uwb_callback,
            10
        )

        # パブリッシャー: フィルタリングされたUWBデータを配信
        self.uwb_pub = self.create_publisher(
            String,
            '/uwb_data_json',
            10
        )

    def uwb_callback(self, msg):
        """
        生のUWBデータを受信し、フィルタリングして配信する。
        """
        try:
            uwb_data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"UWBデータのJSONパースに失敗しました: {e}")
            return

        # フィルタリングされたデータを保存する辞書
        filtered_data = {}

        for twr_id_str, data in uwb_data.items():
            if not data:
                continue

            twr_index = int(twr_id_str.replace('TWR', ''))
            
            # 履歴がない場合は初期化
            if twr_index not in self.distance_history:
                self.distance_history[twr_index] = deque(maxlen=self.history_size)
            
            # 距離を履歴に追加
            self.distance_history[twr_index].append(data['distance'])
            
            # 履歴が溜まるまではフィルタリングせずにそのまま通す
            if len(self.distance_history[twr_index]) < self.history_size:
                filtered_data[twr_id_str] = data
                continue
            
            # 分散を計算
            variance = np.var(self.distance_history[twr_index])
            
            # 分散が閾値を超える場合はデータを採用しない
            if variance > self.variance_threshold:
                self.get_logger().warning(
                    f"❌データ不採用 [不安定] from {twr_id_str} (variance: {variance:.4f})"
                )
                # データを採用しない（filtered_dataに追加しない）
                continue
            
            # 分散が閾値以下の場合はデータを採用
            filtered_data[twr_id_str] = data

        # フィルタリングされたデータを配信
        filtered_json = json.dumps(filtered_data)
        filtered_msg = String()
        filtered_msg.data = filtered_json
        self.uwb_pub.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = UwbDivergenceCorrection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
