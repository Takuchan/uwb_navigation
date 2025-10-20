import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from PIL import Image
import yaml
import os
import sys
import select

class MapSaver(Node):
    """
    /mapトピックのnav_msgs/msg/OccupancyGridを受け取り、
    デスクトップにPGM画像ファイルと対応するYAMLファイルに保存するROS 2ノード。
    保存完了時にディレクトリパスを表示します。
    """
    def __init__(self):
        super().__init__('map_saver')
        
        # デスクトップディレクトリのパスを取得
        desktop_path = os.path.expanduser('~/Desktop')
        
        # パラメータの宣言と取得
        self.declare_parameter('file_name', 'ros2_map')
        self.declare_parameter('save_directory', desktop_path)
        self.declare_parameter('occupied_thresh', 0.65)
        self.declare_parameter('free_thresh', 0.25)

        self.file_name = self.get_parameter('file_name').get_parameter_value().string_value
        self.save_directory = self.get_parameter('save_directory').get_parameter_value().string_value
        self.occupied_thresh = self.get_parameter('occupied_thresh').get_parameter_value().double_value
        self.free_thresh = self.get_parameter('free_thresh').get_parameter_value().double_value

        self.get_logger().info(f"保存ディレクトリ: {self.save_directory}")
        self.get_logger().info("地図データを待機中... /map トピック")
        
        # 保存ディレクトリが存在しない場合は作成を試みる
        if not os.path.exists(self.save_directory):
            try:
                os.makedirs(self.save_directory)
                self.get_logger().info(f"ディレクトリを作成しました: {self.save_directory}")
            except OSError as e:
                self.get_logger().error(f"ディレクトリの作成に失敗しました: {e}")
                self.save_directory = os.getcwd() # 失敗した場合はカレントディレクトリにフォールバック
                self.get_logger().warn(f"代わりにカレントディレクトリに保存します: {self.save_directory}")

        # /mapトピックのサブスクライバ
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/kachaka/mapping/map',
            self.map_callback,
            10
        )
        self.map_data = None

    def map_callback(self, msg):
        """
        /mapトピックのデータを受信し、保存のために保持する。
        """
        self.map_data = msg
        # ログメッセージは一度だけ表示
        if not hasattr(self, 'received_initial_map'):
             self.get_logger().info("地図データを受信しました。's' を入力してEnterでデスクトップに保存します。")
             setattr(self, 'received_initial_map', True)


    def save_map(self):
        """
        受信した地図データをPGMファイルとYAMLファイルとして保存する。
        """
        if self.map_data is None:
            self.get_logger().warn("保存できる地図データがありません。/mapトピックからの受信を確認してください。")
            return

        map_msg = self.map_data
        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y

        self.get_logger().info(f"地図を保存中: {width}x{height}、解像度: {resolution}m/px")

        # 1. PGM画像の生成
        pgm_data = []
        for occ_prob in map_msg.data:
            if occ_prob == -1:
                pgm_data.append(205)  # 不明な領域 (灰色)
            elif occ_prob == 0:
                pgm_data.append(254)  # 自由空間 (白)
            elif occ_prob == 100:
                pgm_data.append(0)    # 占有領域 (黒)
            else:
                pgm_data.append(205) # その他は不明として処理

        # Imageオブジェクトの作成と保存
        img = Image.new('L', (width, height)) 
        img.putdata(pgm_data)

        # 画像を垂直方向に反転 (ROSのOccupancyGridデータ形式に合わせるため)
        img = img.transpose(Image.Transpose.FLIP_TOP_BOTTOM)

        pgm_filepath = os.path.join(self.save_directory, f"{self.file_name}.pgm")
        img.save(pgm_filepath)
        self.get_logger().info(f"✅ PGM画像を保存しました: {pgm_filepath}")

        # 2. YAMLファイルの生成
        yaml_data = {
            'image': f"{self.file_name}.pgm",
            'resolution': resolution,
            'origin': [origin_x, origin_y, 0.0],
            'negate': 0,
            'occupied_thresh': self.occupied_thresh,
            'free_thresh': self.free_thresh,
            'mode': 'trinary' 
        }

        yaml_filepath = os.path.join(self.save_directory, f"{self.file_name}.yaml")
        with open(yaml_filepath, 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False)

        self.get_logger().info(f"✅ YAMLファイルを保存しました: {yaml_filepath}")
        
        # 3. 保存ディレクトリのパスを表示
        self.get_logger().info(f"🎉 地図の保存が完了しました。保存先ディレクトリ: {self.save_directory}")
        self.get_logger().info("----------------------------------------------------------------")


def main(args=None):
    rclpy.init(args=args)

    map_saver = MapSaver()

    try:
        # 地図データを受信するまでスピン
        while rclpy.ok() and map_saver.map_data is None:
            rclpy.spin_once(map_saver, timeout_sec=0.1)

        if map_saver.map_data is None and rclpy.ok():
             map_saver.get_logger().warn("タイムアウト: 地図データを受信できませんでした。")
             return

        # データの受信後、保存を待機
        map_saver.get_logger().info("ノードを実行中です。保存するには 's' を入力してEnterを押してください。")
        map_saver.get_logger().info("ノードを終了するには Ctrl+C を押してください。")

        while rclpy.ok():
            rclpy.spin_once(map_saver, timeout_sec=0.1)
            
            # キーボード入力のチェック
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline().strip()
                if line == 's':
                    map_saver.save_map()
    
    except KeyboardInterrupt:
        pass
    finally:
        map_saver.get_logger().info("ノードをシャットダウンします。")
        map_saver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()