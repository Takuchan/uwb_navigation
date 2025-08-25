# occupancy_grid_manager.py
import numpy as np
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
from skimage.draw import line_aa
import math

class OccupancyGridManager:
    """
    確率的占有格子地図を管理するクラス。
    - Log-odds表現でマップを内部的に保持する。
    - UWBのLOS/nLOS情報に基づいてマップを更新する。
    - ROSのOccupancyGridメッセージへの変換機能を持つ。
    """

    def __init__(self, resolution: float, width_m: float, height_m: float, origin_x: float, origin_y: float):
        """
        マップを初期化します。
        Args:
            resolution (float): マップの解像度 [m/cell]。
            width_m (float): マップの幅 [m]。
            height_m (float): マップの高さ [m]。
            origin_x (float): マップ原点のx座標 [m]。
            origin_y (float): マップ原点のy座標 [m]。
        """
        self.resolution = resolution
        self.width = int(width_m / resolution)
        self.height = int(height_m / resolution)
        self.origin = Pose()
        self.origin.position.x = origin_x
        self.origin.position.y = origin_y
        self.origin.orientation.w = 1.0

        # Log-oddsマップを初期化 (0は確率0.5に対応)
        self.log_odds_map = np.zeros((self.height, self.width), dtype=np.float32)

        # 更新値 (チューニングパラメータ)
        self.prob_hit = 0.70  # nLOSの場合に障害物であると仮定する確率
        self.prob_miss = 0.35 # LOSの場合に自由空間であると仮定する確率
        self.log_odds_hit = np.log(self.prob_hit / (1 - self.prob_hit))
        self.log_odds_miss = np.log((1 - self.prob_miss) / self.prob_miss) # Note: missは逆数

        # Log-oddsの最大値・最小値 (確率が0または1に発散するのを防ぐ)
        self.log_odds_max = 3.5  # 約97%
        self.log_odds_min = -2.0 # 約12%

    def world_to_map(self, wx: float, wy: float) -> tuple[int, int]:
        """世界座標系 (m) からマップ座標系 (cell) へ変換する。"""
        mx = int((wx - self.origin.position.x) / self.resolution)
        my = int((wy - self.origin.position.y) / self.resolution)
        return mx, my

    def update_map(self, robot_pos_m: np.ndarray, anchor_pos_m: np.ndarray, is_los: bool):
        """
        単一のUWB観測に基づいてマップを更新する。
        Bresenham's line algorithmを使用してロボットとアンカー間のセルを更新する。
        """
        robot_mx, robot_my = self.world_to_map(robot_pos_m[0], robot_pos_m[1])
        anchor_mx, anchor_my = self.world_to_map(anchor_pos_m[0], anchor_pos_m[1])
        
        # skiamgeのline_aaを使ってアンチエイリアス付きの線を描画
        rr, cc, _ = line_aa(robot_my, robot_mx, anchor_my, anchor_mx)

        # 境界チェック
        valid_indices = (rr >= 0) & (rr < self.height) & (cc >= 0) & (cc < self.width)
        rr, cc = rr[valid_indices], cc[valid_indices]

        if is_los:
            # LOS: 線上は自由空間である可能性が高い
            self.log_odds_map[rr, cc] -= self.log_odds_miss
        else:
            # nLOS: 線上に障害物がある可能性が高い
            # 簡単なモデルとして、線上全体の確率を少し上げる
            self.log_odds_map[rr, cc] += self.log_odds_hit

        # 値が発散しないようにクリッピング
        np.clip(self.log_odds_map, self.log_odds_min, self.log_odds_max, out=self.log_odds_map)

    def get_ros_message(self, stamp) -> OccupancyGrid:
        """
        現在のLog-oddsマップをROSのOccupancyGridメッセージに変換する。
        """
        grid_msg = OccupancyGrid()
        
        # Header
        grid_msg.header.stamp = stamp
        grid_msg.header.frame_id = "map"
        
        # MetaData
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height
        grid_msg.info.origin = self.origin

        # Data
        # Log-oddsを確率(0-1)に変換し、さらにOccupancyGridのフォーマット(0-100)に変換
        # P(m|z) = 1 - 1 / (1 + exp(l))
        prob_map = 1.0 - 1.0 / (1.0 + np.exp(self.log_odds_map))
        occupancy_data = (prob_map * 100).astype(np.int8)
        
        # NumPy配列をPythonのリストに変換
        grid_msg.data = occupancy_data.flatten().tolist()
        
        return grid_msg