import numpy as np
from typing import Dict, Optional, Tuple, List
from scipy.optimize import least_squares


class Trilateration:
    """
    多辺測位を行うクラス（3つ以上のアンカー対応）
    最小二乗法を使用して、複数のアンカーからの距離データから位置を推定する。
    """

    def __init__(self, anchor_positions: Dict[str, Tuple[float, float, float]]):
        """
        Args:
            anchor_positions: アンカーの座標 {"TWR0": (x, y, z), "TWR1": (x, y, z), ...}
        """
        self.anchor_positions = anchor_positions
        self.anchor_keys = sorted(anchor_positions.keys())

    def set_anchor_positions(self, anchor_positions: Dict[str, Tuple[float, float, float]]):
        """アンカー位置を更新"""
        self.anchor_positions = anchor_positions
        self.anchor_keys = sorted(anchor_positions.keys())

    def get_num_anchors(self) -> int:
        """アンカー数を取得"""
        return len(self.anchor_positions)

    def calculate_position(self, distances: Dict[str, Optional[float]]) -> Optional[Tuple[float, float, float]]:
        """
        多辺測位による位置推定（3つ以上のアンカーに対応）
        
        Args:
            distances: 各アンカーからの距離 {"TWR0": distance, "TWR1": distance, ...}
                      データがない場合はNoneが入る
        
        Returns:
            推定位置 (x, y, z) または計算失敗時はNone
        """
        # 有効なデータのみ抽出
        valid_anchors = []
        valid_distances = []
        
        for key in self.anchor_keys:
            if key in distances and distances[key] is not None and key in self.anchor_positions:
                valid_anchors.append(self.anchor_positions[key])
                valid_distances.append(distances[key])
        
        # 最低3つのアンカーデータが必要
        if len(valid_anchors) < 3:
            return None
        
        # numpy配列に変換
        anchors = np.array(valid_anchors)
        dists = np.array(valid_distances)
        
        # 初期推定位置（アンカーの重心）
        initial_guess = np.mean(anchors, axis=0)
        
        # 最小二乗法による最適化
        def residuals(pos):
            """各アンカーからの距離誤差を計算"""
            return np.sqrt(np.sum((anchors - pos) ** 2, axis=1)) - dists
        
        try:
            # 非線形最小二乗法で位置を推定
            result = least_squares(residuals, initial_guess, method='lm')
            
            if result.success:
                return tuple(result.x)
            else:
                return None
                
        except Exception as e:
            print(f"多辺測位の計算中にエラーが発生: {e}")
            return None

    def calculate_position_2d(self, distances: Dict[str, Optional[float]]) -> Optional[Tuple[float, float]]:
        """
        2D平面での多辺測位（Z=0と仮定）
        
        Args:
            distances: 各アンカーからの距離
        
        Returns:
            推定位置 (x, y) または計算失敗時はNone
        """
        result_3d = self.calculate_position(distances)
        if result_3d is None:
            return None
        return (result_3d[0], result_3d[1])


class AnchorPositionCalculator:
    """
    アンカー間の距離からアンカー位置を計算するクラス
    """

    @staticmethod
    def calculate_positions_from_distances(
        anchor_distances: Dict[Tuple[int, int], float],
        num_anchors: int
    ) -> Dict[str, Tuple[float, float, float]]:
        """
        アンカー間距離から2D平面上の座標を計算（三角形配置）
        
        Args:
            anchor_distances: アンカー間距離 {(0, 1): 3.0, (0, 2): 2.5, (1, 2): 2.0, ...}
            num_anchors: アンカーの総数
        
        Returns:
            アンカー位置 {"TWR0": (x, y, 0), "TWR1": (x, y, 0), ...}
        """
        if num_anchors < 3:
            raise ValueError("アンカーは最低3つ必要です")
        
        # 初期化
        positions = {}
        
        # Anchor 0を原点に配置
        positions[0] = np.array([0.0, 0.0, 0.0])
        
        # Anchor 1をX軸上に配置
        d01 = anchor_distances.get((0, 1), anchor_distances.get((1, 0), 3.0))
        positions[1] = np.array([d01, 0.0, 0.0])
        
        # Anchor 2を三角測量で配置（Y座標が正になるように）
        d02 = anchor_distances.get((0, 2), anchor_distances.get((2, 0), 3.0))
        d12 = anchor_distances.get((1, 2), anchor_distances.get((2, 1), 3.0))
        
        # 余弦定理でAnchor 2の位置を計算
        # Anchor 0が原点、Anchor 1がX軸上にあるとき
        x2 = (d01**2 + d02**2 - d12**2) / (2 * d01)
        y2_squared = d02**2 - x2**2
        
        if y2_squared < 0:
            # 三角形が成立しない場合（距離に矛盾がある）
            # デフォルト配置として60度の角度で配置
            print(f"警告: 指定された距離では三角形が成立しません。デフォルト配置を使用します。")
            x2 = d02 * np.cos(np.pi / 3)
            y2 = d02 * np.sin(np.pi / 3)
        else:
            y2 = np.sqrt(y2_squared)
        
        positions[2] = np.array([x2, y2, 0.0])
        
        # Anchor 3以降を順次配置
        for i in range(3, num_anchors):
            # 既に配置された3つのアンカー（0, 1, 2）からの距離を使用
            d0i = anchor_distances.get((0, i), anchor_distances.get((i, 0)))
            d1i = anchor_distances.get((1, i), anchor_distances.get((i, 1)))
            d2i = anchor_distances.get((2, i), anchor_distances.get((i, 2)))
            
            if d0i is None or d1i is None or d2i is None:
                # 距離情報が不足している場合はデフォルト配置
                angle = (i - 2) * (2 * np.pi / (num_anchors - 2))
                radius = 3.0
                positions[i] = np.array([
                    radius * np.cos(angle) + d01/2,
                    radius * np.sin(angle) + y2/2,
                    0.0
                ])
                continue
            
            # 三辺測量で位置を計算
            pos = AnchorPositionCalculator._trilaterate_2d(
                positions[0][:2], d0i,
                positions[1][:2], d1i,
                positions[2][:2], d2i
            )
            
            if pos is not None:
                positions[i] = np.array([pos[0], pos[1], 0.0])
            else:
                # 計算失敗時はデフォルト配置
                angle = (i - 2) * (2 * np.pi / (num_anchors - 2))
                radius = 3.0
                positions[i] = np.array([
                    radius * np.cos(angle) + d01/2,
                    radius * np.sin(angle) + y2/2,
                    0.0
                ])
        
        # 辞書形式に変換
        result = {}
        for i in range(num_anchors):
            result[f"TWR{i}"] = tuple(positions[i])
        
        return result

    @staticmethod
    def _trilaterate_2d(p1: np.ndarray, r1: float,
                       p2: np.ndarray, r2: float,
                       p3: np.ndarray, r3: float) -> Optional[Tuple[float, float]]:
        """
        2D平面での三辺測量（3つの円の交点を求める）
        
        Args:
            p1, p2, p3: 各アンカーの2D座標 (x, y)
            r1, r2, r3: 各アンカーからの距離
        
        Returns:
            推定位置 (x, y) または計算失敗時はNone
        """
        # 行列を構築
        A = 2 * np.array([
            [p2[0] - p1[0], p2[1] - p1[1]],
            [p3[0] - p1[0], p3[1] - p1[1]]
        ])
        
        b = np.array([
            r1**2 - r2**2 - p1[0]**2 - p1[1]**2 + p2[0]**2 + p2[1]**2,
            r1**2 - r3**2 - p1[0]**2 - p1[1]**2 + p3[0]**2 + p3[1]**2
        ])
        
        try:
            # 連立方程式を解く
            pos = np.linalg.solve(A, b)
            return (pos[0], pos[1])
        except np.linalg.LinAlgError:
            # 行列が特異の場合（アンカーが一直線上にある等）
            return None

    @staticmethod
    def calculate_distance(pos1: Tuple[float, float, float], 
                          pos2: Tuple[float, float, float]) -> float:
        """2点間の距離を計算"""
        return np.sqrt(sum((a - b)**2 for a, b in zip(pos1, pos2)))


# テスト用コード
if __name__ == "__main__":
    # テストケース1: 3つのアンカー（三角形）
    print("=== Test 1: 3 Anchors (Triangle) ===")
    anchor_distances_3 = {
        (0, 1): 4.0,
        (0, 2): 3.0,
        (1, 2): 3.5
    }
    
    positions_3 = AnchorPositionCalculator.calculate_positions_from_distances(
        anchor_distances_3, num_anchors=3
    )
    print("Anchor positions (3 anchors):")
    for name, pos in positions_3.items():
        print(f"  {name}: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
    
    # 距離の検証
    print("\n距離検証:")
    d01_calc = AnchorPositionCalculator.calculate_distance(positions_3["TWR0"], positions_3["TWR1"])
    d02_calc = AnchorPositionCalculator.calculate_distance(positions_3["TWR0"], positions_3["TWR2"])
    d12_calc = AnchorPositionCalculator.calculate_distance(positions_3["TWR1"], positions_3["TWR2"])
    print(f"  TWR0-TWR1: 設定={anchor_distances_3[(0,1)]:.2f}m, 計算={d01_calc:.2f}m")
    print(f"  TWR0-TWR2: 設定={anchor_distances_3[(0,2)]:.2f}m, 計算={d02_calc:.2f}m")
    print(f"  TWR1-TWR2: 設定={anchor_distances_3[(1,2)]:.2f}m, 計算={d12_calc:.2f}m")
    
    # テストケース2: 4つのアンカー
    print("\n=== Test 2: 4 Anchors ===")
    anchor_distances_4 = {
        (0, 1): 5.0,
        (0, 2): 4.0,
        (0, 3): 4.5,
        (1, 2): 4.5,
        (1, 3): 3.5,
        (2, 3): 3.0
    }
    
    positions_4 = AnchorPositionCalculator.calculate_positions_from_distances(
        anchor_distances_4, num_anchors=4
    )
    print("Anchor positions (4 anchors):")
    for name, pos in positions_4.items():
        print(f"  {name}: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
    
    # 測位テスト
    print("\n=== Trilateration Test ===")
    trilat = Trilateration(positions_4)
    
    test_distances = {
        "TWR0": 2.0,
        "TWR1": 2.5,
        "TWR2": 1.8,
        "TWR3": 2.2
    }
    
    position = trilat.calculate_position(test_distances)
    print(f"Estimated position: ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})")