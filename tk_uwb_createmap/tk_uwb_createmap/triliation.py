import numpy as np
import math

class TrilaterationSolver:

    def __init__(self,d01:float,d12:float,d02:float):
        """"
        アンカーの設置距離をcm単位で指定。
        d01 は 0から1の距離 d12は1から2の距離みたいな感じ"""

        self.anchor0 = np.array([0, 0])
        self.anchor1 = np.array([d01, 0])
        
        x2 = (d01**2 + d02**2 - d12**2) / (2 * d01)
        
        y2_squared = d02**2 - x2**2
        if y2_squared < 0:
            raise ValueError("指定されたアンカー間の距離では三角形を形成できません。")
        
        y2 = math.sqrt(y2_squared)
        self.anchor2 = np.array([x2, y2])

        print("--- アンカー座標 (cm) ---")
        print(f"  Anchor 0: {self.anchor0}")
        print(f"  Anchor 1: {self.anchor1}")
        print(f"  Anchor 2: {self.anchor2}")
        print("--------------------------")

    def calculate_position(self, r0: float, r1: float, r2: float) -> np.ndarray:

        x0, y0 = self.anchor0
        x1, y1 = self.anchor1
        x2, y2 = self.anchor2

        # 連立一次方程式 Ax = b の形に変形
        A = np.array([
            [2 * (x1 - x0), 2 * (y1 - y0)],
            [2 * (x2 - x1), 2 * (y2 - y1)]
        ])

        b = np.array([
            (r0**2 - r1**2) + (x1**2 - x0**2) + (y1**2 - y0**2),
            (r1**2 - r2**2) + (x2**2 - x1**2) + (y2**2 - y1**2)
        ])

        try:
            # numpyの連立方程式ソルバーを使用
            position = np.linalg.solve(A, b)
            return position
        except np.linalg.LinAlgError:
            print("警告: 行列が特異であり、位置を一意に決定できません。")
            return np.array([float('nan'), float('nan')])

