import numpy as np

class ExtendedKalmanFilter:
    """
    拡張カルマンフィルタ(EKF)による自己位置推定を行うクラス。
    状態は [x, y, vx, vy] (位置と速度) とする。
    """
    def __init__(self, dt: float, initial_pos: np.ndarray):
        """
        EKFを初期化します。

        Args:
            dt (float): 状態遷移の時間間隔 (秒)。
            initial_pos (np.ndarray): 初期位置 [x, y]。
        """
        self.dt = dt

        # 状態ベクトル [x, y, vx, vy]
        # 初期速度は0と仮定
        self.x = np.array([initial_pos[0], initial_pos[1], 0, 0])

        # 状態遷移行列F (等速直線運動モデル)
        self.F = np.array([
            [1, 0, self.dt, 0],
            [0, 1, 0, self.dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # 観測行列H (状態のうち位置のみを観測)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        # プロセスノイズの共分散行列Q
        # 加速度のばらつきを仮定。値はチューニングが必要。
        q_val = 0.1
        self.Q = np.eye(4) * q_val

        # 観測ノイズの共分散行列R
        # 観測（三辺測位）のばらつき。値はチューニングが必要。
        r_val = 0.5
        self.R = np.eye(2) * r_val

        # 誤差共分散行列P
        self.P = np.eye(4)

    def predict(self) -> np.ndarray:
        """
        次の状態を予測する (予測ステップ)。
        """
        # 状態の予測
        self.x = self.F @ self.x
        # 誤差共分散の予測
        self.P = self.F @ self.P @ self.F.T + self.Q
        
        return self.x

    def update(self, z: np.ndarray):
        """
        観測値zを使って状態を更新する (更新ステップ)。

        Args:
            z (np.ndarray): 観測値 [x, y]。
        """
        # カルマンゲインの計算
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # 状態の更新
        y = z - self.H @ self.x  # 観測残差
        self.x = self.x + K @ y

        # 誤差共分散の更新
        self.P = (np.eye(self.x.shape[0]) - K @ self.H) @ self.P
