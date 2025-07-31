import pygame
import numpy as np
import time
from triliation import TrilaterationSolver
from serialandFilter import SerialFilter
from kalman_filter import ExtendedKalmanFilter

class UwbMapper:
    """
    UWB測位データを可視化し、nLOS/LOSに基づいて環境地図を生成するクラス。
    """
    def __init__(self, solver: TrilaterationSolver, serial_filter: SerialFilter, window_size_m: tuple = (15, 15)):
        self.solver = solver
        self.filter = serial_filter
        self.running = True

        # Pygameの初期化
        pygame.init()
        pygame.display.set_caption("UWB Real-Time Mapper")

        # 画面サイズと座標系の設定
        self.padding_px = 50
        self.screen_width = 800
        self.screen_height = 800
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        
        # ワールド座標(m)からスクリーン座標(px)への変換スケールを計算
        # アンカーが画面内に収まるようにスケールを決定
        all_anchor_coords = np.array([solver.anchor0, solver.anchor1, solver.anchor2])
        min_coords = np.min(all_anchor_coords, axis=0)
        max_coords = np.max(all_anchor_coords, axis=0)
        world_range = max_coords - min_coords
        
        scale_x = (self.screen_width - 2 * self.padding_px) / world_range[0] if world_range[0] > 0 else 1
        scale_y = (self.screen_height - 2 * self.padding_px) / world_range[1] if world_range[1] > 0 else 1
        self.scale = min(scale_x, scale_y)
        
        self.origin_px = np.array([
            self.padding_px - min_coords[0] * self.scale,
            self.screen_height - self.padding_px + min_coords[1] * self.scale
        ])

        # EKFの初期化
        self.dt = 1.0 / 20.0  # 20Hz
        initial_pos = np.array([solver.anchor1[0] / 2, solver.anchor2[1] / 2]) # 適当な初期位置
        self.ekf = ExtendedKalmanFilter(dt=self.dt, initial_pos=initial_pos)
        
        self.prev_pos_px = None

        # 描画色
        self.COLOR_BACKGROUND = (20, 20, 20) # 暗い背景
        self.COLOR_GRID = (40, 40, 40)
        self.COLOR_ANCHOR = (255, 0, 0) # Red
        self.COLOR_LOS_LINE = (100, 100, 100) # Gray
        self.COLOR_NLOS_LINE = (0, 0, 0) # Black
        self.COLOR_PATH = (0, 150, 255) # Blue
        self.COLOR_SELF = (0, 255, 0) # Green
        self.COLOR_TEXT = (255, 255, 255)

        self.font = pygame.font.Font(None, 24)
        
        # 背景（マップ）を一度だけ描画するためのサーフェス
        self.map_surface = pygame.Surface((self.screen_width, self.screen_height))
        self.map_surface.fill(self.COLOR_BACKGROUND)
        self._draw_grid()
        self._draw_anchors()

    def _world_to_screen(self, pos_m: np.ndarray) -> tuple:
        """ワールド座標(m)をスクリーン座標(px)に変換する。"""
        screen_pos = self.origin_px + np.array([pos_m[0], -pos_m[1]]) * self.scale
        return int(screen_pos[0]), int(screen_pos[1])

    def _draw_grid(self):
        """背景にグリッドを描画する。"""
        for x in range(0, self.screen_width, 50):
            pygame.draw.line(self.map_surface, self.COLOR_GRID, (x, 0), (x, self.screen_height))
        for y in range(0, self.screen_height, 50):
            pygame.draw.line(self.map_surface, self.COLOR_GRID, (0, y), (y, self.screen_width))

    def _draw_anchors(self):
        """アンカーの位置を描画する。"""
        for i, anchor_pos_m in enumerate([self.solver.anchor0, self.solver.anchor1, self.solver.anchor2]):
            pos_px = self._world_to_screen(anchor_pos_m)
            pygame.draw.circle(self.map_surface, self.COLOR_ANCHOR, pos_px, 8)
            text = self.font.render(f"A{i}", True, self.COLOR_TEXT)
            self.map_surface.blit(text, (pos_px[0] + 10, pos_px[1]))

    def run(self):
        """メインループを実行する。"""
        clock = pygame.time.Clock()

        while self.running:
            # --- イベント処理 ---
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

            # --- データ取得と計算 ---
            self.ekf.predict() # EKF予測ステップ

            anchor_data = self.filter.read_anchor_data_snapshot(timeout=0.04) # 25Hz
            
            if anchor_data and all(anchor_data.get(f"TWR{i}") for i in range(3)):
                # 三辺測位で生データを計算
                r0 = anchor_data["TWR0"]["distance"]
                r1 = anchor_data["TWR1"]["distance"]
                r2 = anchor_data["TWR2"]["distance"]
                raw_pos_m = self.solver.calculate_position(r0, r1, r2)
                
                # EKF更新ステップ
                if not np.isnan(raw_pos_m).any():
                    self.ekf.update(raw_pos_m)

                # --- 描画処理 ---
                # EKFから平滑化された位置を取得
                current_pos_m = self.ekf.x[:2]
                current_pos_px = self._world_to_screen(current_pos_m)

                # LOS/nLOS線を描画
                for i in range(3):
                    anchor_pos_m = [self.solver.anchor0, self.solver.anchor1, self.solver.anchor2][i]
                    anchor_pos_px = self._world_to_screen(anchor_pos_m)
                    
                    nlos_status = anchor_data[f"TWR{i}"]["nlos_los"]
                    line_color = self.COLOR_NLOS_LINE if nlos_status == "nLOS" else self.COLOR_LOS_LINE
                    
                    pygame.draw.line(self.map_surface, line_color, current_pos_px, anchor_pos_px, 5)

                # 移動軌跡を描画
                if self.prev_pos_px:
                    pygame.draw.line(self.map_surface, self.COLOR_PATH, self.prev_pos_px, current_pos_px, 1)
                self.prev_pos_px = current_pos_px

            # --- 画面更新 ---
            # 累積されたマップをコピー
            self.screen.blit(self.map_surface, (0, 0))

            # 現在位置（動的な部分）を描画
            final_pos_m = self.ekf.x[:2]
            final_pos_px = self._world_to_screen(final_pos_m)
            pygame.draw.circle(self.screen, self.COLOR_SELF, final_pos_px, 6)

            pygame.display.flip()
            clock.tick(20) # 20 FPSに制限

        pygame.quit()
