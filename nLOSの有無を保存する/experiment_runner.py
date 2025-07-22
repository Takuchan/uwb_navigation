# experiment_runner.py
import random
import time
from collections import Counter
from typing import List, Dict, Any, Tuple

class ExperimentRunner:
    """
    Type2BPを用いた実験のロジックを管理するクラス。
    データ収集のシミュレーションと最頻値の計算を行う。
    """
    def __init__(self):
        self.uwb_device = None

    def _simulate_uwb_data_frame(self, current_distance: float, nlos_status: str) -> Dict[str, Any]:
        """
        UWBデバイスから1フレームのデータをシミュレートして取得する。
        実際のType2BP APIからのデータ取得に置き換える。
        """
        simulated_distance = current_distance + random.uniform(-0.1, 0.1)
        simulated_horizontal_angle = random.uniform(-1.0, 1.0)
        simulated_elevation_angle = random.uniform(-1.0, 1.0)

        actual_nlos_los = nlos_status
        if random.random() < 0.05:
            actual_nlos_los = "LOS" if nlos_status == "nLOS" else "nLOS"

        return {
            "distance": round(simulated_distance, 2),
            "nlos_los": actual_nlos_los,
            "horizontal_angle": round(simulated_horizontal_angle, 2),
            "elevation_angle": round(simulated_elevation_angle, 2)
        }

    def collect_data_frames(self, num_frames: int, current_distance: float, nlos_status: str) -> Tuple[List[Dict[str, Any]], int]:
        """
        指定されたフレーム数のデータを収集し、その収集にかかった時間を計測する。
        """
        start_time = time.perf_counter_ns()
        data_frames = []
        for _ in range(num_frames):
            data_frames.append(self._simulate_uwb_data_frame(current_distance, nlos_status))
            time.sleep(random.uniform(0.005, 0.015))

        end_time = time.perf_counter_ns()
        elapsed_time_ms = (end_time - start_time) // 1_000_000

        return data_frames, elapsed_time_ms

    def calculate_modes(self, data_frames: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        収集したデータフレームから各パラメータの最頻値を計算する。
        """
        if not data_frames:
            return {
                "mode_distance_m": None,
                "mode_nlos_los": None,
                "mode_horizontal_angle_deg": None,
                "mode_elevation_angle_deg": None
            }

        distances = [d["distance"] for d in data_frames]
        nlos_los_statuses = [d["nlos_los"] for d in data_frames]
        horizontal_angles = [d["horizontal_angle"] for d in data_frames]
        elevation_angles = [d["elevation_angle"] for d in data_frames]

        def get_mode(data_list):
            if not data_list:
                return None
            counts = Counter(data_list)
            if not counts:
                return None
            max_count = 0
            mode_values = []
            for value, count in counts.items():
                if count > max_count:
                    max_count = count
                    mode_values = [value]
                elif count == max_count:
                    mode_values.append(value)
            return mode_values[0] if mode_values else None

        return {
            "mode_distance_m": get_mode(distances),
            "mode_nlos_los": get_mode(nlos_los_statuses),
            "mode_horizontal_angle_deg": get_mode(horizontal_angles),
            "mode_elevation_angle_deg": get_mode(elevation_angles)
        }

    def run_single_experiment(self,
                              experiment_no: int,
                              distance_m: int,
                              uwb_orientation_condition: str,
                              nlos_status_expected: str,
                              trial_count: int) -> Dict[str, Any]:
        """
        単一の実験試行を実行し、結果を返す。
        生データフレームも結果に含める。
        """
        print(f"実験No.{experiment_no} 距離:{distance_m}m, 向き:{uwb_orientation_condition}, 期待nLOS/LOS:{nlos_status_expected}, 試行:{trial_count} を実行中...")

        data_frames, measurement_time_ms = self.collect_data_frames(
            num_frames=20,
            current_distance=float(distance_m),
            nlos_status=nlos_status_expected
        )

        modes = self.calculate_modes(data_frames)

        result = {
            "experiment_no": experiment_no,
            "distance_m": distance_m,
            "uwb_orientation_condition": uwb_orientation_condition,
            "trial_count": trial_count,
            "measurement_time_ms": measurement_time_ms,
            **modes,
            "raw_frames_data": data_frames # ここで生データを結果に追加
        }
        return result

# 使用例 (このファイル自体を実行した場合)
if __name__ == "__main__":
    runner = ExperimentRunner()
    single_result = runner.run_single_experiment(
        experiment_no=1,
        distance_m=5,
        uwb_orientation_condition="垂直",
        nlos_status_expected="LOS",
        trial_count=1
    )
    print("\n単一実験結果:")
    print(single_result)
    print("生データフレームの数:", len(single_result["raw_frames_data"]))