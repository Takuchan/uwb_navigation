# experiment_runner.py
import random
import time
from collections import Counter
from typing import List, Dict, Any, Tuple,Optional
import serial
import re

class ExperimentRunner:
    """
    Type2BPを用いた実験のロジックを管理するクラス。
    データ収集（シリアル通信）、最頻値の計算を行う。
    """
    def __init__(self, com_port="COM7", baud_rate=3000000):
        self.com_port = com_port
        self.baud_rate = baud_rate
        self.ser = None

        self.connect_serial()

    def connect_serial(self) -> bool:
        """
        シリアルポートに接続を試行する。
        """
        if self.ser and self.ser.is_open:
            print(f"Already connected to {self.com_port}.")
            return True
        try:
            self.ser = serial.Serial(self.com_port, self.baud_rate, timeout=1)
            print(f"✅ {self.com_port}に{self.baud_rate}のボーレートで接続しました。")
            time.sleep(2)
            self.ser.flushInput()
            return True
        except serial.SerialException as e:
            print(f"❌ シリアルポートへの接続に失敗しました: {e}")
            print(f"指定されたCOMポート '{self.com_port}' が正しいか、デバイスが接続されているか確認してください。")
            return False
        except Exception as e:
            print(f"❌ 予期せぬエラーが発生しました: {e}")
            return False

    def disconnect_serial(self):
        """シリアルポートを切断する。"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"🔌 {self.com_port}から切断しました。")

    def _read_uwb_data_from_serial(self) -> Optional[Dict[str, Any]]:
        """
        シリアルポートから1つの完全なTWR[0]データフレームを読み取り、パースする。
        nLos, distance, aoa_azimuth, aoa_elevationの4つの情報が揃うまで待つ。
        """
        if not self.ser or not self.ser.is_open:
            return None

        # 1つの論理的なフレームのデータを格納する辞書
        current_twr0_frame = {}
        # 必要なデータ項目をトラックするためのセット
        required_keys = {"nlos_los", "distance", "horizontal_angle", "elevation_angle"}

        timeout_start = time.time()
        # 1つのフレームが揃うまでのタイムアウト（例えば2秒）
        frame_timeout = 2.0

        while (time.time() - timeout_start) < frame_timeout:
            if self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    # print(f"Raw serial line: {line}") # デバッグ用

                    # 正規表現パターンを定義
                    nlos_pattern = r'APP\s*:\s*INFO\s*:\s*TWR\[(\d+)\]\.nLos\s*:\s*(\d+)'
                    distance_pattern = r'APP\s*:\s*INFO\s*:\s*TWR\[(\d+)\]\.distance\s*:\s*([\d.]+)'
                    azimuth_pattern = r'APP\s*:\s*INFO\s*:\s*TWR\[(\d+)\]\.aoa_azimuth:\s*([-+]?[\d.]+)'
                    elevation_pattern = r'APP\s*:\s*INFO\s*:\s*TWR\[(\d+)\]\.aoa_elevation\s*:\s*([-+]?[\d.]+)'

                    # nLos抽出
                    nlos_match = re.search(nlos_pattern, line)
                    if nlos_match and int(nlos_match.group(1)) == 0: # TWR[0]のみを対象
                        current_twr0_frame["nlos_los"] = "LOS" if int(nlos_match.group(2)) == 0 else "nLOS"
                        # print(f"  nLos: {current_twr0_frame['nlos_los']}") # デバッグ用

                    # distance抽出 (cmをmに変換)
                    distance_match = re.search(distance_pattern, line)
                    if distance_match and int(distance_match.group(1)) == 0:
                        current_twr0_frame["distance"] = round(float(distance_match.group(2)) / 100, 2)
                        # print(f"  Distance: {current_twr0_frame['distance']} m") # デバッグ用

                    # aoa_azimuth抽出
                    azimuth_match = re.search(azimuth_pattern, line)
                    if azimuth_match and int(azimuth_match.group(1)) == 0:
                        current_twr0_frame["horizontal_angle"] = round(float(azimuth_match.group(2)), 2)
                        # print(f"  Azimuth: {current_twr0_frame['horizontal_angle']} deg") # デバッグ用

                    # aoa_elevation抽出
                    elevation_match = re.search(elevation_pattern, line)
                    if elevation_match and int(elevation_match.group(1)) == 0:
                        current_twr0_frame["elevation_angle"] = round(float(elevation_match.group(2)), 2)
                        # print(f"  Elevation: {current_twr0_frame['elevation_angle']} deg") # デバッグ用

                    # 必要な全てのデータが揃ったかチェック
                    if all(key in current_twr0_frame for key in required_keys):
                        return {"TWR0": current_twr0_frame} # TWR0のデータとして返す
                
                except UnicodeDecodeError:
                    # 無効なバイト列をスキップ
                    continue
                except Exception as e:
                    print(f"データパース中にエラーが発生しました: {e} - 行: {line}")
                    continue
            time.sleep(0.01) # シリアルポートポーリング間の短い待機

        # タイムアウトした場合
        print(f"警告: タイムアウトにより完全なTWR[0]フレームを取得できませんでした。取得済みのデータ: {current_twr0_frame}")
        return None


    def collect_data_frames(self, num_frames: int) -> Tuple[List[Dict[str, Any]], int]:
        """
        指定されたフレーム数 (各TWR[0]の全情報が揃った論理フレーム) のデータを
        シリアルポートから収集し、その収集にかかった時間を計測する。
        """
        if not self.ser or not self.ser.is_open:
            print("シリアルポートが接続されていません。データを収集できません。")
            return [], 0

        print(f"{num_frames}個の完全な論理フレームのデータ収集を開始します...")
        start_time = time.perf_counter_ns()
        data_frames = []

        for i in range(num_frames):
            print(f"  フレーム {i+1}/{num_frames} を取得中...")
            frame_data = self._read_uwb_data_from_serial()
            if frame_data:
                data_frames.append(frame_data)
                # print(f"  フレーム {i+1} 収集済み.")
            else:
                print(f"  フレーム {i+1} のデータ取得に失敗またはタイムアウトしました。スキップします。")
                # 不完全なフレームをスキップするか、後で処理するかは要件による
                # ここではスキップし、目標フレーム数に満たない可能性がある
                continue # 次のフレーム取得へ

        end_time = time.perf_counter_ns()
        elapsed_time_ms = (end_time - start_time) // 1_000_000

        print(f"データ収集完了。{len(data_frames)}個の論理フレームを収集しました。所要時間: {elapsed_time_ms} ms")
        return data_frames, elapsed_time_ms

    def calculate_modes(self, data_frames: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        収集したデータフレーム (TWR0データを含む辞書のリスト) から各パラメータの最頻値を計算する。
        """
        if not data_frames:
            return {
                "mode_distance_m": None,
                "mode_nlos_los": None,
                "mode_horizontal_angle_deg": None,
                "mode_elevation_angle_deg": None
            }

        # TWR0のデータのみを抽出して最頻値を計算
        all_distances = []
        all_nlos_los_statuses = []
        all_horizontal_angles = []
        all_elevation_angles = []

        for frame in data_frames:
            # 各フレームは {"TWR0": {...}} の形式を期待
            if "TWR0" in frame:
                twr0_data = frame["TWR0"]
                if 'distance' in twr0_data:
                    all_distances.append(twr0_data['distance'])
                if 'nlos_los' in twr0_data:
                    all_nlos_los_statuses.append(twr0_data['nlos_los'])
                if 'horizontal_angle' in twr0_data:
                    all_horizontal_angles.append(twr0_data['horizontal_angle'])
                if 'elevation_angle' in twr0_data:
                    all_elevation_angles.append(twr0_data['elevation_angle'])

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
            "mode_distance_m": get_mode(all_distances),
            "mode_nlos_los": get_mode(all_nlos_los_statuses),
            "mode_horizontal_angle_deg": get_mode(all_horizontal_angles),
            "mode_elevation_angle_deg": get_mode(all_elevation_angles)
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

        # 20個の完全な論理フレームのデータを収集
        # collect_data_framesはシミュレーションではなく実データ収集を行う
        data_frames, measurement_time_ms = self.collect_data_frames(num_frames=20)

        # 収集したデータから最頻値を計算
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
    # Windowsの場合のCOMポート例: 'COM3'
    # Linuxの場合のCOMポート例: '/dev/ttyUSB0'
    runner = ExperimentRunner(com_port='COM3') # <-- ここをお使いの環境に合わせて修正

    if runner.ser and runner.ser.is_open:
        # 簡易的な実験実行のシミュレーション
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
        runner.disconnect_serial()
    else:
        print("シリアルポートに接続できなかったため、実験を実行できませんでした。")