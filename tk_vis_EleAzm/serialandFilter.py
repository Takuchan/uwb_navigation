import time
from collections import Counter
from typing import List, Dict , Any, Tuple, Optional
import serial 
import re 

class SerialFilter:
    """
    Type2BPを用いた実験のロジックを管理するクラス。
    データ収集を行う
    """

    def __init__(self, com_port="/dev/ttyUSB0", baud_rate=3000000):
        self.com_port = com_port
        self.baud_rate = baud_rate
        self.ser = None

        self.connect_serial()
    
    def connect_serial(self) -> bool:
        """
        シリアル通信に接続を試行する
        """
        if self.ser and self.ser.is_open:
            print(f"すでにポート接続しています: {self.com_port}")
            return True
        try:
            self.ser = serial.Serial(self.com_port,self.baud_rate,timeout=1)
            print(f"✅接続できた。{self.com_port} {self.baud_rate}")
            print("一旦、安定化接続中。２秒待て")
            time.sleep(2)
            return True
        except serial.SerialException as e:
            print(f"シリアルポートの接続に失敗しました。: {e}")
            return False
        except Exception as e:
            print("予期せぬエラー")
            return False
    
    def disconnect_serial(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"🥳{self.com_port}から接続解除しました。")
    
    def _read_uwb_data_from_serial(self) -> Optional[Dict[str,Any]]:
        """
        シリアル通信で１つの完全なTWR[0]のデータフレームを読み取り、パースする。
        nLOS,distance,aoa_azimuth,aoa_elavationの４つの情報が揃うまで待つ
        """

        if not self.ser or not self.ser.is_open:
            return None
        
        current_twr0_frame = {}
        required_keys = {"nlos_los", "distance", "horizontal_angle", "elevation_angle"}

        timeout_start = time.time()
        frame_timeout = 2.0

        while(time.time() - timeout_start) < frame_timeout:
            if self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()

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
                    continue
                except Exception as e:
                    print(f"データパース中にエラーが発生しましたあ。{e} - {line}")
                    continue
            time.sleep(0.01)
        print(f"警告：タイムアウトにより、完全なTWRを取得できませんでした。")
        return None

