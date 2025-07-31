import time
from collections import Counter
from typing import List, Dict, Any, Tuple, Optional
import serial
import re
import pprint

class SerialFilter:
    """
    Type2BPを用いた実験のロジックを管理するクラス。
    データ収集を行う。
    指定された数のAnchorの情報を取得する。
    """

    def __init__(self, com_port="/dev/ttyUSB0", baud_rate=3000000, num_anchors=3):
        """
        Initializes the SerialFilter.
        Args:
            com_port (str): The serial port to connect to (e.g., "/dev/ttyUSB0" or "COM3").
            baud_rate (int): The baud rate for the serial connection.
            num_anchors (int): The number of TWR anchors to expect data from (e.g., 3 for TWR[0], TWR[1], TWR[2]).
        """
        self.com_port = com_port
        self.baud_rate = baud_rate
        self.num_anchors = num_anchors # Number of anchors to expect
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
            self.ser = serial.Serial(self.com_port, self.baud_rate, timeout=0.1) # Use a short timeout for readline
            print(f"✅接続できた。{self.com_port} {self.baud_rate}")
            print("一旦、安定化接続中。２秒待て")
            time.sleep(2)
            self.ser.flushInput() # Clear any old data in the buffer
            return True
        except serial.SerialException as e:
            print(f"シリアルポートの接続に失敗しました。: {e}")
            return False
        except Exception as e:
            print(f"予期せぬエラー: {e}")
            return False

    def disconnect_serial(self):
        """
        Closes the serial connection.
        """
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"🥳{self.com_port}から接続解除しました。")

    def read_anchor_data_snapshot(self, timeout: float = 1.0) -> Optional[Dict[str, Any]]:
        if not self.ser or not self.ser.is_open:
            print("エラー: シリアルポートが開いていません。")
            return None

        # This dictionary will store data as it comes in.
        # Format: {0: {data_for_twr0}, 1: {data_for_twr1}, ...}
        collected_data = {}
        required_keys = {"nlos_los", "distance", "horizontal_angle", "elevation_angle"}
        
        # Regex patterns to capture TWR index and value
        nlos_pattern = re.compile(r'TWR\[(\d+)\]\.nLos\s*:\s*(\d+)')
        distance_pattern = re.compile(r'TWR\[(\d+)\]\.distance\s*:\s*([\d.]+)')
        azimuth_pattern = re.compile(r'TWR\[(\d+)\]\.aoa_azimuth:\s*([-+]?[\d.]+)')
        elevation_pattern = re.compile(r'TWR\[(\d+)\]\.aoa_elevation\s*:\s*([-+]?[\d.]+)')

        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue

                    # Try to match each pattern
                    nlos_match = nlos_pattern.search(line)
                    if nlos_match:
                        twr_id = int(nlos_match.group(1))
                        # Use setdefault to create dict for a new twr_id if it doesn't exist
                        collected_data.setdefault(twr_id, {})["nlos_los"] = "LOS" if int(nlos_match.group(2)) == 0 else "nLOS"
                        continue

                    distance_match = distance_pattern.search(line)
                    if distance_match:
                        twr_id = int(distance_match.group(1))
                        dist_cm = float(distance_match.group(2))
                        collected_data.setdefault(twr_id, {})["distance"] = round(dist_cm / 100, 2)
                        continue
                    
                    azimuth_match = azimuth_pattern.search(line)
                    if azimuth_match:
                        twr_id = int(azimuth_match.group(1))
                        azimuth = float(azimuth_match.group(2))
                        collected_data.setdefault(twr_id, {})["horizontal_angle"] = round(azimuth, 2)
                        continue

                    elevation_match = elevation_pattern.search(line)
                    if elevation_match:
                        twr_id = int(elevation_match.group(1))
                        elevation = float(elevation_match.group(2))
                        collected_data.setdefault(twr_id, {})["elevation_angle"] = round(elevation, 2)
                        continue

            except Exception as e:
                print(f"データパース中にエラーが発生しました: {e}")
                continue
        
        # After timeout, build the final result dictionary
        final_result = {}
        for i in range(self.num_anchors):
            anchor_id_str = f"TWR{i}"
            if i in collected_data and all(key in collected_data[i] for key in required_keys):
                final_result[anchor_id_str] = collected_data[i]
            else:
                final_result[anchor_id_str] = None # Mark as None if incomplete or missing
        
        if not any(final_result.values()):
             print("警告：どのアンカーからも完全なデータを取得できませんでした。")

        return final_result
    
if __name__ == "__main__":
    uwb_filter = SerialFilter(com_port="/dev/ttyUSB0",num_anchors=3)

    if not uwb_filter.ser or not uwb_filter.ser.is_open:
        print("プログラムを終了します") 
    
    try:
        print("😀データ収集を開始する")
        while True:
            anchor_data = uwb_filter.read_anchor_data_snapshot(timeout=0.15)

            print(f"\n--- {time.ctime()} ---")
            if anchor_data:
                pprint.pprint(anchor_data)
            else:
                print("有効なデータがない")
            
    except KeyboardInterrupt:
        print("キーボード割り込みを検出しました。プログラムを終了します")
    finally:
        uwb_filter.disconnect_serial()