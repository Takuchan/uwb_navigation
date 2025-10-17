import csv
import json
import pandas as pd
from typing import Dict, Any, Optional, List, Tuple


class CSVPlayer:
    """
    記録されたCSVファイルを読み込み、再生する機能を提供するクラス
    アンカー位置情報も読み込む
    """

    def __init__(self):
        self.data = None
        self.current_index = 0
        self.filepath = None
        self.is_loaded = False
        self.anchor_positions = None

    def load_csv(self, filepath: str) -> bool:
        """
        CSVファイルを読み込む
        
        Args:
            filepath: CSVファイルのパス
        
        Returns:
            読み込み成功ならTrue
        """
        try:
            # まずファイルを開いてアンカー位置情報を読み取る
            with open(filepath, 'r', encoding='utf-8') as f:
                lines = f.readlines()
                
                # 最初の数行からアンカー位置情報を抽出
                anchor_line = None
                data_start_line = 0
                
                for i, line in enumerate(lines):
                    if line.startswith('# Anchor Positions'):
                        # 次の行がアンカー位置のJSON
                        if i + 1 < len(lines):
                            anchor_line = lines[i + 1].strip()
                        data_start_line = i + 3  # メタデータ、JSON、空行の次
                        break
                
                # アンカー位置情報をパース
                if anchor_line:
                    try:
                        self.anchor_positions = json.loads(anchor_line)
                        # タプルに変換
                        for key in self.anchor_positions:
                            if isinstance(self.anchor_positions[key], list):
                                self.anchor_positions[key] = tuple(self.anchor_positions[key])
                        print(f"アンカー位置を読み込みました: {self.anchor_positions}")
                    except json.JSONDecodeError:
                        print("警告: アンカー位置情報の読み込みに失敗しました")
                        self.anchor_positions = None
            
            # pandasでCSVを読み込み（メタデータ行をスキップ）
            self.data = pd.read_csv(filepath, 
                                   na_values=['None'], 
                                   keep_default_na=True,
                                   skiprows=data_start_line)
            
            self.filepath = filepath
            self.current_index = 0
            self.is_loaded = True
            print(f"CSVファイルを読み込みました: {filepath}")
            print(f"総データ数: {len(self.data)}行")
            return True
            
        except Exception as e:
            print(f"CSVファイルの読み込みに失敗: {e}")
            self.is_loaded = False
            return False

    def get_anchor_positions(self) -> Optional[Dict[str, Tuple[float, float, float]]]:
        """アンカー位置情報を取得"""
        return self.anchor_positions

    def get_total_frames(self) -> int:
        """総フレーム数を取得"""
        if not self.is_loaded or self.data is None:
            return 0
        return len(self.data)

    def get_time_range(self) -> Tuple[float, float]:
        """時間範囲を取得 (開始時刻, 終了時刻)"""
        if not self.is_loaded or self.data is None:
            return (0.0, 0.0)
        
        start_time = self.data['timestamp'].iloc[0]
        end_time = self.data['timestamp'].iloc[-1]
        return (start_time, end_time)

    def get_data_at_index(self, index: int) -> Optional[Dict[str, Any]]:
        """
        指定インデックスのデータを取得
        
        Args:
            index: データのインデックス
        
        Returns:
            データ辞書またはNone
        """
        if not self.is_loaded or self.data is None:
            return None
        
        if index < 0 or index >= len(self.data):
            return None
        
        row = self.data.iloc[index]
        
        # データを辞書形式に変換
        result = {
            'timestamp': row['timestamp'],
            'anchor_data': {},
            'position': None
        }
        
        # 各アンカーのデータを抽出
        for i in range(3):
            anchor_key = f"TWR{i}"
            distance = row[f'{anchor_key}_distance']
            
            # データが存在する場合（NaNでない場合）
            if pd.notna(distance):
                result['anchor_data'][anchor_key] = {
                    'distance': float(distance),
                    'nlos_los': str(row[f'{anchor_key}_nlos']),
                    'horizontal_angle': float(row[f'{anchor_key}_horizontal_angle']) if pd.notna(row[f'{anchor_key}_horizontal_angle']) else None,
                    'elevation_angle': float(row[f'{anchor_key}_elevation_angle']) if pd.notna(row[f'{anchor_key}_elevation_angle']) else None
                }
            else:
                result['anchor_data'][anchor_key] = None
        
        # 推定位置を抽出
        if pd.notna(row['estimated_x']):
            result['position'] = (
                float(row['estimated_x']),
                float(row['estimated_y']),
                float(row['estimated_z'])
            )
        
        return result

    def get_data_at_time(self, timestamp: float) -> Optional[Dict[str, Any]]:
        """
        指定時刻に最も近いデータを取得
        
        Args:
            timestamp: タイムスタンプ
        
        Returns:
            データ辞書またはNone
        """
        if not self.is_loaded or self.data is None:
            return None
        
        # 最も近い時刻のインデックスを検索
        idx = (self.data['timestamp'] - timestamp).abs().idxmin()
        return self.get_data_at_index(idx)

    def get_next_data(self) -> Optional[Dict[str, Any]]:
        """次のデータを取得（順次再生用）"""
        if not self.is_loaded:
            return None
        
        data = self.get_data_at_index(self.current_index)
        self.current_index += 1
        
        # 最後まで到達したらループ
        if self.current_index >= self.get_total_frames():
            self.current_index = 0
        
        return data

    def reset(self):
        """再生位置をリセット"""
        self.current_index = 0

    def seek(self, index: int):
        """指定インデックスにシーク"""
        if 0 <= index < self.get_total_frames():
            self.current_index = index

    def get_all_positions(self) -> List[Tuple[float, float, float]]:
        """
        全ての有効な推定位置を取得（軌跡表示用）
        
        Returns:
            位置のリスト [(x, y, z), ...]
        """
        if not self.is_loaded or self.data is None:
            return []
        
        positions = []
        for _, row in self.data.iterrows():
            if pd.notna(row['estimated_x']):
                positions.append((
                    float(row['estimated_x']),
                    float(row['estimated_y']),
                    float(row['estimated_z'])
                ))
        
        return positions


# テスト用コード
if __name__ == "__main__":
    player = CSVPlayer()
    
    # CSVファイルを読み込み
    if player.load_csv("./test_logs/uwb_log_test.csv"):
        print(f"総フレーム数: {player.get_total_frames()}")
        print(f"アンカー位置: {player.get_anchor_positions()}")
        
        time_range = player.get_time_range()
        print(f"時間範囲: {time_range[0]:.2f} - {time_range[1]:.2f} 秒")
        
        # 最初の5フレームを表示
        print("\n最初の5フレーム:")
        for i in range(min(5, player.get_total_frames())):
            data = player.get_data_at_index(i)
            if data:
                print(f"Frame {i}: Position = {data['position']}")
        
        # 全位置を取得
        positions = player.get_all_positions()
        print(f"\n有効な位置データ数: {len(positions)}")