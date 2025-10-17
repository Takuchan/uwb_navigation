import csv
import json
import os
from datetime import datetime
from typing import Dict, Any, Optional, Tuple


class CSVLogger:
    """
    UWBデータとトライラテレーション結果をCSVに記録するクラス
    可変数のアンカーに対応
    """

    def __init__(self, save_directory: str = "./uwb_logs"):
        """
        Args:
            save_directory: CSVファイルの保存先ディレクトリ
        """
        self.save_directory = save_directory
        self.csv_file = None
        self.csv_writer = None
        self.is_logging = False
        self.current_filepath = None
        self.anchor_positions = None
        self.num_anchors = 0
        
        # ディレクトリが存在しない場合は作成
        if not os.path.exists(save_directory):
            os.makedirs(save_directory)

    def start_logging(self, 
                     anchor_positions: Dict[str, Tuple[float, float, float]],
                     filename: Optional[str] = None) -> str:
        """
        ログ記録を開始
        
        Args:
            anchor_positions: アンカー位置情報
            filename: ファイル名（Noneの場合は自動生成）
        
        Returns:
            作成されたファイルのパス
        """
        if self.is_logging:
            print("既にログ記録中です")
            return self.current_filepath
        
        self.anchor_positions = anchor_positions
        self.num_anchors = len(anchor_positions)
        
        # ファイル名が指定されていない場合は自動生成
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"uwb_log_{timestamp}.csv"
        
        self.current_filepath = os.path.join(self.save_directory, filename)
        
        # CSVファイルを開く
        self.csv_file = open(self.current_filepath, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        
        # メタデータ行（アンカー位置情報）を先頭に記録
        self.csv_writer.writerow(['# Anchor Positions (JSON format)'])
        anchor_json = json.dumps(anchor_positions)
        self.csv_writer.writerow([anchor_json])
        self.csv_writer.writerow([])  # 空行
        
        # ヘッダー行を動的に生成
        header = ['timestamp']
        anchor_keys = sorted(anchor_positions.keys())
        for key in anchor_keys:
            header.extend([
                f'{key}_distance',
                f'{key}_nlos',
                f'{key}_horizontal_angle',
                f'{key}_elevation_angle'
            ])
        header.extend(['estimated_x', 'estimated_y', 'estimated_z'])
        
        self.csv_writer.writerow(header)
        
        self.is_logging = True
        print(f"ログ記録を開始しました: {self.current_filepath}")
        print(f"アンカー数: {self.num_anchors}")
        return self.current_filepath

    def log_data(self, 
                 timestamp: float,
                 anchor_data: Dict[str, Optional[Dict[str, Any]]],
                 position: Optional[Tuple[float, float, float]]):
        """
        データを1行記録
        
        Args:
            timestamp: タイムスタンプ（秒）
            anchor_data: アンカーからのデータ
            position: 推定位置 (x, y, z) またはNone
        """
        if not self.is_logging or self.csv_writer is None:
            return
        
        row = [timestamp]
        
        # 各アンカーのデータを追加（動的に対応）
        anchor_keys = sorted([k for k in self.anchor_positions.keys()])
        for anchor_key in anchor_keys:
            if anchor_key in anchor_data and anchor_data[anchor_key] is not None:
                data = anchor_data[anchor_key]
                row.extend([
                    data.get('distance', 'None'),
                    data.get('nlos_los', 'None'),
                    data.get('horizontal_angle', 'None'),
                    data.get('elevation_angle', 'None')
                ])
            else:
                # データがない場合はNoneを4つ追加
                row.extend(['None', 'None', 'None', 'None'])
        
        # 推定位置を追加
        if position is not None:
            row.extend([position[0], position[1], position[2]])
        else:
            row.extend(['None', 'None', 'None'])
        
        # CSVに書き込み
        self.csv_writer.writerow(row)

    def stop_logging(self):
        """ログ記録を停止"""
        if not self.is_logging:
            return
        
        if self.csv_file:
            self.csv_file.close()
            print(f"ログ記録を停止しました: {self.current_filepath}")
        
        self.csv_file = None
        self.csv_writer = None
        self.is_logging = False

    def is_logging_active(self) -> bool:
        """ログ記録中かどうかを返す"""
        return self.is_logging