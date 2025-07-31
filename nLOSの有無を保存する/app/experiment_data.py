# experiment_data.py
import pandas as pd
from dataclasses import dataclass, asdict, field
from typing import List, Dict, Any, Optional
import os
import json

@dataclass
class ExperimentResult:
    """
    単一の実験試行の結果を保持するデータクラス。
    """
    experiment_no: int
    distance_m: int
    uwb_orientation_condition: str # UWB向き条件 (例: '垂直', '30°傾斜', '下向き固定')
    nlos_los_expected: str # 新しく追加: 期待されるnLOS/LOS条件
    trial_count: int # 試行回数 (LOS/nLOSそれぞれで1, 2, 3)
    measurement_time_ms: Optional[int] = None # 計測時間 (ミリ秒)
    mode_distance_m: Optional[float] = None # 最頻値 - 距離 (m)
    mode_nlos_los: Optional[str] = None # 最頻値 - nLOS/LOS (LOS/nLOS)
    mode_horizontal_angle_deg: Optional[float] = None # 最頻値 - 水平角 (°)
    mode_elevation_angle_deg: Optional[float] = None # 最頻値 - 仰角 (°)
    remarks: Optional[str] = None # 備考
    raw_frames_data: List[Dict[str, Any]] = field(default_factory=list) # 各フレームの生データをリストで格納

@dataclass
class ExperimentDataSet:
    """
    複数の実験結果を管理し、ファイルとして保存・読み込みを行うクラス。
    """
    results: List[ExperimentResult] = field(default_factory=list)
    file_path: str = "experiment_results.csv"

    def add_result(self, result: ExperimentResult):
        """実験結果を追加する。"""
        self.results.append(result)

    def save_to_csv(self):
        """現在の実験結果をCSVファイルに保存する。"""
        if not self.results:
            print("保存するデータがありません。")
            return

        df_data = []
        for r in self.results:
            row_dict = asdict(r)
            row_dict['raw_frames_data'] = json.dumps(row_dict['raw_frames_data'], ensure_ascii=False)
            df_data.append(row_dict)

        df = pd.DataFrame(df_data)
        df.to_csv(self.file_path, index=False, encoding='utf-8-sig')
        print(f"実験結果を {self.file_path} に保存しました。")

    def load_from_csv(self):
        """CSVファイルから実験結果を読み込む。"""
        if os.path.exists(self.file_path):
            df = pd.read_csv(self.file_path, encoding='utf-8-sig')
            self.results = []
            for _, row in df.iterrows():
                try:
                    row_dict = row.where(pd.notna(row), None).to_dict()
                    if 'raw_frames_data' in row_dict and row_dict['raw_frames_data'] is not None:
                        row_dict['raw_frames_data'] = json.loads(row_dict['raw_frames_data'])
                    else:
                        row_dict['raw_frames_data'] = []
                    
                    # 新しいフィールド 'nlos_los_expected' に対応
                    # CSVにこのカラムがない場合（旧バージョンのデータ）はデフォルト値を設定
                    if 'nlos_los_expected' not in row_dict:
                        row_dict['nlos_los_expected'] = "UNKNOWN" # または適切なデフォルト値

                    self.results.append(ExperimentResult(**row_dict))
                except (TypeError, json.JSONDecodeError) as e:
                    print(f"CSVデータの読み込みエラー: {e} - 行データ: {row_dict}")
            print(f"実験結果を {self.file_path} から読み込みました。")
        else:
            print(f"{self.file_path} が見つかりませんでした。新しいデータセットを作成します。")

    def get_all_results(self) -> List[ExperimentResult]:
        return self.results

    def clear_results(self):
        self.results = []
        print("実験結果をクリアしました。")

# 使用例 (このファイル自体を実行した場合)
if __name__ == "__main__":
    import json
    data_set = ExperimentDataSet()
    data_set.load_from_csv()

    new_result = ExperimentResult(
        experiment_no=1,
        distance_m=5,
        uwb_orientation_condition="垂直",
        nlos_los_expected="LOS", # ここに期待条件を追加
        trial_count=1,
        measurement_time_ms=100,
        mode_distance_m=5.01,
        mode_nlos_los="LOS",
        mode_horizontal_angle_deg=0.5,
        mode_elevation_angle_deg=-0.2,
        remarks="最初の試行",
        raw_frames_data=[
            {"distance": 5.02, "nlos_los": "LOS", "horizontal_angle": 0.1, "elevation_angle": -0.1},
            {"distance": 5.00, "nlos_los": "LOS", "horizontal_angle": 0.3, "elevation_angle": -0.3},
        ]
    )
    data_set.add_result(new_result)
    data_set.save_to_csv()

    new_data_set = ExperimentDataSet()
    new_data_set.load_from_csv()
    print("\nロードされたデータ:")
    for res in new_data_set.get_all_results():
        print(res)