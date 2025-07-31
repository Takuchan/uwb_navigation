import pandas as pd 
import io

def load_csv_to_dataframe(file_path=None, csv_content=None):
    """
    CSVデータをpandasのDataFrameに変換する関数

    Args:
        file_path (str): CSVファイルのパス
        csv_content (str): CSV形式の文字列データ

    Returns:
        pd.DataFrame: pandasのデータフレーム
    """
    if file_path:
        return pd.read_csv(file_path)
    elif csv_content:
        return pd.read_csv(io.StringIO(csv_content))
    else:
        raise ValueError("file_pathまたはcsv_contentのいずれかを指定してください。")

def analyze_distance(dataframe,expected_los,expected_distance, actual_los,remarks):
    """
    引数の要件Args:
        dataFrame(pd.DataFrame): 分析対象のデータフレーム
        expedted_los(str): nLOS/LOS期待
        expected_distance (int or float): 期待距離
        actual_los(str): 実際のnLOS/LOS
    
    戻り値
        float: 計算された実際の距離の平均値  
    """

    filterd_df = dataframe[
        (dataframe['nlos_los_expected'] == expected_los) &
        (dataframe['distance_m'] == expected_distance) &
        (dataframe['mode_nlos_los'] == actual_los) &
        (dataframe['remarks'] == remarks)
    ]

    if filterd_df.empty:
        return None
    else:
        average_distance = filterd_df['mode_distance_m'].mean()
        return average_distance
    


nlos_los_expect = 'LOS'
distance_expect = 20
nlos_los_actual = 'LOS'
remarks = "Z位置関係: Z揃え (実験①)"
csv_file_name = "experiment_results.csv"  # 必要に応じてファイル名を変更

# CSVデータをデータフレームに変換
dataframe = load_csv_to_dataframe(file_path=csv_file_name)

average_actual_distance = analyze_distance(dataframe, nlos_los_expect, distance_expect, nlos_los_actual,remarks=remarks)

if average_actual_distance is not None:
    print(f"nLOS/LOS期待: {nlos_los_expect}")
    print(f"期待距離: {distance_expect} m")
    print(f"条件: {remarks} ")
    print(f"実際のnLOS/LOS: {nlos_los_actual}")
    print("-" * 30)
    print(f"👉 実際の距離の平均値: {average_actual_distance:.2f} m")
else:
    print("指定された条件に一致するデータはありませんでした。")