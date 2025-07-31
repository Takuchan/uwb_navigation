import time
import pprint
from serialandFilter import SerialFilter
from triliation import TrilaterationSolver

def main():
    """
    UWBデータを受信し、三辺測位で位置を計算して表示するメインプログラム。
    """
    # --- 事前設定 ---
    # ここでアンカー間の物理的な距離を設定します (m単位)
    DISTANCE_0_TO_1 = 11.40  # m
    DISTANCE_1_TO_2 = 5.52  # m
    DISTANCE_0_TO_2 = 13.49  # m

    # シリアルポートとアンカーの数を設定
    # 実際の環境に合わせて変更してください (Windowsなら "COM3" など)
    SERIAL_PORT = "COM7"
    NUM_ANCHORS = 3
    
    # --- 初期化 ---
    try:
        # 三辺測位ソルバーを、設定したアンカー間距離で初期化
        trilateration_solver = TrilaterationSolver(
            d01=DISTANCE_0_TO_1,
            d12=DISTANCE_1_TO_2,
            d02=DISTANCE_0_TO_2
        )
        
        # シリアル通信フィルターを初期化
        uwb_filter = SerialFilter(com_port=SERIAL_PORT, num_anchors=NUM_ANCHORS)
        if not uwb_filter.ser or not uwb_filter.ser.is_open:
            print("プログラムを終了します。")
            return

    except Exception as e:
        print(f"初期化中にエラーが発生しました: {e}")
        return

    # --- メインループ ---
    try:
        print("\n--- 位置計算を開始します (Ctrl+Cで終了) ---")
        while True:
            anchor_data = uwb_filter.read_anchor_data_snapshot(timeout=0.2)
            
            if not anchor_data or not all(anchor_data.get(f"TWR{i}") for i in range(NUM_ANCHORS)):
                print(f"[{time.time():.2f}] 全てのアンカーからデータを取得できませんでした。スキップします。")
                pprint.pprint(anchor_data)
                time.sleep(0.1) # データが取れない場合は少し待つ
                continue

            # 各アンカーからの距離をcm単位で取得
            # SerialFilterはメートル単位で返すので100を掛ける
            r0 = anchor_data["TWR0"]["distance"] 
            r1 = anchor_data["TWR1"]["distance"] 
            r2 = anchor_data["TWR2"]["distance"]

            
            
            # 取得した距離データを使って位置を計算
            calculated_position = trilateration_solver.calculate_position(r0, r1, r2)
            
            # 結果を表示
            pos_x, pos_y = calculated_position
            print(f"計算位置 (cm):  X = {pos_x:8.2f},  Y = {pos_y:8.2f}")
            
    except KeyboardInterrupt:
        print("\nキーボード割り込みを検出しました。プログラムを終了します。")
    finally:
        # 確実にシリアルポートを閉じる
        if 'uwb_filter' in locals():
            uwb_filter.disconnect_serial()

if __name__ == "__main__":
    main()