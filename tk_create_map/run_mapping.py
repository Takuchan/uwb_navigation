from uwb_mapper import UwbMapper
from triliation import TrilaterationSolver
from serialandFilter import SerialFilter

def main():
    """
    UWBマッピングアプリケーションを初期化して実行する。
    """
    # --- 事前設定 ---
    # アンカー間の物理的な距離を設定します (メートル単位)
    DISTANCE_0_TO_1 = 6.9  # m
    DISTANCE_1_TO_2 = 5.7   # m
    DISTANCE_0_TO_2 = 5.9  # m

    # シリアルポートとアンカーの数を設定
    SERIAL_PORT = "/dev/ttyUSB0"  # ご自身の環境に合わせて変更してください
    NUM_ANCHORS = 3
    
    # --- 初期化と実行 ---
    try:
        # 1. 三辺測位ソルバーを初期化
        trilateration_solver = TrilaterationSolver(
            d01=DISTANCE_0_TO_1,
            d12=DISTANCE_1_TO_2,
            d02=DISTANCE_0_TO_2
        )
        
        # 2. シリアル通信フィルターを初期化
        uwb_filter = SerialFilter(com_port=SERIAL_PORT, num_anchors=NUM_ANCHORS)
        if not uwb_filter.ser or not uwb_filter.ser.is_open:
            print("シリアルポートの接続に失敗しました。プログラムを終了します。")
            return

        # 3. UWBマッパーを初期化して実行
        mapper = UwbMapper(solver=trilateration_solver, serial_filter=uwb_filter)
        mapper.run()

    except Exception as e:
        print(f"プログラムの実行中にエラーが発生しました: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
