import tkinter as tk
from tkinter import messagebox, ttk
import json
import os
import time

from experiment_runner import ExperimentRunner
from experiment_data import ExperimentDataSet, ExperimentResult

class ExperimentApp:
    def __init__(self, root):
        self.root = root
        self.root.title("データ収集実験アプリケーション")
        self.root.geometry("900x650") # ウィンドウサイズを少し広げます

        self.experiment_data_set = ExperimentDataSet()
        self.experiment_data_set.load_from_csv()

        self.experiment_runner = ExperimentRunner(com_port='COM7', baud_rate=3000000) # 環境に合わせてCOMポートを変更してください

        self._setup_experiment_count_frame()
        self._setup_experiment_condition_frame()
        self._setup_delay_setting_frame()

        button_frame = tk.Frame(self.root)
        button_frame.pack(pady=10)

        tk.Button(button_frame, text="実験開始", command=self._start_experiment).pack(side=tk.LEFT, padx=5)
        tk.Button(button_frame, text="全実験結果をCSV保存", command=self.experiment_data_set.save_to_csv).pack(side=tk.LEFT, padx=5)
        tk.Button(button_frame, text="全実験結果をクリア", command=self._clear_all_results).pack(side=tk.LEFT, padx=5)

        self._setup_results_display_frame()

        delete_button_frame = tk.Frame(self.root)
        delete_button_frame.pack(pady=5)
        tk.Button(delete_button_frame, text="選択した結果を削除", command=self._delete_selected_results).pack()

        self._update_results_display()

        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)

    def _on_closing(self):
        if self.experiment_runner:
            self.experiment_runner.disconnect_serial()
        self.root.destroy()

    def _setup_experiment_count_frame(self):
        frame = tk.LabelFrame(self.root, text="実験回数設定", padx=10, pady=10)
        frame.pack(pady=10, padx=10, fill="x")

        self.experiment_count = tk.IntVar(value=3)

        tk.Radiobutton(frame, text="1回", variable=self.experiment_count, value=1).pack(side=tk.LEFT, padx=5)
        tk.Radiobutton(frame, text="3回", variable=self.experiment_count, value=3).pack(side=tk.LEFT, padx=5)
        tk.Radiobutton(frame, text="5回", variable=self.experiment_count, value=5).pack(side=tk.LEFT, padx=5)
        tk.Radiobutton(frame, text="10回", variable=self.experiment_count, value=10).pack(side=tk.LEFT, padx=5)
        tk.Label(frame, text="回数分、連続でデータを取得します。").pack(side=tk.LEFT, padx=10)

    def _setup_experiment_condition_frame(self):
        frame = tk.LabelFrame(self.root, text="実験条件設定", padx=10, pady=10)
        frame.pack(pady=10, padx=10, fill="x")

        tk.Label(frame, text="距離 (m):").grid(row=0, column=0, padx=5, pady=2, sticky="w")
        self.distance_var = tk.StringVar(value="5")
        ttk.Combobox(frame, textvariable=self.distance_var,
                     values=["5", "10", "20", "30", "40", "50"], state="readonly").grid(row=0, column=1, padx=5, pady=2, sticky="ew")

        tk.Label(frame, text="UWB向き条件:").grid(row=1, column=0, padx=5, pady=2, sticky="w")
        self.uwb_orientation_var = tk.StringVar(value="垂直")
        ttk.Combobox(frame, textvariable=self.uwb_orientation_var,
                     values=["垂直", "30°傾斜", "下向き固定"], state="readonly").grid(row=1, column=1, padx=5, pady=2, sticky="ew")

        tk.Label(frame, text="障害物有無 (期待LOS/nLOS):").grid(row=2, column=0, padx=5, pady=2, sticky="w")
        self.nlos_los_var = tk.StringVar(value="LOS")
        ttk.Combobox(frame, textvariable=self.nlos_los_var,
                     values=["LOS", "nLOS"], state="readonly").grid(row=2, column=1, padx=5, pady=2, sticky="ew")

        tk.Label(frame, text="Z位置関係 (実験タイプ):").grid(row=3, column=0, padx=5, pady=2, sticky="w")
        self.z_position_var = tk.StringVar(value="Z揃え (実験①)")
        ttk.Combobox(frame, textvariable=self.z_position_var,
                     values=["Z揃え (実験①)", "Anchor=Tag+1000mm (実験②)", "Anchor電波上から (実験③)"], state="readonly").grid(row=3, column=1, padx=5, pady=2, sticky="ew")

        frame.grid_columnconfigure(1, weight=1)

    def _setup_delay_setting_frame(self):
        frame = tk.LabelFrame(self.root, text="実験開始遅延設定", padx=10, pady=10)
        frame.pack(pady=10, padx=10, fill="x")

        tk.Label(frame, text="実験開始までの遅延 (秒):").pack(side=tk.LEFT, padx=5)
        self.delay_time_var = tk.IntVar(value=0)
        ttk.Spinbox(frame, from_=0, to=60, textvariable=self.delay_time_var, width=5).pack(side=tk.LEFT, padx=5)

    def _setup_results_display_frame(self):
        frame = tk.LabelFrame(self.root, text="実験結果", padx=10, pady=10)
        frame.pack(pady=10, padx=10, fill="both", expand=True)

        # カラムに "期待LOS/nLOS" を追加
        self.results_tree = ttk.Treeview(frame, columns=(
            "No", "距離(m)", "向き", "期待LOS/nLOS", "試行", "計測時間(ms)", "最頻距離(m)", "最頻LOS/nLOS", "最頻水平角(°)", "最頻仰角(°)", "備考"
        ), show="headings")

        self.results_tree.heading("No", text="No", anchor=tk.W)
        self.results_tree.heading("距離(m)", text="距離(m)", anchor=tk.W)
        self.results_tree.heading("向き", text="向き", anchor=tk.W)
        self.results_tree.heading("期待LOS/nLOS", text="期待LOS/nLOS", anchor=tk.W) # 新しいヘッダ
        self.results_tree.heading("試行", text="試行", anchor=tk.W)
        self.results_tree.heading("計測時間(ms)", text="計測時間(ms)", anchor=tk.W)
        self.results_tree.heading("最頻距離(m)", text="最頻距離(m)", anchor=tk.W)
        self.results_tree.heading("最頻LOS/nLOS", text="最頻LOS/nLOS", anchor=tk.W)
        self.results_tree.heading("最頻水平角(°)", text="最頻水平角(°)", anchor=tk.W)
        self.results_tree.heading("最頻仰角(°)", text="最頻仰角(°)", anchor=tk.W)
        self.results_tree.heading("備考", text="備考", anchor=tk.W)

        self.results_tree.column("No", width=40, stretch=tk.NO)
        self.results_tree.column("距離(m)", width=60, stretch=tk.NO)
        self.results_tree.column("向き", width=90, stretch=tk.NO)
        self.results_tree.column("期待LOS/nLOS", width=110, stretch=tk.NO) # 新しいカラムの幅
        self.results_tree.column("試行", width=50, stretch=tk.NO)
        self.results_tree.column("計測時間(ms)", width=90, stretch=tk.NO)
        self.results_tree.column("最頻距離(m)", width=90, stretch=tk.NO)
        self.results_tree.column("最頻LOS/nLOS", width=100, stretch=tk.NO)
        self.results_tree.column("最頻水平角(°)", width=90, stretch=tk.NO)
        self.results_tree.column("最頻仰角(°)", width=90, stretch=tk.NO)
        self.results_tree.column("備考", width=150)

        self.results_tree.pack(side=tk.LEFT, fill="both", expand=True)

        scrollbar = ttk.Scrollbar(frame, orient="vertical", command=self.results_tree.yview)
        self.results_tree.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side=tk.RIGHT, fill="y")

    def _update_results_display(self):
        for item in self.results_tree.get_children():
            self.results_tree.delete(item)

        for res in self.experiment_data_set.get_all_results():
            # valuesの順番に "期待LOS/nLOS" を追加
            self.results_tree.insert("", tk.END, values=(
                res.experiment_no,
                res.distance_m,
                res.uwb_orientation_condition,
                res.nlos_los_expected, # ここで期待条件を表示
                res.trial_count,
                res.measurement_time_ms,
                res.mode_distance_m,
                res.mode_nlos_los,
                res.mode_horizontal_angle_deg,
                res.mode_elevation_angle_deg,
                res.remarks
            ))
            
    def _start_experiment(self):
        if not self.experiment_runner.ser or not self.experiment_runner.ser.is_open:
            messagebox.showerror("エラー", "シリアルポートが接続されていません。実験を開始できません。")
            return

        delay_time = self.delay_time_var.get()
        num_repetitions = self.experiment_count.get()

        distance = int(self.distance_var.get())
        uwb_orientation = self.uwb_orientation_var.get()
        nlos_los_expected = self.nlos_los_var.get() # ここで期待条件を取得
        z_position_type = self.z_position_var.get()

        experiment_type_map = {
            "Z揃え (実験①)": 100,
            "Anchor=Tag+1000mm (実験②)": 200,
            "Anchor電波上から (実験③)": 300
        }
        base_experiment_no = experiment_type_map.get(z_position_type, 0)

        if delay_time > 0:
            messagebox.showinfo("実験開始", f"{delay_time}秒後に実験を開始します。")
            self.root.update_idletasks()
            time.sleep(delay_time)

        try:
            for i in range(num_repetitions):
                current_experiment_no = base_experiment_no + len(self.experiment_data_set.get_all_results()) + 1

                status_message = f"実験 {i+1}/{num_repetitions} 回目を実行中..."
                print(status_message)

                raw_result = self.experiment_runner.run_single_experiment(
                    experiment_no=current_experiment_no,
                    distance_m=distance,
                    uwb_orientation_condition=uwb_orientation,
                    nlos_status_expected=nlos_los_expected, # ここでExperimentRunnerに渡す
                    trial_count=i + 1
                )

                new_experiment_result = ExperimentResult(
                    experiment_no=raw_result["experiment_no"],
                    distance_m=raw_result["distance_m"],
                    uwb_orientation_condition=raw_result["uwb_orientation_condition"],
                    nlos_los_expected=nlos_los_expected, # ExperimentResultにも追加
                    trial_count=raw_result["trial_count"],
                    measurement_time_ms=raw_result["measurement_time_ms"],
                    mode_distance_m=raw_result["mode_distance_m"],
                    mode_nlos_los=raw_result["mode_nlos_los"],
                    mode_horizontal_angle_deg=raw_result["mode_horizontal_angle_deg"],
                    mode_elevation_angle_deg=raw_result["mode_elevation_angle_deg"],
                    remarks=f"Z位置関係: {z_position_type}",
                    raw_frames_data=raw_result["raw_frames_data"]
                )
                self.experiment_data_set.add_result(new_experiment_result)
                self._update_results_display()

            self.experiment_data_set.save_to_csv()
            messagebox.showinfo("実験完了", f"{num_repetitions}回の実験が完了し、結果が保存されました。")

        except Exception as e:
            messagebox.showerror("エラー", f"実験中にエラーが発生しました: {e}")

    def _clear_all_results(self):
        if messagebox.askyesno("確認", "全ての実験結果をクリアしますか？この操作は元に戻せません。"):
            self.experiment_data_set.clear_results()
            self.experiment_data_set.save_to_csv()
            self._update_results_display()
            messagebox.showinfo("完了", "全ての実験結果がクリアされました。")

    def _delete_selected_results(self):
        selected_items = self.results_tree.selection()
        if not selected_items:
            messagebox.showwarning("選択エラー", "削除する結果を選択してください。")
            return

        if not messagebox.askyesno("確認", "選択した結果を削除しますか？"):
            return

        experiment_nos_to_delete = []
        for item_id in selected_items:
            values = self.results_tree.item(item_id, 'values')
            if values:
                # 削除されるカラムの位置が変更されたため、インデックスを再確認
                # 以前はvalues[0]がexperiment_noだったが、今は変わらない
                experiment_nos_to_delete.append(int(values[0]))

        self.experiment_data_set.results = [
            res for res in self.experiment_data_set.results
            if res.experiment_no not in experiment_nos_to_delete
        ]

        self.experiment_data_set.save_to_csv()
        self._update_results_display()
        messagebox.showinfo("削除完了", "選択された結果が削除されました。")


if __name__ == "__main__":
    root = tk.Tk()
    app = ExperimentApp(root)
    root.mainloop()