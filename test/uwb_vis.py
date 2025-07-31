import tkinter as tk
from tkinter import ttk
import re
import math
import threading
from collections import deque
import serial

import matplotlib
matplotlib.use("TkAgg")  # tkinterバックエンドを使用
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation

# --- グローバル設定 ---
MAX_DATA_POINTS = 50 
UPDATE_INTERVAL_MS = 50
DEFAULT_VIEW = {'elev': 30, 'azim': -60} # 原点ボタンで戻る視点

# データを保持するためのスレッドセーフな両端キュー
data_queue = deque(maxlen=MAX_DATA_POINTS)

# --- データ解析 & 座標変換 (前回と同様) ---
def polar_to_cartesian(dist, az, el):
    """極座標 (度) を直交座標に変換"""
    az_rad = math.radians(az)
    el_rad = math.radians(el)
    x = dist * math.cos(el_rad) * math.cos(az_rad)
    y = dist * math.cos(el_rad) * math.sin(az_rad)
    z = dist * math.sin(el_rad)
    return x, y, z

def read_from_serial(ser):
    """シリアルポートから継続的にデータを読み込む"""
    print("Starting serial read thread...")
    current_point = {}
    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                if "nLos" in line:
                    current_point['nlos'] = int(re.search(r"nLos\s*:\s*(\d)", line).group(1))
                elif "distance" in line:
                    current_point['distance'] = float(re.search(r"distance\s*:\s*(-?[\d.]+)", line).group(1))
                elif "aoa_azimuth" in line:
                    current_point['azimuth'] = float(re.search(r"aoa_azimuth\s*:\s*(-?[\d.]+)", line).group(1))
                elif "aoa_elevation" in line:
                    current_point['elevation'] = float(re.search(r"aoa_elevation\s*:\s*(-?[\d.]+)", line).group(1))

                if all(k in current_point for k in ['nlos', 'distance', 'azimuth', 'elevation']):
                    data_queue.append(current_point.copy())
                    current_point.clear()
        except (serial.SerialException, OSError):
            print("Serial port disconnected. Stopping thread.")
            break
        except Exception:
            pass 

# --- Tkinter アプリケーション ---
class UWB_Tkinter_App:
    def __init__(self, root):
        self.root = root
        self.root.title("UWB 3D Visualizer")
        self.root.geometry("800x650")

        # --- コントロールフレーム ---
        control_frame = ttk.Frame(self.root, padding="10")
        control_frame.pack(side=tk.TOP, fill=tk.X)

        center_button = ttk.Button(control_frame, text="原点に合わせる (Center View)", command=self.center_view)
        center_button.pack(side=tk.LEFT)
        
        # --- Matplotlib Figure ---
        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        
        self.center_view() # 初期視点を設定

    def center_view(self):
        """カメラの視点を初期位置に戻す"""
        self.ax.view_init(elev=DEFAULT_VIEW['elev'], azim=DEFAULT_VIEW['azim'])
        self.canvas.draw()

    def update_plot(self, frame):
        """プロットを定期的に更新する関数"""
        # 現在の視点を保存
        current_elev = self.ax.elev
        current_azim = self.ax.azim
        
        self.ax.clear()
        
        # プロットの基本設定
        self.ax.set_title("3D Real-time UWB Visualization")
        self.ax.set_xlabel("X-axis (cm)")
        self.ax.set_ylabel("Y-axis (cm)")
        self.ax.set_zlabel("Z-axis (cm)")
        self.ax.set_xlim([-100, 100])
        self.ax.set_ylim([-100, 100])
        self.ax.set_zlim([-100, 100])
        
        # 向きを示すX軸の矢印を描画
        self.ax.quiver(0, 0, 0, 50, 0, 0, color='blue', label='Positive X-axis', 
                       arrow_length_ratio=0.1, linewidth=2)

        # 原点を表示
        self.ax.scatter([0], [0], [0], c='black', marker='o', s=100, label='Origin (Anchor)')

        # データ点をプロット
        x_coords, y_coords, z_coords, colors = [], [], [], []
        for data_point in list(data_queue):
            x, y, z = polar_to_cartesian(data_point['distance'], data_point['azimuth'], data_point['elevation'])
            x_coords.append(x)
            y_coords.append(y)
            z_coords.append(z)
            colors.append('green' if data_point['nlos'] == 0 else 'red')
        
        if x_coords:
            self.ax.scatter(x_coords, y_coords, z_coords, c=colors, marker='^', s=80)
        
        # 凡例用のダミープロット
        self.ax.scatter([], [], [], c='green', marker='^', s=80, label='Tag (LOS)')
        self.ax.scatter([], [], [], c='red', marker='^', s=80, label='Tag (NLOS)')
        self.ax.legend()
        
        # 視点を復元
        self.ax.view_init(elev=current_elev, azim=current_azim)

def run_with_serial():
    """シリアルポートを使用してプログラムを実行"""
    try:
        ser = serial.Serial('COM7', 3000000, timeout=1)
        print("Connected to COM7 at 3000000 baud.")
        
        # シリアル読み取りをバックグラウンドスレッドで開始
        serial_thread = threading.Thread(target=read_from_serial, args=(ser,), daemon=True)
        serial_thread.start()

    except serial.SerialException as e:
        print(f"FATAL: Could not open serial port COM7. {e}")
        print("Running with sample data instead.")
        run_with_sample_data(start_app=False) # サンプルデータで続行

    root = tk.Tk()
    app = UWB_Tkinter_App(root)
    ani = animation.FuncAnimation(app.fig, app.update_plot, interval=UPDATE_INTERVAL_MS, cache_frame_data=False)
    root.mainloop()

def run_with_sample_data(start_app=True):
    """提供されたサンプルデータでデモを実行"""
    sample_log = """
    APP      :INFO :TWR[0].nLos             : 0
    APP      :INFO :TWR[0].distance         : 50
    APP      :INFO :TWR[0].aoa_azimuth: 0.0
    APP      :INFO :TWR[0].aoa_elevation    : 0.0
    APP      :INFO :TWR[0].nLos             : 0
    APP      :INFO :TWR[0].distance         : 60
    APP      :INFO :TWR[0].aoa_azimuth: 10.0
    APP      :INFO :TWR[0].aoa_elevation    : 15.0
    APP      :INFO :TWR[0].nLos             : 1
    APP      :INFO :TWR[0].distance         : 70
    APP      :INFO :TWR[0].aoa_azimuth: 20.0
    APP      :INFO :TWR[0].aoa_elevation    : 30.0
    """
    
    current_point = {}
    for line in sample_log.strip().splitlines():
        if "nLos" in line:
            current_point['nlos'] = int(re.search(r"nLos\s*:\s*(\d)", line).group(1))
        elif "distance" in line:
            current_point['distance'] = float(re.search(r"distance\s*:\s*(-?[\d.]+)", line).group(1))
        elif "aoa_azimuth" in line:
            current_point['azimuth'] = float(re.search(r"aoa_azimuth\s*:\s*(-?[\d.]+)", line).group(1))
        elif "aoa_elevation" in line:
            current_point['elevation'] = float(re.search(r"aoa_elevation\s*:\s*(-?[\d.]+)", line).group(1))

        if all(k in current_point for k in ['nlos', 'distance', 'azimuth', 'elevation']):
            data_queue.append(current_point.copy())
            current_point.clear()
            
    if start_app:
        root = tk.Tk()
        app = UWB_Tkinter_App(root)
        ani = animation.FuncAnimation(app.fig, app.update_plot, interval=UPDATE_INTERVAL_MS, cache_frame_data=False)
        root.mainloop()

if __name__ == '__main__':
    # --- 実行モードを選択 ---
    # サンプルデータで試す場合
    # run_with_sample_data()
    
    # シリアルポート（COM7）からリアルタイムで読み込む場合
    run_with_serial()