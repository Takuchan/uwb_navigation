import serial
import re
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
from collections import deque
import threading
from matplotlib.animation import FuncAnimation

class UWBTrilateration:
    def __init__(self, com_port='COM7', baud_rate=3000000):
        self.com_port = com_port
        self.baud_rate = baud_rate
        self.ser = None
        
        # Fixed anchor positions based on given distances
        # UWB0が一番ベースとなる。
        self.anchors = self.calculate_anchor_positions()
        
        # Data storage
        self.positions = deque(maxlen=100)  # メモリのことを考慮して100個だけデータを一時ほじほじする。
        self.current_distances = [0, 0, 0]
        self.current_nlos = [0, 0, 0]  # Line of sight status nLOSはAnchorとTagの間に障害物があるかどうかを判定する(1 = あり 0 = なし)
        self.data_lock = threading.Lock()
        self.latest_uwb_data = {}  # Store latest data from each UWB
        
        # Visualization setup
        plt.style.use('dark_background') #なんとなくダーク
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.setup_plot()
        
    def calculate_anchor_positions(self):
        """Calculate anchor positions based on given distances"""
        # UWB0 at origin
        anchor0 = np.array([0, 0, 0])
        
        # UWB1の距離はUWB0よりも776cm離れている
        anchor1 = np.array([776, 0, 0])
        


        # UWB2(Anchor2)のデータはUWB0とUWB1のデータを使って計算する。余弦定理を利用している。
        # Distance from UWB0: 789cm, Distance from UWB1: 530cm
        d01 = 776  # UWB0 to UWB1
        d02 = 789  # UWB0 to UWB2
        d12 = 530  # UWB1 to UWB2
        
        # UWB2(Anchor)はほかのパラメータのデータを使って行う。余弦定理で求めたcosΘを掛け合わせている。
        x2 = (d02**2 - d12**2 + d01**2) / (2 * d01)
        y2 = np.sqrt(d02**2 - x2**2)
        anchor2 = np.array([x2, y2, 0])
        
        return [anchor0, anchor1, anchor2]
    

    # シリアル通信を行う部分。
    def connect_serial(self):
        """Connect to serial port with better error handling"""
        try:
            self.ser = serial.Serial(
                port=self.com_port, 
                baudrate=self.baud_rate, 
                timeout=0.1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            time.sleep(2)  # Wait for connection to stabilize
            print(f"Connected to {self.com_port} at {self.baud_rate} baud")
            
            # Clear any existing data in buffer
            self.ser.flushInput()
            return True
        except Exception as e:
            print(f"Failed to connect to serial port: {e}")
            return False
    

    # UWBのシリアル通信から送られてきたデータの加工 `APP:～ より後ろを持ってくる`
    def parse_uwb_data(self, data_line):
        """Parse UWB data from serial line - improved version"""
        try:
            # Clean the line and look for APP :INFO patterns
            if 'APP' in data_line and 'INFO' in data_line and 'TWR[' in data_line:
                # Extract TWR data using improved regex
                twr_pattern = r'TWR\[(\d+)\]\.(\w+)\s*:\s*([\d.-]+)'
                match = re.search(twr_pattern, data_line)
                
                if match:
                    twr_id = int(match.group(1))
                    param = match.group(2)
                    value = float(match.group(3))
                    
                    # Store data by UWB ID
                    if twr_id not in self.latest_uwb_data:
                        self.latest_uwb_data[twr_id] = {}
                    
                    self.latest_uwb_data[twr_id][param] = value
                    
                    print(f"Parsed: UWB{twr_id}.{param} = {value}")
                    return twr_id, param, value
                    
        except Exception as e:
            print(f"Error parsing data: {e}")
        
        return None, None, None
    
    def check_complete_data_set(self):
        """Check if we have complete distance data from all UWBs"""
        distances = [0, 0, 0]
        nlos_status = [0, 0, 0]
        
        for i in range(3):
            if i in self.latest_uwb_data:
                if 'distance' in self.latest_uwb_data[i]:
                    distances[i] = self.latest_uwb_data[i]['distance']
                if 'nLos' in self.latest_uwb_data[i]:
                    nlos_status[i] = self.latest_uwb_data[i]['nLos']
        
        # Check if all distances are valid (> 0)
        if all(d > 0 for d in distances):
            return distances, nlos_status
        
        return None, None
    
    def trilaterate_3d(self, distances):
        """Perform 3D trilateration using least squares"""
        if len(distances) < 3:
            return None
        
        try:

            #x_diff,y_diffは三平方の定理で求めた結果から得られた物。
            
            A = []
            b = []
            
            for i in range(1, len(distances)):
                if distances[i] > 0 and distances[0] > 0:
                    # 係数 2(x_i - x_0) を計算している
                    x_diff = 2 * (self.anchors[i][0] - self.anchors[0][0])
                    y_diff = 2 * (self.anchors[i][1] - self.anchors[0][1])
                    z_diff = 2 * (self.anchors[i][2] - self.anchors[0][2])
                    
                    A.append([x_diff, y_diff, z_diff])
                   
                    # 方程式の右辺の長い計算をしている
                    b_val = (distances[0]**2 - distances[i]**2 + 
                            np.sum(self.anchors[i]**2) - np.sum(self.anchors[0]**2))
                    b.append(b_val)
            
            if len(A) < 2:
                return None
            
            A = np.array(A)
            b = np.array(b)
            
            # Solve using least squares lstsqはほんの少しの誤差を見逃してくれてよしなに調整してくれる。
            # 連立一次方程式
            position, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
            
            return position
            
        except Exception as e:
            print(f"Trilateration error: {e}")
            return None
    
    def setup_plot(self):
        """Initial plot setup"""
        self.ax.set_xlabel('X (cm)', color='white')
        self.ax.set_ylabel('Y (cm)', color='white')
        self.ax.set_zlabel('Z (cm)', color='white')
        self.ax.set_title('UWB Real-time Position Tracking', color='white', fontsize=14)
        
        # Set reasonable limits
        self.ax.set_xlim(-500, 1000)
        self.ax.set_ylim(-500, 1000)
        self.ax.set_zlim(-200, 500)
        
        # Plot anchors initially
        for i, anchor in enumerate(self.anchors):
            self.ax.scatter(anchor[0], anchor[1], anchor[2], 
                          c='red', s=100, marker='^')
            self.ax.text(anchor[0], anchor[1], anchor[2] + 50, 
                        f'UWB{i}', fontsize=12, color='white')
    
    def update_visualization(self, frame):
        """Update 3D visualization for animation"""
        with self.data_lock:
            if len(self.positions) == 0:
                return
            
            # Clear previous plots except anchors
            self.ax.clear()
            self.setup_plot()
            
            # Plot position history
            if len(self.positions) > 1:
                positions_array = np.array(list(self.positions))
                self.ax.plot(positions_array[:, 0], positions_array[:, 1], 
                            positions_array[:, 2], 'cyan', alpha=0.8, linewidth=2, label='Path')
            
            # Plot current position
            current_pos = self.positions[-1]
            self.ax.scatter(current_pos[0], current_pos[1], current_pos[2], 
                          c='lime', s=200, marker='o', label='Current Position')
            
            # Add distance and nLos information as text
            info_text = f'Position:\nX: {current_pos[0]:.1f}cm\nY: {current_pos[1]:.1f}cm\nZ: {current_pos[2]:.1f}cm\n\n'
            info_text += 'Distances:\n'
            for i in range(3):
                nlos_str = "✓" if self.current_nlos[i] == 1 else "✗"
                info_text += f'UWB{i}: {self.current_distances[i]:.1f}cm {nlos_str}\n'
            
            self.ax.text2D(0.02, 0.98, info_text, transform=self.ax.transAxes, 
                          fontsize=10, verticalalignment='top', color='white',
                          bbox=dict(boxstyle='round', facecolor='black', alpha=0.7))
            
            self.ax.legend(loc='upper right')
    
    def serial_reader(self):
        """Separate thread for reading serial data"""
        print("Serial reader thread started...")
        buffer = ""
        
        try:
            while True:
                if self.ser and self.ser.in_waiting > 0:
                    # Read available data
                    new_data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                    buffer += new_data
                    
                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        
                        if line and 'TWR[' in line:
                            print(f"Raw line: {line}")
                            
                            twr_id, param, value = self.parse_uwb_data(line)
                            
                            if twr_id is not None and param == 'distance':
                                # Check if we have complete data set after distance update
                                distances, nlos_status = self.check_complete_data_set()
                                
                                if distances is not None:
                                    print(f"Complete data set: {distances}")
                                    
                                    # Perform trilateration
                                    position = self.trilaterate_3d(distances)
                                    
                                    if position is not None:
                                        with self.data_lock:
                                            self.positions.append(position)
                                            self.current_distances = distances
                                            self.current_nlos = nlos_status
                                        
                                        print(f"New position: X={position[0]:.1f}, Y={position[1]:.1f}, Z={position[2]:.1f}")
                
                time.sleep(0.001)  # Very small delay
                
        except Exception as e:
            print(f"Serial reader error: {e}")
    
    def process_data(self):
        """Main data processing with real-time visualization"""
        if not self.connect_serial():
            return
        
        # Start serial reading thread
        serial_thread = threading.Thread(target=self.serial_reader, daemon=True)
        serial_thread.start()
        
        # Set up animation for real-time plotting
        ani = FuncAnimation(self.fig, self.update_visualization, interval=100, cache_frame_data=False)
        
        try:
            plt.show()  # This will block and show the real-time plot
        except KeyboardInterrupt:
            print("\nStopping UWB trilateration...")
        finally:
            if self.ser:
                self.ser.close()

def main():
    """Main function to run the UWB trilateration system"""
    print("UWB Trilateration System")
    print("Anchor positions:")
    
    uwb_system = UWBTrilateration()
    
    for i, anchor in enumerate(uwb_system.anchors):
        print(f"UWB{i}: X={anchor[0]:.1f}, Y={anchor[1]:.1f}, Z={anchor[2]:.1f}")
    
    print(f"\nConnecting to {uwb_system.com_port}...")
    print("Press Ctrl+C to stop")
    
    uwb_system.process_data()

if __name__ == "__main__":
    main()
