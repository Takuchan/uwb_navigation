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
    def __init__(self, com_port='/dev/ttyUSB0', baud_rate=3000000):
        self.com_port = com_port
        self.baud_rate = baud_rate
        self.ser = None
        
        # Fixed anchor positions based on given distances
        self.anchors = self.calculate_anchor_positions()
        
        # Data storage
        self.positions = deque(maxlen=100)
        self.current_distances = [0, 0, 0]
        self.current_nlos = [0, 0, 0]  # nLos状態を記録
        self.data_lock = threading.Lock()
        self.running = True
        
        # Debug mode
        self.debug = True
        
        # Visualization setup
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.setup_plot()
        
    def calculate_anchor_positions(self):
        """Calculate anchor positions based on given distances"""
        # UWB0 at origin
        anchor0 = np.array([0, 0, 0])
        
        # UWB1 at distance 776cm from UWB0
        anchor1 = np.array([776, 0, 0])
        
        # UWB2 positioned using triangulation from UWB0 and UWB1
        # Distance from UWB0: 789cm, Distance from UWB1: 530cm
        d01 = 776  # UWB0 to UWB1
        d02 = 789  # UWB0 to UWB2
        d12 = 530  # UWB1 to UWB2
        
        # Calculate UWB2 position using law of cosines
        x2 = (d02**2 - d12**2 + d01**2) / (2 * d01)
        y2 = np.sqrt(d02**2 - x2**2)
        anchor2 = np.array([x2, y2, 0])
        
        return [anchor0, anchor1, anchor2]
    
    def connect_serial(self):
        """Connect to serial port with improved error handling"""
        try:
            self.ser = serial.Serial(self.com_port, self.baud_rate, timeout=1)
            print(f"Connected to {self.com_port} at {self.baud_rate} baud")
            
            # Wait a bit for the connection to stabilize
            time.sleep(2)
            
            # Clear any existing data in buffer
            self.ser.flushInput()
            
            return True
        except Exception as e:
            print(f"Failed to connect to serial port: {e}")
            return False
    
    def parse_uwb_data(self, data_line):
        """Parse UWB data from serial line - improved parsing"""
        try:
            if self.debug:
                print(f"Parsing line: {data_line}")
            
            # より正確な正規表現パターン
            # APP :INFO :TWR[0].distance : 195 のような形式を解析
            distance_pattern = r'APP\s*:\s*INFO\s*:\s*TWR\[(\d+)\]\.distance\s*:\s*([\d.]+)'
            nlos_pattern = r'APP\s*:\s*INFO\s*:\s*TWR\[(\d+)\]\.nLos\s*:\s*(\d+)'
            
            uwb_data = {}
            
            # 距離データを抽出
            distance_matches = re.findall(distance_pattern, data_line)
            for match in distance_matches:
                twr_id = int(match[0])
                distance = float(match[1])
                
                if twr_id not in uwb_data:
                    uwb_data[twr_id] = {}
                uwb_data[twr_id]['distance'] = distance
                
                if self.debug:
                    print(f"Found distance: TWR[{twr_id}] = {distance}cm")
            
            # nLos状態を抽出
            nlos_matches = re.findall(nlos_pattern, data_line)
            for match in nlos_matches:
                twr_id = int(match[0])
                nlos = int(match[1])
                
                if twr_id not in uwb_data:
                    uwb_data[twr_id] = {}
                uwb_data[twr_id]['nLos'] = nlos
                
                if self.debug:
                    print(f"Found nLos: TWR[{twr_id}] = {nlos}")
            
            return uwb_data
            
        except Exception as e:
            if self.debug:
                print(f"Error parsing data: {e}")
            return {}
    
    def trilaterate_3d(self, distances):
        """Perform 3D trilateration using least squares"""
        if len(distances) < 3:
            return None
        
        try:
            # Set up equations for trilateration
            # (x-x1)^2 + (y-y1)^2 + (z-z1)^2 = r1^2
            # Convert to linear system using differences
            
            A = []
            b = []
            
            for i in range(1, len(distances)):
                if distances[i] > 0 and distances[0] > 0:
                    x_diff = 2 * (self.anchors[i][0] - self.anchors[0][0])
                    y_diff = 2 * (self.anchors[i][1] - self.anchors[0][1])
                    z_diff = 2 * (self.anchors[i][2] - self.anchors[0][2])
                    
                    A.append([x_diff, y_diff, z_diff])
                    
                    b_val = (distances[0]**2 - distances[i]**2 + 
                            np.sum(self.anchors[i]**2) - np.sum(self.anchors[0]**2))
                    b.append(b_val)
            
            if len(A) < 2:
                return None
            
            A = np.array(A)
            b = np.array(b)
            
            # Solve using least squares
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
            color = 'lime' if all(nlos == 1 for nlos in self.current_nlos) else 'yellow'
            self.ax.scatter(current_pos[0], current_pos[1], current_pos[2], 
                          c=color, s=200, marker='o', label='Current Position')
            
            # Add detailed information as text
            info_text = f'Position:\nX: {current_pos[0]:.1f}cm\nY: {current_pos[1]:.1f}cm\nZ: {current_pos[2]:.1f}cm\n\n'
            info_text += 'Distances & Signal Quality:\n'
            for i in range(3):
                status = "Good" if self.current_nlos[i] == 1 else "NLOS"
                info_text += f'UWB{i}: {self.current_distances[i]:.1f}cm ({status})\n'
            
            self.ax.text2D(0.02, 0.98, info_text, transform=self.ax.transAxes, 
                          fontsize=9, verticalalignment='top', color='white',
                          bbox=dict(boxstyle='round', facecolor='black', alpha=0.8))
            
            self.ax.legend(loc='upper right')
    
    def serial_reader(self):
        """Separate thread for reading serial data - improved data collection"""
        current_frame_data = {}  # 1つのフレームのデータを蓄積
        
        try:
            while self.running:
                if self.ser and self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    if self.debug and line:
                        print(f"Raw line: {line}")
                    
                    if 'TWR' in line and 'APP' in line:
                        uwb_data = self.parse_uwb_data(line)
                        
                        # データを現在のフレームに追加
                        for twr_id, data in uwb_data.items():
                            if twr_id not in current_frame_data:
                                current_frame_data[twr_id] = {}
                            current_frame_data[twr_id].update(data)
                        
                        # 3つのUWBすべてからdistanceデータが揃ったかチェック
                        if self.has_complete_frame(current_frame_data):
                            distances = [0, 0, 0]
                            nlos_states = [0, 0, 0]
                            
                            for twr_id in range(3):
                                if twr_id in current_frame_data:
                                    distances[twr_id] = current_frame_data[twr_id].get('distance', 0)
                                    nlos_states[twr_id] = current_frame_data[twr_id].get('nLos', 0)
                            
                            if all(d > 0 for d in distances):
                                # Perform trilateration
                                position = self.trilaterate_3d(distances)
                                
                                if position is not None:
                                    with self.data_lock:
                                        self.positions.append(position)
                                        self.current_distances = distances
                                        self.current_nlos = nlos_states
                                    
                                    signal_quality = "Good" if all(nlos == 1 for nlos in nlos_states) else "NLOS detected"
                                    print(f"Position: X={position[0]:.1f}, Y={position[1]:.1f}, Z={position[2]:.1f} ({signal_quality})")
                            
                            # フレームデータをリセット
                            current_frame_data = {}
                
                time.sleep(0.01)
                
        except Exception as e:
            print(f"Serial reader error: {e}")
    
    def has_complete_frame(self, frame_data):
        """Check if we have complete distance data from all 3 UWBs"""
        for twr_id in range(3):
            if twr_id not in frame_data or 'distance' not in frame_data[twr_id]:
                return False
        return True
    
    def process_data(self):
        """Main data processing with real-time visualization"""
        if not self.connect_serial():
            return
        
        print("Starting data collection...")
        print("Waiting for UWB data...")
        
        # Start serial reading thread
        serial_thread = threading.Thread(target=self.serial_reader, daemon=True)
        serial_thread.start()
        
        # Set up animation for real-time plotting
        ani = FuncAnimation(self.fig, self.update_visualization, interval=200, cache_frame_data=False)
        
        try:
            plt.show()
        except KeyboardInterrupt:
            print("\nStopping UWB trilateration...")
        finally:
            self.running = False
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
