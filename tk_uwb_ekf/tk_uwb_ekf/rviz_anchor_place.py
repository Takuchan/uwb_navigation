import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
import tkinter as tk
from tkinter import messagebox
import threading
import yaml
import os

class RvizAnchorPlacer(Node):
    def __init__(self):
        super().__init__('rviz_anchor_placer')

        # ROSパラメータの宣言と取得
        self.declare_parameter('num_anchors', 3)
        self.declare_parameter('save_path', 'anchors.yaml')
        self.num_anchors = self.get_parameter('num_anchors').get_parameter_value().integer_value
        self.save_path = self.get_parameter('save_path').get_parameter_value().string_value
        
        self.anchors = []
        self.lock = threading.Lock()

        # RVizの /clicked_point トピックを購読
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10)
        
        # RVizにマーカーを表示するためのPublisher
        self.marker_publisher = self.create_publisher(MarkerArray, '/anchor_markers', 10)

        self.get_logger().info(f"RViz Anchor Placer started. Waiting for {self.num_anchors} anchors.")
        self.get_logger().info(f"Use the 'Publish Point' tool in RViz to set anchor positions on the map.")
        self.get_logger().info(f"Final configuration will be saved to: {self.save_path}")

        # GUIを別スレッドで実行
        self.gui_thread = threading.Thread(target=self.run_gui)
        self.gui_thread.start()

    def clicked_point_callback(self, msg: PointStamped):
        with self.lock:
            if len(self.anchors) >= self.num_anchors:
                self.get_logger().warn(f"Already have {self.num_anchors} anchors. Reset if you want to place new ones.")
                return

            # クリックされた座標を保存
            x, y = msg.point.x, msg.point.y
            self.anchors.append({'x': x, 'y': y})
            self.get_logger().info(f"Anchor {len(self.anchors)} placed at: (x={x:.2f}, y={y:.2f})")
            
            # RVizにマーカーを描画
            self.publish_markers()

    def publish_markers(self):
        marker_array = MarkerArray()
        # 既存のマーカーをすべて削除するメッセージを追加
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        # 現在のアンカー位置に新しいマーカーを配置
        for i, anchor in enumerate(self.anchors):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "anchors"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = anchor['x']
            marker.pose.position.y = anchor['y']
            marker.pose.position.z = 0.5  # 少し浮かせる
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
        
        self.marker_publisher.publish(marker_array)
    
    def save_anchors_to_yaml(self):
        with self.lock:
            if len(self.anchors) != self.num_anchors:
                messagebox.showwarning("Warning", f"Please place exactly {self.num_anchors} anchors before saving.")
                return
            
            output_data = {'anchors': []}
            for i, anchor in enumerate(self.anchors):
                output_data['anchors'].append({
                    'name': f'A{i}',
                    'x': anchor['x'],
                    'y': anchor['y']
                })
            
            try:
                with open(self.save_path, 'w') as f:
                    yaml.dump(output_data, f, default_flow_style=False)
                self.get_logger().info(f"Successfully saved anchor positions to {self.save_path}")
                messagebox.showinfo("Success", f"アンカーの位置を保存しました:\n{self.save_path}。再度 colcon build を行ってください。")
            except Exception as e:
                self.get_logger().error(f"Failed to save file: {e}")
                messagebox.showerror("Error", f"ファイルの保存に失敗しました:\n{e}")

    def reset_anchors(self):
        with self.lock:
            self.get_logger().info("Anchor positions have been reset.")
            self.anchors.clear()
            self.publish_markers() # マーカーを消去

    def run_gui(self):
        root = tk.Tk()
        root.title("Anchor Setter")

        status_label = tk.Label(root, text="Initializing...", font=("Helvetica", 12), width=40)
        status_label.pack(pady=10, padx=10)

        save_button = tk.Button(root, text="Save Anchors", command=self.save_anchors_to_yaml)
        save_button.pack(pady=5)
        
        reset_button = tk.Button(root, text="Reset", command=self.reset_anchors)
        reset_button.pack(pady=5)

        def update_gui_status():
            with self.lock:
                placed_count = len(self.anchors)
                status_text = f"Placed {placed_count} / {self.num_anchors} anchors."
                if placed_count < self.num_anchors:
                    status_text += f"\nClick in RViz to place Anchor {placed_count}."
                    save_button.config(state=tk.DISABLED)
                else:
                    status_text += "\nReady to save."
                    save_button.config(state=tk.NORMAL)
                
                status_label.config(text=status_text)
            
            # 200msごとにGUIを更新
            root.after(200, update_gui_status)

        root.after(200, update_gui_status)
        root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = RvizAnchorPlacer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()