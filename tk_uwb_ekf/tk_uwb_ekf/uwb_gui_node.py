#!/usr/bin/env python3

import sys
import json
import time
from typing import Dict, Any, Optional

# ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# PyQt5
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QComboBox, QFileDialog, QSlider, QGroupBox,
    QLineEdit, QGridLayout, QStatusBar, QMessageBox, QSpinBox,
    QScrollArea, QTabWidget
)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QFont

# Matplotlib
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import numpy as np

# 自作モジュール
from .trilateration import Trilateration, AnchorPositionCalculator
from .csv_logger import CSVLogger
from .csv_player import CSVPlayer

# デフォルトのアンカー配置
DEFAULT_ANCHORS = {
    "TWR0": (0.0, 0.0, 0.0),
    "TWR1": (3.0, 0.0, 0.0),
    "TWR2": (1.5, 3.0, 0.0)
}


class UWBVisualizationCanvas(FigureCanvas):
    """
    UWBデータを可視化するMatplotlibキャンバス
    ズーム・パン機能付き、可変アンカー数対応
    """

    def __init__(self, parent=None):
        self.fig = Figure(figsize=(8, 6), facecolor='#2b2b2b')
        self.axes = self.fig.add_subplot(111)
        super().__init__(self.fig)
        
        self.setParent(parent)
        
        # データ保持
        self.anchor_positions = DEFAULT_ANCHORS.copy()
        self.trajectory = []  # 軌跡
        self.max_trajectory_points = 100
        
        # スタイル設定
        self.setup_plot_style()
        self.auto_adjust_axes()
        self.update_plot()
        
        # マウスイベント接続
        self.mpl_connect('scroll_event', self.on_scroll)
        self.mpl_connect('button_press_event', self.on_press)
        self.mpl_connect('button_release_event', self.on_release)
        self.mpl_connect('motion_notify_event', self.on_motion)
        
        # ドラッグ用変数
        self.is_dragging = False
        self.drag_start = None

    def setup_plot_style(self):
        """プロットのスタイルを設定"""
        self.axes.set_facecolor('#1e1e1e')
        self.axes.grid(True, color='#3e3e3e', linestyle='--', linewidth=0.5)
        self.axes.set_xlabel('X [m]', color='#e0e0e0', fontsize=12, fontweight='bold')
        self.axes.set_ylabel('Y [m]', color='#e0e0e0', fontsize=12, fontweight='bold')
        self.axes.set_title('UWB Multilateration - Real-time Position', 
                           color='#e0e0e0', fontsize=14, fontweight='bold')
        self.axes.tick_params(colors='#b0b0b0', labelsize=10)
        
        # 軸の範囲を設定（デフォルト）
        self.axes.set_aspect('equal')
        
        # 軸のスパイン（枠線）の色を設定
        for spine in self.axes.spines.values():
            spine.set_edgecolor('#5e5e5e')

    def auto_adjust_axes(self):
        """アンカー位置に基づいて軸範囲を自動調整"""
        if not self.anchor_positions:
            return
        
        # 全アンカーの座標を取得
        x_coords = [pos[0] for pos in self.anchor_positions.values()]
        y_coords = [pos[1] for pos in self.anchor_positions.values()]
        
        # 最小・最大値を計算
        x_min, x_max = min(x_coords), max(x_coords)
        y_min, y_max = min(y_coords), max(y_coords)
        
        # マージンを追加（範囲の30%）
        x_range = x_max - x_min
        y_range = y_max - y_min
        margin_x = max(x_range * 0.3, 1.0)  # 最小マージン1m
        margin_y = max(y_range * 0.3, 1.0)
        
        # 軸範囲を設定
        self.axes.set_xlim(x_min - margin_x, x_max + margin_x)
        self.axes.set_ylim(y_min - margin_y, y_max + margin_y)

    def on_scroll(self, event):
        """マウスホイールでズーム"""
        if event.inaxes != self.axes:
            return
        
        # ズーム係数
        zoom_factor = 1.2 if event.button == 'up' else 1 / 1.2
        
        # 現在の軸範囲を取得
        xlim = self.axes.get_xlim()
        ylim = self.axes.get_ylim()
        
        # マウス位置を中心にズーム
        xdata, ydata = event.xdata, event.ydata
        
        # 新しい範囲を計算
        new_xlim = [xdata - (xdata - xlim[0]) * zoom_factor,
                    xdata + (xlim[1] - xdata) * zoom_factor]
        new_ylim = [ydata - (ydata - ylim[0]) * zoom_factor,
                    ydata + (ylim[1] - ydata) * zoom_factor]
        
        self.axes.set_xlim(new_xlim)
        self.axes.set_ylim(new_ylim)
        self.draw()

    def on_press(self, event):
        """マウスボタン押下"""
        if event.button == 1 and event.inaxes == self.axes:  # 左クリック
            self.is_dragging = True
            self.drag_start = (event.xdata, event.ydata)

    def on_release(self, event):
        """マウスボタン解放"""
        self.is_dragging = False
        self.drag_start = None

    def on_motion(self, event):
        """マウス移動（ドラッグでパン）"""
        if not self.is_dragging or self.drag_start is None:
            return
        
        if event.inaxes != self.axes:
            return
        
        # ドラッグ量を計算
        dx = self.drag_start[0] - event.xdata
        dy = self.drag_start[1] - event.ydata
        
        # 軸範囲を移動
        xlim = self.axes.get_xlim()
        ylim = self.axes.get_ylim()
        
        self.axes.set_xlim(xlim[0] + dx, xlim[1] + dx)
        self.axes.set_ylim(ylim[0] + dy, ylim[1] + dy)
        self.draw()

    def update_plot(self, 
                   current_position: Optional[tuple] = None,
                   distances: Optional[Dict[str, float]] = None):
        """
        プロットを更新
        
        Args:
            current_position: 現在の推定位置 (x, y, z)
            distances: 各アンカーからの距離
        """
        # 現在の軸範囲を保存
        xlim = self.axes.get_xlim()
        ylim = self.axes.get_ylim()
        
        self.axes.clear()
        self.setup_plot_style()
        
        # 軸範囲を復元
        self.axes.set_xlim(xlim)
        self.axes.set_ylim(ylim)
        
        # アンカー位置をプロット（赤色）
        anchor_names = sorted(self.anchor_positions.keys())
        for i, anchor_name in enumerate(anchor_names):
            x, y, z = self.anchor_positions[anchor_name]
            self.axes.plot(x, y, 'rs', markersize=15, 
                          label='Anchors' if i == 0 else "", zorder=10)
            self.axes.text(x, y + (ylim[1] - ylim[0]) * 0.03, anchor_name, 
                          color='#ff5252', ha='center', fontsize=11, fontweight='bold')
            
            # 距離円を描画（薄い赤）
            if distances and anchor_name in distances and distances[anchor_name] is not None:
                circle = plt.Circle((x, y), distances[anchor_name], 
                                   color='#ff5252', fill=False, alpha=0.3, linestyle='--', linewidth=2)
                self.axes.add_patch(circle)
        
        # 現在位置をプロット（青色）
        if current_position:
            x, y, z = current_position
            self.axes.plot(x, y, 'o', color='#42a5f5', markersize=18, label='Current Position', zorder=15)
            
            # 軌跡に追加
            self.trajectory.append((x, y))
            if len(self.trajectory) > self.max_trajectory_points:
                self.trajectory.pop(0)
        
        # 軌跡を描画（薄い青）
        if len(self.trajectory) > 1:
            traj_x = [p[0] for p in self.trajectory]
            traj_y = [p[1] for p in self.trajectory]
            self.axes.plot(traj_x, traj_y, '-', color='#42a5f5', alpha=0.6, linewidth=2, label='Trajectory')
        
        # 凡例の背景色を設定
        legend = self.axes.legend(loc='upper right', facecolor='#2b2b2b', 
                        edgecolor='#5e5e5e', labelcolor='#e0e0e0', fontsize=10)
        legend.get_frame().set_alpha(0.9)
        
        self.draw()

    def clear_trajectory(self):
        """軌跡をクリア"""
        self.trajectory.clear()
        self.update_plot()

    def set_anchor_positions(self, anchors: Dict[str, tuple]):
        """アンカー位置を更新"""
        self.anchor_positions = anchors.copy()
        self.auto_adjust_axes()
        self.update_plot()

    def reset_view(self):
        """ビューをリセット（自動調整）"""
        self.auto_adjust_axes()
        self.update_plot()


class UWBGUINode(QMainWindow):
    """
    UWB 多辺測位システムのメインGUIウィンドウ（ROS2ノード統合）
    可変アンカー数、距離指定対応
    """
    
    # シグナル定義
    data_received = pyqtSignal(dict)

    def __init__(self, ros_node):
        super().__init__()
        
        self.ros_node = ros_node
        self.mode = "live"  # "live" or "playback"
        
        # アンカー数のデフォルト
        self.num_anchors = 3
        
        # コンポーネント初期化
        self.trilateration = Trilateration(DEFAULT_ANCHORS)
        self.csv_logger = CSVLogger()
        self.csv_player = CSVPlayer()
        
        # データ保持
        self.latest_anchor_data = None
        self.latest_position = None
        
        # UI初期化
        self.init_ui()
        
        # ROS2サブスクライバー（動的に生成）
        self.setup_ros_subscription()
        
        # タイマー設定
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_visualization)
        self.update_timer.start(50)  # 20Hz更新
        
        # 再生用タイマー
        self.playback_timer = QTimer()
        self.playback_timer.timeout.connect(self.playback_step)
        self.playback_speed = 1.0
        self.is_playing = False

    def setup_ros_subscription(self):
        """ROS2サブスクライバーを設定"""
        self.ros_node.create_subscription(
            String,
            'uwb_data_json',
            self.ros_data_callback,
            10
        )

    def init_ui(self):
        """UIを初期化"""
        self.setWindowTitle('UWB Multilateration System')
        self.setGeometry(100, 100, 1600, 1000)
        
        # ダークテーマの統一スタイル
        self.setStyleSheet("""
            QMainWindow {
                background-color: #1e1e1e;
            }
            QWidget {
                background-color: #2b2b2b;
                color: #e0e0e0;
            }
            QLabel {
                color: #e0e0e0;
                font-size: 13px;
                background-color: transparent;
            }
            QPushButton {
                background-color: #3e3e3e;
                color: #e0e0e0;
                border: 1px solid #5e5e5e;
                padding: 12px;
                border-radius: 6px;
                font-size: 14px;
                font-weight: bold;
                min-height: 35px;
            }
            QPushButton:hover {
                background-color: #4e4e4e;
                border: 1px solid #7e7e7e;
            }
            QPushButton:pressed {
                background-color: #2e2e2e;
            }
            QPushButton:disabled {
                background-color: #2a2a2a;
                color: #6e6e6e;
            }
            QGroupBox {
                color: #e0e0e0;
                border: 2px solid #5e5e5e;
                border-radius: 5px;
                margin-top: 12px;
                font-weight: bold;
                font-size: 13px;
                background-color: #2b2b2b;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
                color: #81d4fa;
            }
            QLineEdit {
                background-color: #3e3e3e;
                color: #e0e0e0;
                border: 1px solid #5e5e5e;
                padding: 8px;
                border-radius: 3px;
                font-size: 13px;
                selection-background-color: #1976d2;
            }
            QLineEdit:focus {
                border: 1px solid #42a5f5;
            }
            QComboBox, QSpinBox {
                background-color: #3e3e3e;
                color: #e0e0e0;
                border: 1px solid #5e5e5e;
                padding: 8px;
                border-radius: 3px;
                font-size: 13px;
            }
            QComboBox:hover, QSpinBox:hover {
                border: 1px solid #7e7e7e;
            }
            QComboBox::drop-down {
                border: none;
                background-color: #4e4e4e;
            }
            QComboBox::down-arrow {
                image: none;
                border-left: 5px solid transparent;
                border-right: 5px solid transparent;
                border-top: 5px solid #e0e0e0;
                margin-right: 5px;
            }
            QComboBox QAbstractItemView {
                background-color: #3e3e3e;
                color: #e0e0e0;
                selection-background-color: #1976d2;
                border: 1px solid #5e5e5e;
            }
            QSpinBox::up-button, QSpinBox::down-button {
                background-color: #4e4e4e;
                border: 1px solid #5e5e5e;
            }
            QSpinBox::up-button:hover, QSpinBox::down-button:hover {
                background-color: #5e5e5e;
            }
            QTabWidget::pane {
                border: 1px solid #5e5e5e;
                background-color: #2b2b2b;
            }
            QTabBar::tab {
                background-color: #3e3e3e;
                color: #b0b0b0;
                padding: 10px 20px;
                border: 1px solid #5e5e5e;
                font-size: 13px;
                border-bottom: none;
            }
            QTabBar::tab:selected {
                background-color: #1976d2;
                color: #ffffff;
            }
            QTabBar::tab:hover:!selected {
                background-color: #4e4e4e;
                color: #e0e0e0;
            }
            QSlider::groove:horizontal {
                background-color: #3e3e3e;
                height: 6px;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background-color: #42a5f5;
                width: 16px;
                margin: -5px 0;
                border-radius: 8px;
            }
            QSlider::handle:horizontal:hover {
                background-color: #64b5f6;
            }
            QScrollArea {
                border: none;
                background-color: #2b2b2b;
            }
            QScrollBar:vertical {
                background-color: #2b2b2b;
                width: 12px;
                border-radius: 6px;
            }
            QScrollBar::handle:vertical {
                background-color: #5e5e5e;
                border-radius: 6px;
                min-height: 20px;
            }
            QScrollBar::handle:vertical:hover {
                background-color: #7e7e7e;
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                height: 0px;
            }
            QMessageBox {
                background-color: #2b2b2b;
                color: #e0e0e0;
            }
            QMessageBox QLabel {
                color: #e0e0e0;
            }
            QMessageBox QPushButton {
                min-width: 80px;
            }
        """)
        
        # メインウィジェット
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        
        # 左側: 可視化エリア
        viz_layout = QVBoxLayout()
        
        # ツールバー追加
        self.canvas = UWBVisualizationCanvas(self)
        self.toolbar = NavigationToolbar(self.canvas, self)
        self.toolbar.setStyleSheet("""
            QToolBar {
                background-color: #3e3e3e;
                border: 1px solid #5e5e5e;
                spacing: 3px;
                padding: 3px;
            }
            QToolButton {
                background-color: #3e3e3e;
                color: #e0e0e0;
                border: 1px solid #5e5e5e;
                border-radius: 3px;
                padding: 5px;
            }
            QToolButton:hover {
                background-color: #4e4e4e;
            }
        """)
        
        # リセットビューボタンを追加
        reset_view_btn = QPushButton("🔄 Reset View")
        reset_view_btn.clicked.connect(self.canvas.reset_view)
        reset_view_btn.setMaximumWidth(150)
        
        viz_layout.addWidget(self.toolbar)
        viz_layout.addWidget(self.canvas)
        viz_layout.addWidget(reset_view_btn)
        
        main_layout.addLayout(viz_layout, stretch=3)
        
        # 右側: コントロールパネル（スクロール可能）
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        
        control_panel = self.create_control_panel()
        scroll.setWidget(control_panel)
        
        main_layout.addWidget(scroll, stretch=1)
        
        # ステータスバー
        self.statusBar = QStatusBar()
        self.setStatusBar(self.statusBar)
        self.statusBar.setStyleSheet("""
            QStatusBar {
                background-color: #1e1e1e;
                color: #b0b0b0;
                font-size: 12px;
                border-top: 1px solid #5e5e5e;
            }
        """)
        self.statusBar.showMessage("Ready - Live Mode")

    def create_control_panel(self) -> QWidget:
        """コントロールパネルを作成"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # モード選択
        mode_group = QGroupBox("Mode Selection")
        mode_layout = QVBoxLayout()
        
        self.live_btn = QPushButton("🔴 Live Mode")
        self.live_btn.clicked.connect(lambda: self.switch_mode("live"))
        self.live_btn.setStyleSheet("""
            QPushButton {
                background-color: #c62828;
                color: #ffffff;
                font-size: 15px;
                padding: 15px;
            }
            QPushButton:hover {
                background-color: #d32f2f;
            }
        """)
        
        self.playback_btn = QPushButton("▶️ Playback Mode")
        self.playback_btn.clicked.connect(lambda: self.switch_mode("playback"))
        self.playback_btn.setStyleSheet("font-size: 15px; padding: 15px;")
        
        mode_layout.addWidget(self.live_btn)
        mode_layout.addWidget(self.playback_btn)
        mode_group.setLayout(mode_layout)
        layout.addWidget(mode_group)
        
        # 記録制御
        record_group = QGroupBox("Recording Control")
        record_layout = QVBoxLayout()
        
        self.start_record_btn = QPushButton("⏺️ Start Recording")
        self.start_record_btn.clicked.connect(self.start_recording)
        self.start_record_btn.setStyleSheet("font-size: 15px; padding: 15px;")
        
        self.stop_record_btn = QPushButton("⏹️ Stop Recording")
        self.stop_record_btn.clicked.connect(self.stop_recording)
        self.stop_record_btn.setEnabled(False)
        self.stop_record_btn.setStyleSheet("font-size: 15px; padding: 15px;")
        
        self.recording_status_label = QLabel("Status: Not Recording")
        self.recording_status_label.setStyleSheet("font-size: 14px; color: #b0b0b0;")
        
        record_layout.addWidget(self.start_record_btn)
        record_layout.addWidget(self.stop_record_btn)
        record_layout.addWidget(self.recording_status_label)
        record_group.setLayout(record_layout)
        layout.addWidget(record_group)
        
        # 再生制御
        playback_group = QGroupBox("Playback Control")
        playback_layout = QVBoxLayout()
        
        self.load_csv_btn = QPushButton("📁 Load CSV")
        self.load_csv_btn.clicked.connect(self.load_csv_file)
        self.load_csv_btn.setStyleSheet("font-size: 15px; padding: 15px;")
        
        playback_controls = QHBoxLayout()
        self.play_btn = QPushButton("▶️")
        self.play_btn.clicked.connect(self.toggle_playback)
        self.play_btn.setEnabled(False)
        self.play_btn.setStyleSheet("font-size: 18px;")
        
        self.stop_play_btn = QPushButton("⏹️")
        self.stop_play_btn.clicked.connect(self.stop_playback)
        self.stop_play_btn.setEnabled(False)
        self.stop_play_btn.setStyleSheet("font-size: 18px;")
        
        playback_controls.addWidget(self.play_btn)
        playback_controls.addWidget(self.stop_play_btn)
        
        self.speed_combo = QComboBox()
        self.speed_combo.addItems(["0.5x", "1x", "2x", "5x"])
        self.speed_combo.setCurrentText("1x")
        self.speed_combo.currentTextChanged.connect(self.change_playback_speed)
        
        self.playback_slider = QSlider(Qt.Horizontal)
        self.playback_slider.setEnabled(False)
        self.playback_slider.valueChanged.connect(self.seek_playback)
        
        playback_layout.addWidget(self.load_csv_btn)
        playback_layout.addLayout(playback_controls)
        playback_layout.addWidget(QLabel("Speed:"))
        playback_layout.addWidget(self.speed_combo)
        playback_layout.addWidget(QLabel("Timeline:"))
        playback_layout.addWidget(self.playback_slider)
        playback_group.setLayout(playback_layout)
        layout.addWidget(playback_group)
        
        # アンカー設定（タブで切り替え）
        anchor_group = QGroupBox("Anchor Configuration")
        anchor_main_layout = QVBoxLayout()
        
        # アンカー数設定
        num_anchor_layout = QHBoxLayout()
        num_label = QLabel("Number of Anchors:")
        num_label.setStyleSheet("font-size: 13px; color: #e0e0e0; font-weight: bold;")
        num_anchor_layout.addWidget(num_label)
        self.num_anchor_spin = QSpinBox()
        self.num_anchor_spin.setMinimum(3)
        self.num_anchor_spin.setMaximum(10)
        self.num_anchor_spin.setValue(3)
        self.num_anchor_spin.valueChanged.connect(self.on_num_anchors_changed)
        num_anchor_layout.addWidget(self.num_anchor_spin)
        anchor_main_layout.addLayout(num_anchor_layout)
        
        # タブウィジェット
        self.anchor_tabs = QTabWidget()
        
        # タブ1: 距離指定
        self.distance_tab = QWidget()
        self.distance_tab_layout = QVBoxLayout(self.distance_tab)
        self.create_distance_inputs()
        self.anchor_tabs.addTab(self.distance_tab, "📏 Distance Input")
        
        # タブ2: 座標直接指定
        self.coordinate_tab = QWidget()
        self.coordinate_tab_layout = QVBoxLayout(self.coordinate_tab)
        self.create_coordinate_inputs()
        self.anchor_tabs.addTab(self.coordinate_tab, "📍 Coordinate Input")
        
        anchor_main_layout.addWidget(self.anchor_tabs)
        anchor_group.setLayout(anchor_main_layout)
        layout.addWidget(anchor_group)
        
        # データ表示
        self.data_group = QGroupBox("Current Data")
        self.create_data_display()
        layout.addWidget(self.data_group)
        
        # クリアボタン
        clear_btn = QPushButton("🗑️ Clear Trajectory")
        clear_btn.clicked.connect(self.canvas.clear_trajectory)
        clear_btn.setStyleSheet("font-size: 14px; padding: 12px;")
        layout.addWidget(clear_btn)
        
        layout.addStretch()
        
        return panel

    def create_distance_inputs(self):
        """距離入力UIを作成"""
        layout = QGridLayout()
        
        # 説明ラベル
        info_label = QLabel("Enter distances between anchors (meters):")
        info_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #81d4fa;")
        layout.addWidget(info_label, 0, 0, 1, 3)
        
        self.distance_inputs = {}
        row = 1
        
        # 初期アンカー数に応じて距離入力を作成
        for i in range(self.num_anchors):
            for j in range(i + 1, self.num_anchors):
                label = QLabel(f"TWR{i} ↔ TWR{j}:")
                label.setStyleSheet("font-size: 12px; color: #e0e0e0;")
                distance_input = QLineEdit("3.0")
                distance_input.setPlaceholderText("meters")
                
                layout.addWidget(label, row, 0)
                layout.addWidget(distance_input, row, 1)
                
                self.distance_inputs[(i, j)] = distance_input
                row += 1
        
        # 計算ボタン
        calc_btn = QPushButton("🔢 Calculate Positions")
        calc_btn.clicked.connect(self.calculate_from_distances)
        calc_btn.setStyleSheet("font-size: 14px; padding: 12px;")
        layout.addWidget(calc_btn, row, 0, 1, 2)
        
        self.distance_tab_layout.addLayout(layout)
        self.distance_tab_layout.addStretch()

    def create_coordinate_inputs(self):
        """座標入力UIを作成"""
        layout = QGridLayout()
        
        # 説明ラベル
        info_label = QLabel("Enter anchor coordinates directly:")
        info_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #81d4fa;")
        layout.addWidget(info_label, 0, 0, 1, 7)
        
        self.coordinate_inputs = {}
        
        for i in range(self.num_anchors):
            anchor_name = f"TWR{i}"
            label = QLabel(f"{anchor_name}:")
            label.setStyleSheet("font-size: 12px; font-weight: bold; color: #e0e0e0;")
            layout.addWidget(label, i + 1, 0)
            
            x_input = QLineEdit("0.0")
            y_input = QLineEdit("0.0")
            z_input = QLineEdit("0.0")
            
            x_label = QLabel("X:")
            y_label = QLabel("Y:")
            z_label = QLabel("Z:")
            x_label.setStyleSheet("color: #b0b0b0;")
            y_label.setStyleSheet("color: #b0b0b0;")
            z_label.setStyleSheet("color: #b0b0b0;")
            
            layout.addWidget(x_label, i + 1, 1)
            layout.addWidget(x_input, i + 1, 2)
            layout.addWidget(y_label, i + 1, 3)
            layout.addWidget(y_input, i + 1, 4)
            layout.addWidget(z_label, i + 1, 5)
            layout.addWidget(z_input, i + 1, 6)
            
            self.coordinate_inputs[anchor_name] = (x_input, y_input, z_input)
        
        # 更新ボタン
        update_btn = QPushButton("🔄 Update Anchors")
        update_btn.clicked.connect(self.update_anchor_positions)
        update_btn.setStyleSheet("font-size: 14px; padding: 12px;")
        layout.addWidget(update_btn, self.num_anchors + 1, 0, 1, 7)
        
        self.coordinate_tab_layout.addLayout(layout)
        self.coordinate_tab_layout.addStretch()

    def create_data_display(self):
        """データ表示UIを作成"""
        layout = QVBoxLayout()
        
        self.position_label = QLabel("Position: N/A")
        self.position_label.setStyleSheet("font-size: 14px; font-weight: bold; color: #81d4fa;")
        self.distance_labels = {}
        
        layout.addWidget(self.position_label)
        
        separator = QLabel("─" * 30)
        separator.setStyleSheet("color: #5e5e5e;")
        layout.addWidget(separator)
        
        for i in range(self.num_anchors):
            label = QLabel(f"TWR{i}: N/A")
            label.setStyleSheet("font-size: 13px; color: #e0e0e0;")
            self.distance_labels[f"TWR{i}"] = label
            layout.addWidget(label)
        
        self.data_group.setLayout(layout)

    def on_num_anchors_changed(self, value):
        """アンカー数が変更されたときの処理"""
        self.num_anchors = value
        
        # 距離入力UIを再生成
        self.clear_layout(self.distance_tab_layout)
        self.create_distance_inputs()
        
        # 座標入力UIを再生成
        self.clear_layout(self.coordinate_tab_layout)
        self.create_coordinate_inputs()
        
        # データ表示を再生成
        self.clear_layout(self.data_group.layout())
        self.create_data_display()

    def clear_layout(self, layout):
        """レイアウトの全ウィジェットを削除"""
        if layout is not None:
            while layout.count():
                item = layout.takeAt(0)
                widget = item.widget()
                if widget is not None:
                    widget.deleteLater()
                else:
                    self.clear_layout(item.layout())

    def calculate_from_distances(self):
        """距離からアンカー位置を計算"""
        try:
            # 距離データを収集
            anchor_distances = {}
            for (i, j), input_widget in self.distance_inputs.items():
                distance = float(input_widget.text())
                anchor_distances[(i, j)] = distance
            
            # 位置を計算
            positions = AnchorPositionCalculator.calculate_positions_from_distances(
                anchor_distances, self.num_anchors
            )
            
            # 座標入力UIに反映
            for anchor_name, (x, y, z) in positions.items():
                if anchor_name in self.coordinate_inputs:
                    x_input, y_input, z_input = self.coordinate_inputs[anchor_name]
                    x_input.setText(f"{x:.2f}")
                    y_input.setText(f"{y:.2f}")
                    z_input.setText(f"{z:.2f}")
            
            # アンカー位置を更新
            self.update_anchor_positions()
            
            QMessageBox.information(
                self, "Success", 
                f"Anchor positions calculated from distances!\n"
                f"{self.num_anchors} anchors positioned in triangle/polygon formation."
            )
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to calculate positions:\n{str(e)}")

    def update_anchor_positions(self):
        """アンカー位置を更新"""
        try:
            new_anchors = {}
            for anchor_name, (x_input, y_input, z_input) in self.coordinate_inputs.items():
                x = float(x_input.text())
                y = float(y_input.text())
                z = float(z_input.text())
                new_anchors[anchor_name] = (x, y, z)
            
            self.trilateration.set_anchor_positions(new_anchors)
            self.canvas.set_anchor_positions(new_anchors)
            
            self.statusBar.showMessage(
                f"Anchor positions updated! ({self.num_anchors} anchors)"
            )
            
        except ValueError:
            QMessageBox.critical(self, "Error", "Invalid anchor coordinates!")

    def ros_data_callback(self, msg):
        """ROS2トピックからデータを受信"""
        try:
            data = json.loads(msg.data)
            self.latest_anchor_data = data
            
            # 距離データを抽出（動的に対応）
            distances = {}
            for i in range(self.num_anchors):
                key = f"TWR{i}"
                if key in data and data[key] is not None:
                    distances[key] = data[key].get('distance')
                else:
                    distances[key] = None
            
            # 多辺測位
            self.latest_position = self.trilateration.calculate_position(distances)
            
            # ログ記録
            if self.csv_logger.is_logging_active():
                self.csv_logger.log_data(time.time(), data, self.latest_position)
            
        except Exception as e:
            self.ros_node.get_logger().error(f"データ処理エラー: {e}")

    def update_visualization(self):
        """可視化を更新"""
        if self.mode == "live" and self.latest_anchor_data:
            # 距離データを抽出
            distances = {}
            for i in range(self.num_anchors):
                key = f"TWR{i}"
                if key in self.latest_anchor_data and self.latest_anchor_data[key] is not None:
                    distances[key] = self.latest_anchor_data[key].get('distance')
                else:
                    distances[key] = None
            
            # 可視化更新
            self.canvas.update_plot(self.latest_position, distances)
            
            # データ表示更新
            if self.latest_position:
                self.position_label.setText(
                    f"Position: X={self.latest_position[0]:.2f}, "
                    f"Y={self.latest_position[1]:.2f}, "
                    f"Z={self.latest_position[2]:.2f}"
                )
            else:
                self.position_label.setText("Position: N/A")
            
            # 距離表示更新
            for i in range(self.num_anchors):
                key = f"TWR{i}"
                if key in self.distance_labels:
                    if key in self.latest_anchor_data and self.latest_anchor_data[key]:
                        data = self.latest_anchor_data[key]
                        self.distance_labels[key].setText(
                            f"{key}: {data['distance']:.2f}m ({data['nlos_los']})"
                        )
                    else:
                        self.distance_labels[key].setText(f"{key}: N/A")

    def switch_mode(self, mode: str):
        """モードを切り替え"""
        self.mode = mode
        if mode == "live":
            self.live_btn.setStyleSheet("""
                QPushButton {
                    background-color: #c62828;
                    color: #ffffff;
                    font-size: 15px;
                    padding: 15px;
                }
                QPushButton:hover {
                    background-color: #d32f2f;
                }
            """)
            self.playback_btn.setStyleSheet("font-size: 15px; padding: 15px;")
            self.statusBar.showMessage("Live Mode")
            self.stop_playback()
        else:
            self.live_btn.setStyleSheet("font-size: 15px; padding: 15px;")
            self.playback_btn.setStyleSheet("""
                QPushButton {
                    background-color: #1565c0;
                    color: #ffffff;
                    font-size: 15px;
                    padding: 15px;
                }
                QPushButton:hover {
                    background-color: #1976d2;
                }
            """)
            self.statusBar.showMessage("Playback Mode")

    def start_recording(self):
        """記録を開始"""
        anchor_positions = self.trilateration.anchor_positions
        
        if self.csv_logger.start_logging(anchor_positions=anchor_positions):
            self.start_record_btn.setEnabled(False)
            self.stop_record_btn.setEnabled(True)
            self.recording_status_label.setText("Status: 🔴 Recording...")
            self.recording_status_label.setStyleSheet("color: #ff5252; font-weight: bold; font-size: 14px;")
            self.statusBar.showMessage("Recording started")

    def stop_recording(self):
        """記録を停止"""
        self.csv_logger.stop_logging()
        self.start_record_btn.setEnabled(True)
        self.stop_record_btn.setEnabled(False)
        self.recording_status_label.setText("Status: Not Recording")
        self.recording_status_label.setStyleSheet("color: #b0b0b0; font-size: 14px;")
        self.statusBar.showMessage("Recording stopped")

    def load_csv_file(self):
        """CSVファイルを読み込み"""
        filepath, _ = QFileDialog.getOpenFileName(
            self, "Open CSV File", "./uwb_logs", "CSV Files (*.csv)"
        )
        
        if filepath:
            if self.csv_player.load_csv(filepath):
                total_frames = self.csv_player.get_total_frames()
                self.playback_slider.setMaximum(total_frames - 1)
                self.playback_slider.setEnabled(True)
                self.play_btn.setEnabled(True)
                self.stop_play_btn.setEnabled(True)
                
                # アンカー位置を復元
                anchors = self.csv_player.get_anchor_positions()
                if anchors:
                    # アンカー数を更新
                    self.num_anchor_spin.setValue(len(anchors))
                    
                    self.trilateration.set_anchor_positions(anchors)
                    self.canvas.set_anchor_positions(anchors)
                    
                    # 座標入力UIを更新
                    for anchor_name, (x, y, z) in anchors.items():
                        if anchor_name in self.coordinate_inputs:
                            x_input, y_input, z_input = self.coordinate_inputs[anchor_name]
                            x_input.setText(str(x))
                            y_input.setText(str(y))
                            z_input.setText(str(z))
                    
                    self.statusBar.showMessage(
                        f"Loaded: {filepath} ({total_frames} frames, {len(anchors)} anchors)"
                    )
                else:
                    self.statusBar.showMessage(f"Loaded: {filepath} ({total_frames} frames)")
                
                QMessageBox.information(self, "Success", f"Loaded {total_frames} frames")
            else:
                QMessageBox.critical(self, "Error", "Failed to load CSV file")

    def toggle_playback(self):
        """再生/一時停止をトグル"""
        if self.is_playing:
            self.playback_timer.stop()
            self.play_btn.setText("▶️")
            self.is_playing = False
        else:
            interval = int(50 / self.playback_speed)
            self.playback_timer.start(interval)
            self.play_btn.setText("⏸️")
            self.is_playing = True

    def stop_playback(self):
        """再生を停止"""
        self.playback_timer.stop()
        self.play_btn.setText("▶️")
        self.is_playing = False
        self.csv_player.reset()
        self.playback_slider.setValue(0)
        self.canvas.clear_trajectory()

    def playback_step(self):
        """再生を1ステップ進める"""
        data = self.csv_player.get_next_data()
        if data:
            distances = {}
            for i in range(self.num_anchors):
                key = f"TWR{i}"
                if key in data['anchor_data'] and data['anchor_data'][key]:
                    distances[key] = data['anchor_data'][key]['distance']
                else:
                    distances[key] = None
            
            self.canvas.update_plot(data['position'], distances)
            self.playback_slider.setValue(self.csv_player.current_index)
        else:
            self.stop_playback()

        
    def seek_playback(self, value):
        """再生位置をシーク"""
        if not self.is_playing:
            self.csv_player.seek(value)
            data = self.csv_player.get_data_at_index(value)
            if data:
                distances = {}
                for i in range(self.num_anchors):
                    key = f"TWR{i}"
                    if key in data['anchor_data'] and data['anchor_data'][key]:
                        distances[key] = data['anchor_data'][key]['distance']
                    else:
                        distances[key] = None
                
                self.canvas.update_plot(data['position'], distances)

    def change_playback_speed(self, speed_text):
        """再生速度を変更"""
        speed_map = {"0.5x": 0.5, "1x": 1.0, "2x": 2.0, "5x": 5.0}
        self.playback_speed = speed_map.get(speed_text, 1.0)
        
        if self.is_playing:
            interval = int(50 / self.playback_speed)
            self.playback_timer.setInterval(interval)

    def closeEvent(self, event):
        """ウィンドウを閉じる時の処理"""
        if self.csv_logger.is_logging_active():
            self.csv_logger.stop_logging()
        event.accept()


class ROS2GUINode(Node):
    """ROS2ノードラッパー"""
    
    def __init__(self):
        super().__init__('uwb_multilateration_gui')
        self.get_logger().info('UWB Multilateration GUI Node started')


def main(args=None):
    # ROS2初期化
    rclpy.init(args=args)
    
    # Qt Application
    app = QApplication(sys.argv)
    
    # ROS2ノード作成
    ros_node = ROS2GUINode()
    
    # GUIウィンドウ作成
    window = UWBGUINode(ros_node)
    window.show()
    
    # ROS2スピンをタイマーで実行
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    timer.start(10)  # 100Hz
    
    # Qt実行
    exit_code = app.exec_()
    
    # クリーンアップ
    ros_node.destroy_node()
    rclpy.shutdown()
    
    sys.exit(exit_code)


if __name__ == '__main__':
    main()