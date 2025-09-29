from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tk_uwb_ekf',
            # 実行するスクリプト名を変更
            executable='uwb_ekf_node',
            name='uwb_ekf_node',
            output='screen',
            emulate_tty=True, # シリアルからのprint出力をコンソールに表示するために推奨
            parameters=[
                # シリアル通信パラメータ
                {'com_port': '/dev/ttyUSB1'},
                {'baud_rate': 3000000},
                {'num_anchors': 3},
                {'uwb_timeout': 0.04}, # タイマー周期(0.05s)より少し短く設定
                
                # アンカー座標
                {'anchor_a_pos': [0.0, 5.0]}, # TWR0
                {'anchor_b_pos': [5.0, 5.0]}, # TWR1
                {'anchor_c_pos': [2.5, 0.0]}, # TWR2
            ]
        )
    ])
