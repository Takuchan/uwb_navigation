from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tk_uwb_ekf',
            executable='uwb_ekf_node',
            name='uwb_ekf_node',
            output='screen',
            emulate_tty=True, # シリアルからのprint出力をコンソールに表示するために推奨
            parameters=[
                {'com_port': '/dev/ttyUSB1'},
                {'baud_rate': 3000000},
                {'num_anchors': 3},
                {'uwb_timeout': 0.05},
                
                {'anchor_a_pos': [0.0, 0.0]},    # TWR0 (Anchor 0)
                {'anchor_b_pos': [8.4, 0.0]},    # TWR1 (Anchor 1)
                {'anchor_c_pos': [7.29, 4.57]},  # TWR2 (Anchor 2)
            ]
        )
    ])