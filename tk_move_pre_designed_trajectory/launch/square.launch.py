from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tk_move_pre_designed_trajectory',
            executable='square_node',
            name='square_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'linear_velocity': 1.0},
                {'angular_velocity_limit': 1.0},
                {'frequency': 1000.0},
                {'side_length': 3.0},
                {'laps': 1.0},
                {'turn_angular_velocity': 0.9},
                {'stop_duration': 2.0}
            ]
        )
    ])
