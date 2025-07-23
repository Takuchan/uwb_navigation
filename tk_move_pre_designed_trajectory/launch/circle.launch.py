from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tk_move_pre_designed_trajectory',
            executable='circle_node',
            name='circle_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'linear_velocity': 1.5},
                {'angular_velocity_limit': 2.0},
                {'frequency': 20.0},
                {'radius': 10.0},
                {'laps': 2.0}
            ]
        )
    ])
