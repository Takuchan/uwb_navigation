from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('side_length', default_value='1.0'),
        DeclareLaunchArgument('linear_velocity', default_value='0.2'),
        DeclareLaunchArgument('turn_velocity', default_value='0.5'),
        DeclareLaunchArgument('angular_velocity_limit', default_value='1.0'),
        
        Node(
            package='tk_move_pre_designed_trajectory',
            executable='square_trajectory',
            name='square_trajectory_node',
            parameters=[{
                'side_length': LaunchConfiguration('side_length'),
                'linear_velocity': LaunchConfiguration('linear_velocity'),
                'turn_velocity': LaunchConfiguration('turn_velocity'),
                'angular_velocity_limit': LaunchConfiguration('angular_velocity_limit'),
            }],
            output='screen'
        )
    ])
