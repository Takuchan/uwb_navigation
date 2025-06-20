from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_segments', default_value='3'),
        DeclareLaunchArgument('period_length', default_value='10.0'),
        DeclareLaunchArgument('amplitude', default_value='0.5'),
        DeclareLaunchArgument('linear_velocity', default_value='0.2'),
        DeclareLaunchArgument('angular_velocity_limit', default_value='1.5'),
        DeclareLaunchArgument('frequency', default_value='10.0'),
        
        Node(
            package='tk_move_pre_designed_trajectory',
            executable='tan_trajectory',
            name='tan_trajectory_node',
            parameters=[{
                'num_segments': LaunchConfiguration('num_segments'),
                'period_length': LaunchConfiguration('period_length'),
                'amplitude': LaunchConfiguration('amplitude'),
                'linear_velocity': LaunchConfiguration('linear_velocity'),
                'angular_velocity_limit': LaunchConfiguration('angular_velocity_limit'),
                'frequency': LaunchConfiguration('frequency'),
            }],
            output='screen'
        )
    ])
