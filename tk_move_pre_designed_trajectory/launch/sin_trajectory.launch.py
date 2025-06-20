from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'num_waves',
            default_value='3',
            description='Number of sin waves to generate'
        ),
        DeclareLaunchArgument(
            'period_length',
            default_value='10.0',
            description='Length of one period in seconds'
        ),
        DeclareLaunchArgument(
            'amplitude',
            default_value='1.0',
            description='Amplitude of sin wave'
        ),
        DeclareLaunchArgument(
            'linear_velocity',
            default_value='0.5',
            description='Linear velocity in m/s'
        ),
        DeclareLaunchArgument(
            'angular_velocity_limit',
            default_value='1.0',
            description='Maximum allowed angular velocity in rad/s'
        ),
        DeclareLaunchArgument(
            'frequency',
            default_value='10.0',
            description='Control frequency in Hz'
        ),
        
        Node(
            package='tk_move_pre_designed_trajectory',
            executable='sin_trajectory',
            name='sin_trajectory_node',
            parameters=[{
                'num_waves': LaunchConfiguration('num_waves'),
                'period_length': LaunchConfiguration('period_length'),
                'amplitude': LaunchConfiguration('amplitude'),
                'linear_velocity': LaunchConfiguration('linear_velocity'),
                'angular_velocity_limit': LaunchConfiguration('angular_velocity_limit'),
                'frequency': LaunchConfiguration('frequency'),
            }],
            output='screen'
        )
    ])
