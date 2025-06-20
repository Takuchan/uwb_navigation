from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'radius',
            default_value='2.0',
            description='Radius of circle in meters'
        ),
        DeclareLaunchArgument(
            'num_rotations',
            default_value='2',
            description='Number of full rotations'
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
            executable='circle_trajectory',
            name='circle_trajectory_node',
            parameters=[{
                'radius': LaunchConfiguration('radius'),
                'num_rotations': LaunchConfiguration('num_rotations'),
                'linear_velocity': LaunchConfiguration('linear_velocity'),
                'angular_velocity_limit': LaunchConfiguration('angular_velocity_limit'),
                'frequency': LaunchConfiguration('frequency'),
            }],
            output='screen'
        )
    ])
