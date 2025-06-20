from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('semi_major_axis', default_value='2.0'),
        DeclareLaunchArgument('semi_minor_axis', default_value='1.0'),
        DeclareLaunchArgument('num_rotations', default_value='2'),
        DeclareLaunchArgument('parameter_speed', default_value='0.5'),
        DeclareLaunchArgument('linear_velocity_limit', default_value='1.0'),
        DeclareLaunchArgument('angular_velocity_limit', default_value='1.0'),
        DeclareLaunchArgument('frequency', default_value='20.0'),
        
        Node(
            package='tk_move_pre_designed_trajectory',
            executable='ellipse_trajectory',
            name='ellipse_trajectory_node',
            parameters=[{
                'semi_major_axis': LaunchConfiguration('semi_major_axis'),
                'semi_minor_axis': LaunchConfiguration('semi_minor_axis'),
                'num_rotations': LaunchConfiguration('num_rotations'),
                'parameter_speed': LaunchConfiguration('parameter_speed'),
                'linear_velocity_limit': LaunchConfiguration('linear_velocity_limit'),
                'angular_velocity_limit': LaunchConfiguration('angular_velocity_limit'),
                'frequency': LaunchConfiguration('frequency'),
            }],
            output='screen'
        )
    ])
