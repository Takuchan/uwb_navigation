import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    tk_uwb_ekf_dir = get_package_share_directory('tk_uwb_ekf')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(tk_uwb_ekf_dir, 'maps', 'map.yaml')) 
    params_file = LaunchConfiguration('params_file', default=os.path.join(tk_uwb_ekf_dir, 'param', 'nav2_params_uwb.yaml'))
    anchor_height = LaunchConfiguration('anchor_height', default='0.0')
    tag_height = LaunchConfiguration('tag_height', default='0.0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=map_yaml_file,
            description='Full path to map file to load'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Full path to the ROS2 parameters file to use'
        ),
        DeclareLaunchArgument(
            'anchor_height',
            default_value='0.0',
            description='Height of UWB anchors from ground (in meters)'
        ),
        DeclareLaunchArgument(
            'tag_height',
            default_value='0.0',
            description='Height of UWB tag from ground (in meters)'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tk_uwb_ekf_dir, 'launch', 'ekf_with_serial.launch.py')
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_yaml_file,
                'use_sim_time': use_sim_time,
                'params_file': params_file
            }.items()
        ),
    ])