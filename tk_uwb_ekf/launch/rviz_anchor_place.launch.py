import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    tk_uwb_ekf_dir = get_package_share_directory('tk_uwb_ekf')
    
    map_file = os.path.join(tk_uwb_ekf_dir, 'maps', 'map.yaml')
    default_save_path = os.path.join(tk_uwb_ekf_dir, 'param', 'anchors.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'yaml_filename': map_file}
            ]
        ),
        
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
        ),

        Node(
            package='tk_uwb_ekf',
            executable='rviz_anchor_place',
            name='rviz_anchor_place',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'num_anchors': 3},
                {'save_path': default_save_path}
            ]
        )
    ])