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
    num_anchors = LaunchConfiguration('num_anchors', default='3')
    anchor_height = LaunchConfiguration('anchor_height', default='0.0')
    tag_height = LaunchConfiguration('tag_height', default='0.0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'num_anchors',
            default_value='3',
            description='Initial number of anchors to place'
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
                {'num_anchors': num_anchors},
                {'save_path': default_save_path},
                {'anchor_height': anchor_height},
                {'tag_height': tag_height}
            ]
        )
    ])