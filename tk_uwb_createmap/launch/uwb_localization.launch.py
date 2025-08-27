# launch/localization.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('tk_uwb_createmap') # ★自分のパッケージ名に変更

    # robot_localizationの設定ファイルパス
    ekf_config_path = os.path.join(pkg_share, 'config/ekf.yaml')

    # UWBマッパーノードの起動設定
    uwb_mapper_node = Node(
        package='tk_uwb_createmap', # ★自分のパッケージ名に変更
        executable='uwb_mapper',     # ★setup.pyで設定した実行可能ファイル名
        name='uwb_grid_mapper_node',
        output='screen'
    )

    # robot_localizationノードの起動設定
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )

    return LaunchDescription([
        uwb_mapper_node,
        robot_localization_node
    ])