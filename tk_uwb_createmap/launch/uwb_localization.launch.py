import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    
    # --- アンカー間距離のLaunch引数を宣言 ---
    declare_d01_arg = DeclareLaunchArgument(
        'd01', default_value='7.12',
        description='Distance between anchor 0 and 1 in meters.'
    )
    declare_d12_arg = DeclareLaunchArgument(
        'd12', default_value='1.66',
        description='Distance between anchor 1 and 2 in meters.'
    )
    declare_d02_arg = DeclareLaunchArgument(
        'd02', default_value='7.12',
        description='Distance between anchor 0 and 2 in meters.'
    )
    
    # シリアルポート設定
    declare_com_port_arg = DeclareLaunchArgument(
        'com_port', default_value='/dev/ttyUSB0',
        description='Serial port for UWB communication.'
    )
    declare_baud_rate_arg = DeclareLaunchArgument(
        'baud_rate', default_value='3000000',
        description='Baud rate for serial communication.'
    )

    # UWBタグの位置をロボットのベースリンクからの静的TFとして設定
    static_tf_tag = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_tag',
        output='screen',
        arguments=['0.1', '0.0', '0.2', '0', '0', '0', 'base_link', 'uwb_tag']
    )


    # UWBローカライゼーションノード
    uwb_localizer_node = Node(
        package='tk_uwb_createmap',
        executable='uwb_localizer_node',
        name='uwb_localizer_node',
        output='screen',
        parameters=[
            {
                # Launch引数で受け取った値をノードのパラメータとして渡す
                'd01': LaunchConfiguration('d01'),
                'd12': LaunchConfiguration('d12'),
                'd02': LaunchConfiguration('d02'),
                'com_port': LaunchConfiguration('com_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'num_anchors': 3,
                'ekf_dt': 0.1,
                'fov_angle': 120.0,  # 視野角 [度]
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'uwb_tag_frame': 'uwb_tag',
                'publish_frequency': 10.0
            }
        ]
    )
    

    return LaunchDescription([
        declare_d01_arg,
        declare_d12_arg,
        declare_d02_arg,
        declare_com_port_arg,
        declare_baud_rate_arg,
        static_tf_tag,
        uwb_localizer_node,
    ])