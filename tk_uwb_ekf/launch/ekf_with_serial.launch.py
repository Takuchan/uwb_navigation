import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # パッケージの共有ディレクトリへのパスを取得
    tk_uwb_ekf_dir = get_package_share_directory('tk_uwb_ekf')
    # anchors.yaml ファイルへのフルパスを構築
    anchor_config_file = os.path.join(tk_uwb_ekf_dir, 'param', 'anchors.yaml')

    if not os.path.exists(anchor_config_file):
            error_message = f"❌ 必須ファイルが見つかりません: {anchor_config_file}"
            print("="*60)
            print(f"\033[91m{error_message}\033[0m")
            print("Launchファイルを実行する前に、'tk_uwb_ekf/param/anchors.yaml' を設置し、colcon buildしてください。")
            print("="*60)
            # Launch処理を強制終了させるために例外を発生させる
            raise FileNotFoundError(error_message)

    # 【必須】必ず実行前にデフォルト値の確認をする。特にTag
    default_anchor_height = 0.0
    default_tag_height = 0.6
    final_anchor_positions = {}


    with open(anchor_config_file, 'r') as f:
        anchor_data = yaml.safe_load(f)
    
    anchors_list = anchor_data.get('anchors', [])
    num_anchors = len(anchors_list)
    
    # 高さ情報を取得（オプション）
    anchor_height_from_yaml = anchor_data.get('anchor_height', default_anchor_height)
    tag_height_from_yaml = anchor_data.get('tag_height', default_tag_height)
    

    anchor_params = {}
    for i, anchor in enumerate(anchors_list):
        param_name = f'anchor_{chr(97 + i)}_pos' 
        # YAMLファイルにz座標があればそれを使用、なければanchor_heightを使用
        if 'z' in anchor:
            anchor_params[param_name] = [anchor['x'], anchor['y'], anchor['z']]
        else:
            anchor_params[param_name] = [anchor['x'], anchor['y']]
        
    final_anchor_positions = anchor_params
    
    
    # Launch引数（uwb_nav.launch.pyからオーバーライド可能）
    anchor_height_arg = LaunchConfiguration('anchor_height', default=str(anchor_height_from_yaml))
    tag_height_arg = LaunchConfiguration('tag_height', default=str(tag_height_from_yaml))

    # --- ノードの定義 ---
    uwb_ekf_node = Node(
        package='tk_uwb_ekf',
        executable='uwb_ekf_node',
        name='uwb_ekf_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'com_port': '/dev/ttyUSB1'},
            {'baud_rate': 3000000},
            {'uwb_timeout': 0.05},
            {'num_anchors': num_anchors},
            # {'anchor_height': anchor_height_arg}, # なんか使えなかったので、直接ノードで編集せよ。
            # {'tag_height': tag_height_arg},
            # final_anchor_positions を展開してパラメータとして渡す
            final_anchor_positions 
        ]
    )
    
    # serialAndFliterPublisher ノードの定義を追加
    serial_publisher_node = Node(
        package='tk_uwb_ekf',
        executable='serialAndFliterPublisher',
        name='serialAndFliterPublisher',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'com_port': '/dev/ttyUSB1'},
            {'baud_rate': 3000000},
            {'uwb_timeout': 0.05},
            {'num_anchors': num_anchors},
        ]
    )

    experiment_gui_node = Node(
        package='tk_uwb_ekf',
        executable='experiment_gui',
        name='experiment_gui',
        output='screen',
        emulate_tty=True
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'anchor_height',
            default_value=str(anchor_height_from_yaml),
            description='Height of UWB anchors from ground (in meters)'
        ),
        DeclareLaunchArgument(
            'tag_height',
            default_value=str(tag_height_from_yaml),
            description='Height of UWB tag from ground (in meters)'
        ),
        uwb_ekf_node,
        serial_publisher_node,
        experiment_gui_node
    ])