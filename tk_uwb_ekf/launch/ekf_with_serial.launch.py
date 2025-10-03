import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # パッケージの共有ディレクトリへのパスを取得
    tk_uwb_ekf_dir = get_package_share_directory('tk_uwb_ekf')
    # anchors.yaml ファイルへのフルパスを構築
    anchor_config_file = os.path.join(tk_uwb_ekf_dir, 'param', 'anchors.yaml')

    # デフォルトのアンカー座標
    default_anchor_positions = {
        'anchor_a_pos': [0.0, 0.0],
        'anchor_b_pos': [8.4, 0.0],
        'anchor_c_pos': [7.29, 4.57],
    }
    
    if os.path.exists(anchor_config_file):
        with open(anchor_config_file, 'r') as f:
            anchor_data = yaml.safe_load(f)
        
        num_anchors = len(anchor_data.get('anchors', []))
        
        anchor_params = {}
        for i, anchor in enumerate(anchor_data.get('anchors', [])):
            param_name = f'anchor_{chr(97 + i)}_pos' 
            anchor_params[param_name] = [anchor['x'], anchor['y']]
            
        final_anchor_positions = anchor_params
    

    else:
        # ファイルが存在しない場合
        num_anchors = 3 # デフォルト値
        final_anchor_positions = default_anchor_positions
        print("アンカーの設置ポイントが保存された anchors.yaml プログラムがありません。")

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
            # final_anchor_positions を展開してパラメータとして渡す
            final_anchor_positions 
        ]
    )

    return LaunchDescription([
        uwb_ekf_node
    ])