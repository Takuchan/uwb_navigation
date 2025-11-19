# UWB Data Processing Architecture

## Overview
UWBデータの処理は以下の3つのノードで構成されています。

## Data Flow

```
Serial Port (UWB Device)
    ↓
[serialAndFliterPublisher]
    ↓ publishes to: /uwb_data_raw
[uwb_divergence_correction]
    ↓ publishes to: /uwb_data_json
[uwb_ekf_node] & [uwb_gui_node]
```

## Node Descriptions

### 1. serialAndFliterPublisher
- **Purpose**: シリアルポートからUWBデバイスの生データを読み取る
- **Publishes**: `/uwb_data_raw` (String/JSON)
- **Description**: UWBデバイスからの生の距離・角度データを読み取り、JSONフォーマットで配信

### 2. uwb_divergence_correction (NEW)
- **Purpose**: UWBデータの揺れを補正する
- **Subscribes**: `/uwb_data_raw`
- **Publishes**: `/uwb_data_json`
- **Description**: 分散ベースのフィルタリングを適用し、不安定なデータを除外
- **Parameters**:
  - `history_size` (default: 5): 履歴サイズ
  - `variance_threshold` (default: 0.05): 分散の閾値

### 3. uwb_ekf_node
- **Purpose**: EKFを使用した自己位置推定
- **Subscribes**: `/uwb_data_json`, `/odom`
- **Publishes**: `/odometry/filtered`
- **Description**: フィルタリング済みのUWBデータとオドメトリを使用してEKFで位置推定
- **Note**: 以前はこのノードで分散フィルタリングを行っていたが、現在は`uwb_divergence_correction`に分離

### 4. uwb_gui_node
- **Purpose**: UWBデータの可視化
- **Subscribes**: `/uwb_data_json`
- **Description**: フィルタリング済みのUWBデータをGUIで表示

## Launch

```bash
ros2 launch tk_uwb_ekf ekf_with_serial.launch.py
```

このコマンドで以下のノードが起動します:
- serialAndFliterPublisher
- uwb_divergence_correction
- uwb_ekf_node

## Configuration

### Variance Filtering Parameters
`uwb_divergence_correction`のパラメータは`ekf_with_serial.launch.py`で設定可能:

```python
uwb_divergence_correction_node = Node(
    package='tk_uwb_ekf',
    executable='uwb_divergence_correction',
    parameters=[
        {'history_size': 5},           # 履歴サイズ
        {'variance_threshold': 0.05}   # 分散の閾値
    ]
)
```

## Benefits of This Architecture

1. **Separation of Concerns**: データ取得、フィルタリング、位置推定が分離され、各モジュールの責務が明確
2. **Reusability**: フィルタリング済みのデータを複数のノード（EKF、GUI等）で利用可能
3. **Maintainability**: フィルタリングロジックの変更が容易
4. **Testability**: 各ノードを独立してテスト可能
