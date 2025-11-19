# UWB Anchor Configuration Guide

このガイドでは、UWBアンカーの設定方法と、動的なアンカー数のサポート機能について説明します。

## 主な変更点

1. **動的なアンカー数のサポート**: 3個のアンカーに限定されなくなり、任意の数のアンカーを設定できます（最小3個）
2. **高さパラメータの追加**: アンカーとタグの地面からの高さを設定でき、より正確な3D測位が可能
3. **GUIでのアンカー数調整**: RVizでアンカーを配置する際に、GUI上でアンカー数を増減できます

## アンカー設置位置の設定

### 方法1: RVizを使った対話的な設定（推奨）

1. 以下のコマンドでアンカー設置ツールを起動：

```bash
ros2 launch tk_uwb_ekf rviz_anchor_place.launch.py
```

パラメータ:
- `num_anchors`: 初期アンカー数（デフォルト: 3）
- `anchor_height`: アンカーの高さ [m]（デフォルト: 0.0）
- `tag_height`: タグの高さ [m]（デフォルト: 0.0）

例:
```bash
ros2 launch tk_uwb_ekf rviz_anchor_place.launch.py \
  num_anchors:=5 \
  anchor_height:=1.5 \
  tag_height:=0.3
```

2. GUIウィンドウが表示されます：
   - **+/-ボタン**: アンカー数を増減
   - **Reset**: 配置したアンカーをリセット
   - **Save Anchors**: 設定を保存

3. RVizで「Publish Point」ツールを使用して、地図上でアンカー位置をクリック

4. すべてのアンカーを配置したら「Save Anchors」ボタンをクリック

5. 保存後、`colcon build`を実行して設定を反映

### 方法2: 手動でYAMLファイルを編集

`tk_uwb_ekf/param/anchors.yaml` を直接編集：

```yaml
anchors:
- name: A0
  x: 0.0
  y: 0.0
- name: A1
  x: 5.0
  y: 0.0
- name: A2
  x: 2.5
  y: 4.0
- name: A3
  x: 7.0
  y: 3.0
- name: A4
  x: 3.0
  y: 7.0
anchor_height: 1.5  # アンカーの高さ [m]
tag_height: 0.3     # タグの高さ [m]
```

編集後、`colcon build`を実行して設定を反映してください。

## ナビゲーションの起動

### 基本的な起動

```bash
ros2 launch tk_uwb_ekf uwb_nav.launch.py
```

### 高さパラメータを指定して起動

```bash
ros2 launch tk_uwb_ekf uwb_nav.launch.py \
  anchor_height:=1.5 \
  tag_height:=0.3
```

このパラメータは、`anchors.yaml`ファイルの設定を上書きします。

## パラメータの詳細

### uwb_nav.launch.py

| パラメータ | デフォルト値 | 説明 |
|-----------|------------|------|
| `use_sim_time` | false | シミュレーション時刻を使用するか |
| `map` | maps/map.yaml | 地図ファイルのパス |
| `params_file` | param/nav2_params_uwb.yaml | Nav2パラメータファイル |
| `anchor_height` | 0.0 | アンカーの地面からの高さ [m] |
| `tag_height` | 0.0 | タグの地面からの高さ [m] |

### rviz_anchor_place.launch.py

| パラメータ | デフォルト値 | 説明 |
|-----------|------------|------|
| `num_anchors` | 3 | 初期アンカー数（最小3個） |
| `anchor_height` | 0.0 | アンカーの地面からの高さ [m] |
| `tag_height` | 0.0 | タグの地面からの高さ [m] |

## 高さパラメータの効果

アンカーとタグに高さの差がある場合、2D距離だけでなく3D距離を考慮した測位が行われます。

例：
- アンカー高さ: 1.5m
- タグ高さ: 0.3m
- 平面距離: 2.0m
- 実際のUWB距離: √(2.0² + (1.5-0.3)²) ≈ 2.33m

この補正により、より正確な自己位置推定が可能になります。

## トラブルシューティング

### アンカー数を変更しても反映されない

`colcon build`を実行して、パッケージを再ビルドしてください。

### GUIでアンカー数を減らせない

最小アンカー数は3個です。これ以下には設定できません。

### 既存の設定との互換性

後方互換性を保つため、`anchors.yaml`に高さ情報がない場合は、デフォルト値（0.0）が使用されます。
