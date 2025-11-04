# UWB Navigation - Usage Examples

このドキュメントでは、UWBアンカー設定の具体的な使用例を紹介します。

## 例1: デフォルト設定（3アンカー）

既存の設定と完全に互換性があります。

```bash
# アンカー配置ツールの起動
ros2 launch tk_uwb_ekf rviz_anchor_place.launch.py

# ナビゲーションの起動
ros2 launch tk_uwb_ekf uwb_nav.launch.py
```

## 例2: 5つのアンカーを使用

```bash
# アンカー配置ツールの起動（5アンカー）
ros2 launch tk_uwb_ekf rviz_anchor_place.launch.py num_anchors:=5

# RVizで5つのアンカー位置をクリックして配置
# GUIの「Save Anchors」ボタンで保存

# colcon buildで設定を反映
cd ~/ros2_ws
colcon build --packages-select tk_uwb_ekf

# ナビゲーションの起動
ros2 launch tk_uwb_ekf uwb_nav.launch.py
```

## 例3: アンカーとタグの高さを指定

天井にアンカーを設置し、ロボットにタグを取り付ける場合：

```bash
# アンカー配置（高さ情報付き）
ros2 launch tk_uwb_ekf rviz_anchor_place.launch.py \
  num_anchors:=4 \
  anchor_height:=2.5 \
  tag_height:=0.5

# 配置後、保存してcolcon build

# ナビゲーションの起動（高さパラメータ付き）
ros2 launch tk_uwb_ekf uwb_nav.launch.py \
  anchor_height:=2.5 \
  tag_height:=0.5
```

## 例4: 広いエリアに10個のアンカーを配置

```bash
# 10アンカーでの配置
ros2 launch tk_uwb_ekf rviz_anchor_place.launch.py \
  num_anchors:=10 \
  anchor_height:=2.0 \
  tag_height:=0.3

# RVizで10箇所のアンカー位置をクリック
# GUIで必要に応じて+/-ボタンでアンカー数を調整可能
# 保存してcolcon build

# ナビゲーション起動
ros2 launch tk_uwb_ekf uwb_nav.launch.py \
  anchor_height:=2.0 \
  tag_height:=0.3
```

## GUIの使い方

### アンカー配置GUIの機能

起動すると以下のようなGUIウィンドウが表示されます：

```
┌──────────────────────────────────────┐
│       Anchor Setter                  │
├──────────────────────────────────────┤
│                                      │
│  Placed 3 / 5 anchors.               │
│  Click in RViz to place Anchor 3.    │
│                                      │
├──────────────────────────────────────┤
│  Number of Anchors:  [-] [5] [+]     │
├──────────────────────────────────────┤
│  Anchor Height: 1.5m | Tag Height: 0.3m │
├──────────────────────────────────────┤
│           [Save Anchors]             │
│           [Reset]                    │
└──────────────────────────────────────┘
```

### ボタンの説明

- **[-]**: アンカー数を1つ減らす（最小3個まで）
- **[+]**: アンカー数を1つ増やす
- **Save Anchors**: 配置したアンカー位置をYAMLファイルに保存
- **Reset**: 配置したアンカーをすべてクリア

### RVizでの操作

1. RVizの上部ツールバーから「Publish Point」ツールを選択
2. 地図上でアンカーを配置したい位置をクリック
3. クリックした位置に赤い球体のマーカーが表示される
4. GUIで設定した数のアンカーを配置し終えたら「Save Anchors」をクリック

## YAMLファイルの形式

### 基本形式（後方互換）

```yaml
anchors:
- name: A0
  x: 0.0
  y: 0.0
- name: A1
  x: 8.4
  y: 0.0
- name: A2
  x: 7.29
  y: 4.57
```

### 高さパラメータ付き

```yaml
anchors:
- name: A0
  x: 0.0
  y: 0.0
- name: A1
  x: 8.4
  y: 0.0
- name: A2
  x: 7.29
  y: 4.57
- name: A3
  x: 5.0
  y: 5.0
- name: A4
  x: 10.0
  y: 3.0
anchor_height: 2.5  # 全アンカーの高さ [m]
tag_height: 0.5     # タグの高さ [m]
```

### 個別のz座標指定（高度な使用）

```yaml
anchors:
- name: A0
  x: 0.0
  y: 0.0
  z: 2.0  # この高さを使用
- name: A1
  x: 8.4
  y: 0.0
  z: 2.5  # 異なる高さも指定可能
- name: A2
  x: 7.29
  y: 4.57
  z: 2.3
anchor_height: 2.5  # 個別z座標がない場合のデフォルト
tag_height: 0.5
```

## トラブルシューティング

### 問題: アンカー数を変更しても反映されない

**解決策**: 設定変更後は必ず `colcon build` を実行してください。

```bash
cd ~/ros2_ws
colcon build --packages-select tk_uwb_ekf
source install/setup.bash
```

### 問題: GUIでアンカー数を3以下に減らせない

**原因**: 多辺測位には最低3つのアンカーが必要です。

**解決策**: これは正常な動作です。最小3個のアンカーは必須です。

### 問題: 高さパラメータが効いているか確認したい

**確認方法**: ノードを起動したときのログを確認してください。

```bash
ros2 launch tk_uwb_ekf uwb_nav.launch.py anchor_height:=1.5 tag_height:=0.3
```

ログに以下のような出力が表示されます：

```
[uwb_ekf_node]: Number of anchors: 5
[uwb_ekf_node]: Anchor height: 1.5m, Tag height: 0.3m
[uwb_ekf_node]: Anchor positions: {'a': [0.0, 0.0, 1.5], 'b': [5.0, 0.0, 1.5], ...}
```

### 問題: 既存のanchors.yamlとの互換性

**解決策**: 既存のanchors.yamlファイルは自動的に認識されます。height パラメータがない場合は、デフォルト値（0.0）が使用されます。

## ベストプラクティス

1. **アンカー数の決定**: 部屋のサイズに応じて適切な数を選択
   - 小部屋（~20㎡）: 3-4個
   - 中部屋（20-50㎡）: 4-6個
   - 大部屋（50㎡~）: 6-10個以上

2. **アンカー配置**: 
   - なるべく部屋の角や壁沿いに配置
   - ロボットの動作範囲をカバーするように配置
   - 三角形や四角形の頂点に配置すると精度が向上

3. **高さの設定**:
   - アンカーを天井近くに設置する場合、正確な高さを測定
   - タグの高さもロボットの実際の取り付け位置を測定
   - 高さの差が大きいほど、3D補正の効果が大きくなる
