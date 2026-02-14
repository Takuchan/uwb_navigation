
# tk_uwb_ekf: UWBロバスト自己位置推定

自分が書いた文章→Geminiで校閲。
README松村、最終チェック済み。

## 📖 概要
**「電波の死角でも、止まらない。」**

このプログラムは、UWB（Ultra-Wideband）とホイールオドメトリを互いに補いながら、安定した自己位置推定と自律移動を行うためのパッケージだ。

一番の強みは、**nLOS（非見通し）判定**を利用した動的なUWBの信頼性調節にある。nLOS判定が行われた場合、カルマンフィルタ側でUWBの重みを下げ、ホイールオドメトリを最優先する。これにより、工場や電波の届きにくい死角にロボットが入っても、安定した自律移動を継続できる。

---

## 🧠 アルゴリズムの核心
拡張カルマンフィルタ（EKF）をベースに、非線形な入力に対応している。

* **予測：** ホイールオドメトリを利用。
* **補正：** UWBの距離データを利用。
* **3D補正：** UWBの生データ（斜め距離）を、設定された `anchor_height` と `tag_height` の差を利用して水平距離に変換してから計算に使う。

> **詳細資料：**
> [UWBとセンサーフュージョンによる広域空間におけるロバストな自己位置推定と自律移動の実証（金沢工業大学）](https://www.ipsj.or.jp/kenkyukai/event/dps205.html)

---


## 📂 構成ファイル解説

### 🛰️ 主要プログラム
- `ekf_with_serial.launch.py`: **自己位置推定の心臓部。** シリアル通信とフィルタを同時に立ち上げる。
- `uwb_nav.launch.py`: **自律移動の実行。** Nav2を利用する。※先に`ekf_with_serial.launch.py`が動いている必要がある。

### 🛠️ ツール・スクリプト
- `rviz_anchor_place.py`: **アンカー設置ツール。** Rviz上で地図をタッチするだけで、アンカー座標を`anchors.yaml`に自動保存できる。（詳細は使い方セクション）
- `experiment_gui.py`: 論文執筆時に使用したGUIツール。
- `get_map.py`: Kachaka APIと接続し、自動でマップファイルを取得する。
- `serialandFilter.py` / `serialAndFliterPublisher.py`: シリアル通信とデータ加工の担当。
- `uwb_ekf_node.py`: アルゴリズムの実体。

### ⚙️ パラメータ
- `maps/`: 実験で使用したKachaka製の地図データを格納。
- `param/anchors.yaml`: アンカーの設置位置。
- `param/nav2_params_uwb.yaml`: UWBナビゲーション用のNav2設定。AMCLを排した特化設定だ。

---

## ⚡ 使い方

### 1. 事前準備
リポジトリをクローンしてビルドする。
```bash
git clone [https://github.com/Takuchan/opencampus_robot](https://github.com/Takuchan/opencampus_robot)
colcon build
source install/setup.bash
```

利用する部屋の環境地図を作成しておく。

### 2. アンカー設置
自律移動を始める前に、必ずこの手順でアンカーの座標を設定すること。

#### 1. アンカー座標の取得
Rviz2を起動した状態で、`rviz_anchor_place.launch.py` を起動すると、Rviz上で地図データが表示される。地図データが出ない場合は、`uwb_nav.launch.py`を起動して出すことをおすすめする。
1.  Rviz上のツールバーにある **「Publish Point」** を押す。
2.  **アンカー番号の順番通り（A0, A1...）**に、地図上の正確な設置場所をタッチしていく。
3.  すべてのアンカーを打ち終えると、GUIのステータスが「Ready to save」に変わる。
4.  **Save Anchors** を押すと、`~/ros2_ws/install/tk_uwb_ekf/share/tk_uwb_ekf/param/anchors.yaml` に保存される。
5.  **重要：** そのデータをros2_ws/src/tk_uwb_ekf/paramsnに保存完了後、再度 `colcon build` を行って設定を反映させること。

#### 2. ローンチファイルでの高さ設定
UWBは「斜めの距離」を測るため、高さの設定がズレると位置推定が狂う。
地面からの高さをメートル単位で正確に指定して起動すること。

* `anchor_height`: アンカーの設置高度（デフォルト: 0.0）
* `tag_height`: ロボットに載せているタグの高度（デフォルト: 0.0）

例：
```bash
ros2 launch tk_uwb_ekf uwb_nav.launch.py anchor_height:=1.43 tag_height:=0.69
```

### 3.実行の順番（俺用）
1. 台車とシリアル通信
2. ホイールOdometryを発行
3. IMUを利用
4. 俺の作ったメインの自己位置推定のプログラムを実行
```:bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v4
ros2 launch megarover3_bringup robot.launch.py
ros2 launch oc_megarover_bringup bringup.launch.py
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
ros2 launch tk_uwb_ekf uwb_nav.launch.py
```

