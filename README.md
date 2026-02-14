# UWBとセンサーフュージョンによる広域空間におけるロバストな自己位置推定と自律移動の実証
English: [English version / README_en.md](README_en.md)
最終更新日：2026年2月14日



動画を作成しました。ざっくり概要をご覧ください。
<video src="https://s3-ap-northeast-1.amazonaws.com/mp4.video.honko.kanazawa-it.ac.jp/20251118nakazawa_jiritsu.mp4" controls="true"></video>
動画引用元：[【国際ロボット展2025に情報工学科 中沢実研究室が出展】
12月3日(水)から6日(土)まで東京ビッグサイトで。対話型AI、自己位置推定ロボット、交通流解析、遠隔コミュニケーション支援に関する最新成果を展示](https://www.kanazawa-it.ac.jp/kitnews/2025/1118_nakazawa.html)

[別タブで動画を見る](https://s3-ap-northeast-1.amazonaws.com/mp4.video.honko.kanazawa-it.ac.jp/20251118nakazawa_jiritsu.mp4)

## 功績
- [ 第205回マルチメディア通信と分散処理研究発表会 論文発表](https://www.ipsj.or.jp/kenkyukai/event/dps205.html)

- 国際ロボット展 2025 展示
- 物語の始まりへで取り上げられました。

ライセンス：商用利用可能です。ただし、利用した場合は明記してほしいです！

## 📖 研究概要
**「電波の死角でも、止まらない。」**

このプログラムは、UWB（Ultra-Wideband）とホイールオドメトリを互いに補いながら、安定した自己位置推定と自律移動を行うためのパッケージです。

一番の強みは、**nLOS（非見通し）判定**を利用した動的なUWBの信頼性調節にある。nLOS判定が行われた場合、カルマンフィルタ側でUWBの重みを下げ、ホイールオドメトリを最優先する。これにより、工場や電波の届きにくい死角にロボットが入っても、安定した自律移動を継続できる。

## 研究背景
|①近年工場DXが進んでいる|②LiDARをベースとしたAMRロボットが多い|③しかし、LiDARには問題がある|
|---|---|---|
|![reason1](/images/reason1.png)|![reason2](/images/reason2.png)|![reason3](/images/reason3.png)|

LiDARをベースとしたAMRロボットは、現在業界のデファクトスタンダードとなっている。実際に国際ロボット展2025で様々な企業のブースを訪れて話を聞いているとLiDARを使ってロボットを運用していることが当たり前のようだ。

しかし、LiDARには③以外の欠点として、特徴点が少ない環境では運用が難しい点が挙げられる。国際ロボット展ではトンネル内の自動運転の研究を行っている方や、とても大きなタンクの中を自律移動をしたいという方とお話をした。これらの環境は特徴点が少なく、自己位置を把握しにくい。LiDAR以外の手法を模索し、これらの環境下でも安定した自律移動を目指したい。

今回の研究ではUWBという測距が可能な電波を利用して自律移動を可能にすることを目指し、実現ができた。

資料：https://www.automation-news.jp/2023/09/75132/

## 🧠 内部アルゴリズムの概略
拡張カルマンフィルタ（EKF）をベースに、非線形な入力に対応している。
![system_input_output](/images/system_input_output.png)

- **予測：** ホイールオドメトリを利用。
- **補正：** UWBの距離データを利用。
- **3D補正：** UWBの生データ（斜め距離）を、設定された `anchor_height` と `tag_height` の差を利用して水平距離に変換してから計算に使う。

## 実験機器
|機器名|説明|
|---|---|
|メガローバーVer3.0|実験用台車。内部にホイールオドメトリを出力するエンコーダーが内蔵されている。|
|Type2BP EVK| 村田製作所が販売しているUWB開発者キット|
|Kachaka|地図作成用に利用 |
|Livox MID360 | 実験比較用に利用 | 
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
git clone https://github.com/Takuchan/opencampus_robot.git
colcon build
source install/setup.bash
```

- **利用する部屋の環境地図**を作成する。本実験ではKachakaで作成した地図を利用した。
- 環境地図を作成したら、`tk_uwb_ekf`フォルダーの`maps`の中に`map.pgm`,`map.yaml`で保存する。
- この際`map.yaml`の中の`image`の指定先`map.pgm`になっているか確認する。

最後に
```bash
colcon build
```
をする。

### 2. アンカー設置
自律移動を始める前に、必ずこの手順でアンカーの座標を設定すること。

#### 1. アンカー座標の取得
Rviz2を起動した状態で、`rviz_anchor_place.launch.py` を起動すると、Rviz上で地図データが表示される。地図データが出ない場合は、`uwb_nav.launch.py`を起動して地図データを表示することをおすすめする。
1.  Rviz上のツールバーにある **「Publish Point」** を押す。
2.  **UWBアンカー番号の順番通り（A0, A1...）** に、地図上の正確な設置場所をタッチしていく。
3.  すべてのアンカーを打ち終えると、GUIのステータスが「Ready to save」に変わる。
4.  **Save Anchors** を押すと、`~/ros2_ws/install/tk_uwb_ekf/share/tk_uwb_ekf/param/anchors.yaml` に保存される。
5.  **重要：** そのデータを`ros2_ws/src/tk_uwb_ekf/params`にコピーする。そして再度 `colcon build` を行って設定を反映させること。

#### 2. ローンチファイルでの高さ設定
UWBは「斜めの距離」を測るため、高さの設定がズレると位置推定が狂う。
地面からの高さをメートル単位で正確に指定して起動すること。
`param/anchors.yaml`の中を変更する。

* `anchor_height`: アンカーの設置高度（デフォルト: 0.0）
* `tag_height`: ロボットに載せているタグの高度（デフォルト: 0.0）

例：この方法でも可能だ。
```bash
ros2 launch tk_uwb_ekf uwb_nav.launch.py anchor_height:=1.43 tag_height:=0.69
```

### 3.実行の順番
1. 台車とシリアル通信
2. ホイールOdometryを発行
3. IMUを利用
4. 自作の自己位置推定プログラムを実行
```:bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v4
ros2 launch megarover3_bringup robot.launch.py
ros2 launch oc_megarover_bringup bringup.launch.py
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
ros2 launch tk_uwb_ekf uwb_nav.launch.py
```

