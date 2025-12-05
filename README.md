# UWB Navigation
研究用のシステムになります。
参考になれば幸いです。
開発中ののブランチはadd/match...にあります。

UWBを使ったナビゲーションシステム。
UWBで自己位置推定を行い、そのデータを使ってNav2でナビゲーションを行う。

メインで編集しているのは、`tk_uwb_ekf` フォルダー

# 実行の順番（俺用）
1. 台車とシリアル通信
2. ホイールOdometryを発行
3. IMUを利用
4. 俺の作ったメインの自己位置推定のプログラムを実行
```:bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v4
ros2 launch megarover3_bringup robot.launch.py
ros2 launch oc_megarover_bringup bringup.launch.py
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch tk_uwb_ekf ekf_with_serial.launch.py
