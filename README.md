# UWB Trilateration System

This system performs real-time 3D positioning using UWB (Ultra-Wideband) technology with three fixed anchor points.

# 実行の順番（俺用）
## ①台車とシリアル通信をするため
```:bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v4
```

## ②ホイールOdometryを発行するため。
```:bash
ros2 launch megarover3_bringup robot.launch.py
```

##  ③IMUを利用するため。
```:bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```
## ④EKF
```:bash
ros2 launch tk_uwb_ekf ekf_with_serial.launch.py
```

## Features

- Real-time serial data acquisition from UWB device
- 3D trilateration using least squares method
- Live 3D visualization of position
- Position history tracking

## Setup

1. Install required packages:
```bash
pip install -r requirements.txt
```

2. Connect your UWB device to COM7 (or modify the port in the code)

3. Run the system:
```bash
python uwb_trilateration.py
```

## Anchor Configuration

The system uses three fixed UWB anchors with known distances:
- UWB0: Origin (0, 0, 0)
- UWB1: 776cm from UWB0
- UWB2: 789cm from UWB0, 530cm from UWB1

## Usage

- The program will automatically parse incoming serial data
- 3D position is calculated using trilateration
- Real-time visualization shows current position and movement path
- Press Ctrl+C to stop the system
