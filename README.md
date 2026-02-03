# DM-IMU ROS2 Example

DM-IMU（L1）用の ROS2 サンプルです。`dm_imu` パッケージとして構成されています。

## 前提
- ROS 2 Humble（または互換のある環境）
- Python 3
- `pyserial`

> 注意: `serial` という別パッケージが入っていると衝突します。
> その場合は `pip uninstall serial` してから `pip install pyserial` を入れてください。

## ビルド
```
colcon build --symlink-install
```

## 環境読み込み
```
source install/setup.bash
# zsh の場合
# source install/setup.zsh
```

## 起動
```
ros2 launch dm_imu dm_imu.launch.py
```

## 主要パラメータ
`config/params.yaml` で設定します。

- `port`: シリアルポート（例: `/dev/ttyACM0` または `/dev/tty.usbmodem...`）
- `baudrate`: ボーレート（例: `921600`）
- `publish_imu_data`: `/imu/data` を出すか
- `publish_rpy`: `/imu/rpy` を出すか
- `publish_pose`: `/imu/pose` を出すか
- `publish_rpy_in_degree`: RPY を度で出すか
- `debug_raw`: 生の受信バイト列ログ（トラブルシュート用）
- `debug_raw_max_bytes`: ログ出力最大バイト数

## ディレクトリ構成
- `dm_imu/`: Python ノード実装
- `config/`: パラメータ
- `launch/`: launch ファイル
- `rviz/`: RViz 設定
