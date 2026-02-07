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
- `publish_rate_hz`: publish の周期（Hz）。`1000.0` で 1kHz
- `gyro_in_degree`: 角速度が deg/s の場合 true（rad/s に変換）
- `accel_in_g`: 加速度が g の場合 true（m/s^2 に変換）
- `orientation_covariance`: 姿勢の共分散（対角）
- `angular_velocity_covariance`: 角速度の共分散（対角）
- `linear_acceleration_covariance`: 加速度の共分散（対角）
- `data_type_accel`: 加速度データのタイプID（PDFのUSBデータフレーム表で `01`）
- `data_type_gyro`: 角速度データのタイプID（`02`）
- `data_type_rpy`: オイラー角データのタイプID（`03`）
- `data_type_quat`: 四元数データのタイプID（`04`）
- `warn_unknown_data_type`: 未知タイプIDの警告ログ
- `debug_raw`: 生の受信バイト列ログ（トラブルシュート用）
- `debug_raw_max_bytes`: ログ出力最大バイト数

### USB 設定コマンド（自動送信）
起動時に USB の「設定モード」に入り、出力データと周波数を設定してから通常モードへ戻します。
（DM-IMU-L1 使用説明書 V1.0 / V1.2 互換）

- `usb_configure`: 起動時に USB コマンドを送るか
- `usb_protocol_version`: `v1_0`（推奨）または `v1_2`
- `usb_output_rate_hz`: USB 送信周波数（100-1000Hz）
- `usb_enable_accel`: 加速度の出力を有効化
- `usb_enable_gyro`: 角速度の出力を有効化
- `usb_enable_rpy`: オイラー角の出力を有効化
- `usb_enable_quat_cmd`: 四元数出力要求コマンド（`AA 01 17 0D`）を送るか
- `usb_force_output_interface`: 出力IF設定コマンド（v1.2系）を送るか
- `usb_factory_reset_on_start`: 起動時に工場出荷リセットを実行するか
- `usb_factory_reset_wait_ms`: 工場出荷リセット後の再起動待ち時間（ms）
- `usb_save_params`: 設定保存（電源断後も保持）
- `usb_config_sleep_ms`: コマンド間の待ち時間（ms）

`usb_factory_reset_on_start=true` の場合、起動時に次を送信します（PDF準拠）。
- `AA 06 01 0D`（設定モード）
- `AA 0B 01 0D`（工場出荷）
- `AA 03 01 0D`（保存）
- `AA 00 00 0D`（再起動）
その後、通常の出力有効化・周波数設定コマンドを再送します。

## 変更内容（2026-02-06）
- `/imu/data` に `linear_acceleration` と `angular_velocity` を追加
- 1kHz publish のため `publish_rate_hz` を追加（Python/C++）
- USB のクイックコマンドで出力有効化と周波数設定を実装
- `dm_imu/modules/dm_serial.py` に送信用 `write_bytes()` を追加
- USB フレームの `ID` と `タイプID` を区別して扱うように修正

## ディレクトリ構成
- `dm_imu/`: Python ノード実装
- `config/`: パラメータ
- `launch/`: launch ファイル
- `rviz/`: RViz 設定
