import math
import threading
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped, PoseStamped

# 使用你的串口实现（保持路径）
from .modules.dm_serial import DM_Serial


def euler_rpy_to_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """ZYX intrinsic (yaw->pitch->roll); roll/pitch/yaw in radians."""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qx, qy, qz, qw)


class DmImuNode(Node):
    def __init__(self):
        super().__init__('dm_imu')

        # ---------- Parameters ----------
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rpy_in_degree', True) # 若希望 /imu/rpy 以“度”发布，把 False 改 True
        self.declare_parameter('verbose', True)          # 终端打印
        self.declare_parameter('qos_reliable', True)     # 发布端 QoS（默认 Reliable，RViz 直接可见）
        self.declare_parameter('debug_raw', False)       # 输出原始串口字节（仅用于排查）
        self.declare_parameter('debug_raw_max_bytes', 64) # 原始字节日志最大长度
        # 新增：三个话题的开关（默认全开）
        self.declare_parameter('publish_imu_data', False)  # /imu/data
        self.declare_parameter('publish_rpy', True)       # /imu/rpy
        self.declare_parameter('publish_pose', False)      # /imu/pose
        self.declare_parameter('publish_rate_hz', 1000.0)  # publish timer rate

        # Unit conversion + covariance
        self.declare_parameter('gyro_in_degree', True)  # deg/s -> rad/s
        self.declare_parameter('accel_in_g', True)      # g -> m/s^2
        self.declare_parameter('orientation_covariance', 0.02)
        self.declare_parameter('angular_velocity_covariance', 0.02)
        self.declare_parameter('linear_acceleration_covariance', 0.02)

        # USB quick commands (see PDF)
        self.declare_parameter('usb_configure', True)
        self.declare_parameter('usb_output_rate_hz', 1000.0)
        self.declare_parameter('usb_enable_accel', True)
        self.declare_parameter('usb_enable_gyro', True)
        self.declare_parameter('usb_enable_rpy', True)
        self.declare_parameter('usb_save_params', True)
        self.declare_parameter('usb_config_sleep_ms', 50)

        def _p(name, default=None):
            try:
                v = self.get_parameter(name).value
                return default if v in (None, '') else v
            except Exception:
                return default

        self.port = _p('port', '/dev/ttyACM0')
        self.frame_id = _p('frame_id', 'imu_link')
        self.publish_rpy = bool(_p('publish_rpy', True))
        self.publish_rpy_in_degree = bool(_p('publish_rpy_in_degree', True))
        self.verbose = bool(_p('verbose', True))
        qos_reliable = bool(_p('qos_reliable', True))
        self.debug_raw = bool(_p('debug_raw', True))
        try:
            self.debug_raw_max_bytes = int(_p('debug_raw_max_bytes', 64))
        except Exception:
            self.debug_raw_max_bytes = 64
        # 新增：三个话题的总开关
        self.publish_imu_data = bool(_p('publish_imu_data', True))
        self.publish_rpy = bool(_p('publish_rpy', True))
        self.publish_pose = bool(_p('publish_pose', False))
        try:
            self.publish_rate_hz = float(_p('publish_rate_hz', 1000.0))
        except Exception:
            self.publish_rate_hz = 1000.0

        self.gyro_in_degree = bool(_p('gyro_in_degree', True))
        self.accel_in_g = bool(_p('accel_in_g', True))
        try:
            self.orientation_cov = float(_p('orientation_covariance', 0.02))
            self.angular_velocity_cov = float(_p('angular_velocity_covariance', 0.02))
            self.linear_acceleration_cov = float(_p('linear_acceleration_covariance', 0.02))
        except Exception:
            self.orientation_cov = 0.02
            self.angular_velocity_cov = 0.02
            self.linear_acceleration_cov = 0.02

        self.usb_configure = bool(_p('usb_configure', True))
        try:
            self.usb_output_rate_hz = float(_p('usb_output_rate_hz', self.publish_rate_hz))
        except Exception:
            self.usb_output_rate_hz = self.publish_rate_hz
        self.usb_enable_accel = bool(_p('usb_enable_accel', True))
        self.usb_enable_gyro = bool(_p('usb_enable_gyro', True))
        self.usb_enable_rpy = bool(_p('usb_enable_rpy', True))
        self.usb_save_params = bool(_p('usb_save_params', True))
        try:
            self.usb_config_sleep_ms = float(_p('usb_config_sleep_ms', 50))
        except Exception:
            self.usb_config_sleep_ms = 50.0

        baud = _p('baudrate', 921600)
        try:
            self.baudrate = int(baud)
        except (TypeError, ValueError):
            self.get_logger().warn(f'Invalid baudrate "{baud}", fallback to 921600')
            self.baudrate = 921600

        if self.publish_rate_hz <= 0:
            self.get_logger().warn(f'Invalid publish_rate_hz "{self.publish_rate_hz}", fallback to 1000.0')
            self.publish_rate_hz = 1000.0
        self.publish_period_sec = 1.0 / float(self.publish_rate_hz)
        self._no_frame_warn_every = max(1, int(round(self.publish_rate_hz)))

        # ---------- QoS ----------
        if qos_reliable:
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=50,
                durability=DurabilityPolicy.VOLATILE,
            )
        else:
            from rclpy.qos import qos_profile_sensor_data  # BestEffort
            qos = qos_profile_sensor_data

        # ---------- Publishers ----------
        self.pub_imu  = self.create_publisher(Imu, 'imu/data', qos)                 if self.publish_imu_data else None
        self.pub_rpy  = self.create_publisher(Vector3Stamped, 'imu/rpy', qos)       if self.publish_rpy      else None
        self.pub_pose = self.create_publisher(PoseStamped, 'imu/pose', 10)          if self.publish_pose     else None

        # ---------- Serial ----------
        try:
            # 按你的要求：初始化不传 timeout
            self.ser = DM_Serial(self.port, baudrate=self.baudrate)
            if self.usb_configure:
                self._configure_device()
            self.ser.start_reader()  # 后台读线程交给你的类
            self.get_logger().info(f'Opened serial {self.port} @ {self.baudrate}')
        except Exception as e:
            self.get_logger().fatal(f'Init serial failed: {e}')
            raise

        # ---------- Timers ----------
        # 用时间戳做去重；拿不到就不去重
        self._last_stamp_ts: Optional[float] = None
        self._last_count: int = 0
        self._closing = threading.Event()
        self._logged_bad_fmt_once = False
        self._no_frame_ticks = 0
        self._pub_count = 0
        self._has_rpy = False
        self._has_gyro = False
        self._has_accel = False
        self._logged_missing_gyro = False
        self._logged_missing_accel = False
        self._r_deg = 0.0
        self._p_deg = 0.0
        self._y_deg = 0.0
        self._gyro_x = 0.0
        self._gyro_y = 0.0
        self._gyro_z = 0.0
        self._accel_x = 0.0
        self._accel_y = 0.0
        self._accel_z = 0.0

        # publish_rate_hz 轮询
        self.timer_pub = self.create_timer(self.publish_period_sec, self._on_timer_publish)
        # 每 2s 打一次统计（若你的类提供 get_stats）
        self.timer_stat = self.create_timer(2.0, self._on_timer_stats)

    # ----------- Timers -----------
    def _on_timer_publish(self):
        try:
            latest = self.ser.get_latest()
        except Exception as e:
            if self.verbose:
                self.get_logger().warn(f'get_latest() exception: {e}')
            return

        if latest is None:
            self._no_frame_ticks += 1
            if self._no_frame_ticks % self._no_frame_warn_every == 0 and self.verbose:  # ≈每秒一次
                self.get_logger().warn('No frames yet from serial (≈1s). Check IMU streaming/baud/crc.')
            return
        pkt = None
        stamp_ts = None
        count = None
        if isinstance(latest, (tuple, list)) and len(latest) >= 3:
            pkt, stamp_ts, count = latest[0], latest[1], latest[2]
        else:
            pkt = latest

        # DM_Serial.get_latest() の戻り (pkt, ts, count) で pkt=None の場合は「未受信」
        if pkt is None:
            self._no_frame_ticks += 1
            if self._no_frame_ticks % self._no_frame_warn_every == 0 and self.verbose:  # ≈每秒一次
                self.get_logger().warn('No frames yet from serial (≈1s). Check IMU streaming/baud/crc.')
            return

        # 去重（优先用 count；没有就用时间戳）
        if count is not None:
            try:
                count_int = int(count)
            except Exception:
                count_int = None
            if count_int is not None:
                if count_int == self._last_count:
                    return
                self._last_count = count_int
        elif stamp_ts is not None:
            if stamp_ts == self._last_stamp_ts:
                return
            self._last_stamp_ts = stamp_ts

        if not self._update_from_packet(pkt):
            ok, stamp_ts, r_deg, p_deg, y_deg = self._extract_latest(latest)  # 提取数据!!
            if not ok:
                if not self._logged_bad_fmt_once and self.verbose:
                    self.get_logger().warn(f'Unknown latest frame format; example: {repr(latest)}')
                    self._logged_bad_fmt_once = True
                return
            self._r_deg = r_deg
            self._p_deg = p_deg
            self._y_deg = y_deg
            self._has_rpy = True

        if not self._has_rpy:
            return

        # 度→弧度
        r = self._r_deg * math.pi / 180.0
        p = self._p_deg * math.pi / 180.0
        y = self._y_deg * math.pi / 180.0

        stamp = self.get_clock().now().to_msg()

        # /imu/data
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.frame_id
        qx, qy, qz, qw = euler_rpy_to_quat(r, p, y)

        # /imu/rpy
        if self.pub_rpy is not None:
            rpy_msg = Vector3Stamped()
            rpy_msg.header.stamp = stamp
            rpy_msg.header.frame_id = self.frame_id
            if self.publish_rpy_in_degree:
                rpy_msg.vector.x = float(self._r_deg)
                rpy_msg.vector.y = float(self._p_deg)
                rpy_msg.vector.z = float(self._y_deg)
            else:
                rpy_msg.vector.x = float(r)
                rpy_msg.vector.y = float(p)
                rpy_msg.vector.z = float(y)
            # /imu/rpy
            self.pub_rpy.publish(rpy_msg)

        if self.publish_imu_data == False and self.publish_pose == False:
            return
        # 有效性检查 + 归一化（防 Unvisualizable）
        def _finite(*vals):
            return all(math.isfinite(v) for v in vals)

        if not _finite(qx, qy, qz, qw):
            if self.verbose:
                self.get_logger().warn('Quaternion has NaN/Inf, publishing identity (0,0,0,1)')
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        else:
            n = (qx*qx + qy*qy + qz*qz + qw*qw) ** 0.5
            if n < 1e-6:
                if self.verbose:
                    self.get_logger().warn('Quaternion norm ~0, publishing identity (0,0,0,1)')
                qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
            else:
                qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n

        # /imu/data
        if self.pub_imu is not None:
            imu.orientation.x, imu.orientation.y = qx, qy
            imu.orientation.z, imu.orientation.w = qz, qw
            imu.orientation_covariance[0] = self.orientation_cov
            imu.orientation_covariance[4] = self.orientation_cov
            imu.orientation_covariance[8] = self.orientation_cov

            gx = self._gyro_x
            gy = self._gyro_y
            gz = self._gyro_z
            if self.gyro_in_degree:
                scale = math.pi / 180.0
                gx *= scale
                gy *= scale
                gz *= scale
            imu.angular_velocity.x = gx
            imu.angular_velocity.y = gy
            imu.angular_velocity.z = gz

            ax = self._accel_x
            ay = self._accel_y
            az = self._accel_z
            if self.accel_in_g:
                g = 9.80665
                ax *= g
                ay *= g
                az *= g
            imu.linear_acceleration.x = ax
            imu.linear_acceleration.y = ay
            imu.linear_acceleration.z = az

            if self._has_gyro:
                for i in range(9):
                    imu.angular_velocity_covariance[i] = 0.0
                imu.angular_velocity_covariance[0] = self.angular_velocity_cov
                imu.angular_velocity_covariance[4] = self.angular_velocity_cov
                imu.angular_velocity_covariance[8] = self.angular_velocity_cov
            else:
                for i in range(9):
                    imu.angular_velocity_covariance[i] = -1.0

            if self._has_accel:
                for i in range(9):
                    imu.linear_acceleration_covariance[i] = 0.0
                imu.linear_acceleration_covariance[0] = self.linear_acceleration_cov
                imu.linear_acceleration_covariance[4] = self.linear_acceleration_cov
                imu.linear_acceleration_covariance[8] = self.linear_acceleration_cov
            else:
                for i in range(9):
                    imu.linear_acceleration_covariance[i] = -1.0
            self.pub_imu.publish(imu)

        # /imu/pose（原点+IMU姿态），便于 RViz 直接看 Pose
        if self.pub_pose is not None:
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = 0.0
            pose.pose.position.y = 0.0
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            self.pub_pose.publish(pose)

        # 终端打印
        self._pub_count += 1
        self._no_frame_ticks = 0
        if self.verbose:
            self.get_logger().info(
                f'#{self._pub_count} RPY(deg)=({self._r_deg:.2f}, {self._p_deg:.2f}, {self._y_deg:.2f}) '
                f'RPY(rad)=({r:.3f}, {p:.3f}, {y:.3f})'
            )

    def _on_timer_stats(self):
        try:
            if hasattr(self.ser, 'get_stats'):
                stats = self.ser.get_stats()
                msg = " ".join([f"{k}={v}" for k, v in stats.items()]) if isinstance(stats, dict) else str(stats)
                if self.verbose:
                    self.get_logger().info(f'[stats] {msg}')
            if self.debug_raw and hasattr(self.ser, 'get_debug'):
                dbg = self.ser.get_debug()
                raw_hex = dbg.get("last_read_hex") or ""
                if self.debug_raw_max_bytes > 0 and raw_hex:
                    raw_hex = raw_hex[: self.debug_raw_max_bytes * 2]
                if self._no_frame_ticks > 0:
                    self.get_logger().info(
                        f'[raw] in_waiting={dbg.get("in_waiting")} '
                        f'buf_len={dbg.get("buf_len")} '
                        f'last_read_len={dbg.get("last_read_len")} '
                        f'cnt_ok={dbg.get("cnt_ok")} cnt_crc={dbg.get("cnt_crc")} '
                        f'cnt_short={dbg.get("cnt_short")} cnt_nohdr={dbg.get("cnt_nohdr")} '
                        f'last_error={dbg.get("last_error")} '
                        f'last_read_hex={raw_hex}'
                    )
        except Exception:
            pass  # 统计异常不影响主流程

    # ----------- Helpers -----------
    def _configure_device(self):
        """USB 快捷指令配置（见说明书 V1.2）。"""
        if not hasattr(self.ser, 'write_bytes'):
            if self.verbose:
                self.get_logger().warn('DM_Serial has no write_bytes(); skip USB configuration.')
            return

        if self.usb_output_rate_hz <= 0:
            interval_ms = None
        else:
            if not (100.0 <= self.usb_output_rate_hz <= 1000.0):
                self.get_logger().warn(
                    f'usb_output_rate_hz={self.usb_output_rate_hz} out of 100-1000Hz range; '
                    'sending anyway.'
                )
            interval_ms = int(round(1000.0 / float(self.usb_output_rate_hz)))
            interval_ms = max(1, min(interval_ms, 0xFFFF))

        cmds = []
        cmds.append(bytes([0xAA, 0x06, 0x01, 0x0D]))  # 进入设置模式
        if self.usb_enable_accel:
            cmds.append(bytes([0xAA, 0x01, 0x14, 0x0D]))  # 开启加速度输出
        if self.usb_enable_gyro:
            cmds.append(bytes([0xAA, 0x01, 0x15, 0x0D]))  # 开启角速度输出
        if self.usb_enable_rpy:
            cmds.append(bytes([0xAA, 0x01, 0x16, 0x0D]))  # 开启欧拉角输出
        if interval_ms is not None:
            lo = interval_ms & 0xFF
            hi = (interval_ms >> 8) & 0xFF
            cmds.append(bytes([0xAA, 0x02, lo, hi, 0x0D]))  # 修改反馈频率
        if self.usb_save_params:
            cmds.append(bytes([0xAA, 0x03, 0x01, 0x0D]))  # 保存参数
        cmds.append(bytes([0xAA, 0x06, 0x00, 0x0D]))  # 返回正常模式

        sleep_s = max(0.0, float(self.usb_config_sleep_ms) / 1000.0)
        for cmd in cmds:
            n = self.ser.write_bytes(cmd)
            if self.verbose:
                self.get_logger().info(f'USB cmd sent ({n}B): {cmd.hex()}')
            if sleep_s > 0:
                time.sleep(sleep_s)

    def _update_from_packet(self, pkt) -> bool:
        """更新最新传感器数据，返回是否成功解析。"""
        if not isinstance(pkt, (tuple, list)) or len(pkt) != 2:
            return False
        rid, vals = pkt[0], pkt[1]
        if not isinstance(vals, (tuple, list)) or len(vals) < 3:
            return False
        try:
            rid = int(rid)
            v1 = float(vals[0])
            v2 = float(vals[1])
            v3 = float(vals[2])
        except Exception:
            return False

        if rid == 0x01:
            self._accel_x, self._accel_y, self._accel_z = v1, v2, v3
            self._has_accel = True
        elif rid == 0x02:
            self._gyro_x, self._gyro_y, self._gyro_z = v1, v2, v3
            self._has_gyro = True
        elif rid == 0x03:
            self._r_deg, self._p_deg, self._y_deg = v1, v2, v3
            self._has_rpy = True
        else:
            return False

        if not self._has_gyro and not self._logged_missing_gyro:
            self._logged_missing_gyro = True
            if self.verbose:
                self.get_logger().warn('Gyro frame not received yet; angular_velocity will be zero until it arrives.')
        if not self._has_accel and not self._logged_missing_accel:
            self._logged_missing_accel = True
            if self.verbose:
                self.get_logger().warn('Accel frame not received yet; linear_acceleration will be zero until it arrives.')
        return True

    def _extract_latest(self, latest) -> Tuple[bool, Optional[float], float, float, float]:
        """
        返回 (ok, stamp_ts, roll_deg, pitch_deg, yaw_deg)
        - 兼容你的嵌套：((rid, (r,p,y)), ts, extra)
        - 也兼容 dict / 扁平 tuple / 对象字段
        """
        try:
            # dict
            if isinstance(latest, dict):
                r = latest.get('roll') or latest.get('r') or latest.get('Roll')
                p = latest.get('pitch') or latest.get('p') or latest.get('Pitch')
                y = latest.get('yaw') or latest.get('y') or latest.get('Yaw')
                ts = latest.get('ts') or latest.get('timestamp') or latest.get('time') or None
                if r is not None and p is not None and y is not None:
                    return True, (float(ts) if ts is not None else None), float(r), float(p), float(y)

            # tuple/list
            if isinstance(latest, (tuple, list)):
                # ((rid, (r,p,y)), ts, extra)
                if len(latest) >= 2 and isinstance(latest[0], (tuple, list)):
                    rid_part = latest[0]
                    ts = latest[1] if isinstance(latest[1], (int, float)) else None
                    if len(rid_part) == 2 and isinstance(rid_part[1], (tuple, list)) and len(rid_part[1]) >= 3:
                        r, p, y = rid_part[1][0], rid_part[1][1], rid_part[1][2]
                        return True, (float(ts) if ts is not None else None), float(r), float(p), float(y)
                # (rid, r, p, y)
                if len(latest) >= 4 and not isinstance(latest[0], (tuple, list)):
                    _, r, p, y = latest[0], latest[1], latest[2], latest[3]
                    return True, None, float(r), float(p), float(y)
                # (r, p, y)
                if len(latest) == 3:
                    r, p, y = latest[0], latest[1], latest[2]
                    return True, None, float(r), float(p), float(y)

            # 对象字段
            r = getattr(latest, 'roll', None)
            p = getattr(latest, 'pitch', None)
            y = getattr(latest, 'yaw', None)
            ts = getattr(latest, 'ts', None) or getattr(latest, 'timestamp', None) or None
            if r is not None and p is not None and y is not None:
                return True, (float(ts) if ts is not None else None), float(r), float(p), float(y)

            return False, None, 0.0, 0.0, 0.0
        except Exception as e:
            if self.verbose:
                self.get_logger().debug(f'_extract_latest exception: {e}')
            return False, None, 0.0, 0.0, 0.0

    # ----------- Shutdown -----------
    def destroy_node(self):
        if getattr(self, '_closing', None) is None or self._closing.is_set():
            try:
                super().destroy_node()
            except Exception:
                pass
            return
        self._closing.set()
        try:
            if hasattr(self.ser, 'stop_reader'):
                self.ser.stop_reader()
        except Exception:
            pass
        try:
            if hasattr(self.ser, 'close'):
                self.ser.close()
        except Exception:
            pass
        try:
            super().destroy_node()
        except Exception:
            pass


def main():
    rclpy.init()
    node = DmImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
