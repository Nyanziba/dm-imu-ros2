# models/dm_serial.py
# -*- coding: utf-8 -*-
"""
DM_Serial: 达妙 IMU 串口读取类（支持“后台读线程” + 主线程按需取最新）
- 固定 timeout=0（非阻塞）
- read(): Drain+ParseAll+Latest（也可单线程调用）
- start_reader()/stop_reader(): 在内部起一个只负责“刷新数据”的线程，不打印
- get_latest(): 主线程随时取“最新一帧”与其时间戳/计数
- destory()/reopen(): 资源管理
- CRC：默认“包含帧头 0x55,0xAA”，失败自动再试“不含帧头”

帧格式：
[0,1]=0x55,0xAA | [2]=? | [3]=RID | [4:16]=3*float32(LE) | [16:18]=CRC16(LE) | [18]=0x0A
"""
from __future__ import annotations

import struct
import threading
import time
from typing import Optional, Tuple, List

import serial  # pip install pyserial

from .dm_crc import dm_crc16, dm_crc16_ccitt_false

HDR = b'\x55\xAA'
TAIL = 0x0A
FRAME_LEN = 19

# v1.0 设备存在 CRC 实现差异：同时接受 doc-style 与 CCITT(FALSE)，并兼容含/不含帧头。
SKIP_HDR_IN_CRC = False

class DM_Serial:
    def __init__(self, port: str, baudrate: int):
        self.port = port
        self.baudrate = int(baudrate)
        self.timeout = 0.0  # 非阻塞
        self.ser: Optional[serial.Serial] = None
        self._buf = bytearray()

        # 统计
        self.cnt_ok = 0
        self.cnt_crc = 0
        self.cnt_short = 0
        self.cnt_nohdr = 0

        # 后台读线程状态
        self._th: Optional[threading.Thread] = None
        self._stop_evt: Optional[threading.Event] = None
        self._read_sleep = 0.001  # 读线程小睡控制 CPU

        # 最新数据（线程安全）
        self._latest_lock = threading.Lock()
        self._latest_pkt: Optional[Tuple[int, Tuple[float, float, float], int]] = None
        self._latest_by_type: dict[int, Tuple[int, Tuple[float, float, float], int]] = {}
        self._latest_ts: float = 0.0
        self._latest_count: int = 0
        self._last_error: Optional[str] = None
        self._last_read: bytes = b""
        self._last_read_ts: float = 0.0

        if not self._open():
            raise RuntimeError(self._last_error or "Failed to open serial")

    # ------------ 公共 API ------------
    def read(self, max_bytes: int | None = None) -> Optional[Tuple[int, Tuple[float, float, float]]]:
        """一次性读入串口当前可读字节，解析所有完整帧，只返回“最新一帧”。"""
        if not self.ser or not self.ser.is_open:
            return None
        self._read_into_buf(max_bytes)
        frames = self._parse_all()
        return frames[-1] if frames else None

    def start_reader(self, read_sleep: float = 0.001) -> bool:
        """启动只负责刷新数据的后台线程；不打印。"""
        if self._th and self._th.is_alive():
            self._read_sleep = read_sleep
            return True
        if not self.is_open:
            if not self._open():
                return False
        self._stop_evt = threading.Event()
        self._read_sleep = read_sleep
        self._th = threading.Thread(target=self._reader_loop, daemon=True)
        self._th.start()
        return True

    def stop_reader(self) -> None:
        """停止后台读线程。"""
        if self._stop_evt:
            self._stop_evt.set()
        if self._th:
            self._th.join(timeout=1.0)
        self._th = None
        self._stop_evt = None

    def get_latest(self) -> Tuple[Optional[Tuple[int, Tuple[float, float, float]]], float, int]:
        """线程安全地获取（pkt, timestamp, count）。"""
        with self._latest_lock:
            return self._latest_pkt, self._latest_ts, self._latest_count

    def get_latest_by_type(self) -> Tuple[dict[int, Tuple[int, Tuple[float, float, float], int]], float, int]:
        """线程安全地获取（{data_type: pkt}, timestamp, count）。"""
        with self._latest_lock:
            return dict(self._latest_by_type), self._latest_ts, self._latest_count

    def get_debug(self) -> dict:
        """返回调试信息（仅用于日志）。"""
        in_waiting = getattr(self.ser, "in_waiting", None) if self.ser else None
        return {
            "buf_len": len(self._buf),
            "last_read_len": len(self._last_read),
            "last_read_hex": self._last_read.hex(),
            "last_read_ts": self._last_read_ts,
            "in_waiting": in_waiting,
            "cnt_ok": self.cnt_ok,
            "cnt_crc": self.cnt_crc,
            "cnt_short": self.cnt_short,
            "cnt_nohdr": self.cnt_nohdr,
            "last_error": self._last_error,
        }

    def write_bytes(self, data: bytes) -> int:
        """向串口写入原始字节；返回写入字节数。"""
        if not self.ser or not self.ser.is_open:
            self._last_error = "serial not open"
            return 0
        try:
            n = self.ser.write(data)
            try:
                self.ser.flush()
            except Exception:
                pass
            return int(n)
        except Exception as e:
            self._last_error = str(e)
            return 0

    def last_error(self) -> Optional[str]:
        return self._last_error

    def destory(self) -> None:
        """立即关闭串口（按你的拼写保留）。"""
        self.stop_reader()
        if self.ser:
            try:
                self.ser.close()
            finally:
                self.ser = None

    # 别名
    def destroy(self) -> None:
        self.destory()

    def reopen(self) -> bool:
        """关闭并重新打开串口。"""
        self.destory()
        return self._open()

    def clear_input(self) -> None:
        """Drop pending RX bytes from driver and local parse buffer."""
        self._buf.clear()
        if self.ser and self.ser.is_open:
            try:
                self.ser.reset_input_buffer()
            except Exception:
                pass

    @property
    def is_open(self) -> bool:
        return bool(self.ser and self.ser.is_open)

    # ------------ 内部实现 ------------
    def _open(self) -> bool:
        try:
            if not hasattr(serial, "Serial"):
                self._last_error = (
                    "pyserial not available: module 'serial' has no attribute 'Serial'. "
                    f"serial.__file__={getattr(serial, '__file__', None)}"
                )
                self.ser = None
                return False
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout, write_timeout=0)
            try:
                self.ser.reset_input_buffer()
            except Exception:
                pass
            return True
        except Exception as e:
            self._last_error = str(e)
            self.ser = None
            return False

    def _reader_loop(self):
        """后台线程：不断刷新最新数据，不打印。"""
        evt = self._stop_evt
        try:
            while evt and not evt.is_set():
                self._read_into_buf(None)
                frames = self._parse_all()
                if frames:
                    now = time.time()
                    with self._latest_lock:
                        for pkt in frames:
                            self._latest_pkt = pkt
                            try:
                                data_type = int(pkt[0])
                                self._latest_by_type[data_type] = pkt
                            except Exception:
                                pass
                        self._latest_ts = now
                        self._latest_count += len(frames)
                if self._read_sleep > 0.0:
                    time.sleep(self._read_sleep)
        except Exception as e:
            # 不打印，记录错误字符串，便于主线程查询
            self._last_error = f"reader_loop: {e!r}"

    def _read_into_buf(self, max_bytes: Optional[int]) -> int:
        """把串口里“当前可读”的字节读入缓冲；返回读取字节数。"""
        n = getattr(self.ser, "in_waiting", 0) if self.ser else 0
        if max_bytes is not None and n > max_bytes:
            n = max_bytes
        if n <= 0:
            return 0
        data = self.ser.read(n)
        self._buf.extend(data)
        self._last_read = bytes(data)
        self._last_read_ts = time.time()
        return n

    def _parse_all(self) -> List[Tuple[int, Tuple[float, float, float]]]:
        """
        Parse all frames from buffer.
        Re-sync behavior intentionally matches test.py (v1.0 reference).
        """
        results: List[Tuple[int, Tuple[float, float, float]]] = []

        while True:
            if len(self._buf) < FRAME_LEN:
                if self._buf:
                    self.cnt_short += 1
                break

            j = self._buf.find(HDR)
            if j < 0:
                self._buf.clear()
                self.cnt_nohdr += 1
                break
            if j > 0:
                del self._buf[:j]
            if len(self._buf) < FRAME_LEN:
                self.cnt_short += 1
                break

            frame = bytes(self._buf[:FRAME_LEN])
            if frame[0] != 0x55 or frame[1] != 0xAA or frame[-1] != TAIL:
                # Same as test.py: shift by 2 bytes when header/tail is invalid.
                del self._buf[:2]
                continue

            if not self._crc_matches(frame):
                self.cnt_crc += 1
                # Same as test.py: shift by 1 byte on CRC mismatch.
                del self._buf[:1]
                continue

            dev_id = frame[2]
            rid = frame[3]
            f1 = struct.unpack('<f', frame[4:8])[0]
            f2 = struct.unpack('<f', frame[8:12])[0]
            f3 = struct.unpack('<f', frame[12:16])[0]
            results.append((rid, (f1, f2, f3), dev_id))
            del self._buf[:FRAME_LEN]

        self.cnt_ok += len(results)
        return results

    @staticmethod
    def _crc_matches(frame: bytes) -> bool:
        """Match test.py behavior for v1.0: header-included doc-style OR CCITT(FALSE)."""
        rx_crc = frame[16] | (frame[17] << 8)
        crc_data = frame[0:16]
        c1 = dm_crc16(crc_data)
        c2 = dm_crc16_ccitt_false(crc_data)
        return rx_crc == c1 or rx_crc == c2
