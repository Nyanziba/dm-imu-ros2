#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import struct
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import serial


# --- CRC16 ---
# マニュアルには「附录四のCRC16コードを使え」とあるが、掲載コードの演算が一般的なCCITTとズレて見える。
# なので (A)掲載コード風 と (B)標準CRC-CCITT(FALSE) の両方を計算し、どちらか一致したらOKにする。
# これで「マニュアルの誤植」でも「実装が標準」でも落ちにくい。

CRC16_TABLE = [
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129,
    0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF, 0x1231, 0x0210, 0x3273, 0x2252,
    0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C,
    0xF3FF, 0xE3DE, 0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D, 0x3653, 0x2672,
    0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4, 0xB75B, 0xA77A, 0x9719, 0x8738,
    0xF7DF, 0xE7FE, 0xD79D, 0xC7BC, 0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861,
    0x2802, 0x3823, 0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC,
    0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A, 0x6CA6, 0x7C87, 0x4CE4, 0x5CC5,
    0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B,
    0x8D68, 0x9D49, 0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78, 0x9188, 0x81A9,
    0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3,
    0x5004, 0x4025, 0x7046, 0x6067, 0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C,
    0xE37F, 0xF35E, 0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D, 0x34E2, 0x24C3,
    0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xA7DB, 0xB7FA, 0x8799, 0x97B8,
    0xE75F, 0xF77E, 0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676,
    0x4615, 0x5634, 0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3, 0xCB7D, 0xDB5C,
    0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A, 0x4A75, 0x5A54, 0x6A37, 0x7A16,
    0x0AF1, 0x1AD0, 0x2AB3, 0x3A92, 0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B,
    0x9DE8, 0x8DC9, 0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36,
    0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
]

def crc16_doc_style(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        idx = ((crc >> 8) ^ b) & 0xFF
        crc = (((crc << 1) & 0xFFFF) ^ CRC16_TABLE[idx]) & 0xFFFF
    return crc

def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# --- IMU USB framing ---
FRAME_LEN = 19
HDR0 = 0x55
HDR1 = 0xAA
TAIL = 0x0A

REG_ACCEL = 0x01
REG_GYRO  = 0x02
REG_EULER = 0x03

# --- USB control commands (4 bytes) ---
# 0xAA 0x01 0x14: accel ON
# 0xAA 0x01 0x15: gyro  ON
# 0xAA 0x01 0x16: euler ON
# 0xAA 0x01 0x17: quat  ON (ただしUSBでは現状出ないと明記)
# 0xAA 0x02 0x06: 1000Hz
# 0xAA 0x03 0x01: save
# 0xAA 0x06 0x01: enter setting mode
# 0xAA 0x06 0x00: exit setting mode
CMD = {
    "enter_setting": bytes([0xAA, 0x06, 0x01, 0x0D]),
    "exit_setting":  bytes([0xAA, 0x06, 0x00, 0x0D]),
    "accel_on":      bytes([0xAA, 0x01, 0x14, 0x0D]),
    "gyro_on":       bytes([0xAA, 0x01, 0x15, 0x0D]),
    "euler_on":      bytes([0xAA, 0x01, 0x16, 0x0D]),
    "quat_on":       bytes([0xAA, 0x01, 0x17, 0x0D]),
    "save":          bytes([0xAA, 0x03, 0x01, 0x0D]),
}
RATE_CMD = {
    100:  bytes([0xAA, 0x02, 0x01, 0x0D]),
    125:  bytes([0xAA, 0x02, 0x02, 0x0D]),
    200:  bytes([0xAA, 0x02, 0x03, 0x0D]),
    250:  bytes([0xAA, 0x02, 0x04, 0x0D]),
    500:  bytes([0xAA, 0x02, 0x05, 0x0D]),
    1000: bytes([0xAA, 0x02, 0x06, 0x0D]),
}

@dataclass
class IMUState:
    accel: Optional[Tuple[float, float, float]] = None  # m/s^2
    gyro:  Optional[Tuple[float, float, float]] = None  # rad/s
    euler_deg: Optional[Tuple[float, float, float]] = None  # deg
    quat_xyzw: Optional[Tuple[float, float, float, float]] = None
    last_print: float = 0.0


def euler_rpy_to_quat(roll_rad: float, pitch_rad: float, yaw_rad: float) -> Tuple[float, float, float, float]:
    """ZYX intrinsic (yaw->pitch->roll)."""
    cy, sy = math.cos(yaw_rad * 0.5), math.sin(yaw_rad * 0.5)
    cp, sp = math.cos(pitch_rad * 0.5), math.sin(pitch_rad * 0.5)
    cr, sr = math.cos(roll_rad * 0.5), math.sin(roll_rad * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qx, qy, qz, qw)

def try_parse_frame(buf: bytearray) -> Optional[Tuple[int, int, bytes]]:
    """
    buf から1フレーム抜き出して (slave_id, reg_id, payload12) を返す。
    失敗なら None。成功したらbufから消費する。
    """
    # ヘッダ探索
    while True:
        if len(buf) < FRAME_LEN:
            return None
        i = buf.find(bytes([HDR0, HDR1]))
        if i < 0:
            # ヘッダっぽいものが無い -> 全捨て
            buf.clear()
            return None
        if i > 0:
            del buf[:i]
        if len(buf) < FRAME_LEN:
            return None

        frame = bytes(buf[:FRAME_LEN])
        if frame[0] != HDR0 or frame[1] != HDR1 or frame[-1] != TAIL:
            # たまたま0x55 0xAAが出ただけ
            del buf[:2]
            continue

        data0_15 = frame[0:16]
        rx_crc = frame[16] | (frame[17] << 8)

        c1 = crc16_doc_style(data0_15)
        c2 = crc16_ccitt_false(data0_15)
        if rx_crc != c1 and rx_crc != c2:
            # CRC不一致 -> 1バイトずらして再同期
            del buf[:1]
            continue

        slave_id = frame[2]
        reg_id = frame[3]
        payload = frame[4:16]  # 12 bytes
        del buf[:FRAME_LEN]
        return slave_id, reg_id, payload

def decode_3floats_le(payload12: bytes) -> Tuple[float, float, float]:
    # Ax_L1..Ax_H2 の並び = little-endian 32-bit float だと読む（説明書の「指针类型转换」より）
    return struct.unpack("<fff", payload12)

def configure_usb(ser: serial.Serial, rate_hz: int) -> None:
    # 設定モードに入って、必要なデータをON、周波数設定、保存、抜ける
    ser.write(CMD["enter_setting"])
    time.sleep(0.05)

    ser.write(CMD["accel_on"])
    ser.write(CMD["gyro_on"])
    ser.write(CMD["euler_on"])
    ser.write(CMD["quat_on"])   # USBでは出ないのが普通だが、FW次第なので一応送る
    ser.write(RATE_CMD[rate_hz])
    ser.write(CMD["save"])
    time.sleep(0.05)

    ser.write(CMD["exit_setting"])
    time.sleep(0.05)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="例: COM5 or /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=921600, help="USB側が仮想COMの場合、まずは921600で試す")
    ap.add_argument("--rate", type=int, default=500, choices=sorted(RATE_CMD.keys()))
    ap.add_argument("--print_hz", type=float, default=100.0, help="統合出力の最大表示レート")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    configure_usb(ser, args.rate)

    state = IMUState()
    buf = bytearray()
    min_print_dt = 1.0 / args.print_hz

    print("# columns: t accel_x accel_y accel_z gyro_x gyro_y gyro_z roll_deg pitch_deg yaw_deg qx qy qz qw")
    print("# units: accel=m/s^2, gyro=rad/s, euler=deg")

    while True:
        chunk = ser.read(4096)
        if chunk:
            buf.extend(chunk)

        # バッファにある分だけ全部パース
        while True:
            parsed = try_parse_frame(buf)
            if not parsed:
                break

            slave_id, reg_id, payload = parsed
            if reg_id == REG_ACCEL:
                state.accel = decode_3floats_le(payload)
            elif reg_id == REG_GYRO:
                state.gyro = decode_3floats_le(payload)
            elif reg_id == REG_EULER:
                roll_deg, pitch_deg, yaw_deg = decode_3floats_le(payload)
                state.euler_deg = (roll_deg, pitch_deg, yaw_deg)
                roll = roll_deg * math.pi / 180.0
                pitch = pitch_deg * math.pi / 180.0
                yaw = yaw_deg * math.pi / 180.0
                state.quat_xyzw = euler_rpy_to_quat(roll, pitch, yaw)

        now = time.perf_counter()  # time.timeよりこっちが安定（単調増加）
        if now - state.last_print >= min_print_dt:
            ax, ay, az = state.accel if state.accel else (float("nan"),)*3
            gx, gy, gz = state.gyro  if state.gyro  else (float("nan"),)*3
            rd, pd, yd = state.euler_deg if state.euler_deg else (float("nan"),)*3
            qx, qy, qz, qw = state.quat_xyzw if state.quat_xyzw else (float("nan"),)*4
            print(
                f"{now:.6f} "
                f"{ax:.6f} {ay:.6f} {az:.6f} "
                f"{gx:.6f} {gy:.6f} {gz:.6f} "
                f"{rd:.6f} {pd:.6f} {yd:.6f} "
                f"{qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}"
            )
            state.last_print = now



if __name__ == "__main__":
    main()
