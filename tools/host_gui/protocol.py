#!/usr/bin/env python3
"""Wire format for the RK3566 <-> STM32 link (docs/architecture/protocol.md, v1).

Pure Python and Qt-free on purpose: the frame codec is the part that has to
match the MCU byte for byte, so it is kept small, testable and reusable by any
other host tool.

    frame = AA 55 LEN CMD SEQ PAYLOAD CRC_L CRC_H
    LEN   = 2 + len(PAYLOAD)          (CMD + SEQ + PAYLOAD)
    CRC   = CRC-16/CCITT-FALSE over LEN..PAYLOAD, stored little-endian
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum

HEAD = b"\xAA\x55"
# ESTOP has no payload, so LEN = CMD + SEQ = 2 is the shortest legal frame.
LEN_MIN = 2
LEN_MAX = 66
PAYLOAD_MAX = LEN_MAX - 2
ACK_FLAG = 0x80


class Cmd(IntEnum):
    """Host -> MCU commands (0x01..0x3F)."""

    SET_VELOCITY = 0x01
    SET_ARM_JOINTS = 0x02
    SET_GRIPPER = 0x03
    SET_ENABLE = 0x04
    SET_PARAM = 0x05
    ESTOP = 0x06
    CLEAR_FAULT = 0x07


class Report(IntEnum):
    """MCU -> Host unsolicited reports (0xC0..0xFF)."""

    ODOM = 0xC1
    ARM = 0xC2
    STATUS = 0xC3
    EVENT = 0xC4
    HEARTBEAT = 0xC5


class Result(IntEnum):
    OK = 0x00
    OUT_OF_RANGE = 0x01
    BAD_STATE = 0x02
    UNKNOWN_CMD = 0x03
    FAILED = 0x04


RESULT_NAMES = {
    Result.OK: "成功",
    Result.OUT_OF_RANGE: "参数越界",
    Result.BAD_STATE: "当前状态不允许",
    Result.UNKNOWN_CMD: "未知命令",
    Result.FAILED: "执行失败",
}

# Provisional until protocol.md fixes the REPORT_STATUS state enum; unknown
# values fall back to "0xNN" in the UI, so a mismatch is visible, not silent.
STATE_NAMES = {
    0x00: "初始化",
    0x01: "空闲",
    0x02: "手动",
    0x03: "自动",
    0x04: "急停",
    0x05: "故障",
}

FAULT_NAMES = {
    0x0101: "电机过流",
    0x0102: "编码器异常",
    0x0201: "CRC 错误率超限",
    0x0202: "通信超时",
    0x0301: "关节超限位",
    0x0302: "关节堵转",
    0x0303: "轨迹执行异常",
    0x0401: "电池低压",
    0x0402: "温度过高",
    0x0501: "急停触发",
}

PARAM_NAMES = {
    0x01: "轮距",
    0x02: "轮径",
    0x03: "左轮 Kp",
    0x04: "左轮 Ki",
    0x05: "左轮 Kd",
    0x06: "右轮 Kp",
    0x07: "右轮 Ki",
    0x08: "右轮 Kd",
    0x10: "控制超时",
    0x11: "最大线速度",
    0x12: "最大角速度",
}

GRIPPER_ACTION_NAMES = {0: "张开", 1: "闭合"}
GRIPPER_STATE_NAMES = {0: "张开", 1: "夹持", 2: "夹空"}


# ------------------------------------------------------------------- codec

def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 \
                else (crc << 1) & 0xFFFF
    return crc


def _crc_step(crc: int, byte: int) -> int:
    """One byte of the same CRC, for the incremental parser."""
    crc ^= byte << 8
    for _ in range(8):
        crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 \
            else (crc << 1) & 0xFFFF
    return crc


def build_frame(cmd: int, seq: int, payload: bytes = b"") -> bytes:
    """Encode one frame. Raises ValueError on an impossible payload."""
    cmd = int(cmd)
    seq = int(seq) & 0xFF
    payload = bytes(payload)
    if len(payload) > PAYLOAD_MAX:
        raise ValueError(f"payload {len(payload)} > {PAYLOAD_MAX}")
    if not 0 < cmd <= 0xFF:
        raise ValueError(f"bad command 0x{cmd:02X}")
    body = bytes((len(payload) + 2, cmd, seq)) + payload
    crc = crc16_ccitt(body)
    return HEAD + body + bytes((crc & 0xFF, crc >> 8))


@dataclass(frozen=True)
class Frame:
    cmd: int
    seq: int
    payload: bytes
    raw: bytes

    @property
    def is_ack(self) -> bool:
        return ACK_FLAG <= self.cmd < 0xC0

    @property
    def acked_cmd(self) -> int | None:
        return (self.cmd & 0x7F) if self.is_ack else None

    @property
    def acked_seq(self) -> int | None:
        return self.payload[0] if self.is_ack and len(self.payload) >= 1 else None

    @property
    def result(self) -> int | None:
        return self.payload[1] if self.is_ack and len(self.payload) >= 2 else None


class FrameParser:
    """Incremental byte-stream parser (protocol.md section 9).

    feed() accepts any chunking, including one byte at a time. Invalid LEN or
    CRC returns to the head-search state, so a corrupt frame cannot cascade.
    """

    _HEAD1, _HEAD2, _LEN, _BODY, _CRC_LO, _CRC_HI = range(6)

    def __init__(self) -> None:
        self.crc_errors = 0
        self.frames_ok = 0
        self.reset()

    def reset(self) -> None:
        self._state = self._HEAD1
        self._crc = 0xFFFF
        self._need = 0
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[Frame]:
        frames: list[Frame] = []

        for byte in data:
            if self._state == self._HEAD1:
                if byte == 0xAA:
                    self._state = self._HEAD2

            elif self._state == self._HEAD2:
                # A repeated 0xAA is still a plausible head, keep waiting.
                self._state = self._LEN if byte == 0x55 else (
                    self._HEAD2 if byte == 0xAA else self._HEAD1)

            elif self._state == self._LEN:
                if LEN_MIN <= byte <= LEN_MAX:
                    self._buf = bytearray((byte,))
                    self._crc = _crc_step(0xFFFF, byte)
                    self._need = byte
                    self._state = self._BODY
                else:
                    # Resynchronise: this byte may itself be the next 0xAA.
                    self._state = self._HEAD2 if byte == 0xAA else self._HEAD1

            elif self._state == self._BODY:
                self._buf.append(byte)
                self._crc = _crc_step(self._crc, byte)
                self._need -= 1
                if self._need == 0:
                    self._crc_lo = 0
                    self._state = self._CRC_LO

            elif self._state == self._CRC_LO:
                self._crc_lo = byte
                self._state = self._CRC_HI

            else:  # _CRC_HI
                rx_crc = self._crc_lo | (byte << 8)
                if rx_crc == self._crc:
                    self.frames_ok += 1
                    frames.append(Frame(
                        cmd=self._buf[1],
                        seq=self._buf[2],
                        payload=bytes(self._buf[3:]),
                        raw=HEAD + bytes(self._buf) + bytes((self._crc_lo, byte)),
                    ))
                else:
                    self.crc_errors += 1
                self._state = self._HEAD1

        return frames


# ----------------------------------------------------------- payloads (down)

def _i16(value: float) -> int:
    return max(-32768, min(32767, int(round(value))))


def set_velocity_payload(v_mm_s: float, w_mrad_s: float) -> bytes:
    return struct.pack("<hh", _i16(v_mm_s), _i16(w_mrad_s))


def set_arm_joints_payload(angles_centi_deg) -> bytes:
    values = list(angles_centi_deg)
    if len(values) != 6:
        raise ValueError("SET_ARM_JOINTS needs exactly 6 angles")
    return struct.pack("<6h", *(_i16(v) for v in values))


def set_gripper_payload(action: int, force: int) -> bytes:
    return struct.pack("<BB", int(action) & 0xFF, max(0, min(100, int(force))))


def set_enable_payload(enable: bool) -> bytes:
    return struct.pack("<B", 1 if enable else 0)


def set_param_payload(param_id: int, value: int) -> bytes:
    return struct.pack("<Bi", int(param_id) & 0xFF,
                       max(-2**31, min(2**31 - 1, int(value))))


def clear_fault_payload(code: int = 0xFFFF) -> bytes:
    return struct.pack("<H", int(code) & 0xFFFF)


# ---------------------------------------------------------- payloads (up)

def decode_odom(payload: bytes):
    if len(payload) != 14:
        return None
    x, y, theta, vl, vr = struct.unpack("<iihhh", payload)
    return {"x_mm": x, "y_mm": y, "theta_cdeg": theta,
            "vl_mm_s": vl, "vr_mm_s": vr}


def decode_arm(payload: bytes):
    if len(payload) != 15:
        return None
    values = struct.unpack("<6hhB", payload)
    return {"angles_cdeg": values[:6], "gripper_pos": values[6],
            "gripper_state": values[7]}


def decode_status(payload: bytes):
    if len(payload) != 8:
        return None
    state, fault, batt_mv, temp_c, jitter_us = struct.unpack("<BHHbH", payload)
    return {"state": state, "fault": fault, "battery_mv": batt_mv,
            "temp_c": temp_c, "jitter_us": jitter_us}


def decode_event(payload: bytes):
    if len(payload) != 6:
        return None
    code, timestamp_ms = struct.unpack("<HI", payload)
    return {"code": code, "timestamp_ms": timestamp_ms}


def decode_heartbeat(payload: bytes):
    if len(payload) != 3:
        return None
    state, counter = struct.unpack("<BH", payload)
    return {"state": state, "counter": counter}


def describe(frame: Frame) -> str:
    """One-line human summary for the log pane."""
    if frame.is_ack:
        result = frame.result
        try:
            name = RESULT_NAMES.get(Result(result), f"0x{result:02X}")
        except (TypeError, ValueError):
            name = "短帧" if result is None else f"0x{result:02X}"
        try:
            cmd_name = Cmd(frame.cmd & 0x7F).name
        except ValueError:
            cmd_name = f"0x{frame.cmd:02X}"
        return f"ACK {cmd_name} seq={frame.acked_seq} {name}"

    try:
        name = Cmd(frame.cmd).name
    except ValueError:
        try:
            name = Report(frame.cmd).name
        except ValueError:
            name = f"0x{frame.cmd:02X}"

    if frame.cmd == Report.ODOM:
        odom = decode_odom(frame.payload) or {}
        return (f"{name} x={odom.get('x_mm')} y={odom.get('y_mm')} "
                f"theta={odom.get('theta_cdeg')} vl={odom.get('vl_mm_s')} "
                f"vr={odom.get('vr_mm_s')}")
    if frame.cmd == Report.STATUS:
        status = decode_status(frame.payload) or {}
        return (f"{name} state=0x{status.get('state', 0):02X} "
                f"fault=0x{status.get('fault', 0):04X} "
                f"batt={status.get('battery_mv')}mV "
                f"temp={status.get('temp_c')}C jitter={status.get('jitter_us')}us")
    if frame.cmd == Report.EVENT:
        event = decode_event(frame.payload) or {}
        code = event.get("code", 0)
        return f"{name} {FAULT_NAMES.get(code, f'0x{code:04X}')} " \
               f"t={event.get('timestamp_ms')}ms"
    if frame.cmd == Report.HEARTBEAT:
        beat = decode_heartbeat(frame.payload) or {}
        return (f"{name} state=0x{beat.get('state', 0):02X} "
                f"count={beat.get('counter')}")
    if frame.cmd == Report.ARM:
        arm = decode_arm(frame.payload) or {}
        return (f"{name} joints={arm.get('angles_cdeg')} "
                f"grip={arm.get('gripper_pos')} state={arm.get('gripper_state')}")

    return f"{name} seq={frame.seq} len={len(frame.payload)}"


# --------------------------------------------------------------- vectors

VECTOR_CRC = (b"123456789", 0x29B1)
VECTOR_VELOCITY = bytes.fromhex("AA 55 06 01 01 2C 01 F4 01 72 3F")
VECTOR_ENABLE = bytes.fromhex("AA 55 03 04 02 01 9F B5")
VECTOR_ESTOP = bytes.fromhex("AA 55 02 06 03 39 38")
