#!/usr/bin/env python3
"""Virtual MCU for the host console.

The firmware does not implement the protocol yet (M2.2/M2.3), so the console
ships with a simulated peer: it consumes the exact frames the GUI would send,
answers the ones that need an ACK, and produces the periodic reports. The GUI
therefore runs end to end today and the same code path is exercised once the
real board is on the other end of the cable.

The physics is deliberately small: a differential-drive integrator, a
rate-limited six-joint arm and a gripper. It is a protocol peer, not a plant
model.
"""

from __future__ import annotations

import math
import queue
import random
import struct
import threading
import time

from PySide6.QtCore import QObject, Signal

from .protocol import (ACK_FLAG, COUNTS_PER_MM_IDS, CRASH_SLOT_COUNT,
                       CRASH_SLOT_LATEST, DIRECTION_IDS, DUTY_FWD_IDS,
                       DUTY_REV_IDS, PID_PARAM_IDS, Cmd, FrameParser, Report,
                       Result, build_frame)

TICK_S = 0.005
VELOCITY_TIMEOUT_S = 0.200
ODOM_PERIOD_S = 0.020
WHEEL_SPEED_PERIOD_S = 0.020
ARM_PERIOD_S = 0.050
STATUS_PERIOD_S = 0.100
HEARTBEAT_PERIOD_S = 0.100
TRACK_WIDTH_MM = 200.0
JOINT_RATE = 6000.0        # centi-degrees per second
GRIPPER_RATE = 500.0       # position units per second

# Storage side (protocol.md 5.5). The virtual device starts with values that
# match the calibration page's defaults, so a write followed by a read-back
# shows the same numbers.
LOG_EXPORT_PERIOD_S = 0.050        # the firmware's 20 Hz ceiling
CRASH_CHUNK_BYTES = 32
DEFAULT_LIMITS = {"timeout_ms": 200, "max_linear": 500, "max_angular": 1500}
DEFAULT_CALIB = {
    "wheel_base": 240,
    "wheel_diameter": 60,
    "counts": [24828] * 4,
    "duty_fwd": [75, 67, 82, 80],
    "duty_rev": [67, 75, 80, 100],
    "direction": [1] * 4,
}
LIMIT_PARAM_IDS = {0x10: "timeout_ms", 0x11: "max_linear", 0x12: "max_angular"}
# Same bounds as firmware/app/01_APP/app_robot/app_params.c. The velocity
# limits are capped at what the chassis can actually do (500 mm/s per wheel,
# 1500 mrad/s), so a host cannot configure a limit that would never be reached
# and then believe commands above the real capability were accepted.
PARAM_RANGES = {
    0x01: (50, 2000),        # wheel base, mm
    0x02: (10, 500),         # wheel diameter, mm
    0x10: (10, 5000),        # command timeout, ms
    0x11: (10, 500),         # max linear speed, mm/s
    0x12: (10, 1500),        # max angular speed, mrad/s
}
COUNTS_PER_MM_RANGE = (1000, 100000)

# param id -> (calibration field, index or None)
CALIB_FIELDS = {0x01: ("wheel_base", None), 0x02: ("wheel_diameter", None)}
for _wheel, _param_id in enumerate(COUNTS_PER_MM_IDS):
    CALIB_FIELDS[_param_id] = ("counts", _wheel)
for _wheel, _param_id in enumerate(DUTY_FWD_IDS):
    CALIB_FIELDS[_param_id] = ("duty_fwd", _wheel)
for _wheel, _param_id in enumerate(DUTY_REV_IDS):
    CALIB_FIELDS[_param_id] = ("duty_rev", _wheel)
for _wheel, _param_id in enumerate(DIRECTION_IDS):
    CALIB_FIELDS[_param_id] = ("direction", _wheel)


class SimulatorLink(QObject):
    connectedChanged = Signal(bool)
    frameReceived = Signal(object)
    errorOccurred = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._out: queue.Queue[bytes] = queue.Queue()
        self.rx_frames = 0
        self.crc_errors = 0
        self._running = False

    @property
    def is_open(self) -> bool:
        return self._running

    def open(self, port: str | None = None, baud: int = 0) -> None:
        self.close()
        self._reset_state()
        self._stop.clear()
        self._running = True
        self._thread = threading.Thread(target=self._run, name="simulator",
                                        daemon=True)
        self._thread.start()
        self.connectedChanged.emit(True)

    def close(self) -> None:
        was_open = self._running
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        self._running = False
        while not self._out.empty():
            try:
                self._out.get_nowait()
            except queue.Empty:
                break
        if was_open:
            self.connectedChanged.emit(False)

    def send(self, data: bytes) -> None:
        if self._running:
            self._out.put(data)

    # -- state -------------------------------------------------------------

    def _reset_state(self) -> None:
        self._enabled = False
        self._estopped = False
        self._fault = 0
        self._v = 0.0              # mm/s, commanded and applied
        self._w = 0.0              # mrad/s
        self._last_velocity_cmd = 0.0
        self._x = 0.0              # mm
        self._y = 0.0
        self._theta = 0.0          # rad
        self._joints = [0.0] * 6   # centi-degrees
        self._joint_targets = [0.0] * 6
        self._gripper = 0.0        # 0..1000
        self._gripper_target = 0.0
        self._gripper_state = 0
        self._gains = [[5.0, 6.0, 0.0] for _ in range(4)]
        self._heartbeat = 0
        self._tx_seq = 0
        self._rng = random.Random(7)
        self._out_parser = FrameParser()
        self._in_parser = FrameParser()
        self._started_at = time.monotonic()

        # -- storage subsystem --------------------------------------------
        self._param_seq = 4
        self._calib_seq = 2
        self._boot_total = 23
        self._log_erase = 11
        self._last_save_s = 0
        self._log_level = 1
        self._param_valid = True
        self._calib_valid = True
        self._dirty_param = False
        self._dirty_calib = False
        self._save_failed = False
        self._crash_unread = True
        self._log_wrapped = False
        self._limits = dict(DEFAULT_LIMITS)
        self._calib = {key: (list(value) if isinstance(value, list) else value)
                       for key, value in DEFAULT_CALIB.items()}
        self._logs = self._seed_logs()
        self._faults = self._seed_faults()
        self._export: dict | None = None
        self._crash_records = [self._make_crash()]
        self._crash_tx: dict | None = None

    # -- worker ------------------------------------------------------------

    def _run(self) -> None:
        last = time.monotonic()
        next_report = {"odom": 0.0, "wheel": 0.0, "arm": 0.0,
                       "status": 0.0, "beat": 0.0}

        while not self._stop.is_set():
            now = time.monotonic()
            dt = min(0.05, now - last)
            last = now

            try:
                while True:
                    for frame in self._in_parser.feed(self._out.get_nowait()):
                        self._handle(frame, now)
            except queue.Empty:
                pass
            self.crc_errors = self._in_parser.crc_errors

            self._step(dt, now)
            self._step_export(now)
            self._step_crash()

            for key, period, builder in (
                    ("odom", ODOM_PERIOD_S, self._odom_payload),
                    ("wheel", WHEEL_SPEED_PERIOD_S, self._wheel_speed_payload),
                    ("arm", ARM_PERIOD_S, self._arm_payload),
                    ("status", STATUS_PERIOD_S, self._status_payload),
                    ("beat", HEARTBEAT_PERIOD_S, self._heartbeat_payload)):
                if now >= next_report[key]:
                    next_report[key] = now + period
                    self._emit(builder())

            time.sleep(TICK_S)

    def _step(self, dt: float, now: float) -> None:
        if now - self._last_velocity_cmd > VELOCITY_TIMEOUT_S:
            self._v = 0.0
            self._w = 0.0
        if not self._enabled or self._estopped:
            self._v = 0.0
            self._w = 0.0

        speed_mm_s = self._v
        self._theta += self._w / 1000.0 * dt
        self._x += speed_mm_s * math.cos(self._theta) * dt
        self._y += speed_mm_s * math.sin(self._theta) * dt

        for index in range(6):
            delta = self._joint_targets[index] - self._joints[index]
            step = JOINT_RATE * dt
            self._joints[index] += max(-step, min(step, delta))

        delta = self._gripper_target - self._gripper
        step = GRIPPER_RATE * dt
        self._gripper += max(-step, min(step, delta))
        if abs(self._gripper_target - self._gripper) < 1.0:
            self._gripper = self._gripper_target
            if self._gripper_target <= 0.0:
                self._gripper_state = 0          # open
            elif self._gripper_target > 0.0:
                self._gripper_state = 2          # closed, nothing gripped

    # -- inbound -----------------------------------------------------------

    def _handle(self, frame, now: float) -> None:
        cmd = frame.cmd
        payload = frame.payload

        if cmd == Cmd.SET_VELOCITY:
            self._last_velocity_cmd = now
            if len(payload) != 4:
                return
            v, w = struct.unpack("<hh", payload)
            self._v, self._w = float(v), float(w)
            return

        if cmd == Cmd.SET_ARM_JOINTS:
            if len(payload) != 12:
                return
            if self._enabled and not self._estopped:
                self._joint_targets = [float(v) for v in
                                       struct.unpack("<6h", payload)]
            return

        if cmd == Cmd.SET_GRIPPER:
            if len(payload) != 2:
                self._ack(cmd, frame.seq, Result.OUT_OF_RANGE)
                return
            action, force = payload[0], payload[1]
            if action > 1 or force > 100:
                self._ack(cmd, frame.seq, Result.OUT_OF_RANGE)
            elif not self._enabled or self._estopped:
                self._ack(cmd, frame.seq, Result.BAD_STATE)
            else:
                self._gripper_target = 1000.0 if action else 0.0
                self._gripper_state = 1 if action else 0
                self._ack(cmd, frame.seq, Result.OK)
            return

        if cmd == Cmd.SET_ENABLE:
            if len(payload) != 1 or payload[0] > 1:
                self._ack(cmd, frame.seq, Result.OUT_OF_RANGE)
            elif self._estopped and payload[0]:
                self._ack(cmd, frame.seq, Result.BAD_STATE)
            else:
                self._enabled = bool(payload[0])
                if not self._enabled:
                    self._v = self._w = 0.0
                self._ack(cmd, frame.seq, Result.OK)
            return

        if cmd == Cmd.SET_PARAM:
            if len(payload) != 5:
                self._ack(cmd, frame.seq, Result.OUT_OF_RANGE)
                return
            param_id = payload[0]
            value = struct.unpack("<i", payload[1:])[0]
            self._handle_set_param(frame, param_id, value)
            return

        if cmd == Cmd.PARAM_SAVE:
            self._handle_param_save(frame, now)
            return

        if cmd == Cmd.PARAM_RESET:
            self._handle_param_reset(frame)
            return

        if cmd == Cmd.STORAGE_QUERY:
            self._ack(cmd, frame.seq, Result.OK)
            self._emit(self._storage_payload())
            return

        if cmd == Cmd.LOG_EXPORT:
            self._handle_log_export(frame, now)
            return

        if cmd == Cmd.CRASH_READ:
            self._handle_crash_read(frame, now)
            return

        if cmd == Cmd.LOG_CLEAR:
            self._handle_log_clear(frame)
            return

        if cmd == Cmd.ESTOP:
            self._estopped = True
            self._enabled = False
            self._v = self._w = 0.0
            self._fault = 0x0501
            self._gripper_state = 0
            self._ack(cmd, frame.seq, Result.OK)
            self._emit(self._event_payload(0x0501, now))
            return

        if cmd == Cmd.CLEAR_FAULT:
            if len(payload) != 2:
                self._ack(cmd, frame.seq, Result.OUT_OF_RANGE)
                return
            code = struct.unpack("<H", payload)[0]
            if code in (0xFFFF, self._fault):
                self._fault = 0
                self._estopped = False
                self._ack(cmd, frame.seq, Result.OK)
            else:
                self._ack(cmd, frame.seq, Result.OUT_OF_RANGE)
            return

        if 0x01 <= cmd <= 0x3F:
            self._ack(cmd, frame.seq, Result.UNKNOWN_CMD)

    # -- parameters and calibration ---------------------------------------

    def _handle_set_param(self, frame, param_id: int, value: int) -> None:
        cmd = int(Cmd.SET_PARAM)

        if param_id == 0x40:                       # log level
            if not 0 <= value <= 3:
                self._ack(cmd, frame.seq, Result.OUT_OF_RANGE)
                return
            self._log_level = value
            self._dirty_param = True
            self._ack(cmd, frame.seq, Result.OK)
            return

        if param_id in CALIB_FIELDS:
            field, index = CALIB_FIELDS[param_id]
            low, high = PARAM_RANGES.get(param_id, (0, 1000))
            if field == "direction":
                in_range = value in (1, -1)
            elif field == "counts":
                in_range = COUNTS_PER_MM_RANGE[0] <= value <= \
                    COUNTS_PER_MM_RANGE[1]
            else:
                in_range = low <= value <= high
            if not in_range:
                self._ack(cmd, frame.seq, Result.OUT_OF_RANGE)
                return
            # Calibration may not move under a powered chassis (5.4).
            if self._enabled:
                self._ack(cmd, frame.seq, Result.BAD_STATE)
                return
            if index is None:
                self._calib[field] = value
            else:
                self._calib[field][index] = value
            self._dirty_calib = True
            self._ack(cmd, frame.seq, Result.OK)
            return

        if param_id in LIMIT_PARAM_IDS:
            low, high = PARAM_RANGES[param_id]
            if not low <= value <= high:
                self._ack(cmd, frame.seq, Result.OUT_OF_RANGE)
                return
            self._limits[LIMIT_PARAM_IDS[param_id]] = value
            self._dirty_param = True
            self._ack(cmd, frame.seq, Result.OK)
            return

        target = self._param_target(param_id)
        if target is None:
            self._ack(cmd, frame.seq, Result.OUT_OF_RANGE)
            return
        gain, wheels = target
        limit = 5000 if gain == 2 else 10000
        if not 0 <= value <= limit:
            self._ack(cmd, frame.seq, Result.OUT_OF_RANGE)
            return
        for wheel in wheels:
            self._gains[wheel][gain] = value / 1000.0
        self._dirty_param = True
        self._ack(cmd, frame.seq, Result.OK)

    def _handle_param_save(self, frame, now: float) -> None:
        cmd = int(Cmd.PARAM_SAVE)
        payload = frame.payload
        if len(payload) != 1 or payload[0] > 2:
            self._ack(cmd, frame.seq, Result.OUT_OF_RANGE)
            return
        scope = payload[0]
        if scope in (1, 2) and self._enabled:
            self._ack(cmd, frame.seq, Result.BAD_STATE)
            return
        if scope in (0, 2):
            self._dirty_param = False
            self._param_seq = (self._param_seq + 1) & 0xFFFF
        if scope in (1, 2):
            self._dirty_calib = False
            self._calib_seq = (self._calib_seq + 1) & 0xFFFF
        self._save_failed = False
        self._last_save_s = int(now - self._started_at)
        self._ack(cmd, frame.seq, Result.OK)
        self._emit(self._storage_payload())

    def _handle_param_reset(self, frame) -> None:
        cmd = int(Cmd.PARAM_RESET)
        payload = frame.payload
        if len(payload) != 2 or payload[1] != 0xA5 or payload[0] > 2:
            self._ack(cmd, frame.seq, Result.OUT_OF_RANGE)
            return
        scope = payload[0]
        if scope in (1, 2) and self._enabled:
            self._ack(cmd, frame.seq, Result.BAD_STATE)
            return
        if scope in (0, 2):
            self._gains = [[5.0, 6.0, 0.0] for _ in range(4)]
            self._limits = dict(DEFAULT_LIMITS)
            self._log_level = 1
            self._dirty_param = True
        if scope in (1, 2):
            self._calib = {
                key: (list(value) if isinstance(value, list) else value)
                for key, value in DEFAULT_CALIB.items()}
            self._dirty_calib = True
        self._ack(cmd, frame.seq, Result.OK)

    # -- storage reads -----------------------------------------------------

    def _handle_log_export(self, frame, now: float) -> None:
        cmd = int(Cmd.LOG_EXPORT)
        payload = frame.payload
        if len(payload) != 6:
            self._ack(cmd, frame.seq, Result.OUT_OF_RANGE)
            return
        source, action = payload[0], payload[1]
        from_seq = struct.unpack("<I", payload[2:])[0]
        if source > 1 or action > 1:
            self._ack(cmd, frame.seq, Result.OUT_OF_RANGE)
            return

        if not action:
            self._export = None
            self._ack(cmd, frame.seq, Result.OK)
            return

        if self._export is not None:
            self._ack(cmd, frame.seq, Result.BUSY)
            return

        records = self._records_for(source)
        index = 0
        if from_seq:
            index = len(records)
            for position, record in enumerate(records):
                if record["seq"] >= from_seq:
                    index = position
                    break
        self._export = {"source": source, "index": index, "next_at": now}
        self._ack(cmd, frame.seq, Result.OK)

    def _handle_crash_read(self, frame, now: float) -> None:
        cmd = int(Cmd.CRASH_READ)
        payload = frame.payload
        if len(payload) != 2:
            self._ack(cmd, frame.seq, Result.OUT_OF_RANGE)
            return
        slot, flags = payload[0], payload[1]
        # Slot numbering is checked first, exactly like the firmware: 16..0xFE
        # is a bad request (RANGE), a valid slot with no record is FAILED.
        if slot != CRASH_SLOT_LATEST and slot >= CRASH_SLOT_COUNT:
            self._ack(cmd, frame.seq, Result.OUT_OF_RANGE)
            return
        if not self._crash_records:
            self._ack(cmd, frame.seq, Result.FAILED)
            return
        if slot == CRASH_SLOT_LATEST:
            slot = len(self._crash_records) - 1
        if slot >= len(self._crash_records):
            self._ack(cmd, frame.seq, Result.FAILED)
            return
        if flags & 0x01:
            self._crash_unread = False
        self._crash_tx = {"slot": slot, "offset": 0}
        self._ack(cmd, frame.seq, Result.OK)

    def _handle_log_clear(self, frame) -> None:
        cmd = int(Cmd.LOG_CLEAR)
        payload = frame.payload
        if len(payload) != 2 or payload[1] != 0xA5 or payload[0] > 3:
            self._ack(cmd, frame.seq, Result.OUT_OF_RANGE)
            return
        source = payload[0]
        if source in (0, 3):
            self._logs = []
        if source in (1, 3):
            self._faults = []
        if source in (2, 3):
            self._crash_records = []
            self._crash_unread = False
            self._crash_tx = None
        self._ack(cmd, frame.seq, Result.OK)
        self._emit(self._storage_payload())

    @staticmethod
    def _param_target(param_id: int):
        """Return (gain_index, wheel_indices) or None for an unsupported id."""
        for wheel, ids in enumerate(PID_PARAM_IDS):
            if param_id in ids:
                return ids.index(param_id), (wheel,)
        if 0x03 <= param_id <= 0x05:
            return param_id - 0x03, (0, 2)
        if 0x06 <= param_id <= 0x08:
            return param_id - 0x06, (1, 3)
        return None

    def _ack(self, cmd: int, seq: int, result: Result) -> None:
        self._emit_frame(int(cmd) | ACK_FLAG,
                         bytes((seq & 0xFF, int(result) & 0xFF)))

    # -- reports -----------------------------------------------------------

    def _state_byte(self) -> int:
        if self._estopped:
            return 0x04
        if self._fault:
            return 0x05
        return 0x02 if self._enabled else 0x01

    def _odom_payload(self) -> tuple[int, bytes]:
        vl = self._v - self._w * (TRACK_WIDTH_MM / 2.0) / 1000.0
        vr = self._v + self._w * (TRACK_WIDTH_MM / 2.0) / 1000.0
        degrees = (math.degrees(self._theta) + 180.0) % 360.0 - 180.0
        theta = int(round(degrees * 100.0))
        payload = struct.pack("<iihhh", int(round(self._x)),
                              int(round(self._y)), theta,
                              int(round(vl)), int(round(vr)))
        return Report.ODOM, payload

    def _wheel_speed_payload(self) -> tuple[int, bytes]:
        half = TRACK_WIDTH_MM / 2.0
        left = self._v - self._w * half / 1000.0
        right = self._v + self._w * half / 1000.0
        payload = struct.pack("<4h", int(round(left)), int(round(right)),
                              int(round(left)), int(round(right)))
        return Report.WHEEL_SPEED, payload

    def _arm_payload(self) -> tuple[int, bytes]:
        payload = struct.pack("<6hhB",
                             *(int(round(v)) for v in self._joints),
                             int(round(self._gripper)),
                             self._gripper_state)
        return Report.ARM, payload

    def _status_payload(self) -> tuple[int, bytes]:
        elapsed = time.monotonic() - self._started_at
        battery_mv = 11820 - int(elapsed) + self._rng.randint(-8, 8)
        temp = 31 + self._rng.choice((0, 0, 0, 1))
        jitter = 40 + self._rng.randint(0, 70)
        payload = struct.pack("<BHHbH", self._state_byte(), self._fault,
                              battery_mv, temp, jitter)
        return Report.STATUS, payload

    def _heartbeat_payload(self) -> tuple[int, bytes]:
        self._heartbeat = (self._heartbeat + 1) & 0xFFFF
        payload = struct.pack("<BH", self._state_byte(), self._heartbeat)
        return Report.HEARTBEAT, payload

    def _event_payload(self, code: int, now: float) -> tuple[int, bytes]:
        stamp = int((now - self._started_at) * 1000.0) & 0xFFFFFFFF
        return Report.EVENT, struct.pack("<HI", code, stamp)

    # -- storage reports ---------------------------------------------------

    @staticmethod
    def _seed_logs():
        rows = (
            (12, 1, 6, "存储子系统就绪：W25Q64 8 MiB，分区表有效"),
            (12, 1, 1, "底盘标定载入：轮距 240 mm，编码器 24828 counts/m"),
            (13, 1, 2, "协议 v1 链路建立，ACK 超时 200 ms"),
            (14, 1, 5, "急停回路自检通过"),
            (15, 1, 4, "电池电压 11.8 V，正常"),
            (18, 2, 1, "速度环输出限幅：左前轮需求 620 mm/s，限到 500"),
            (19, 1, 3, "关节 2 目标角 -45.0 deg"),
            (21, 2, 2, "REPORT_WHEEL_SPEED 队列积压 2 帧，已丢弃最旧"),
            (23, 1, 6, "日志环已用 1.6 KB，擦除 11 次"),
            (26, 3, 1, "编码器 A 相计数抖动 3 counts，已滤波"),
            (28, 1, 2, "收到 LOG_EXPORT，速率上限 20 Hz"),
        )
        return [{"seq": index + 1, "ts": ts, "level": level, "tag": tag,
                 "text": text, "code": 0}
                for index, (ts, level, tag, text) in enumerate(rows)]

    @staticmethod
    def _seed_faults():
        rows = ((41, 0x0201, 2), (97, 0x0402, 2), (160, 0x0102, 3))
        return [{"seq": index + 1, "ts": ts, "level": level, "tag": 0,
                 "text": "", "code": code}
                for index, (ts, code, level) in enumerate(rows)]

    @staticmethod
    def _make_crash() -> bytes:
        header = (b"CRASH r0=0x20001234 r1=0x00000000 r2=0x0800ABCD "
                  b"psr=0x01000000 lr=0x08004A11 pc=0x08004A2E")
        body = bytes((index * 7 + 3) & 0xFF for index in range(1024))
        return (header + body)[:1024]

    def _records_for(self, source: int):
        return self._faults if source == 1 else self._logs

    def _log_used(self) -> int:
        return 128 + 64 * (len(self._logs) + len(self._faults))

    def _storage_payload(self) -> tuple[int, bytes]:
        flags = 0
        if not (self._dirty_param or self._dirty_calib):
            flags |= 0x01
        if self._param_valid:
            flags |= 0x02
        if self._calib_valid:
            flags |= 0x04
        if self._save_failed:
            flags |= 0x08
        flags |= 0x10                    # one application image, always valid
        if self._crash_unread:
            flags |= 0x40
        if self._log_wrapped:
            flags |= 0x80
        payload = struct.pack("<BBHHHIIII", flags, self._log_level,
                              self._param_seq, self._calib_seq, 0,
                              self._boot_total, self._log_used(),
                              self._log_erase, self._last_save_s)
        return Report.STORAGE, payload

    def _step_export(self, now: float) -> None:
        if self._export is None or now < self._export["next_at"]:
            return
        source = self._export["source"]
        records = self._records_for(source)
        index = self._export["index"]

        if index >= len(records):
            self._emit(self._log_end(source, 0xFFFF))
            self._export = None
            return

        record = records[index]
        self._export["index"] = index + 1
        self._export["next_at"] = now + LOG_EXPORT_PERIOD_S
        self._emit(self._log_record(source, record))

        if index + 1 >= len(records):
            # The end frame closes the export; every record before it carries
            # "more" (protocol.md 5.5.6).
            self._emit(self._log_end(source, (record["seq"] + 1) & 0xFFFFFFFF))
            self._export = None

    def _log_record(self, source: int, record: dict) -> tuple[int, bytes]:
        text = record["text"]
        while len(text.encode("utf-8")) > 49:
            text = text[:-1]
        data = text.encode("utf-8")
        head = struct.pack("<BBBBIIHB", source, record["level"],
                           record["tag"], 0x01, record["ts"],
                           record["seq"], record["code"], len(data))
        return Report.LOG, head + data

    @staticmethod
    def _log_end(source: int, seq: int) -> tuple[int, bytes]:
        head = struct.pack("<BBBBIIHB", source, 0, 0, 0x02, 0, seq, 0, 0)
        return Report.LOG, head

    def _step_crash(self) -> None:
        if self._crash_tx is None:
            return
        slot = self._crash_tx["slot"]
        data = self._crash_records[slot]
        offset = self._crash_tx["offset"]
        chunk = data[offset:offset + CRASH_CHUNK_BYTES]
        more = offset + len(chunk) < len(data)
        head = struct.pack("<BBHHB", slot, 0x01 if more else 0x00,
                           offset, len(data), len(chunk))
        self._emit((Report.CRASH, head + chunk))
        self._crash_tx["offset"] = offset + len(chunk)
        if not more:
            self._crash_tx = None

    # -- outbound ----------------------------------------------------------

    def _emit(self, message: tuple[int, bytes]) -> None:
        self._emit_frame(*message)

    def _emit_frame(self, cmd: int, payload: bytes) -> None:
        if self._stop.is_set():
            return
        raw = build_frame(cmd, self._tx_seq, payload)
        self._tx_seq = (self._tx_seq + 1) & 0xFF
        for frame in self._out_parser.feed(raw):
            self.rx_frames += 1
            self.frameReceived.emit(frame)
