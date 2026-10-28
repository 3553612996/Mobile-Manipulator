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

from .protocol import (ACK_FLAG, Cmd, FrameParser, PARAM_NAMES, Report, Result,
                       build_frame)

TICK_S = 0.005
VELOCITY_TIMEOUT_S = 0.200
ODOM_PERIOD_S = 0.020
ARM_PERIOD_S = 0.050
STATUS_PERIOD_S = 0.100
HEARTBEAT_PERIOD_S = 0.100
TRACK_WIDTH_MM = 200.0
JOINT_RATE = 6000.0        # centi-degrees per second
GRIPPER_RATE = 500.0       # position units per second


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
        self._heartbeat = 0
        self._tx_seq = 0
        self._rng = random.Random(7)
        self._out_parser = FrameParser()
        self._in_parser = FrameParser()
        self._started_at = time.monotonic()

    # -- worker ------------------------------------------------------------

    def _run(self) -> None:
        last = time.monotonic()
        next_report = {"odom": 0.0, "arm": 0.0,
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

            for key, period, builder in (
                    ("odom", ODOM_PERIOD_S, self._odom_payload),
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
            self._ack(cmd, frame.seq,
                      Result.OK if param_id in PARAM_NAMES
                      else Result.OUT_OF_RANGE)
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
