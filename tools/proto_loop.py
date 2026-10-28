#!/usr/bin/env python3
"""Closed-loop test for the RK3566 <-> STM32 serial protocol (docs/protocol.md).

Two modes:

    python tools/proto_loop.py --list
    python tools/proto_loop.py --port COM3 --monitor          # passive listen
    python tools/proto_loop.py --port COM3 --auto             # safe suite
    python tools/proto_loop.py --port COM3 --auto --move      # + motion + timeout
    python tools/proto_loop.py --sim --auto --move            # against the GUI simulator

Why the split: the only way to prove the link is to send a command and check
both the ACK and the following report, so the suite always starts with an ACK
and then looks at STATUS/ODOM/EVENT. `--move` is opt-in because it commands
200 mm/s: without it the wheels never turn and the suite still covers enable,
ESTOP, clear-fault, unknown command and length rejection.

The frame codec is imported from tools/host_gui/protocol.py on purpose. One
implementation, already covered by tests, is what makes this a check of the
firmware rather than of a second host-side encoder that could drift.
"""

from __future__ import annotations

import argparse
import queue
import sys
import threading
import time
from collections import Counter, deque
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS_DIR))

from host_gui import protocol as P  # noqa: E402

ACK_TIMEOUT_S = 0.30
REPORT_TIMEOUT_S = 0.60
BYTE_GAP_RESET_S = 0.050
RX_POLL_S = 0.002            # idle sleep when the port has nothing buffered
DEFAULT_BAUD = 115200
DEFAULT_SPEED = 200          # mm/s, used by --move
FAULT_COMM_TIMEOUT = 0x0202
FAULT_BATTERY_LOW = 0x0401
FAULT_ESTOP = 0x0501
BATTERY_LOW_MV = 6800        # APP_BATTERY_LOW_MV in app_robot_config.h (2S)

PASS, FAIL, SKIP, INFO = "PASS", "FAIL", "SKIP", "INFO"


# --------------------------------------------------------------- transport

class SerialLink:
    """pyserial plus a reader thread; decodes frames off the UI thread."""

    def __init__(self, port: str, baud: int) -> None:
        import serial

        self._serial = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            # Non-blocking reads. With a timeout set, Windows ReadFile keeps
            # accumulating until the requested 512 bytes arrive as long as the
            # firmware's report stream never leaves a >timeout gap -- one read
            # then sat for 300 ms and held an ACK hostage. Poll instead.
            timeout=0,
            write_timeout=0.5,
        )
        self.frames: queue.Queue = queue.Queue()
        self.parser = P.FrameParser()
        self.rx_bytes = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="proto-rx",
                                        daemon=True)
        self._thread.start()

    def _run(self) -> None:
        last = 0.0
        while not self._stop.is_set():
            try:
                pending = self._serial.in_waiting
                data = self._serial.read(pending) if pending else b""
            except Exception:                      # port yanked
                return
            if not data:
                time.sleep(RX_POLL_S)
                continue
            now = time.monotonic()
            # protocol.md section 9: 50 ms without a byte abandons the frame.
            if (now - last) > BYTE_GAP_RESET_S:
                self.parser.reset()
            last = now
            self.rx_bytes += len(data)
            for frame in self.parser.feed(data):
                self.frames.put(frame)

    @property
    def frames_ok(self) -> int:
        return self.parser.frames_ok

    @property
    def crc_errors(self) -> int:
        return self.parser.crc_errors

    def send(self, data: bytes) -> None:
        self._serial.write(data)

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=0.5)
        try:
            self._serial.close()
        except Exception:
            pass


class SimLink:
    """The GUI's virtual MCU, so the suite can be rehearsed without a board."""

    def __init__(self) -> None:
        from PySide6.QtCore import Qt

        from host_gui.simulator import SimulatorLink

        self.frames: queue.Queue = queue.Queue()
        self._link = SimulatorLink()
        self._slot = self._on_frame
        self._link.frameReceived.connect(self._slot, Qt.DirectConnection)
        self._link.open()

    def _on_frame(self, frame) -> None:
        self.frames.put(frame)

    @property
    def frames_ok(self) -> int:
        return self._link.rx_frames

    @property
    def crc_errors(self) -> int:
        return self._link.crc_errors

    def send(self, data: bytes) -> None:
        self._link.send(data)

    def close(self) -> None:
        self._link.close()


# ------------------------------------------------------------------ client

class Client:
    def __init__(self, link, echo: bool = True, echo_reports: bool = False,
                 echo_odom: bool = False) -> None:
        self.link = link
        self.echo = echo
        self.echo_reports = echo_reports
        self.echo_odom = echo_odom
        self._seq = 0
        self.rx_frames = 0
        self.seen: Counter = Counter()
        self.history: deque = deque(maxlen=4000)
        self.checks: list[tuple[str, str, str]] = []

    # -- wire -----------------------------------------------------------

    def send(self, cmd, payload: bytes = b"") -> int:
        seq = self._seq
        self._seq = (self._seq + 1) & 0xFF
        data = P.build_frame(cmd, seq, payload)
        self.link.send(data)
        if self.echo:
            frame = P.Frame(cmd=int(cmd), seq=seq, payload=payload, raw=data)
            print(f"  > {P.describe(frame)}")
        return seq

    def _next(self, timeout: float):
        try:
            frame = self.link.frames.get(timeout=timeout)
        except queue.Empty:
            return None
        self.rx_frames += 1
        self.seen[frame.cmd] += 1
        self.history.append((time.monotonic(), frame))
        if self.echo:
            report = 0xC0 <= frame.cmd <= 0xFF
            interesting = (not report) or self.echo_reports \
                or frame.cmd == int(P.Report.EVENT) \
                or (self.echo_odom and frame.cmd == int(P.Report.ODOM))
            if interesting:
                print(f"  < {P.describe(frame)}")
        return frame

    # -- waiting --------------------------------------------------------

    def wait(self, pred, timeout: float, since: float | None = None):
        """Return the first matching frame at/after `since`.

        Checking history first matters: an EVENT often arrives between the
        command and its ACK, and a plain queue scan would lose it.
        """
        start = time.monotonic() if since is None else since

        for stamp, frame in self.history:
            if stamp >= start and pred(frame):
                return frame

        end = start + timeout
        while True:
            remain = end - time.monotonic()
            if remain <= 0:
                return None
            frame = self._next(remain)
            if frame is None:
                return None
            if pred(frame):
                return frame

    def expect_ack(self, cmd, seq: int, timeout: float = ACK_TIMEOUT_S):
        return self.wait(lambda f: f.is_ack and f.acked_cmd == int(cmd)
                         and f.acked_seq == seq, timeout)

    def wait_status(self, pred, timeout: float = REPORT_TIMEOUT_S,
                    since: float | None = None):
        """Wait for a REPORT_STATUS whose decoded fields satisfy `pred`."""
        def match(frame) -> bool:
            if frame.cmd != int(P.Report.STATUS):
                return False
            status = P.decode_status(frame.payload)
            return bool(status) and pred(status)

        frame = self.wait(match, timeout, since=since)
        return _status_of(frame)

    def collect(self, duration: float) -> Counter:
        counts: Counter = Counter()
        end = time.monotonic() + duration
        while True:
            remain = end - time.monotonic()
            if remain <= 0:
                return counts
            frame = self._next(min(remain, 0.2))
            if frame is not None:
                counts[frame.cmd] += 1

    # -- results --------------------------------------------------------

    def check(self, name: str, status: str, detail: str = "") -> None:
        self.checks.append((name, status, detail))
        line = f"[{status}] {name}"
        if detail:
            line += f"  ({detail})"
        print(line)

    def summary(self) -> int:
        failed = [c for c in self.checks if c[1] == FAIL]
        passed = [c for c in self.checks if c[1] == PASS]
        skipped = [c for c in self.checks if c[1] == SKIP]
        print("\n" + "-" * 58)
        print(f"通过 {len(passed)} / 失败 {len(failed)} / 跳过 {len(skipped)}")
        if failed:
            for name, _, detail in failed:
                print(f"  FAIL: {name} ({detail})")
        print(f"链路统计: 合法帧 {self.link.frames_ok}, "
              f"CRC 错误 {self.link.crc_errors}, 收到 {self.rx_frames} 帧")
        return 1 if failed else 0


# ------------------------------------------------------------------ helpers

def _state_name(state: int) -> str:
    return P.STATE_NAMES.get(state, f"0x{state:02X}")


def _fault_name(code: int) -> str:
    return P.FAULT_NAMES.get(code, f"0x{code:04X}")


def _result_name(ack) -> str:
    if ack is None:
        return "无 ACK"
    return P.RESULT_NAMES.get(P.Result(ack.result), f"0x{ack.result:02X}")


def _status_of(frame):
    return P.decode_status(frame.payload) if frame is not None else None


# -------------------------------------------------------------- the suite

def run_auto(client: Client, move: bool, simulation: bool) -> None:
    print("\n== M2 串口协议闭环 ==")
    if move:
        print("!! 即将下发速度指令，请确认底盘已架空或前方留出 1 m 空间")
    else:
        print("静态模式：不下发速度指令；加 --move 测运动与 200 ms 断流停车")

    # 1. The MCU streams reports with no prompting. If nothing arrives, no
    #    later check can be trusted, so abort with the usual causes.
    counts = client.collect(2.0)
    hb = counts[int(P.Report.HEARTBEAT)]
    status_n = counts[int(P.Report.STATUS)]
    odom_n = counts[int(P.Report.ODOM)]
    arm_n = counts[int(P.Report.ARM)]

    client.check("链路在线", PASS if client.rx_frames >= 10 else FAIL,
                 f"2 s 收到 {client.rx_frames} 帧")
    if client.rx_frames < 10:
        client.check("中止", FAIL,
                     "检查 COM 口/波特率/TX-RX 交叉，以及固件是否已烧录")
        return

    client.check("HEARTBEAT 10 Hz", PASS if hb >= 10 else FAIL, f"{hb} 帧/2 s")
    client.check("STATUS 10 Hz", PASS if status_n >= 10 else FAIL,
                 f"{status_n} 帧/2 s")
    client.check("ODOM 50 Hz", PASS if odom_n >= 50 else FAIL,
                 f"{odom_n} 帧/2 s")
    client.check("ARM 20 Hz", PASS if arm_n >= 20 else FAIL, f"{arm_n} 帧/2 s")

    boot = client.wait_status(lambda s: True, 0.3)
    if boot:
        client.check("上电状态", INFO,
                     f"state={_state_name(boot['state'])} "
                     f"fault={_fault_name(boot['fault'])} "
                     f"batt={boot['battery_mv']} mV")

    # REPORT_STATUS carries one code, so a pack under the firmware threshold
    # keeps state at FAULT(0x0401) whatever the host does -- MANUAL and IDLE are
    # unreachable. That is a bench precondition, not a protocol bug, so those
    # two checks say SKIP with the voltage instead of turning the run red.
    batt_mv = boot["battery_mv"] if boot else 0
    pack_low = 0 < batt_mv < BATTERY_LOW_MV
    if pack_low:
        client.check("电池电压", SKIP,
                     f"{batt_mv} mV < {BATTERY_LOW_MV} mV，0x0401 锁存；"
                     "MANUAL/IDLE 需充电后复测")

    # 2. Enable. The ACK proves the command path; STATUS proves the handler
    #    actually changed the machine, not just replied.
    t = time.monotonic()
    seq = client.send(P.Cmd.SET_ENABLE, P.set_enable_payload(True))
    ack = client.expect_ack(P.Cmd.SET_ENABLE, seq)
    client.check("SET_ENABLE(1) 有 ACK", PASS if ack else FAIL,
                 _result_name(ack))
    if ack is not None and ack.result == P.Result.OK:
        status = client.wait_status(lambda s: s["state"] in (2, 5), 1.0,
                                    since=t)
        state = status["state"] if status else -1
        if state == 2:
            client.check("使能后进入 MANUAL", PASS, f"state={_state_name(state)}")
        elif state == 5 and pack_low and status["fault"] == FAULT_BATTERY_LOW:
            client.check("使能后进入 MANUAL", SKIP,
                         f"电池 {batt_mv} mV 低压，state 停在 FAULT")
        elif state == 5:
            client.check("使能后进入 MANUAL", FAIL,
                         f"state=FAULT，fault={_fault_name(status['fault'])}")
        else:
            client.check("使能后进入 MANUAL", FAIL, "没有收到状态变化")

    # 3. Velocity stream, then stop. The MCU requires SET_VELOCITY at 50 Hz
    #    whenever the chassis is enabled -- an idle enabled robot that stops
    #    hearing from the host must stop, so the stream continues with zero
    #    speed in static mode. That makes the watchdog testable without
    #    turning a wheel.
    stream_speed = DEFAULT_SPEED if move else 0
    peak = 0
    deadline = time.monotonic() + 2.0
    next_tx = time.monotonic()
    while True:
        now = time.monotonic()
        if now >= deadline:
            break
        if now >= next_tx:
            client.send(P.Cmd.SET_VELOCITY,
                        P.set_velocity_payload(stream_speed, 0))
            next_tx = max(next_tx + 0.02, now)
        frame = client._next(0.004)
        if frame is not None and frame.cmd == int(P.Report.ODOM):
            odom = P.decode_odom(frame.payload)
            if odom:
                peak = max(peak, abs(odom["vl_mm_s"]),
                           abs(odom["vr_mm_s"]))

    if move:
        client.check("速度指令被执行",
                     PASS if peak >= DEFAULT_SPEED * 0.5 else FAIL,
                     f"ODOM 峰值 {peak} mm/s，目标 {DEFAULT_SPEED} mm/s")
    else:
        client.check("零速指令下不动", PASS if peak < 30 else FAIL,
                     f"ODOM 峰值 {peak} mm/s")

    # Stop sending. The chassis must zero the output within 200 ms and latch
    # 0x0202, which is the last line of defence against a dead host. The
    # simulator only zeroes the speed; the latch exists in firmware, so this
    # check is skipped there rather than reported as a failure.
    t = time.monotonic()
    if simulation:
        client.check("断流 200 ms → 0x0202", SKIP,
                     "模拟器不产生通信超时故障，需真机验证")
    else:
        timeout_frame = client.wait_status(
            lambda s: s["fault"] == FAULT_COMM_TIMEOUT, 1.0, since=t)
        client.check("断流 200 ms → 0x0202", PASS if timeout_frame else FAIL,
                     _fault_name(timeout_frame["fault"]) if timeout_frame
                     else "没有等到通信超时故障")

    def stopped(frame) -> bool:
        if frame.cmd != int(P.Report.ODOM):
            return False
        odom = P.decode_odom(frame.payload)
        return bool(odom) and abs(odom["vl_mm_s"]) < 30 \
            and abs(odom["vr_mm_s"]) < 30

    if move:
        client.check("断流后轮速归零", PASS if client.wait(stopped, 0.5) else FAIL)
    else:
        client.check("断流后轮速归零", SKIP, "静态模式没有转动")

    # 4. ESTOP: ACK, EVENT, and the state byte. All three must agree.
    t = time.monotonic()
    seq = client.send(P.Cmd.ESTOP)
    ack = client.expect_ack(P.Cmd.ESTOP, seq)
    client.check("ESTOP 有 ACK", PASS if ack else FAIL, _result_name(ack))

    event = client.wait(
        lambda f: f.cmd == int(P.Report.EVENT)
        and (P.decode_event(f.payload) or {}).get("code") == FAULT_ESTOP,
        REPORT_TIMEOUT_S, since=t)
    client.check("ESTOP 事件帧 0x0501", PASS if event else FAIL)

    estop_status = client.wait_status(lambda s: s["state"] == 4, 1.0, since=t)
    client.check("ESTOP 状态 4", PASS if estop_status else FAIL,
                 _state_name(estop_status["state"]) if estop_status else "")

    # 5. Clear. A low pack is the one condition the firmware can still
    #    measure, so STATE here is a correct answer, not a bug.
    t = time.monotonic()
    seq = client.send(P.Cmd.CLEAR_FAULT, P.clear_fault_payload())
    ack = client.expect_ack(P.Cmd.CLEAR_FAULT, seq)
    client.check("CLEAR_FAULT 有 ACK", PASS if ack else FAIL, _result_name(ack))

    if ack is not None and ack.result == P.Result.OK:
        cleared = client.wait_status(lambda s: s["fault"] == 0 and s["state"] == 1,
                                     1.0, since=t)
        if cleared:
            client.check("清除后回到 IDLE", PASS,
                         f"state={_state_name(cleared['state'])}")
        elif pack_low:
            client.check("清除后回到 IDLE", SKIP,
                         f"已清除，但电池 {batt_mv} mV 仍低压，0x0401 重新上报")
        else:
            after = client.wait_status(lambda s: True, 0.5)
            client.check("清除后回到 IDLE", FAIL,
                         f"state={_state_name(after['state'])}, "
                         f"fault={_fault_name(after['fault'])}" if after
                         else "没有收到 STATUS")
    elif ack is not None and ack.result == P.Result.BAD_STATE:
        client.check("清除后回到 IDLE", SKIP,
                     "电池电压仍低于阈值，按协议 §8 拒绝清除")
    else:
        client.check("清除后回到 IDLE", FAIL, "清除失败")

    # 6. Rejections. These prove the dispatcher checks the table instead of
    #    forwarding anything it sees.
    t = time.monotonic()
    client.send(0x3A)
    ack = client.wait(lambda f: f.is_ack and f.acked_cmd == 0x3A, 0.4, since=t)
    client.check("未知命令 0x3A → UNKNOWN_CMD",
                 PASS if ack and ack.result == P.Result.UNKNOWN_CMD else FAIL,
                 _result_name(ack))

    t = time.monotonic()
    seq = client.send(P.Cmd.SET_ENABLE, b"\x01\x00")
    ack = client.wait(lambda f: f.is_ack and f.acked_cmd == int(P.Cmd.SET_ENABLE)
                      and f.acked_seq == seq, 0.4, since=t)
    client.check("长度错误 → RANGE",
                 PASS if ack and ack.result == P.Result.OUT_OF_RANGE else FAIL,
                 _result_name(ack))

    t = time.monotonic()
    seq = client.send(P.Cmd.SET_GRIPPER, P.set_gripper_payload(1, 50))
    ack = client.wait(lambda f: f.is_ack and f.acked_cmd == int(P.Cmd.SET_GRIPPER)
                      and f.acked_seq == seq, 0.4, since=t)
    client.check("SET_GRIPPER 有 ACK", PASS if ack else FAIL, _result_name(ack))
    if ack is not None and not simulation:
        # No gripper hardware on this board: the handler must say so rather
        # than pretend the gripper moved.
        client.check("SET_GRIPPER 如实返回 FAILED",
                     PASS if ack.result == P.Result.FAILED else FAIL,
                     _result_name(ack))

    # 7. Disable and link integrity.
    t = time.monotonic()
    seq = client.send(P.Cmd.SET_ENABLE, P.set_enable_payload(False))
    ack = client.expect_ack(P.Cmd.SET_ENABLE, seq)
    client.check("SET_ENABLE(0) 有 ACK", PASS if ack else FAIL, _result_name(ack))

    client.check("下行合法（无 CRC 错误）",
                 PASS if client.link.crc_errors == 0 else FAIL,
                 f"CRC 错误 {client.link.crc_errors} 帧")


# ---------------------------------------------------------------- monitor

def run_monitor(client: Client, duration: float | None) -> None:
    print("\n监听中，Ctrl-C 退出\n")
    start = time.monotonic()
    window_start = start
    window: Counter = Counter()

    try:
        while True:
            if duration is not None and time.monotonic() - start >= duration:
                break
            frame = client._next(0.2)
            if frame is not None:
                window[frame.cmd] += 1
            now = time.monotonic()
            if now - window_start >= 2.0:
                span = now - window_start
                parts = []
                for cmd, name in ((P.Report.ODOM, "ODOM"),
                                  (P.Report.ARM, "ARM"),
                                  (P.Report.STATUS, "STATUS"),
                                  (P.Report.HEARTBEAT, "HB"),
                                  (P.Report.EVENT, "EVT")):
                    count = window[int(cmd)]
                    if count:
                        parts.append(f"{name} {count / span:5.1f} Hz")
                print(f"-- 速率: {'  '.join(parts) or '无帧'}"
                      f"  CRC 错误 {client.link.crc_errors}")
                window.clear()
                window_start = now
    except KeyboardInterrupt:
        pass

    print(f"\n共 {client.rx_frames} 帧，CRC 错误 {client.link.crc_errors}，"
          f"收到 {client.link.rx_bytes if hasattr(client.link, 'rx_bytes') else '-'} 字节")


# ------------------------------------------------------------------- main

def list_ports() -> None:
    try:
        from serial.tools import list_ports as lp
    except ImportError:
        print("未安装 pyserial：pip install pyserial")
        return
    ports = sorted(lp.comports(), key=lambda item: item.device)
    if not ports:
        print("没有找到串口设备。")
        return
    for port in ports:
        print(f"{port.device:<12} {port.description}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="M2 串口协议闭环测试（docs/protocol.md v1）")
    parser.add_argument("--list", action="store_true", help="列出串口后退出")
    parser.add_argument("--port", help="串口名，例如 COM3 或 /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--sim", action="store_true",
                        help="接内置模拟器，不需要硬件")
    parser.add_argument("--auto", action="store_true", help="跑验收序列")
    parser.add_argument("--move", action="store_true",
                        help="允许底盘转动（测速度与断流停车）")
    parser.add_argument("--monitor", action="store_true",
                        help="只监听并打印（默认行为）")
    parser.add_argument("--duration", type=float,
                        help="监听模式持续时间，秒")
    parser.add_argument("--verbose", action="store_true",
                        help="连 50 Hz 的 ODOM 也打印")
    args = parser.parse_args(argv)

    if args.list:
        list_ports()
        return 0

    if not args.sim and not args.port:
        parser.error("需要 --port COM3，或 --sim")

    link = SimLink() if args.sim else SerialLink(args.port, args.baud)
    # The acceptance run prints commands, ACKs and events; periodic reports
    # would bury them under 50 lines per second. Monitor mode prints them all.
    client = Client(link, echo=True,
                    echo_reports=(not args.auto) or args.verbose,
                    echo_odom=args.verbose)

    print(f"== {'模拟器' if args.sim else f'{args.port} @ {args.baud}'} ==")

    try:
        if args.auto:
            run_auto(client, args.move, args.sim)
            return client.summary()
        run_monitor(client, args.duration)
        return 0
    finally:
        link.close()


if __name__ == "__main__":
    sys.exit(main())
