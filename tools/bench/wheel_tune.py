#!/usr/bin/env python3
"""Four-wheel bring-up measurements: motor dead zone and speed-loop response.

    python tools/bench/wheel_tune.py --list
    python tools/bench/wheel_tune.py --port COM5 deadzone --yes
    python tools/bench/wheel_tune.py --port COM5 deadzone --wheels A --step 40
    python tools/bench/wheel_tune.py --port COM5 step --speed 200 --yes
    python tools/bench/wheel_tune.py --port COM5 step --speed 200 \\
        --kp 1.0,1.5 --ki 0.6 --yes

Two measurements, both against the real firmware:

deadzone  Drives one wheel at a time in open loop through SET_PARAM 0x30..0x33
          (test duty, permille). The firmware reports the duty it actually
          wrote (0xC7) and the encoder speed (0xC6) at 50 Hz, so the result is
          "smallest duty that makes this wheel turn", per wheel and per
          direction, not an estimate from the command we sent.

step      Closed loop. Streams SET_VELOCITY at 50 Hz and records the wheel
          speed response; prints rise time, overshoot, steady-state error and
          ripple per wheel, so two gain sets can be compared with numbers
          instead of by eye on the GUI chart.

The gains are applied with SET_PARAM 0x20..0x2B (value = gain * 1000) so a run
is "set gains, measure" without reflashing.

Safety: the wheels DO move. Lift the chassis or leave 1 m clear; every command
needs --yes. The MCU also refuses test duty unless the chassis is enabled, caps
it at +-400 permille, and drops a wheel that stops being refreshed for 300 ms.
On exit (including Ctrl-C) the tool disables the chassis.

Results are printed as a Markdown table and appended to CSV for
docs/bench/pid-tuning.md.
"""

from __future__ import annotations

import argparse
import csv
import statistics
import sys
import time
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(TOOLS_DIR))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from host_gui import protocol as P  # noqa: E402
from proto_loop import Client, SerialLink  # noqa: E402

WHEELS = ("A", "B", "C", "D")          # FL, FR, RL, RR
DUTY_REFRESH_S = 0.10                  # firmware expiry is 300 ms
VELOCITY_PERIOD_S = 0.02               # 50 Hz, firmware watchdog is 200 ms
COAST_S = 0.25                         # zero-duty settle before each step
DEFAULT_POLL_S = 0.005


# ------------------------------------------------------------------ plumbing

class Tune:
    """Thin wrapper over proto_loop.Client: enable/disable and report sinks."""

    def __init__(self, port: str, baud: int, echo: bool = False) -> None:
        self.link = SerialLink(port, baud)
        self.client = Client(self.link, echo=echo)

    def close(self) -> None:
        self._safe_disable()
        self.link.close()

    # -- commands -------------------------------------------------------

    def set_param(self, param_id: int, value: int, timeout: float = 0.4):
        seq = self.client.send(P.Cmd.SET_PARAM,
                               P.set_param_payload(param_id, value))
        ack = self.client.expect_ack(P.Cmd.SET_PARAM, seq, timeout)
        if ack is None:
            raise RuntimeError(f"SET_PARAM 0x{param_id:02X} 无 ACK")
        if ack.result != P.Result.OK:
            name = P.RESULT_NAMES.get(P.Result(ack.result), hex(ack.result))
            raise RuntimeError(f"SET_PARAM 0x{param_id:02X} 被拒绝: {name}")
        return ack

    def enable(self, on: bool, timeout: float = 0.6) -> None:
        seq = self.client.send(P.Cmd.SET_ENABLE, P.set_enable_payload(on))
        ack = self.client.expect_ack(P.Cmd.SET_ENABLE, seq, timeout)
        if ack is None or ack.result != P.Result.OK:
            name = "无 ACK" if ack is None else \
                P.RESULT_NAMES.get(P.Result(ack.result), hex(ack.result))
            raise RuntimeError(f"SET_ENABLE({int(on)}) 失败: {name}")

    def _safe_disable(self) -> None:
        try:
            self.enable(False)
        except Exception:
            pass

    # -- sampling -------------------------------------------------------

    def pump(self, until: float, period: float, emit, sink) -> None:
        """Call `emit()` every `period` until `until`; feed frames to `sink`."""
        next_tx = time.monotonic()
        while True:
            now = time.monotonic()
            if now >= until:
                return
            if now >= next_tx:
                emit()
                next_tx = max(next_tx + period, time.monotonic())
            frame = self.client._next(DEFAULT_POLL_S)
            if frame is not None:
                sink(time.monotonic(), frame)


def _decode(frame) -> tuple[list[int] | None, list[int] | None]:
    """(speeds_mm_s, duty_permille) for the two wheel reports, else (None, None)."""
    if frame.cmd == int(P.Report.WHEEL_SPEED):
        data = P.decode_wheel_speed(frame.payload)
        if data:
            return ([data["fl_mm_s"], data["fr_mm_s"],
                     data["rl_mm_s"], data["rr_mm_s"]], None)
    if frame.cmd == int(P.Report.WHEEL_DUTY):
        data = P.decode_wheel_duty(frame.payload)
        if data:
            return (None, [data["fl_permille"], data["fr_permille"],
                           data["rl_permille"], data["rr_permille"]])
    return (None, None)


# ---------------------------------------------------------------- dead zone

def hold_duty(tune: Tune, wheel: int, duty: int, hold_s: float) -> dict:
    """Drive one wheel open loop for `hold_s` and summarise the response.

    The duty is re-sent every DUTY_REFRESH_S because the firmware expires it
    after 300 ms; the measured speed is taken from the tail of the window so
    the start-up transient does not enter the average.
    """
    speeds: list[tuple[float, int]] = []    # (t, this wheel's mm/s)
    applied = [-1]
    rejected: list[int] = []
    t0 = time.monotonic()

    # Refresh without waiting for the ACK: a blocking wait would swallow the
    # wheel-speed frames this measurement is built on. ACK results are checked
    # after the hold instead.
    def emit() -> None:
        tune.client.send(P.Cmd.SET_PARAM,
                         P.set_param_payload(P.TEST_DUTY_IDS[wheel], duty))

    def sink(t: float, frame) -> None:
        if frame.is_ack and frame.acked_cmd == int(P.Cmd.SET_PARAM):
            if frame.result != P.Result.OK:
                rejected.append(frame.result)
            return
        wheel_speeds, wheel_duty = _decode(frame)
        if wheel_speeds is not None:
            speeds.append((t, wheel_speeds[wheel]))
        if wheel_duty is not None:
            applied[0] = wheel_duty[wheel]

    tune.pump(t0 + hold_s, DUTY_REFRESH_S, emit, sink)

    if rejected:
        name = P.RESULT_NAMES.get(P.Result(rejected[-1]), hex(rejected[-1]))
        raise RuntimeError(f"测试占空比 {duty}‰ 被拒绝: {name}"
                           "（底盘是否已使能？是否在急停？）")

    tail = [abs(v) for t, v in speeds if t >= t0 + hold_s * 0.6]
    mean = statistics.fmean(tail) if tail else 0.0
    peak = max(tail) if tail else 0
    return {"duty": duty, "mean_mm_s": mean, "peak_mm_s": peak,
            "applied": applied[0], "samples": len(speeds)}


def measure_wheel(tune: Tune, wheel: int, direction: int, step: int,
                  max_duty: int, hold_s: float, threshold: float,
                  verbose: bool = True) -> dict | None:
    """Coarse ramp then bisection, for one wheel and one sign of duty."""

    def tries(duty: int) -> dict:
        hold_duty(tune, wheel, 0, COAST_S)      # settle before the step
        result = hold_duty(tune, wheel, duty, hold_s)
        if verbose:
            print(f"    {WHEELS[wheel]} duty={duty:+5d} -> "
                  f"{result['mean_mm_s']:7.1f} mm/s")
        return result

    # Phase 1: ramp up in `step` until the wheel turns. Starting the ramp at
    # `step` (not 0) keeps a failed search cheap.
    previous = 0
    found = None
    for duty in range(step, max_duty + 1, step):
        result = tries(direction * duty)
        if result["mean_mm_s"] >= threshold:
            found = (previous, duty, result)
            break
        previous = duty

    if found is None:
        return None

    low, high, first = found
    # Phase 2: bisect the one-step window. Four halvings turn a 40 permille
    # step into 2.5, which is finer than the driver resolution matters for.
    for _ in range(4):
        if high - low <= 1:
            break
        mid = (low + high) // 2
        result = tries(direction * mid)
        if result["mean_mm_s"] >= threshold:
            high = mid
        else:
            low = mid

    # Re-measure at the decision point so the table carries a real speed.
    final = tries(direction * high)
    hold_duty(tune, wheel, 0, COAST_S)

    return {"wheel": WHEELS[wheel], "dir": "+" if direction > 0 else "-",
            "deadzone": high, "speed_at_deadzone": final["mean_mm_s"],
            "speed_monotonic_at": first["mean_mm_s"],
            "applied_at_deadzone": final["applied"]}


def run_deadzone(tune: Tune, args) -> int:
    wheels = _parse_wheels(args.wheels)
    print(f"\n== 死区扫描 (步长 {args.step}‰, 上限 {args.max_duty}‰, "
          f"判定 {args.threshold:.0f} mm/s) ==")
    print("   每个轮子单独驱动，其余三轮保持在 0\n")

    rows = []
    tune.enable(True)

    for wheel in wheels:
        # Enter test mode once per wheel so the very first command lands while
        # the chassis is enabled and the 200 ms command watchdog is still quiet.
        tune.set_param(P.TEST_DUTY_IDS[wheel], 0)

        for direction in (1, -1):
            label = "正向" if direction > 0 else "反向"
            print(f"  {WHEELS[wheel]} 轮 {label}:")
            row = measure_wheel(tune, wheel, direction, args.step,
                                args.max_duty, args.hold, args.threshold)
            if row is None:
                print(f"    !! {args.max_duty}‰ 内未起转")
                rows.append({"wheel": WHEELS[wheel],
                             "dir": "+" if direction > 0 else "-",
                             "deadzone": "", "speed_at_deadzone": "",
                             "speed_monotonic_at": "", "applied_at_deadzone": ""})
            else:
                rows.append(row)

    # Leaving test mode: disable drops the bridges and clears the test state.
    tune.enable(False)

    _print_deadzone_table(rows, args.threshold)
    _append_csv(args.out, "deadzone", rows)
    return 0


def _print_deadzone_table(rows, threshold: float) -> None:
    print("\n| 轮 | 方向 | 死区 (‰) | 死区处速度 (mm/s) | 起转步速度 (mm/s) "
          "| 回读占空比 (‰) |")
    print("|----|------|----------|-------------------|-------------------|"
          "----------------|")
    for row in rows:
        if row["deadzone"] == "":
            print(f"| {row['wheel']} | {row['dir']} | 未起转 | - | - | - |")
            continue
        print(f"| {row['wheel']} | {row['dir']} | {row['deadzone']} "
              f"| {row['speed_at_deadzone']:.0f} "
              f"| {row['speed_monotonic_at']:.0f} "
              f"| {row['applied_at_deadzone']} |")
    print(f"\n判定阈值 {threshold:.0f} mm/s；占空比以满量程 1000‰ 计。")


# ------------------------------------------------------------ step response

def _step_metrics(t: list[float], v: list[int], target: int) -> dict:
    if not v:
        return {"rise_s": None, "overshoot_pct": None,
                "steady_mm_s": None, "error_pct": None,
                "ripple_mm_s": None}

    sign = 1 if target >= 0 else -1
    t0 = t[0]
    steady_window = [x for ts, x in zip(t, v) if ts >= t[-1] - 0.8]
    steady = statistics.fmean(steady_window) if steady_window else v[-1]

    # 10 % -> 90 % of the *achieved* steady value, not of the target: a wheel
    # that never reaches target still gets a meaningful rise time.
    lo_level = 0.1 * steady
    hi_level = 0.9 * steady
    t_lo = t_hi = None
    for ts, x in zip(t, v):
        if t_lo is None and (x - lo_level) * sign >= 0:
            t_lo = ts
        if t_hi is None and (x - hi_level) * sign >= 0:
            t_hi = ts
            break
    rise = None if (t_lo is None or t_hi is None) else t_hi - t_lo

    peak = max(v, key=lambda x: x * sign)
    overshoot = (abs(peak) / abs(steady) - 1.0) * 100.0 if steady else None
    error = ((steady - target) / target * 100.0) if target else None
    ripple = (max(steady_window) - min(steady_window)) if steady_window else None

    return {"rise_s": rise, "overshoot_pct": overshoot,
            "steady_mm_s": steady, "error_pct": error,
            "ripple_mm_s": ripple, "elapsed_s": t[-1] - t0}


def run_step(tune: Tune, args) -> int:
    wheels = _parse_wheels(args.wheels)
    combos = _gain_combos(args)
    rows = []

    print(f"\n== 阶跃响应 (目标 {args.speed} mm/s, 记录 {args.hold:.1f} s) ==")
    print("   轮速环闭环；速度指令 50 Hz 持续下发\n")

    for kp, ki, kd in combos:
        if kp is not None or ki is not None or kd is not None:
            for wheel in range(4):
                for gain, value in ((0, kp), (1, ki), (2, kd)):
                    if value is not None:
                        tune.set_param(P.PID_PARAM_IDS[wheel][gain],
                                       int(round(value * 1000)))
            print(f"  增益 Kp={kp} Ki={ki} Kd={kd} 已下发")

        tune.enable(True)
        series = [[] for _ in range(4)]
        times = [[] for _ in range(4)]
        t0 = time.monotonic()

        def emit() -> None:
            tune.client.send(P.Cmd.SET_VELOCITY,
                             P.set_velocity_payload(args.speed, 0))

        def sink(t: float, frame) -> None:
            speeds, _ = _decode(frame)
            if speeds is not None:
                for i in range(4):
                    times[i].append(t)
                    series[i].append(speeds[i])

        tune.pump(t0 + args.hold, VELOCITY_PERIOD_S, emit, sink)
        tune.enable(False)
        time.sleep(0.2)

        for wheel in wheels:
            metrics = _step_metrics(times[wheel], series[wheel], args.speed)
            rows.append({"wheel": WHEELS[wheel], "kp": kp, "ki": ki, "kd": kd,
                         **metrics})

    _print_step_table(rows)
    _append_csv(args.out, "step", rows)
    return 0


def _print_step_table(rows) -> None:
    def fmt(value, spec=".2f", unit=""):
        return "-" if value is None else format(value, spec) + unit

    print("\n| 轮 | Kp | Ki | Kd | 稳态 (mm/s) | 误差 (%) | 上升时间 (s) "
          "| 超调 (%) | 纹波 (mm/s) |")
    print("|----|----|----|----|-------------|----------|--------------"
          "|----------|--------------|")
    for r in rows:
        print(f"| {r['wheel']} | {fmt(r['kp'], '.2f')} | {fmt(r['ki'], '.2f')} "
              f"| {fmt(r['kd'], '.2f')} | {fmt(r['steady_mm_s'], '.0f')} "
              f"| {fmt(r['error_pct'], '.1f')} | {fmt(r['rise_s'], '.3f')} "
              f"| {fmt(r['overshoot_pct'], '.1f')} | {fmt(r['ripple_mm_s'], '.0f')} |")


def _gain_combos(args) -> list[tuple[float | None, float | None, float | None]]:
    kps = args.kp if args.kp else [None]
    kis = args.ki if args.ki else [None]
    kds = args.kd if args.kd else [None]
    return [(kp, ki, kd) for kp in kps for ki in kis for kd in kds]


# ------------------------------------------------------------------ results

def _append_csv(path: str | None, kind: str, rows: list[dict]) -> None:
    if not path or not rows:
        return
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    # One file per run kind keeps the header stable when both commands write
    # into the same results directory.
    out = target.with_name(f"{target.stem}-{kind}{target.suffix}")
    write_header = not out.exists() or out.stat().st_size == 0
    with out.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        if write_header:
            writer.writeheader()
        writer.writerows(rows)
    print(f"\n已写入 {out}")


def _parse_wheels(text: str) -> list[int]:
    selected = [WHEELS.index(w.strip().upper()) for w in text.split(",")
                if w.strip()]
    if not selected:
        raise SystemExit("--wheels 需要 A、B、C、D 的一部分")
    return selected


# ---------------------------------------------------------------------- main

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="四轮死区与速度环调参测量（真机）")
    parser.add_argument("--list", action="store_true", help="列出串口后退出")
    parser.add_argument("--port", help="串口名，例如 COM5 或 /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--wheels", default="A,B,C,D")
    parser.add_argument("--out", default="tools/bench/results/wheel_tune.csv",
                        help="CSV 输出路径（按测量类型加后缀）")
    parser.add_argument("--yes", action="store_true",
                        help="确认轮子会转动（必须显式给出）")
    parser.add_argument("--echo", action="store_true", help="打印收发帧")

    sub = parser.add_subparsers(dest="command")
    dz = sub.add_parser("deadzone", help="开环死区扫描")
    dz.add_argument("--step", type=int, default=40, help="粗扫步长，‰")
    dz.add_argument("--max-duty", type=int, default=400, help="扫描上限，‰")
    dz.add_argument("--hold", type=float, default=0.6, help="每档保持时间，s")
    dz.add_argument("--threshold", type=float, default=10.0,
                    help="判定起转的速度阈值，mm/s")

    st = sub.add_parser("step", help="闭环阶跃响应")
    st.add_argument("--speed", type=int, default=200, help="目标轮速，mm/s")
    st.add_argument("--hold", type=float, default=2.0, help="记录时长，s")
    st.add_argument("--kp", type=float, nargs="+")
    st.add_argument("--ki", type=float, nargs="+")
    st.add_argument("--kd", type=float, nargs="+")

    args = parser.parse_args(argv)

    if args.list:
        from proto_loop import list_ports
        list_ports()
        return 0

    if not args.port:
        parser.error("需要 --port COM5")
    if not args.command:
        parser.error("需要 deadzone 或 step")

    if not args.yes:
        print("这个工具会让轮子转动。确认底盘架空或四周留出 1 m 后加 --yes 再运行。")
        return 2

    tune = Tune(args.port, args.baud, echo=args.echo)
    try:
        try:
            if args.command == "deadzone":
                return run_deadzone(tune, args)
            return run_step(tune, args)
        except RuntimeError as exc:
            print(f"\n中止: {exc}")
            return 1
    except KeyboardInterrupt:
        print("\n中断，正在关电机…")
        return 130
    finally:
        tune.close()


if __name__ == "__main__":
    sys.exit(main())
