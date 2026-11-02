#!/usr/bin/env python3
"""Odometry calibration on the floor: straight-line scale and in-place yaw.

    python tools/bench/odom_calib.py --list
    python tools/bench/odom_calib.py --port COM7 straight --yes
    python tools/bench/odom_calib.py --port COM7 straight --distance 2000 \\
        --actual 1960 --apply --yes
    python tools/bench/odom_calib.py --port COM7 spin --angle 360 \\
        --actual-angle 383 --yes
    python tools/bench/odom_calib.py --port COM7 all --apply --yes

Why two runs
------------
The firmware turns encoder counts into millimetres with counts_per_mm, and
turns the four wheel distances into yaw with k = wheelbase/2 + track/2. A
straight run measures the first; a spin measures the second. A spin run on
its own cannot separate the two, so run ``straight`` first (or use ``all``,
which feeds the straight result into the spin calculation).

Procedure
---------
straight  Mark the start, keep 2.5 m clear, run until the *reported* distance
          reaches --distance, then measure the real distance with a tape and
          enter it. The tool prints the correction factor and, with --apply,
          writes the same factor to all four counts_per_mm ids (0x50..0x53)
          and saves the calibration area.

spin      Mark the robot's heading, keep 0.6 m clear, run until the reported
          angle reaches --angle, then measure how far it actually turned and
          enter it. The tool prints the implied k and the track width that
          would correct it. --apply-track writes 0x01; do that only after the
          straight scale is correct, otherwise the error is split between the
          two runs.

Link: USB-TTL or the HC-05 Bluetooth SPP port. The latter lets the laptop sit
at the start line instead of being carried behind the robot; pair the module in
the OS, find the outgoing "Standard Serial over Bluetooth link" COM port, and
pass it to --port. USART3 is shared, so unplug the USB-TTL side first.

Safety: the wheels move and the robot travels. Every run needs --yes, and the
tool disables the chassis on exit (including Ctrl-C). The straight run needs a
clear 2.5 m; the spin run needs about 0.6 m around the robot.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(TOOLS_DIR))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from host_gui import protocol as P  # noqa: E402
from proto_loop import list_ports  # noqa: E402
from wheel_tune import Tune  # noqa: E402

VELOCITY_PERIOD_S = 0.02            # firmware watchdog is 200 ms
POLL_S = 0.002
STOP_SETTLE_S = 0.35                # let the robot come to rest and report
DEFAULT_COUNTS = 24828              # APP_CHASSIS_COUNTS_PER_MM_X1000
COUNTS_MIN, COUNTS_MAX = 1000, 100000
TRACK_MIN, TRACK_MAX = 50, 2000
CSV_FIELDS = ("mode", "reported", "actual", "factor", "old_counts",
              "new_counts", "k_old", "k_new", "track_new", "note")


# ------------------------------------------------------------------ odometry

def decode_odom(frame):
    if frame.cmd == int(P.Report.ODOM):
        return P.decode_odom(frame.payload)
    return None


def wait_odom(client, timeout: float = 1.0):
    """Return one decoded ODOM frame, or None."""
    end = time.monotonic() + timeout
    while True:
        remain = end - time.monotonic()
        if remain <= 0:
            return None
        frame = client._next(min(remain, 0.05))
        if frame is not None:
            data = decode_odom(frame)
            if data is not None:
                return data


class Heading:
    """Accumulate wrapped centidegree reports into an unwrapped total."""

    def __init__(self, start_cdeg: int) -> None:
        self.prev = start_cdeg
        self.total_cdeg = 0.0

    def update(self, cdeg: int) -> float:
        # ODOM heading wraps to (-18000, 18000]; one sample can never jump
        # more than half a turn at the report rate used here.
        delta = ((cdeg - self.prev + 18000) % 36000) - 18000
        self.prev = cdeg
        self.total_cdeg += delta
        return self.total_cdeg


def drive(client, vx_mm_s: int, wz_mrad_s: int, done, timeout_s: float):
    """Stream a velocity until done() or timeout, then command zero.

    Returns (stopped_by_predicate, final_odom, samples). The final ODOM is
    taken after the stop command and a short settle, so it includes coast.
    """
    samples = []
    next_tx = time.monotonic()
    deadline = next_tx + timeout_s
    stopped = False

    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_tx:
            client.send(P.Cmd.SET_VELOCITY,
                        P.set_velocity_payload(vx_mm_s, wz_mrad_s))
            next_tx = max(next_tx + VELOCITY_PERIOD_S, now)

        frame = client._next(POLL_S)
        if frame is None:
            continue
        data = decode_odom(frame)
        if data is None:
            continue

        samples.append(data)
        if done(data):
            stopped = True
            break

    # Always zero, even on timeout or an exception path.
    for _ in range(3):
        client.send(P.Cmd.SET_VELOCITY, P.set_velocity_payload(0, 0))
        time.sleep(0.02)

    final = drain_odom(client, STOP_SETTLE_S, samples)
    return stopped, final, samples


def drain_odom(client, duration: float, samples: list):
    """Collect ODOM frames for `duration`; return the last one seen."""
    end = time.monotonic() + duration
    last = samples[-1] if samples else None
    while True:
        remain = end - time.monotonic()
        if remain <= 0:
            return last
        frame = client._next(min(remain, 0.05))
        if frame is not None:
            data = decode_odom(frame)
            if data is not None:
                last = data
                samples.append(data)


# --------------------------------------------------------------- calibration

def _prompt_float(label: str, fallback: float | None = None) -> float:
    text = input(f"{label}" + (f" [默认 {fallback:g}]" if fallback else "") +
                 ": ").strip()
    if not text and fallback is not None:
        return fallback
    return float(text)


def reported_distance_mm(start: dict, end: dict) -> float:
    return math.hypot(end["x_mm"] - start["x_mm"],
                      end["y_mm"] - start["y_mm"])


def straight_factor(reported_mm: float, actual_mm: float) -> float:
    """multiplier to apply to counts_per_mm.

    reported = counts / stored_cpm and actual = counts / true_cpm, so
    true_cpm = stored_cpm * reported / actual.
    """
    if actual_mm <= 0:
        raise ValueError("实测距离必须大于 0")
    return reported_mm / actual_mm


def implied_k(k_old: float, reported_deg: float, actual_deg: float,
              scale_factor: float = 1.0) -> float:
    """k that makes the reported yaw match the real one.

    If the spin ran with the *old* counts scale, the reported yaw is already
    inflated by `scale_factor`; divide it out. Pass 1.0 when the straight
    correction was written to the board before this spin.
    """
    if actual_deg <= 0:
        raise ValueError("实测转角必须大于 0")
    return k_old * (reported_deg / actual_deg) / scale_factor


def _resolve_counts(args) -> list[int]:
    if not args.counts:
        return [DEFAULT_COUNTS] * 4
    if len(args.counts) == 1:
        return [args.counts[0]] * 4
    if len(args.counts) == 4:
        return list(args.counts)
    raise SystemExit("--counts 需要 1 个或 4 个值（×1000）")


def _check_counts(values: list[int]) -> None:
    for value in values:
        if not COUNTS_MIN <= value <= COUNTS_MAX:
            raise SystemExit(
                f"counts_per_mm×1000={value} 超出 {COUNTS_MIN}..{COUNTS_MAX}")


def query_storage(client, timeout: float = 0.8):
    since = time.monotonic()
    seq = client.send(P.Cmd.STORAGE_QUERY)
    client.expect_ack(P.Cmd.STORAGE_QUERY, seq, timeout)
    frame = client.wait(lambda f: f.cmd == int(P.Report.STORAGE),
                        timeout, since=since)
    return P.decode_storage(frame.payload) if frame is not None else None


def save_calibration(tune: Tune, timeout: float = 3.0) -> dict:
    before = query_storage(tune.client)
    seq = tune.client.send(P.Cmd.PARAM_SAVE,
                           P.param_save_payload(P.SCOPE_CALIB))
    ack = tune.client.expect_ack(P.Cmd.PARAM_SAVE, seq, 0.5)
    if ack is None or ack.result != P.Result.OK:
        name = "无 ACK" if ack is None else \
            P.RESULT_NAMES.get(P.Result(ack.result), hex(ack.result))
        raise RuntimeError(f"PARAM_SAVE scope=1 失败: {name}")

    end = time.monotonic() + timeout
    info = before
    while time.monotonic() < end:
        info = query_storage(tune.client)
        if info is not None and info["clean"] and info["calib_valid"]:
            if before is None or info["calib_seq"] != before["calib_seq"]:
                return info
        time.sleep(0.1)
    return info or {}


def apply_counts(tune: Tune, factor: float, old: list[int]) -> list[int]:
    new = [int(round(value * factor)) for value in old]
    _check_counts(new)
    for param_id, value in zip(P.COUNTS_PER_MM_IDS, new):
        tune.set_param(param_id, value)
    return new


def apply_track(tune: Tune, track_mm: int) -> None:
    if not TRACK_MIN <= track_mm <= TRACK_MAX:
        raise SystemExit(f"轮距 {track_mm} mm 超出 {TRACK_MIN}..{TRACK_MAX}")
    tune.set_param(0x01, track_mm)


# --------------------------------------------------------------------- modes

def run_straight(tune: Tune, args) -> dict:
    client = tune.client
    start = wait_odom(client)
    if start is None:
        raise RuntimeError("没有 ODOM 上报，检查串口/波特率")

    clearance_m = max(1.0, args.distance / 1000.0 + 0.5)
    print(f"\n== 直行 {args.distance} mm，目标速度 {args.speed} mm/s ==")
    print(f"   起点已记录，保持 {clearance_m:g} m 直行空间，轮子即将转动\n")

    heading = Heading(start["theta_cdeg"])
    timeout_s = max(10.0, args.distance / max(args.speed, 1) * 3.0 + 5.0)

    def done(data: dict) -> bool:
        heading.update(data["theta_cdeg"])
        return reported_distance_mm(start, data) >= args.distance

    tune.enable(True)
    stopped, final, samples = drive(client, args.speed, 0, done, timeout_s)
    tune.enable(False)

    if final is None:
        raise RuntimeError("直行过程中没有收到 ODOM")
    reported = reported_distance_mm(start, final)

    print(f"   上报位移 {reported:.1f} mm，"
          f"航向漂移 {heading.total_cdeg / 100.0:+.1f}°，"
          f"样本 {len(samples)}")
    if not stopped:
        print("   !! 超时停止，上报位移未达到目标；结果仍可用，但请检查是否打滑")

    actual = args.actual
    if actual is None:
        actual = _prompt_float("用卷尺量起点到终点的直线距离 (mm)", reported)

    factor = straight_factor(reported, actual)
    old_counts = _resolve_counts(args)
    new_counts = [int(round(value * factor)) for value in old_counts]
    _check_counts(new_counts)

    print(f"\n   比例系数 f = {factor:.5f}")
    print(f"   旧 counts_per_mm×1000 = {old_counts}")
    print(f"   新 counts_per_mm×1000 = {new_counts}")
    print(f"   按本次实测距离外推的修正 {reported - actual:+.1f} mm")

    applied = False
    if args.apply:
        apply_counts(tune, factor, old_counts)
        info = save_calibration(tune)
        applied = True
        print(f"   已写入 0x50~0x53 并保存，calib_seq={info.get('calib_seq')} "
              f"flags=0x{info.get('flags', 0):02X}")
        print("   下次再跑标定要带 --counts 传入上面的新值，"
              "否则会按默认 24828 重复修正")
    else:
        print("   未写回；确认数值后加 --apply，或用 --counts 指定当前值")

    return {"mode": "straight", "reported": f"{reported:.1f}",
            "actual": f"{actual:.1f}", "factor": f"{factor:.5f}",
            "old_counts": "/".join(map(str, old_counts)),
            "new_counts": "/".join(map(str, new_counts)),
            "k_old": "", "k_new": "", "track_new": "",
            "note": "applied" if applied else "report-only",
            "applied": applied, "factor_value": factor}


def run_spin(tune: Tune, args, scale_factor: float = 1.0,
             applied_scale: bool = False) -> dict:
    client = tune.client
    start = wait_odom(client)
    if start is None:
        raise RuntimeError("没有 ODOM 上报，检查串口/波特率")

    target_cdeg = abs(args.angle) * 100.0
    direction = 1 if args.wz >= 0 else -1
    print(f"\n== 原地旋转 {args.angle:g}°，角速度 {args.wz} mrad/s ==")
    print("   标记起始朝向，保持 0.6 m 周围净空，轮子即将转动\n")

    heading = Heading(start["theta_cdeg"])
    timeout_s = max(10.0, math.radians(abs(args.angle)) /
                    (abs(args.wz) / 1000.0) * 3.0 + 5.0)

    def done(data: dict) -> bool:
        total = heading.update(data["theta_cdeg"])
        return abs(total) >= target_cdeg

    tune.enable(True)
    stopped, final, samples = drive(client, 0, args.wz, done, timeout_s)
    tune.enable(False)

    if final is None:
        raise RuntimeError("旋转过程中没有收到 ODOM")

    heading.update(final["theta_cdeg"])
    reported_deg = heading.total_cdeg / 100.0

    print(f"   上报转角 {reported_deg:+.1f}°，样本 {len(samples)}")
    if not stopped:
        print("   !! 超时停止，上报转角未达到目标；请检查地面摩擦/限速")

    actual_deg = args.actual_angle
    if actual_deg is None:
        actual_deg = _prompt_float("用角度尺/标记量实际转角（度）",
                                   abs(args.angle))
    actual_deg = abs(actual_deg)

    k_old = args.wheelbase / 2.0 + args.track / 2.0
    # If the straight correction was not written before this spin, the yaw
    # report is inflated by the old wheel scale and must be divided out.
    k_new = implied_k(k_old, abs(reported_deg), actual_deg,
                      1.0 if applied_scale else scale_factor)
    track_new = int(round(2.0 * (k_new - args.wheelbase / 2.0)))

    print(f"\n   k_old = {k_old:.1f} mm "
          f"(wheelbase {args.wheelbase} / 2 + track {args.track} / 2)")
    print(f"   k_new = {k_new:.1f} mm")
    print(f"   建议轮距 = {track_new} mm (0x01)")
    if not TRACK_MIN <= track_new <= TRACK_MAX:
        print("   !! 建议值越界：先确认已烧入 170 mm 轴距的固件，"
              "并检查 --wheelbase/--track 是否与实际一致")

    applied = False
    if args.apply_track:
        apply_track(tune, track_new)
        info = save_calibration(tune)
        applied = True
        print(f"   已写入 0x01 并保存，calib_seq={info.get('calib_seq')} "
              f"flags=0x{info.get('flags', 0):02X}")
    else:
        print("   未写回；确认数值后加 --apply-track")

    return {"mode": "spin", "reported": f"{reported_deg:.1f}",
            "actual": f"{actual_deg:.1f}",
            "factor": f"{reported_deg / actual_deg:.5f}" if actual_deg else "",
            "old_counts": "", "new_counts": "",
            "k_old": f"{k_old:.1f}", "k_new": f"{k_new:.1f}",
            "track_new": track_new,
            "note": "applied" if applied else "report-only"}


def run_all(tune: Tune, args) -> int:
    straight = run_straight(tune, args)
    # run_straight already wrote the counts when --apply was given. Pass that
    # fact to the spin so it does not divide by the scale factor twice.
    spin = run_spin(tune, args,
                    scale_factor=straight["factor_value"],
                    applied_scale=straight["applied"])
    rows = [straight, spin]
    _print_summary(rows)
    _append_csv(args.out, rows)
    return 0


def _print_summary(rows: list[dict]) -> None:
    print("\n| 模式 | 上报 | 实测 | 比例 | k_old | k_new | 建议轮距 | 处理 |")
    print("|------|------|------|------|-------|-------|----------|------|")
    for row in rows:
        print(f"| {row['mode']} | {row['reported']} | {row['actual']} "
              f"| {row['factor']} | {row['k_old'] or '-'} "
              f"| {row['k_new'] or '-'} | {row['track_new'] or '-'} "
              f"| {row['note']} |")


def _append_csv(path: str | None, rows: list[dict]) -> None:
    if not path or not rows:
        return
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    write_header = not target.exists() or target.stat().st_size == 0
    with target.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS,
                                extrasaction="ignore")
        if write_header:
            writer.writeheader()
        writer.writerows(rows)
    print(f"\n已写入 {target}")


# ---------------------------------------------------------------------- main

def _run_self_test() -> int:
    """Pure arithmetic checks, no serial port."""
    f = straight_factor(2000.0, 1960.0)
    assert abs(f - 1.020408) < 1e-6, f
    assert abs(implied_k(212.0, 360.0, 350.0) - 218.057) < 0.01
    # Old wheel scale not yet applied: 5 % over-report must be divided out.
    assert abs(implied_k(212.0, 360.0, 350.0, 1.02) - 213.78) < 0.01
    h = Heading(17900)
    h.update(-17900)
    assert abs(h.total_cdeg - 200.0) < 1e-9, h.total_cdeg
    print("PASS: odom_calib self-test")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="里程计直行/原地标定（真机，会让车移动）")
    parser.add_argument("--list", action="store_true", help="列出串口后退出")
    parser.add_argument("--port", help="串口名，例如 COM7 或 /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--yes", action="store_true",
                        help="确认车轮会转动且场地已清空（必须显式给出）")
    parser.add_argument("--echo", action="store_true", help="打印收发帧")
    parser.add_argument("--out", default="build/results/odom_calib.csv",
                        help="CSV 输出路径（默认在 build/ 下，不入库）")
    parser.add_argument("--self-test", action="store_true",
                        help="只跑标定算术自检，不打开串口")

    # Measurement options live on the subcommands so they read naturally after
    # the verb ("straight --distance 2000"). --yes is duplicated there too.
    measure = argparse.ArgumentParser(add_help=False)
    measure.add_argument("--speed", type=int, default=150,
                         help="直行速度 mm/s")
    measure.add_argument("--distance", type=int, default=2000,
                         help="直行目标（按上报值）mm")
    measure.add_argument("--actual", type=float,
                         help="直行实测距离 mm；不给则交互输入")
    measure.add_argument("--counts", type=int, nargs="+",
                         help="当前四轮 counts_per_mm×1000；默认 24828")
    measure.add_argument("--apply", action="store_true",
                         help="把直行比例写入 0x50~0x53 并 PARAM_SAVE scope=1")
    measure.add_argument("--wz", type=int, default=500,
                         help="旋转角速度 mrad/s，正数逆时针")
    measure.add_argument("--angle", type=float, default=360.0,
                         help="旋转目标（按上报值）度")
    measure.add_argument("--actual-angle", type=float,
                         help="实测转角 度；不给则交互输入")
    measure.add_argument("--wheelbase", type=int, default=170,
                         help="固件当前轴距 mm（默认 170）")
    measure.add_argument("--track", type=int, default=165,
                         help="固件当前轮距 mm（默认 165，0x01 的值）")
    measure.add_argument("--apply-track", action="store_true",
                         help="把旋转建议写入 0x01 并 PARAM_SAVE scope=1")
    measure.add_argument("--yes", action="store_true",
                         default=argparse.SUPPRESS, help=argparse.SUPPRESS)

    sub = parser.add_subparsers(dest="command")
    sub.add_parser("straight", parents=[measure], help="直行比例标定")
    sub.add_parser("spin", parents=[measure], help="原地旋转标定")
    sub.add_parser("all", parents=[measure], help="先直行再旋转")

    args = parser.parse_args(argv)

    if args.self_test:
        return _run_self_test()

    if args.list:
        list_ports()
        return 0

    if not args.port:
        parser.error("需要 --port COM7")
    if not args.command:
        parser.error("需要 straight / spin / all")

    if not args.yes:
        print("这个工具会让车直行/旋转。清空场地并确认后加 --yes 再运行。")
        return 2

    tune = Tune(args.port, args.baud, echo=args.echo)
    try:
        try:
            if args.command == "straight":
                row = run_straight(tune, args)
                _print_summary([row])
                _append_csv(args.out, [row])
                return 0
            if args.command == "spin":
                row = run_spin(tune, args)
                _print_summary([row])
                _append_csv(args.out, [row])
                return 0
            return run_all(tune, args)
        except (RuntimeError, ValueError) as exc:
            print(f"\n中止: {exc}")
            return 1
    except KeyboardInterrupt:
        print("\n中断，正在关电机…")
        return 130
    finally:
        tune.close()


if __name__ == "__main__":
    sys.exit(main())
