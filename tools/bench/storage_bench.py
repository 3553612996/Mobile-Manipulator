#!/usr/bin/env python3
"""Bench suite for the calibration, tuning and storage half of protocol v1
(docs/architecture/protocol.md 5.4 / 5.5).

    python tools/bench/storage_bench.py --list
    python tools/bench/storage_bench.py --port COM7 --baud 115200
    python tools/bench/storage_bench.py --port COM7 --write
    python tools/bench/storage_bench.py --port COM7 --write --clear --guard
    python tools/bench/storage_bench.py --sim --write

What each level does:

    (default)  read-only tests plus SET_PARAM. SET_PARAM only touches RAM and
               every value written is the compile-time default from
               app_robot_config.h, so a run on an already-calibrated board is
               idempotent. The chassis must be disabled; the suite disables it.
    --write    adds PARAM_SAVE / PARAM_RESET. These erase and rewrite the flash
               parameter records. Values are still the compile-time defaults.
    --clear    adds LOG_CLEAR. This destroys the log rings, and with source 3
               the crash dump (the only copy). No other level touches them.
    --guard    enables the chassis for the "calibration must be refused while
               enabled" checks. Zero velocity only, but wheels must be free.

Why the suite talks to the low-level commands instead of the GUI: the GUI
proves the user path; this proves the ACK/REPORT contract, including the
rejections the GUI never sends on purpose (bad scope, bad confirm, BUSY).

The frame codec and transport are imported from tools/host_gui and
tools/bench/proto_loop.py so this suite cannot drift from the host encoder.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

BENCH_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(BENCH_DIR))

from proto_loop import (  # noqa: E402
    FAIL, INFO, PASS, SKIP, Client, SerialLink, SimLink, _result_name,
)

from host_gui import protocol as P  # noqa: E402

# Compile-time defaults (app_robot_config.h). Writing these back is the least
# invasive way to exercise SET_PARAM: on a board that is already at defaults
# the RAM contents do not change at all.
CFG_TRACK_MM = 2 * 120              # APP_CHASSIS_HALF_WIDTH_MM * 2
CFG_WHEEL_DIAMETER_MM = 60
CFG_COUNTS_PER_MM_X1000 = 24828
CFG_START_DUTY_FWD = (75, 67, 82, 80)   # FL/FR/RL/RR, docs/bench/pid-tuning.md 3.3
CFG_START_DUTY_REV = (67, 75, 80, 100)
CFG_TIMEOUT_MS = 200
CFG_MAX_LINEAR_MM_S = 500
CFG_MAX_ANGULAR_MRAD_S = 1500
CFG_PID = (5000, 6000, 0)           # Kp/Ki/Kd, x1000, docs/bench/pid-tuning.md 4.4

SAVE_SETTLE_S = 3.0                 # flash erase is up to 400 ms + retries
EXPORT_TIMEOUT_S = 3.0
CRASH_TIMEOUT_S = 2.0


# ------------------------------------------------------------------ helpers

def ack_for(client: Client, t: float, cmd, seq: int, timeout: float = 0.4):
    return client.wait(
        lambda f: f.is_ack and f.acked_cmd == int(cmd) and f.acked_seq == seq,
        timeout, since=t)


def expect(client: Client, name: str, cmd, payload: bytes, want,
           timeout: float = 0.4, detail: str = ""):
    """Send one command and check its ACK result. Returns the ACK frame."""
    t = time.monotonic()
    seq = client.send(cmd, payload)
    ack = ack_for(client, t, cmd, seq, timeout)
    ok = ack is not None and ack.result == int(want)
    text = _result_name(ack)
    if detail:
        text = f"{detail} · {text}"
    client.check(name, PASS if ok else FAIL, text)
    return ack


def query_storage(client: Client, timeout: float = 0.8):
    """STORAGE_QUERY, then the REPORT_STORAGE that must follow it."""
    t = time.monotonic()
    seq = client.send(P.Cmd.STORAGE_QUERY)
    ack = ack_for(client, t, P.Cmd.STORAGE_QUERY, seq)
    frame = client.wait(lambda f: f.cmd == int(P.Report.STORAGE),
                        timeout, since=t)
    info = P.decode_storage(frame.payload) if frame is not None else None
    return ack, info


def poll_storage(client: Client, pred, timeout: float = SAVE_SETTLE_S):
    """Poll STORAGE_QUERY until pred(info) holds, or return the last report.

    A save is asynchronous: the ACK means "queued", and the flash erase can
    take 400 ms before REPORT_STORAGE shows the new seq or the clean bit.
    """
    end = time.monotonic() + timeout
    info = None
    while True:
        _ack, info = query_storage(client)
        if info is not None and pred(info):
            return info
        if time.monotonic() >= end:
            return info
        time.sleep(0.1)


def wait_clean(client: Client, timeout: float = SAVE_SETTLE_S):
    return poll_storage(client, lambda info: info["clean"], timeout)


def collect_logs(client: Client, timeout: float = EXPORT_TIMEOUT_S):
    """Drain REPORT_LOG frames until the end frame closes the export.

    The end frame is not a record: it carries the closing seq and an empty
    text, so counting it would make an empty ring look like one record.
    """
    records = []
    end = time.monotonic() + timeout
    while True:
        remain = end - time.monotonic()
        if remain <= 0:
            return records, False
        frame = client._next(min(remain, 0.2))
        if frame is None or frame.cmd != int(P.Report.LOG):
            continue
        record = P.decode_log(frame.payload)
        if record is None:
            continue
        if record["end"]:
            return records, True
        records.append(record)


def collect_crash(client: Client, timeout: float = CRASH_TIMEOUT_S):
    """Reassemble REPORT_CRASH chunks with the same assembler the GUI uses."""
    asm = P.CrashAssembler()
    end = time.monotonic() + timeout
    while True:
        remain = end - time.monotonic()
        if remain <= 0:
            return asm, "timeout"
        frame = client._next(min(remain, 0.2))
        if frame is None or frame.cmd != int(P.Report.CRASH):
            continue
        chunk = P.decode_crash(frame.payload)
        if chunk is None:
            return asm, "bad"
        state = asm.feed(chunk)
        if state in ("bad", "gap"):
            return asm, state
        if state == "done":
            return asm, "done"


def reload_link(client: Client) -> None:
    """Drop stale reports so a later wait() cannot match an earlier frame."""
    while True:
        try:
            client.link.frames.get_nowait()
        except Exception:
            return


# --------------------------------------------------------------- test steps

def step_link(client: Client) -> bool:
    counts = client.collect(1.5)
    total = sum(counts.values())
    client.check("链路在线", PASS if total >= 10 else FAIL, f"1.5 s 收到 {total} 帧")
    if total < 10:
        client.check("中止", FAIL,
                     "检查 COM 口/波特率/TX-RX 交叉，以及固件是否已烧录")
        return False
    return True


def step_disable(client: Client) -> None:
    t = time.monotonic()
    seq = client.send(P.Cmd.SET_ENABLE, P.set_enable_payload(False))
    ack = ack_for(client, t, P.Cmd.SET_ENABLE, seq)
    client.check("底盘失能（标定前置条件）",
                 PASS if ack is not None and ack.result == P.Result.OK else FAIL,
                 _result_name(ack))

    # ESTOP/FAULT blocks scope=1 saves even though the chassis is disabled.
    t = time.monotonic()
    status = client.wait_status(lambda s: s["state"] in (4, 5), 0.3, since=t)
    if status is not None:
        t = time.monotonic()
        seq = client.send(P.Cmd.CLEAR_FAULT, P.clear_fault_payload())
        ack = ack_for(client, t, P.Cmd.CLEAR_FAULT, seq)
        client.check("清除锁存故障", PASS if ack is not None else FAIL,
                     _result_name(ack))


def step_storage_read(client: Client):
    ack, info = query_storage(client)
    client.check("STORAGE_QUERY 有 ACK",
                 PASS if ack is not None and ack.result == P.Result.OK else FAIL,
                 _result_name(ack))
    client.check("REPORT_STORAGE 24 字节", PASS if info is not None else FAIL)
    if info is None:
        return None

    client.check("参数镜像有效",
                 PASS if info["param_valid"] else FAIL,
                 f"param_seq={info['param_seq']}")
    client.check("标定镜像有效",
                 PASS if info["calib_valid"] else FAIL,
                 f"calib_seq={info['calib_seq']}")
    client.check("至少一个镜像有效",
                 PASS if info["image_a_valid"] or info["image_b_valid"] else FAIL,
                 f"A={info['image_a_valid']} B={info['image_b_valid']}")
    client.check("启动计数已累加", PASS if info["boot_total"] > 0 else FAIL,
                 f"boot_total={info['boot_total']}")
    client.check("日志占用合理",
                 PASS if info["log_used_bytes"] < 64 * 1024 * 1024 else FAIL,
                 f"{info['log_used_bytes']} B, 擦除 {info['log_erase_count']} 次")

    _ack2, info2 = query_storage(client)
    stable = info2 is not None and (
        info2["param_seq"], info2["calib_seq"]) == (
        info["param_seq"], info["calib_seq"])
    client.check("连续查询 seq 稳定", PASS if stable else FAIL,
                 f"{info['param_seq']}/{info['calib_seq']} → "
                 f"{info2['param_seq'] if info2 else '?'}/"
                 f"{info2['calib_seq'] if info2 else '?'}")
    return info


def step_tuning(client: Client, before: dict | None) -> None:
    print("\n-- 调参：PID 增益（RAM only）--")
    kp, ki, kd = CFG_PID
    expect(client, "写 A 轮 Kp=1.000", P.Cmd.SET_PARAM,
           P.set_param_payload(0x20, kp), P.Result.OK)
    expect(client, "写 A 轮 Ki=0.600", P.Cmd.SET_PARAM,
           P.set_param_payload(0x21, ki), P.Result.OK)
    expect(client, "写 A 轮 Kd=0.000", P.Cmd.SET_PARAM,
           P.set_param_payload(0x22, kd), P.Result.OK)

    expect(client, "Kp 越界 10001 → RANGE", P.Cmd.SET_PARAM,
           P.set_param_payload(0x20, 10001), P.Result.OUT_OF_RANGE)
    expect(client, "Kd 越界 5001 → RANGE", P.Cmd.SET_PARAM,
           P.set_param_payload(0x22, 5001), P.Result.OUT_OF_RANGE)
    expect(client, "负增益 → RANGE", P.Cmd.SET_PARAM,
           P.set_param_payload(0x21, -1), P.Result.OUT_OF_RANGE)
    expect(client, "未知参数 id 0x7F → RANGE", P.Cmd.SET_PARAM,
           P.set_param_payload(0x7F, 0), P.Result.OUT_OF_RANGE)

    # Log level lives in the parameter area too. Writing the default back is
    # harmless; 4 must be rejected rather than clamped.
    expect(client, "日志等级 = INFO", P.Cmd.SET_PARAM,
           P.set_param_payload(0x40, 1), P.Result.OK)
    expect(client, "日志等级 4 → RANGE", P.Cmd.SET_PARAM,
           P.set_param_payload(0x40, 4), P.Result.OUT_OF_RANGE)

    # Dirty is only evidence when the board started clean; say so otherwise.
    _ack, info = query_storage(client)
    if info is None:
        client.check("调参后置脏", FAIL, "没有 REPORT_STORAGE")
    elif before is not None and before["clean"]:
        client.check("调参后置脏（bit0=0）", PASS if not info["clean"] else FAIL,
                     f"flags=0x{info['flags']:02X}")
    else:
        client.check("调参后置脏", INFO,
                     "运行前已有脏数据，无法用 bit0 判定")


def step_calibration(client: Client) -> None:
    print("\n-- 标定：几何 / 编码器 / 起转占空比 / 方向 --")
    expect(client, "写轮距 240 mm", P.Cmd.SET_PARAM,
           P.set_param_payload(0x01, CFG_TRACK_MM), P.Result.OK)
    expect(client, "写轮径 60 mm", P.Cmd.SET_PARAM,
           P.set_param_payload(0x02, CFG_WHEEL_DIAMETER_MM), P.Result.OK)

    ok = True
    for wheel, param_id in enumerate(P.COUNTS_PER_MM_IDS):
        t = time.monotonic()
        seq = client.send(P.Cmd.SET_PARAM,
                          P.set_param_payload(param_id,
                                              CFG_COUNTS_PER_MM_X1000))
        ack = ack_for(client, t, P.Cmd.SET_PARAM, seq)
        ok = ok and ack is not None and ack.result == P.Result.OK
    client.check("四轮 counts_per_mm=24.828", PASS if ok else FAIL,
                 "0x50~0x53")

    for label, ids, values in (
            ("正向起转占空比 75/67/82/80‰", P.DUTY_FWD_IDS,
             CFG_START_DUTY_FWD),
            ("反向起转占空比 67/75/80/100‰", P.DUTY_REV_IDS,
             CFG_START_DUTY_REV),
            ("四轮方向 +1", P.DIRECTION_IDS, (1, 1, 1, 1))):
        ok = True
        for param_id, value in zip(ids, values):
            t = time.monotonic()
            seq = client.send(P.Cmd.SET_PARAM,
                              P.set_param_payload(param_id, value))
            ack = ack_for(client, t, P.Cmd.SET_PARAM, seq)
            ok = ok and ack is not None and ack.result == P.Result.OK
        client.check(label, PASS if ok else FAIL,
                     "/".join(f"0x{i:02X}" for i in ids))

    expect(client, "轮距 5000 → RANGE", P.Cmd.SET_PARAM,
           P.set_param_payload(0x01, 5000), P.Result.OUT_OF_RANGE)
    expect(client, "轮径 0 → RANGE", P.Cmd.SET_PARAM,
           P.set_param_payload(0x02, 0), P.Result.OUT_OF_RANGE)
    expect(client, "counts_per_mm 0 → RANGE", P.Cmd.SET_PARAM,
           P.set_param_payload(0x50, 0), P.Result.OUT_OF_RANGE)
    expect(client, "起转占空比 1001 → RANGE", P.Cmd.SET_PARAM,
           P.set_param_payload(0x54, 1001), P.Result.OUT_OF_RANGE)
    expect(client, "方向 2 → RANGE", P.Cmd.SET_PARAM,
           P.set_param_payload(0x5C, 2), P.Result.OUT_OF_RANGE)

    print("\n-- 参数区：超时 / 限速 --")
    expect(client, "控制超时 200 ms", P.Cmd.SET_PARAM,
           P.set_param_payload(0x10, CFG_TIMEOUT_MS), P.Result.OK)
    expect(client, "最大线速度 500 mm/s", P.Cmd.SET_PARAM,
           P.set_param_payload(0x11, CFG_MAX_LINEAR_MM_S), P.Result.OK)
    expect(client, "最大角速度 1500 mrad/s", P.Cmd.SET_PARAM,
           P.set_param_payload(0x12, CFG_MAX_ANGULAR_MRAD_S), P.Result.OK)


def step_log_read(client: Client) -> None:
    print("\n-- 读取：日志导出 --")
    t = time.monotonic()
    seq = client.send(P.Cmd.LOG_EXPORT,
                      P.log_export_payload(0, 1, 0))
    ack = ack_for(client, t, P.Cmd.LOG_EXPORT, seq)
    client.check("开始导出有 ACK",
                 PASS if ack is not None and ack.result == P.Result.OK else FAIL,
                 _result_name(ack))

    # A second start while the first is running must be refused, not queued.
    if ack is not None and ack.result == P.Result.OK:
        expect(client, "导出中再次开始 → BUSY", P.Cmd.LOG_EXPORT,
               P.log_export_payload(0, 1, 0), P.Result.BUSY, timeout=0.6)

    records, closed = collect_logs(client)
    client.check("导出收到结束帧", PASS if closed else FAIL,
                 f"{len(records)} 条")
    client.check("导出非空（种子日志）", PASS if records else SKIP,
                 f"{len(records)} 条")

    if records:
        seqs = [r["seq"] for r in records if not r["end"]]
        ordered = all(b >= a for a, b in zip(seqs, seqs[1:]))
        client.check("日志 seq 单调", PASS if ordered else FAIL,
                     f"{seqs[0]}..{seqs[-1]}" if seqs else "")
        levels = {r["level"] for r in records}
        client.check("日志等级在 0..3", PASS if levels <= {0, 1, 2, 3} else FAIL,
                     f"levels={sorted(levels)}")

    expect(client, "开始导出 source=2 → RANGE", P.Cmd.LOG_EXPORT,
           P.log_export_payload(2, 1, 0), P.Result.OUT_OF_RANGE)
    expect(client, "开始导出 action=2 → RANGE", P.Cmd.LOG_EXPORT,
           P.log_export_payload(0, 2, 0), P.Result.OUT_OF_RANGE)
    expect(client, "停止导出（未在导出）", P.Cmd.LOG_EXPORT,
           P.log_export_payload(0, 0, 0), P.Result.OK, timeout=0.6)

    # Fault-event ring: same format, different source.
    t = time.monotonic()
    seq = client.send(P.Cmd.LOG_EXPORT, P.log_export_payload(1, 1, 0))
    ack = ack_for(client, t, P.Cmd.LOG_EXPORT, seq)
    client.check("故障事件导出有 ACK",
                 PASS if ack is not None and ack.result == P.Result.OK else FAIL,
                 _result_name(ack))
    if ack is not None and ack.result == P.Result.OK:
        records, closed = collect_logs(client)
        client.check("故障事件导出结束", PASS if closed else FAIL,
                     f"{len(records)} 条")


def step_crash_read(client: Client, mark_read: bool) -> None:
    print("\n-- 读取：崩溃现场 --")
    t = time.monotonic()
    seq = client.send(P.Cmd.CRASH_READ,
                      P.crash_read_payload(P.CRASH_SLOT_LATEST, mark_read))
    ack = ack_for(client, t, P.Cmd.CRASH_READ, seq)
    if ack is None:
        client.check("CRASH_READ 有 ACK", FAIL, "无应答")
    elif ack.result == P.Result.FAILED:
        client.check("CRASH_READ 无记录 → FAILED", SKIP,
                     _result_name(ack))
    else:
        client.check("CRASH_READ 有 ACK",
                     PASS if ack.result == P.Result.OK else FAIL,
                     _result_name(ack))
        if ack.result == P.Result.OK:
            asm, state = collect_crash(client)
            client.check("崩溃分片拼接完成", PASS if state == "done" else FAIL,
                         f"{state}, {asm.offset}/{asm.total_len} B")
            if state == "done":
                client.check("崩溃记录长度一致",
                             PASS if len(asm.record()) == asm.total_len else FAIL,
                             f"{len(asm.record())} B")

    expect(client, "崩溃槽位 16 → RANGE", P.Cmd.CRASH_READ,
           P.crash_read_payload(16, False), P.Result.OUT_OF_RANGE)


def step_write(client: Client, before: dict | None) -> None:
    print("\n-- 写入：PARAM_SAVE / PARAM_RESET --")
    base_param = before["param_seq"] if before else None
    base_calib = before["calib_seq"] if before else None

    expect(client, "保存参数区 scope=0", P.Cmd.PARAM_SAVE,
           P.param_save_payload(P.SCOPE_PARAM), P.Result.OK)
    info = poll_storage(client,
                        lambda i: base_param is None or i["param_seq"] != base_param)
    if info is not None and base_param is not None:
        client.check("param_seq 递增",
                     PASS if info["param_seq"] != base_param else FAIL,
                     f"{base_param} → {info['param_seq']}")
    # scope=0 only clears the parameter area. The calibration area was just
    # written by step_calibration, so bit0 may legitimately stay 0 here --
    # that is exactly the "恢复出厂不丢标定" rule, not a failure.
    client.check("参数区保存只清参数区脏标志", INFO,
                 f"flags=0x{info['flags']:02X}, clean={info['clean']}"
                 if info else "没有报告")

    expect(client, "保存标定区 scope=1", P.Cmd.PARAM_SAVE,
           P.param_save_payload(P.SCOPE_CALIB), P.Result.OK)
    info = poll_storage(client,
                        lambda i: base_calib is None or i["calib_seq"] != base_calib)
    if info is not None and base_calib is not None:
        client.check("calib_seq 递增",
                     PASS if info["calib_seq"] != base_calib else FAIL,
                     f"{base_calib} → {info['calib_seq']}")
    client.check("两个区落盘后 bit0=1",
                 PASS if info is not None and info["clean"] else FAIL,
                 f"flags=0x{info['flags']:02X}" if info else "没有报告")

    expect(client, "保存 scope=3 → RANGE", P.Cmd.PARAM_SAVE,
           bytes([3]), P.Result.OUT_OF_RANGE)

    expect(client, "恢复出厂 confirm 错 → RANGE", P.Cmd.PARAM_RESET,
           bytes([0, 0x00]), P.Result.OUT_OF_RANGE)
    expect(client, "恢复出厂 scope=3 → RANGE", P.Cmd.PARAM_RESET,
           bytes([3, 0xA5]), P.Result.OUT_OF_RANGE)
    expect(client, "恢复出厂参数区", P.Cmd.PARAM_RESET,
           P.param_reset_payload(0), P.Result.OK)

    _ack, info = query_storage(client)
    client.check("恢复后置脏（bit0=0）",
                 PASS if info is not None and not info["clean"] else FAIL,
                 f"flags=0x{info['flags']:02X}" if info else "没有报告")

    # Leave the board clean so the operator can power it off.
    expect(client, "恢复后重新保存参数区", P.Cmd.PARAM_SAVE,
           P.param_save_payload(P.SCOPE_PARAM), P.Result.OK)
    info = wait_clean(client)
    client.check("最终无脏数据", PASS if info is not None and info["clean"]
                 else FAIL, f"flags=0x{info['flags']:02X}" if info else "")


def step_clear(client: Client) -> None:
    print("\n-- 写入：LOG_CLEAR（破坏性）--")
    _ack, before = query_storage(client)

    expect(client, "清空 confirm 错 → RANGE", P.Cmd.LOG_CLEAR,
           bytes([0, 0x00]), P.Result.OUT_OF_RANGE)
    expect(client, "清空 source=4 → RANGE", P.Cmd.LOG_CLEAR,
           bytes([4, 0xA5]), P.Result.OUT_OF_RANGE)
    expect(client, "清空全部日志与崩溃现场", P.Cmd.LOG_CLEAR,
           P.log_clear_payload(3), P.Result.OK, timeout=0.6)

    # log_used counts region bookkeeping as well as records, so the portable
    # check is that it dropped, not that it reached zero.
    if before is None:
        client.check("清空后日志占用下降", SKIP, "没有清空前的 REPORT_STORAGE")
    else:
        info = poll_storage(
            client,
            lambda i: i["log_used_bytes"] < before["log_used_bytes"],
            timeout=1.0)
        client.check("清空后日志占用下降",
                     PASS if info is not None and info["log_used_bytes"]
                     < before["log_used_bytes"] else FAIL,
                     f"{before['log_used_bytes']} → "
                     f"{info['log_used_bytes'] if info else '?'} B")

    # An empty ring still has to close the export with an end frame.
    for source, name in ((0, "运行日志"), (1, "故障事件")):
        t = time.monotonic()
        seq = client.send(P.Cmd.LOG_EXPORT, P.log_export_payload(source, 1, 0))
        ack = ack_for(client, t, P.Cmd.LOG_EXPORT, seq)
        if ack is None or ack.result != P.Result.OK:
            client.check(f"空{name}导出", FAIL, _result_name(ack))
            continue
        records, closed = collect_logs(client, timeout=1.0)
        client.check(f"空{name}导出立即结束",
                     PASS if closed and not records else FAIL,
                     f"{len(records)} 条")


def step_guard(client: Client) -> None:
    """Calibration writes and scope=1 saves must be refused while enabled."""
    print("\n-- 使能状态下的标定拒绝（底盘会处于使能，轮子必须悬空）--")
    t = time.monotonic()
    seq = client.send(P.Cmd.SET_ENABLE, P.set_enable_payload(True))
    ack = ack_for(client, t, P.Cmd.SET_ENABLE, seq)
    if ack is None or ack.result != P.Result.OK:
        client.check("使能底盘", FAIL, _result_name(ack))
        return
    client.check("使能底盘", PASS,
                 f"state={_state_name(client, t)}")

    # Send all four back to back: waiting for each ACK separately would let the
    # 200 ms velocity watchdog trip between commands, and a rejection caused by
    # FAULT would not prove the "enabled → refuse" rule.
    cases = (
        ("轮距", P.Cmd.SET_PARAM, P.set_param_payload(0x01, CFG_TRACK_MM),
         P.Result.BAD_STATE),
        ("counts_per_mm", P.Cmd.SET_PARAM,
         P.set_param_payload(0x50, CFG_COUNTS_PER_MM_X1000),
         P.Result.BAD_STATE),
        ("PARAM_SAVE scope=1", P.Cmd.PARAM_SAVE,
         P.param_save_payload(P.SCOPE_CALIB), P.Result.BAD_STATE),
        ("PID 调参（参数区）", P.Cmd.SET_PARAM,
         P.set_param_payload(0x20, CFG_PID[0]), P.Result.OK),
    )
    sent = [(name, cmd, client.send(cmd, payload), want)
            for name, cmd, payload, want in cases]

    for name, cmd, seq, want in sent:
        ack = client.wait(
            lambda f, c=cmd, s=seq: f.is_ack and f.acked_cmd == int(c)
            and f.acked_seq == s, 0.8, since=t)
        ok = ack is not None and ack.result == int(want)
        suffix = "使能时 → BAD_STATE" if want == P.Result.BAD_STATE else "仍可写"
        client.check(f"{name} {suffix}", PASS if ok else FAIL,
                     _result_name(ack))

    t = time.monotonic()
    seq = client.send(P.Cmd.SET_ENABLE, P.set_enable_payload(False))
    ack = ack_for(client, t, P.Cmd.SET_ENABLE, seq)
    client.check("失能底盘", PASS if ack is not None and ack.result == P.Result.OK
                 else FAIL, _result_name(ack))

    # A watchdog trip during the guard test latches 0x0202; clear it so the
    # next run does not start in FAULT.
    t = time.monotonic()
    seq = client.send(P.Cmd.CLEAR_FAULT, P.clear_fault_payload())
    ack_for(client, t, P.Cmd.CLEAR_FAULT, seq)


def _state_name(client: Client, since: float) -> str:
    status = client.wait_status(lambda s: True, 0.3, since=since)
    return P.STATE_NAMES.get(status["state"], "?") if status else "?"


# ------------------------------------------------------------------- main

def run_suite(client: Client, sim: bool, do_write: bool, do_clear: bool,
              do_guard: bool) -> int:
    print("\n== 存储 / 标定 / 调参台架测试 ==")
    print("默认级别：只读 + SET_PARAM（RAM）；--write 才动 Flash；"
          "--clear 会删日志与崩溃现场")

    if not step_link(client):
        return client.summary()

    step_disable(client)
    before = step_storage_read(client)
    step_tuning(client, before)
    step_calibration(client)
    step_log_read(client)
    step_crash_read(client, mark_read=do_write)

    if do_write:
        step_write(client, before)
    else:
        client.check("PARAM_SAVE / PARAM_RESET", SKIP, "加 --write 才执行")

    if do_clear:
        step_clear(client)
    else:
        client.check("LOG_CLEAR", SKIP, "加 --clear 才执行（破坏性）")

    if do_guard:
        step_guard(client)
    else:
        client.check("使能时拒绝标定", SKIP, "加 --guard 才执行")

    # Always leave the chassis disabled, whatever happened above.
    t = time.monotonic()
    seq = client.send(P.Cmd.SET_ENABLE, P.set_enable_payload(False))
    ack_for(client, t, P.Cmd.SET_ENABLE, seq)

    client.check("下行合法（无 CRC 错误）",
                 PASS if client.link.crc_errors == 0 else FAIL,
                 f"CRC 错误 {client.link.crc_errors} 帧")
    return client.summary()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="标定 / 调参 / 存储读写台架测试（protocol.md v1 5.4-5.5）")
    parser.add_argument("--list", action="store_true", help="列出串口后退出")
    parser.add_argument("--port", help="串口名，例如 COM7 或 /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--sim", action="store_true", help="接内置模拟器")
    parser.add_argument("--write", action="store_true",
                        help="执行 PARAM_SAVE / PARAM_RESET（写 Flash）")
    parser.add_argument("--clear", action="store_true",
                        help="执行 LOG_CLEAR（删除日志与崩溃现场，破坏性）")
    parser.add_argument("--guard", action="store_true",
                        help="使能底盘验证标定拒绝（轮子必须悬空）")
    args = parser.parse_args(argv)

    if args.list:
        from proto_loop import list_ports
        list_ports()
        return 0

    if not args.sim and not args.port:
        parser.error("需要 --port COM7，或 --sim")

    link = SimLink() if args.sim else SerialLink(args.port, args.baud)
    client = Client(link, echo=True, echo_reports=False, echo_odom=False)

    print(f"== {'模拟器' if args.sim else f'{args.port} @ {args.baud}'} ==")

    try:
        return run_suite(client, args.sim, args.write, args.clear, args.guard)
    finally:
        link.close()


if __name__ == "__main__":
    sys.exit(main())
