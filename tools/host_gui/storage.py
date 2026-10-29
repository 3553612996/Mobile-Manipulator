#!/usr/bin/env python3
"""Storage page: read back what the MCU saved (protocol.md 5.5).

Three readers live here, all of them cache-free:
  * STORAGE_QUERY  -> one REPORT_STORAGE frame with the flags and lifetime
    counters. This is the only way to know a PARAM_SAVE really finished.
  * LOG_EXPORT     -> REPORT_LOG at up to 20 Hz; the end frame (flags bit1)
    closes the export, and the viewer de-duplicates by seq because the ring
    keeps growing while we read it.
  * CRASH_READ     -> REPORT_CRASH chunks with no end frame; the record is
    complete at offset + total. A gap aborts the read instead of showing a
    dump with a hole in it.

LOG_CLEAR is behind a confirmation dialog: the crash record is the only copy
of that event.
"""

from __future__ import annotations

import csv
import datetime
import html
import time

from PySide6.QtCore import QTimer, Qt, Signal
from PySide6.QtWidgets import (QCheckBox, QComboBox, QFileDialog, QGridLayout,
                               QHBoxLayout, QLabel, QMessageBox,
                               QPlainTextEdit, QPushButton, QSpinBox,
                               QVBoxLayout, QWidget)

from . import protocol as p
from .protocol import Cmd, CrashAssembler
from .widgets import (CRITICAL, GOOD, INK, INK_2, MUTED, WARNING, Card,
                      StatTile)

# Longest silence the firmware may leave between two records / chunks before
# the page calls the transfer dead (firmware stops on 500 ms of link idle).
EXPORT_TIMEOUT_S = 1.5
CRASH_TIMEOUT_S = 1.0
MAX_RECORDS = 5000

LEVEL_COLOURS = {0: MUTED, 1: INK_2, 2: WARNING, 3: CRITICAL}


def _button(text: str, kind: str | None = None, height: int = 32) -> QPushButton:
    button = QPushButton(text)
    if kind:
        button.setProperty("kind", kind)
    button.setMinimumHeight(height)
    button.setCursor(Qt.PointingHandCursor)
    return button


class StoragePage(QWidget):
    backRequested = Signal()

    def __init__(self, host, parent=None):
        super().__init__(parent)
        self.host = host
        self.records: list[dict] = []
        self._seen: set[tuple[int, int]] = set()
        self.export_active = False
        self.export_source = 0
        self.export_last_at = 0.0
        self.crash = CrashAssembler()
        self.crash_active = False
        self.crash_last_at = 0.0
        self._build_ui()
        self._build_timer()

    # ================================================================ UI

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 12, 16, 12)
        layout.setSpacing(10)

        header = QHBoxLayout()
        header.setSpacing(10)
        back = _button("← 返回", "ghost", height=30)
        back.clicked.connect(self.backRequested.emit)
        title = QLabel("存储器")
        title.setObjectName("appTitle")
        subtitle = QLabel("STORAGE_QUERY 0x0A · LOG_EXPORT 0x0B · CRASH_READ 0x0C")
        subtitle.setObjectName("appSub")
        header.addWidget(back)
        header.addWidget(title)
        header.addWidget(subtitle)
        header.addStretch(1)
        self.status = QLabel("")
        self.status.setObjectName("muted")
        header.addWidget(self.status)
        layout.addLayout(header)

        body = QHBoxLayout()
        body.setSpacing(14)
        body.addWidget(self._build_left(), 0)
        body.addWidget(self._build_right(), 1)
        layout.addLayout(body, 1)

    def _build_left(self) -> QWidget:
        column = QWidget()
        column.setFixedWidth(380)
        layout = QVBoxLayout(column)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(14)

        card = Card("存储状态", "REPORT_STORAGE 0xC8 · 24 B")
        grid = QGridLayout()
        grid.setSpacing(8)
        self.tile_param_seq = StatTile("参数区 seq")
        self.tile_calib_seq = StatTile("标定区 seq")
        self.tile_boot = StatTile("累计启动", "次")
        self.tile_log_used = StatTile("日志环占用", "KB")
        self.tile_erase = StatTile("擦除次数")
        self.tile_last_save = StatTile("上次保存", "s")
        self.tile_last_save.setToolTip("开机秒数；0 表示从未保存")
        tiles = (self.tile_param_seq, self.tile_calib_seq, self.tile_boot,
                 self.tile_log_used, self.tile_erase, self.tile_last_save)
        for index, tile in enumerate(tiles):
            grid.addWidget(tile, index // 3, index % 3)
            grid.setColumnStretch(index % 3, 1)
        card.body.addLayout(grid)

        flags = QGridLayout()
        flags.setSpacing(4)
        self.flag_labels: list[QLabel] = []
        for index, name in enumerate(p.STORAGE_FLAG_NAMES):
            label = QLabel(f"○ {name}")
            label.setObjectName("fieldLabel")
            self.flag_labels.append(label)
            flags.addWidget(label, index // 2, index % 2)
        card.body.addLayout(flags)

        self.query_btn = _button("刷新存储状态", None, height=32)
        self.query_btn.clicked.connect(self._query)
        card.body.addWidget(self.query_btn)
        card.body.addStretch(1)
        layout.addWidget(card, 1)

        layout.addWidget(self._build_crash_card(), 1)
        return column

    def _build_crash_card(self) -> QWidget:
        card = Card("崩溃现场", "CRASH_READ 0x0C · slot 0–15")
        row = QHBoxLayout()
        row.setSpacing(8)
        self.slot_combo = QComboBox()
        self.slot_combo.addItem("最新有效", p.CRASH_SLOT_LATEST)
        for slot in range(p.CRASH_SLOT_COUNT):
            self.slot_combo.addItem(f"slot {slot}", slot)
        row.addWidget(self.slot_combo, 1)
        self.mark_read = QCheckBox("读后标记已读")
        self.mark_read.setChecked(True)
        self.mark_read.setToolTip(
            "置 CRASH_READ 的 flags bit0，清除 REPORT_STORAGE 的 bit6")
        row.addWidget(self.mark_read)
        card.body.addLayout(row)

        row = QHBoxLayout()
        row.setSpacing(8)
        self.read_btn = _button("读取", "primary", height=32)
        self.read_btn.clicked.connect(self._on_read_crash)
        self.save_crash_btn = _button("保存 .bin", None, height=32)
        self.save_crash_btn.clicked.connect(self._on_save_crash)
        row.addWidget(self.read_btn, 1)
        row.addWidget(self.save_crash_btn, 1)
        card.body.addLayout(row)

        self.crash_status = QLabel("未读取")
        self.crash_status.setObjectName("muted")
        card.body.addWidget(self.crash_status)

        self.dump = QPlainTextEdit()
        self.dump.setObjectName("log")
        self.dump.setReadOnly(True)
        self.dump.setMinimumHeight(150)
        self.dump.setPlaceholderText("读取后在这里显示十六进制 + ASCII")
        card.body.addWidget(self.dump, 1)
        return card

    def _build_right(self) -> QWidget:
        column = QWidget()
        layout = QVBoxLayout(column)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(14)

        export_card = Card("日志导出", "LOG_EXPORT 0x0B · 上限 20 Hz")
        row = QHBoxLayout()
        row.setSpacing(8)
        label = QLabel("来源")
        label.setObjectName("fieldLabel")
        row.addWidget(label)
        self.source_combo = QComboBox()
        for source, name in p.LOG_SOURCE_NAMES.items():
            self.source_combo.addItem(name, source)
        row.addWidget(self.source_combo)
        row.addSpacing(10)
        label = QLabel("起始 seq")
        label.setObjectName("fieldLabel")
        row.addWidget(label)
        self.from_seq = QSpinBox()
        self.from_seq.setRange(0, 999999999)
        self.from_seq.setValue(0)
        self.from_seq.setFixedWidth(110)
        self.from_seq.setToolTip("0 = 从环内最旧一条开始")
        row.addWidget(self.from_seq)
        row.addStretch(1)

        self.start_btn = _button("开始导出", "primary", height=32)
        self.start_btn.clicked.connect(self._on_start_export)
        self.stop_btn = _button("停止", None, height=32)
        self.stop_btn.clicked.connect(lambda: self._stop_export())
        row.addWidget(self.start_btn)
        row.addWidget(self.stop_btn)
        export_card.body.addLayout(row)

        row = QHBoxLayout()
        row.setSpacing(8)
        self.clear_combo = QComboBox()
        for source, name in enumerate(("运行日志", "故障事件", "崩溃现场", "全部")):
            self.clear_combo.addItem(name, source)
        row.addWidget(self.clear_combo)
        self.clear_btn = _button("清空…", "ghost", height=28)
        self.clear_btn.clicked.connect(self._on_clear)
        row.addWidget(self.clear_btn)
        row.addStretch(1)
        self.export_status = QLabel("空闲")
        self.export_status.setObjectName("fieldValue")
        row.addWidget(self.export_status)
        export_card.body.addLayout(row)
        layout.addWidget(export_card)

        records_card = Card("记录", "REPORT_LOG 0xC9 · 按 seq 去重")
        self.count_label = QLabel("0 条")
        self.count_label.setObjectName("muted")
        records_card.add_header_widget(self.count_label)
        save_csv = _button("保存 CSV", "ghost", height=24)
        save_csv.setFixedWidth(76)
        save_csv.clicked.connect(self._on_save_csv)
        records_card.add_header_widget(save_csv)
        clear_view = _button("清屏", "ghost", height=24)
        clear_view.setFixedWidth(52)
        clear_view.clicked.connect(self._on_clear_view)
        records_card.add_header_widget(clear_view)

        self.view = QPlainTextEdit()
        self.view.setObjectName("log")
        self.view.setReadOnly(True)
        self.view.setMaximumBlockCount(MAX_RECORDS)
        self.view.setPlaceholderText(
            "选择来源后点击“开始导出”，MCU 会逐条上报日志或故障事件")
        records_card.body.addWidget(self.view, 1)
        layout.addWidget(records_card, 1)
        return column

    def _build_timer(self) -> None:
        self.timer = QTimer(self)
        self.timer.setInterval(200)
        self.timer.timeout.connect(self._tick)
        self.timer.start()

    # ========================================================== storage

    def _query(self) -> None:
        if not self.host.connected:
            return
        self.host._send(
            Cmd.STORAGE_QUERY, b"", acked=True,
            on_fail=lambda reason: self.status.setText(
                f"STORAGE_QUERY 失败：{reason}"))

    def on_storage(self, info: dict) -> None:
        self.tile_param_seq.set_value(str(info["param_seq"]))
        self.tile_calib_seq.set_value(str(info["calib_seq"]))
        self.tile_boot.set_value(str(info["boot_total"]))
        self.tile_log_used.set_value(f"{info['log_used_bytes'] / 1024:.1f}")
        self.tile_erase.set_value(str(info["log_erase_count"]))
        self.tile_last_save.set_value(
            "—" if info["last_save_s"] == 0 else str(info["last_save_s"]))

        for index, (name, set_) in enumerate(p.storage_flag_states(info)):
            label = self.flag_labels[index]
            if set_:
                colour = {"保存失败": CRITICAL, "有未读崩溃": WARNING,
                          "日志已回卷": WARNING}.get(name, GOOD)
                label.setText(f"● {name}")
                label.setStyleSheet(f"color: {colour};")
            else:
                label.setText(f"○ {name}")
                label.setStyleSheet(f"color: {MUTED};")

        state = "已全部落盘" if info["clean"] else "有未落盘修改"
        if info["save_failed"]:
            state = "上次保存失败"
        self.status.setText(f"存储状态已刷新 · {state}")

    # ============================================================ export

    def _on_start_export(self) -> None:
        if not self.host.connected:
            return
        self.export_source = int(self.source_combo.currentData())
        self.export_last_at = time.monotonic()
        self.export_active = True
        self.export_status.setStyleSheet("")
        self.export_status.setText("导出中…")
        self.host._send(
            Cmd.LOG_EXPORT,
            p.log_export_payload(self.export_source, 1, self.from_seq.value()),
            acked=True,
            on_ok=lambda: self.export_status.setText("已受理，逐条接收…"),
            on_fail=self._export_rejected)
        self.start_btn.setEnabled(False)

    def _export_rejected(self, reason: str) -> None:
        self.export_active = False
        self.export_status.setText(f"导出被拒绝：{reason}")
        self.export_status.setStyleSheet(f"color: {CRITICAL};")

    def _stop_export(self, quiet: bool = False) -> None:
        was_active = self.export_active
        self.export_active = False
        if not quiet:
            self.export_status.setText("已停止")
            self.export_status.setStyleSheet("")
        if was_active and self.host.connected:
            self.host._send(
                Cmd.LOG_EXPORT,
                p.log_export_payload(self.export_source, 0, 0), acked=True)

    def on_log(self, record: dict) -> None:
        now = time.monotonic()
        if self.export_active:
            self.export_last_at = now

        if record["end"]:
            self.export_active = False
            self.export_status.setText(f"导出完成 · 共 {len(self.records)} 条")
            self.export_status.setStyleSheet(f"color: {GOOD};")
            return

        key = (int(record["source"]), int(record["seq"]))
        if key in self._seen:
            # The ring keeps growing during the export; seq is the identity.
            return
        self._seen.add(key)

        self.records.append(record)
        if len(self.records) > MAX_RECORDS:
            del self.records[:len(self.records) - MAX_RECORDS]
        self._append_record(record)
        self.count_label.setText(f"{len(self.records)} 条")

    def _append_record(self, record: dict) -> None:
        source = p.LOG_SOURCE_NAMES.get(record["source"], "?")
        level = p.LOG_LEVEL_NAMES.get(record["level"], "?")
        tag = p.LOG_TAG_NAMES.get(record["tag"], f"#{record['tag']}")
        if record["source"] == 1:
            body = p.FAULT_NAMES.get(record["code"],
                                     f"0x{record['code']:04X}")
        else:
            body = record["text"]
        line = (f"t={record['ts_s']:>7}s  #{record['seq']:<6d}  "
                f"{level:<5s}  {source}  {tag:<3s}  {body}")
        colour = LEVEL_COLOURS.get(record["level"], MUTED)
        self.view.appendHtml(
            f'<span style="color:{colour};">{html.escape(line)}</span>')
        bar = self.view.verticalScrollBar()
        bar.setValue(bar.maximum())

    def _rebuild_view(self) -> None:
        self.view.clear()
        for record in self.records:
            self._append_record(record)
        self.count_label.setText(f"{len(self.records)} 条")

    def _on_clear_view(self) -> None:
        self.records.clear()
        self._seen.clear()
        self.view.clear()
        self.count_label.setText("0 条")

    def _on_clear(self) -> None:
        if not self.host.connected:
            return
        source = int(self.clear_combo.currentData())
        name = self.clear_combo.currentText()
        answer = QMessageBox.question(
            self, "清空存储内容",
            f"将清空「{name}」。崩溃现场是唯一副本，清空后无法恢复。\n\n"
            "是否继续？", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if answer != QMessageBox.Yes:
            return
        self.host._send(
            Cmd.LOG_CLEAR, p.log_clear_payload(source), acked=True,
            on_ok=lambda: self._clear_accepted(source),
            on_fail=lambda reason: self.export_status.setText(
                f"清空被拒绝：{reason}"))

    def _clear_accepted(self, source: int) -> None:
        if source >= 3:
            self.records.clear()
            self._seen.clear()
        else:
            self.records = [item for item in self.records
                            if item["source"] != source]
            self._seen = {key for key in self._seen if key[0] != source}
        self._rebuild_view()
        self.export_status.setStyleSheet(f"color: {INK};")
        self.export_status.setText("清空已受理（后台擦除）")
        self.host._log(f"LOG_CLEAR source={source} 已受理", "warn")

    def _on_save_csv(self) -> None:
        if not self.records:
            return
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path, _ = QFileDialog.getSaveFileName(
            self, "保存日志", f"log_{stamp}.csv", "CSV (*.csv)")
        if not path:
            return
        try:
            with open(path, "w", newline="", encoding="utf-8-sig") as handle:
                writer = csv.writer(handle)
                writer.writerow(["开机秒", "seq", "来源", "等级", "标签",
                                 "故障码", "文本"])
                for record in self.records:
                    writer.writerow([
                        record["ts_s"], record["seq"],
                        p.LOG_SOURCE_NAMES.get(record["source"], ""),
                        p.LOG_LEVEL_NAMES.get(record["level"], ""),
                        p.LOG_TAG_NAMES.get(record["tag"], ""),
                        f"0x{record['code']:04X}" if record["code"] else "",
                        record["text"]])
        except OSError as exc:
            self.host._log(f"日志保存失败：{exc}", "error")
            return
        self.host._log(f"日志已保存到 {path}（{len(self.records)} 条）", "ok")

    # ============================================================= crash

    def _on_read_crash(self) -> None:
        if not self.host.connected:
            return
        slot = int(self.slot_combo.currentData())
        self.crash.reset()
        self.crash_active = True
        self.crash_last_at = time.monotonic()
        self.crash_status.setStyleSheet("")
        self.crash_status.setText("读取中…")
        self.dump.clear()
        self.host._send(
            Cmd.CRASH_READ,
            p.crash_read_payload(slot, self.mark_read.isChecked()), acked=True,
            on_ok=lambda: self.crash_status.setText("接收分片…"),
            on_fail=self._crash_refused)

    def _crash_refused(self, reason: str) -> None:
        self.crash_active = False
        self.crash_status.setText(f"没有可读的崩溃记录：{reason}")
        self.crash_status.setStyleSheet(f"color: {CRITICAL};")

    def on_crash(self, chunk: dict) -> None:
        if not self.crash_active:
            return          # a stray chunk from an aborted read
        self.crash_last_at = time.monotonic()

        outcome = self.crash.feed(chunk)
        if outcome == "bad":
            self.crash_active = False
            self.crash.reset()
            self.crash_status.setText("分片字段非法，已丢弃；请重新读取")
            self.crash_status.setStyleSheet(f"color: {CRITICAL};")
            return
        if outcome == "gap":
            self.crash_active = False
            self.crash.reset()
            self.crash_status.setText("分片不连续（offset 跳变），已中止；请重新读取")
            self.crash_status.setStyleSheet(f"color: {CRITICAL};")
            return

        self.crash_status.setText(
            f"接收中 slot={self.crash.slot} "
            f"{self.crash.offset}/{self.crash.total_len} B")

        if outcome == "done":
            self.crash_active = False
            self.crash_status.setText(
                f"完成 slot={self.crash.slot} · {self.crash.total_len} B")
            self.crash_status.setStyleSheet(f"color: {GOOD};")
            self._render_dump(self.crash.record())
            self.host._log(
                f"崩溃记录已读取 slot={self.crash.slot} "
                f"{self.crash.total_len} B", "ok")
            if self.mark_read.isChecked():
                # flags bit0 clears REPORT_STORAGE bit6; refresh so the panel
                # does not keep claiming the record is unread.
                self._query()

    def _render_dump(self, data: bytes) -> None:
        lines = []
        for offset in range(0, len(data), 16):
            chunk = data[offset:offset + 16]
            hex_part = " ".join(f"{byte:02X}" for byte in chunk).ljust(47)
            text = "".join(chr(byte) if 32 <= byte < 127 else "."
                           for byte in chunk)
            lines.append(f"{offset:04X}  {hex_part}  {text}")
        self.dump.setPlainText("\n".join(lines))

    def _on_save_crash(self) -> None:
        data = self.crash.record()
        if not data:
            return
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path, _ = QFileDialog.getSaveFileName(
            self, "保存崩溃现场", f"crash_{stamp}.bin", "Binary (*.bin)")
        if not path:
            return
        try:
            with open(path, "wb") as handle:
                handle.write(data)
        except OSError as exc:
            self.host._log(f"崩溃现场保存失败：{exc}", "error")
            return
        self.host._log(f"崩溃现场已保存到 {path}（{len(data)} B）", "ok")

    # ============================================================== state

    def on_enter(self) -> None:
        self.status.setStyleSheet("")
        self.status.setText("进入即查询一次存储状态")
        self.export_status.setStyleSheet("")
        self._update_states()
        self._query()

    def on_leave(self) -> None:
        if self.crash_active:
            self.crash_active = False
            self.crash.reset()
            self.crash_status.setText("已中止")
        self._stop_export(quiet=True)

    def on_link_lost(self) -> None:
        self.export_active = False
        self.crash_active = False
        self.crash.reset()
        self.start_btn.setEnabled(False)
        self.export_status.setText("链路已断开")
        self.crash_status.setText("链路已断开")

    def _tick(self) -> None:
        self._update_states()
        now = time.monotonic()

        if self.export_active and now - self.export_last_at > EXPORT_TIMEOUT_S:
            self.export_active = False
            self.export_status.setText(
                f"已停止：{EXPORT_TIMEOUT_S:.1f} s 没有新记录")
            self.export_status.setStyleSheet(f"color: {WARNING};")

        if self.crash_active and now - self.crash_last_at > CRASH_TIMEOUT_S:
            self.crash_active = False
            self.crash.reset()
            self.crash_status.setText("超时：记录不完整，请重新读取")
            self.crash_status.setStyleSheet(f"color: {CRITICAL};")

    def _update_states(self) -> None:
        connected = self.host.connected
        self.query_btn.setEnabled(connected)
        self.start_btn.setEnabled(connected and not self.export_active)
        self.stop_btn.setEnabled(connected and self.export_active)
        self.clear_btn.setEnabled(connected)
        self.read_btn.setEnabled(connected and not self.crash_active)
        self.save_crash_btn.setEnabled(
            not self.crash_active and len(self.crash.buffer) > 0)
