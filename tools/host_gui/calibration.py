#!/usr/bin/env python3
"""Calibration page for the host console.

Every control maps to one SET_PARAM id from protocol.md 5.4 and writes RAM
only; persisting is a separate PARAM_SAVE into the calibration region. v1 has
no GET_PARAM, so the page cannot read what the device actually holds: the
initial numbers are the compile-time defaults from app_robot_config.h and the
page remembers only what this session has written.

Calibration ids (0x01, 0x02, 0x50..0x5F) require a disabled chassis, and so
does a PARAM_SAVE with scope 1. The page greys those controls out while the
motors are enabled; the firmware re-checks and answers 0x02 if the host races
it. The parameter-area limits (0x10..0x12) are writable while running.
"""

from __future__ import annotations

from PySide6.QtCore import QTimer, Qt, Signal
from PySide6.QtWidgets import (QComboBox, QGridLayout, QHBoxLayout, QLabel,
                               QMessageBox, QPushButton, QSpinBox,
                               QVBoxLayout, QWidget)

from . import protocol as p
from .protocol import Cmd
from .widgets import CRITICAL, GOOD, WARNING, Card

# app_robot_config.h: half_width 120 (track = 240 mm), wheel diameter 60 mm,
# counts_per_mm = 13 x 4 x 90 / (pi x 60) = 24.828. The startup duty is the
# measured reliable start per wheel (docs/bench/pid-tuning.md 3.3), FL/FR/RL/RR.
DEFAULT_WHEEL_BASE_MM = 240
DEFAULT_WHEEL_DIAMETER_MM = 60
DEFAULT_COUNTS_PER_MM = 24828
DEFAULT_DUTY_FWD = (75, 67, 82, 80)
DEFAULT_DUTY_REV = (67, 75, 80, 100)
DEFAULT_TIMEOUT_MS = 200
DEFAULT_MAX_LINEAR = 500
DEFAULT_MAX_ANGULAR = 1500


def _button(text: str, kind: str | None = None, height: int = 32) -> QPushButton:
    button = QPushButton(text)
    if kind:
        button.setProperty("kind", kind)
    button.setMinimumHeight(height)
    button.setCursor(Qt.PointingHandCursor)
    return button


class CalibrationPage(QWidget):
    backRequested = Signal()

    def __init__(self, host, parent=None):
        super().__init__(parent)
        self.host = host
        # (param id, widget, label, belongs to the calibration region)
        self._entries: list[tuple[int, QWidget, str, bool]] = []
        self._defaults: dict[int, int] = {}
        # What this session believes the device holds. Starts from the
        # defaults above, because v1 cannot read the device.
        self._written: dict[int, int] = {}
        self._queue: list[tuple[int, int, str]] = []
        self._writing = False
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
        title = QLabel("标定")
        title.setObjectName("appTitle")
        subtitle = QLabel("SET_PARAM 0x01/0x02/0x50–0x5F · 标定区 · 写入要求底盘失能")
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
        body.addWidget(self._build_geometry_card(), 1)
        body.addWidget(self._build_startup_card(), 1)
        body.addWidget(self._build_write_card(), 0)
        layout.addLayout(body, 1)

        layout.addWidget(self._build_limits_card(), 0)

    def _spin(self, minimum: int, maximum: int, value: int,
              step: int = 1) -> QSpinBox:
        box = QSpinBox()
        box.setRange(minimum, maximum)
        box.setSingleStep(step)
        box.setValue(value)
        box.setAlignment(Qt.AlignRight)
        box.setFixedWidth(104)
        # Do not re-count on every keystroke while the user types.
        box.setKeyboardTracking(False)
        box.valueChanged.connect(lambda _value: self._update_pending())
        return box

    def _field(self, label: str, widget: QWidget, unit: str = "",
               name_width: int = 72) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(8)
        name = QLabel(label)
        name.setObjectName("fieldLabel")
        name.setFixedWidth(name_width)
        row.addWidget(name)
        row.addWidget(widget, 1)
        if unit:
            suffix = QLabel(unit)
            suffix.setObjectName("fieldValue")
            row.addWidget(suffix)
        return row

    def _hint(self, text: str) -> QLabel:
        hint = QLabel(text)
        hint.setObjectName("hint")
        hint.setWordWrap(True)
        return hint

    def _build_geometry_card(self) -> QWidget:
        card = Card("标定区 · 几何与编码器", "0x01 / 0x02 / 0x50–0x53")

        self.spin_base = self._spin(50, 2000, DEFAULT_WHEEL_BASE_MM, 5)
        self.spin_diameter = self._spin(10, 500, DEFAULT_WHEEL_DIAMETER_MM)
        card.body.addLayout(self._field("轮距", self.spin_base, "mm"))
        card.body.addLayout(self._field("轮径", self.spin_diameter, "mm"))
        self._register(p.GEOMETRY_IDS["wheel_base"], self.spin_base, "轮距", True)
        self._register(p.GEOMETRY_IDS["wheel_diameter"], self.spin_diameter,
                       "轮径", True)

        separator = QLabel("编码器比例 counts_per_mm ×1000")
        separator.setObjectName("fieldLabel")
        card.body.addWidget(separator)

        grid = QGridLayout()
        grid.setSpacing(8)
        self.spin_counts: list[QSpinBox] = []
        for wheel, name in enumerate(p.WHEEL_NAMES):
            spin = self._spin(1000, 100000, DEFAULT_COUNTS_PER_MM, 10)
            self.spin_counts.append(spin)
            cell = QHBoxLayout()
            cell.setSpacing(6)
            label = QLabel(name.split()[0])
            label.setObjectName("fieldLabel")
            label.setFixedWidth(16)
            cell.addWidget(label)
            cell.addWidget(spin, 1)
            grid.addLayout(cell, wheel // 2, wheel % 2)
            self._register(p.COUNTS_PER_MM_IDS[wheel], spin,
                           f"{name} 编码器比例", True)
        card.body.addLayout(grid)

        card.body.addWidget(self._hint(
            "协议 v1 没有 GET_PARAM，页面读不到设备当前值；上面的数字是 "
            "app_robot_config.h 的编译期默认值，按实测结果改。轮距按两轮中心距填写。"))
        card.body.addStretch(1)
        return card

    def _build_startup_card(self) -> QWidget:
        card = Card("标定区 · 起转与方向", "0x54–0x5F")

        grid = QGridLayout()
        grid.setSpacing(8)
        for column, name in enumerate(("轮", "正向 ‰", "反向 ‰", "方向")):
            label = QLabel(name)
            label.setObjectName("fieldLabel")
            grid.addWidget(label, 0, column)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 1)

        self.spin_duty_fwd: list[QSpinBox] = []
        self.spin_duty_rev: list[QSpinBox] = []
        self.combo_direction: list[QComboBox] = []
        for wheel, name in enumerate(p.WHEEL_NAMES):
            tag = QLabel(name.split()[0])
            tag.setObjectName("fieldLabel")
            grid.addWidget(tag, wheel + 1, 0)

            forward = self._spin(0, 1000, DEFAULT_DUTY_FWD[wheel], 10)
            reverse = self._spin(0, 1000, DEFAULT_DUTY_REV[wheel], 10)
            forward.setMinimumWidth(84)
            reverse.setMinimumWidth(84)
            self.spin_duty_fwd.append(forward)
            self.spin_duty_rev.append(reverse)
            grid.addWidget(forward, wheel + 1, 1)
            grid.addWidget(reverse, wheel + 1, 2)

            direction = QComboBox()
            direction.addItem("+1 正转", 1)
            direction.addItem("-1 反转", -1)
            direction.setFixedWidth(92)
            direction.currentIndexChanged.connect(
                lambda _index: self._update_pending())
            self.combo_direction.append(direction)
            grid.addWidget(direction, wheel + 1, 3)

            self._register(p.DUTY_FWD_IDS[wheel], forward,
                           f"{name} 正向起转", True)
            self._register(p.DUTY_REV_IDS[wheel], reverse,
                           f"{name} 反向起转", True)
            self._register(p.DIRECTION_IDS[wheel], direction,
                           f"{name} 轮方向", True)

        card.body.addLayout(grid)
        card.body.addWidget(self._hint(
            "起转占空比要开环实测：从 0 慢慢加，直到轮子可靠起步，正反两个方向"
            "分别记录。方向写 -1 就翻转该轮的编码器与出力方向，里程计同向修正。"))
        card.body.addStretch(1)
        return card

    def _build_write_card(self) -> QWidget:
        column = QWidget()
        column.setFixedWidth(330)
        layout = QVBoxLayout(column)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        card = Card("写入与保存", "SET_PARAM → PARAM_SAVE")
        self.pending_label = QLabel("待写入 0 项")
        self.pending_label.setObjectName("fieldValue")
        card.body.addWidget(self.pending_label)

        self.write_btn = _button("写入已修改项", "primary", height=36)
        self.write_btn.clicked.connect(self._on_write)
        card.body.addWidget(self.write_btn)

        self.save_calib_btn = _button("保存到标定区（scope=1）", None, height=34)
        self.save_calib_btn.clicked.connect(
            lambda: self._on_save(p.SCOPE_CALIB))
        card.body.addWidget(self.save_calib_btn)

        self.save_param_btn = _button("保存参数区（scope=0）", None, height=34)
        self.save_param_btn.clicked.connect(
            lambda: self._on_save(p.SCOPE_PARAM))
        card.body.addWidget(self.save_param_btn)

        self.reset_btn = _button("恢复出厂参数…", "ghost", height=32)
        self.reset_btn.setToolTip("PARAM_RESET scope=0：只重置参数区，标定区保留")
        self.reset_btn.clicked.connect(self._on_reset)
        card.body.addWidget(self.reset_btn)

        card.body.addWidget(self._hint(
            "SET_PARAM 只写 RAM 并置脏；PARAM_SAVE 才擦写 Flash。保存是否完成看"
            "下方状态，不看 ACK。"))

        self.save_state = QLabel("存储状态未知")
        self.save_state.setObjectName("fieldValue")
        self.save_state.setWordWrap(True)
        card.body.addWidget(self.save_state)

        self.seq_label = QLabel("")
        self.seq_label.setObjectName("muted")
        card.body.addWidget(self.seq_label)
        card.body.addStretch(1)

        layout.addWidget(card, 1)
        return column

    def _build_limits_card(self) -> QWidget:
        card = Card("参数区 · 运行限制", "0x10–0x12 · 运行中可写")
        grid = QGridLayout()
        grid.setSpacing(8)

        self.spin_timeout = self._spin(10, 5000, DEFAULT_TIMEOUT_MS, 10)
        # Bounds match app_params.c: the chassis cannot exceed 500 mm/s per
        # wheel or 1500 mrad/s, so a higher configured limit would only accept
        # commands the wheels silently saturate on.
        self.spin_max_linear = self._spin(10, 500, DEFAULT_MAX_LINEAR, 10)
        self.spin_max_angular = self._spin(10, 1500, DEFAULT_MAX_ANGULAR, 10)
        for column, (label, spin, unit) in enumerate((
                ("控制指令超时", self.spin_timeout, "ms"),
                ("最大线速度", self.spin_max_linear, "mm/s"),
                ("最大角速度", self.spin_max_angular, "mrad/s"))):
            grid.addLayout(self._field(label, spin, unit,
                                       name_width=88), 0, column)
        card.body.addLayout(grid)
        self._register(p.LIMIT_IDS["timeout_ms"], self.spin_timeout,
                       "控制指令超时", False)
        self._register(p.LIMIT_IDS["max_linear"], self.spin_max_linear,
                       "最大线速度", False)
        self._register(p.LIMIT_IDS["max_angular"], self.spin_max_angular,
                       "最大角速度", False)
        card.body.addWidget(self._hint(
            "这三项属于参数区：「恢复出厂」会把它们一并重置，标定区不受影响。"))
        return card

    def _register(self, param_id: int, widget: QWidget, label: str,
                  calib: bool) -> None:
        value = self._current(widget)
        self._entries.append((param_id, widget, label, calib))
        self._defaults[param_id] = value
        self._written[param_id] = value

    @staticmethod
    def _current(widget: QWidget) -> int:
        if isinstance(widget, QComboBox):
            return int(widget.currentData())
        return int(widget.value())

    # ============================================================== timers

    def _build_timer(self) -> None:
        self.refresh_timer = QTimer(self)
        self.refresh_timer.setInterval(100)
        self.refresh_timer.timeout.connect(self._refresh)
        self.refresh_timer.start()

    # ============================================================ writing

    def _changed(self) -> list[tuple[int, int, str, bool]]:
        changed = []
        for param_id, widget, label, calib in self._entries:
            value = self._current(widget)
            if value != self._written.get(param_id):
                changed.append((param_id, value, label, calib))
        return changed

    def _update_pending(self) -> None:
        changed = self._changed()
        self.pending_label.setText(f"待写入 {len(changed)} 项")
        # Grey the button out when the chassis state forbids the change.
        blocked = any(item[3] for item in changed) and self.host.enabled
        self.write_btn.setEnabled(
            self.host.connected and not self._writing and not blocked)
        if blocked:
            self.pending_label.setText(
                f"待写入 {len(changed)} 项 · 标定要求底盘失能")

    def _on_write(self) -> None:
        if not self.host.connected:
            self.status.setText("链路未连接")
            return

        changed = self._changed()
        if not changed:
            self.status.setText("没有修改")
            return
        if any(item[3] for item in changed) and self.host.enabled:
            self.status.setText("标定写入要求底盘失能，请先关闭电机使能")
            return

        self._queue = [(param_id, value, label)
                       for param_id, value, label, _calib in changed]
        self._writing = True
        self.write_btn.setEnabled(False)
        self.status.setText(f"写入 {len(self._queue)} 项…")
        self._send_next()

    def _send_next(self) -> None:
        if not self._queue:
            self._writing = False
            self._update_pending()
            self.status.setText("写入完成；如要断电保存，请再点“保存到标定区”")
            return

        param_id, value, label = self._queue.pop(0)
        seq = self.host._send(
            Cmd.SET_PARAM, p.set_param_payload(param_id, value), acked=True,
            on_ok=lambda pid=param_id, val=value, name=label:
                self._write_acked(pid, val, name),
            on_fail=lambda reason, name=label: self._write_failed(name, reason))
        if seq is None:
            self._writing = False
            self._queue.clear()
            self.status.setText("链路已断开，写入中止")
            self._update_pending()

    def _write_acked(self, param_id: int, value: int, label: str) -> None:
        self._written[param_id] = value
        self.host._log(f"标定/参数写入 {label} = {value}", "ok")
        self._send_next()

    def _write_failed(self, label: str, reason: str) -> None:
        self._writing = False
        self._queue.clear()
        self.status.setText(f"{label} 写入失败：{reason}")
        self.status.setStyleSheet(f"color: {CRITICAL};")
        self.host._log(f"标定/参数写入失败 {label}: {reason}", "error")
        self._update_pending()

    # ============================================================= saving

    def _on_save(self, scope: int) -> None:
        if not self.host.connected:
            self.status.setText("链路未连接")
            return
        if scope == p.SCOPE_CALIB and self.host.enabled:
            self.status.setText("保存标定区要求底盘失能")
            return
        name = "标定区" if scope == p.SCOPE_CALIB else "参数区"
        self.status.setText(f"已请求保存{name}…")
        self.host._send(
            Cmd.PARAM_SAVE, p.param_save_payload(scope), acked=True,
            on_ok=lambda: self._save_accepted(name),
            on_fail=lambda reason: self._save_failed(name, reason))

    def _save_accepted(self, name: str) -> None:
        self.status.setText(f"{name}保存已受理，等待 Flash 完成（看右侧状态）")
        self.status.setStyleSheet(f"color: {WARNING};")

    def _save_failed(self, name: str, reason: str) -> None:
        self.status.setText(f"{name}保存被拒绝：{reason}")
        self.status.setStyleSheet(f"color: {CRITICAL};")
        self.host._log(f"{name} PARAM_SAVE 失败：{reason}", "error")

    def _on_reset(self) -> None:
        if not self.host.connected:
            self.status.setText("链路未连接")
            return
        answer = QMessageBox.question(
            self, "恢复出厂参数",
            "把参数区恢复为固件编译期默认值（PID 增益、日志等级、运行限制），"
            "标定区保留。\n\n恢复只改 RAM 并置脏，还需要再点“保存参数区”才写进 "
            "Flash。是否继续？",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if answer != QMessageBox.Yes:
            return
        self.host._send(
            Cmd.PARAM_RESET, p.param_reset_payload(p.SCOPE_PARAM), acked=True,
            on_ok=self._reset_acked,
            on_fail=lambda reason: self._save_failed("参数区恢复", reason))

    def _reset_acked(self) -> None:
        for param_id, widget, _label, calib in self._entries:
            if calib:
                continue
            value = self._defaults[param_id]
            if isinstance(widget, QComboBox):
                index = widget.findData(value)
                if index >= 0:
                    widget.setCurrentIndex(index)
            else:
                widget.setValue(value)
            self._written[param_id] = value
        self.status.setText("参数区已恢复默认值（未落盘）")
        self.status.setStyleSheet(f"color: {WARNING};")
        self.host._log("参数区已恢复默认值，等待 PARAM_SAVE", "warn")
        self._update_pending()

    # ============================================================== state

    def on_enter(self) -> None:
        self.status.setStyleSheet("")
        self.status.setText("修改后点“写入已修改项”，再保存到 Flash")
        self._update_pending()
        self._refresh()

    def on_leave(self) -> None:
        self.status.setStyleSheet("")
        self.status.setText("")

    def _refresh(self) -> None:
        if not self.isVisible():
            return

        connected = self.host.connected
        can_calib = connected and not self.host.enabled

        for param_id, widget, _label, calib in self._entries:
            widget.setEnabled(connected and (can_calib or not calib))

        self.save_calib_btn.setEnabled(can_calib)
        self.save_param_btn.setEnabled(connected)
        self.reset_btn.setEnabled(connected and not self._writing)
        if not self._writing:
            self._update_pending()

        info = self.host.last_storage
        if info is None:
            self.save_state.setText("存储状态未知 · 发一次 STORAGE_QUERY")
            self.save_state.setStyleSheet("")
            self.seq_label.setText("")
            return

        if info.get("save_failed"):
            text, colour = "上次保存失败（REPORT_STORAGE bit3）", CRITICAL
        elif info.get("dirty"):
            text, colour = "有未落盘修改（bit0 未置位）", WARNING
        else:
            text, colour = "全部已落盘，可以断电", GOOD
        self.save_state.setText(text)
        self.save_state.setStyleSheet(f"color: {colour};")
        self.seq_label.setText(
            f"参数区 seq {info['param_seq']} · 标定区 seq {info['calib_seq']}"
            f" · 参数区{'有效' if info['param_valid'] else '无效'}"
            f" · 标定区{'有效' if info['calib_valid'] else '无效'}")
