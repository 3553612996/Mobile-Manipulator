#!/usr/bin/env python3
"""PID tuning page for the host console.

One set of Kp/Ki/Kd sliders writes to every selected wheel, matching the
firmware's SET_PARAM ids 0x20..0x2B. A/B/C/D map to FL/FR/RL/RR.

The page owns the low-speed test command as well: the tuning session is allowed
to command at most TUNING_MAX_V mm/s and zero yaw, so a slider mistake cannot
send the chassis across the bench. On entry and exit the host stops motion.
"""

from __future__ import annotations

import time

from PySide6.QtCore import QTimer, Qt, Signal
from PySide6.QtWidgets import (QGridLayout, QHBoxLayout, QLabel, QPushButton,
                               QVBoxLayout, QWidget)

from . import protocol as p
from .protocol import Cmd
from .widgets import (CRITICAL, GOOD, Card, SliderRow, StatTile,
                      WheelTraceChart)

TUNING_MAX_V = 150          # mm/s, host-side only
GAIN_SCALE = 1000           # slider int -> gain, matches SET_PARAM
SEND_DEBOUNCE_MS = 250


def _button(text: str, kind: str | None = None, checkable: bool = False,
            height: int = 32) -> QPushButton:
    button = QPushButton(text)
    if kind:
        button.setProperty("kind", kind)
    button.setCheckable(checkable)
    button.setMinimumHeight(height)
    button.setCursor(Qt.PointingHandCursor)
    return button


class TuningPage(QWidget):
    backRequested = Signal()

    def __init__(self, host, parent=None):
        super().__init__(parent)
        self.host = host
        self.selected = [True, True, True, True]
        self.gains = [[5.0, 6.0, 0.0] for _ in range(4)]
        self._build_ui()
        self._build_timers()

    # ================================================================ UI

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 12, 16, 12)
        layout.setSpacing(10)

        header = QHBoxLayout()
        header.setSpacing(10)
        back = _button("← 返回", "ghost", height=30)
        back.clicked.connect(self.backRequested.emit)
        title = QLabel("PID 调参")
        title.setObjectName("appTitle")
        subtitle = QLabel("SET_PARAM · 0x20–0x2B · 运行中生效")
        subtitle.setObjectName("appSub")
        header.addWidget(back)
        header.addWidget(title)
        header.addWidget(subtitle)
        header.addStretch(1)
        self.status = QLabel("选择轮后拖动滑条；松开 250 ms 后写入")
        self.status.setObjectName("muted")
        header.addWidget(self.status)
        layout.addLayout(header)

        columns = QHBoxLayout()
        columns.setSpacing(14)
        columns.addWidget(self._build_gain_card(), 1)
        columns.addWidget(self._build_speed_card(), 0)
        layout.addLayout(columns, 1)

        trace_card = Card("速度跟随",
                          "实线 = 实测轮速，虚线 = 目标 · 最近 6 s")
        self.trace = WheelTraceChart(p.WHEEL_NAMES)
        trace_card.body.addWidget(self.trace, 1)
        layout.addWidget(trace_card, 1)

    def _build_gain_card(self) -> QWidget:
        card = Card("轮速环增益", "同一组滑条写入所有选中的轮")

        wheel_grid = QGridLayout()
        wheel_grid.setSpacing(8)
        self.wheel_buttons: list[QPushButton] = []
        for index, name in enumerate(p.WHEEL_NAMES):
            button = _button(name, "toggle", checkable=True, height=38)
            button.setChecked(True)
            button.toggled.connect(self._on_selection_changed)
            self.wheel_buttons.append(button)
            wheel_grid.addWidget(button, index // 2, index % 2)
        card.body.addLayout(wheel_grid)

        hint = QLabel("A/B/C/D = 左前/右前/左后/右后。只选一个轮时，滑条显示"
                      "该轮当前值；多选时写入所有选中轮。")
        hint.setObjectName("hint")
        hint.setWordWrap(True)
        card.body.addWidget(hint)

        self.gain_rows: list[SliderRow] = []
        for gain, name in enumerate(("Kp", "Ki", "Kd")):
            maximum = 5000 if gain == 2 else 10000
            value = 1000 if gain == 0 else (600 if gain == 1 else 0)
            row = SliderRow(name, 0, maximum, value, GAIN_SCALE, 3,
                            center_fill=False, name_width=34)
            row.valueChanged.connect(
                lambda _value, g=gain: self._timers[g].start())
            self.gain_rows.append(row)
            card.body.addWidget(row)

        note = QLabel("改增益会复位该轮积分并把当前编码器作为新基线；"
                      "滑条在上位机限速 %d mm/s 内测试。" % TUNING_MAX_V)
        note.setObjectName("hint")
        note.setWordWrap(True)
        card.body.addWidget(note)
        card.body.addStretch(1)

        summary = QLabel("当前缓存")
        summary.setObjectName("fieldLabel")
        card.body.addWidget(summary)
        self.summary_labels: list[QLabel] = []
        for index in range(4):
            label = QLabel("")
            label.setObjectName("fieldValue")
            self.summary_labels.append(label)
            card.body.addWidget(label)
            self._update_summary(index)

        return card

    def _build_speed_card(self) -> QWidget:
        column = QWidget()
        column.setFixedWidth(380)
        layout = QVBoxLayout(column)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(14)

        speed_card = Card("轮速反馈", "REPORT_WHEEL_SPEED · 50 Hz")
        grid = QGridLayout()
        grid.setSpacing(8)
        self.speed_tiles: list[StatTile] = []
        for index, name in enumerate(p.WHEEL_NAMES):
            tile = StatTile(name, "mm/s")
            self.speed_tiles.append(tile)
            grid.addWidget(tile, index // 2, index % 2)
        speed_card.body.addLayout(grid)
        layout.addWidget(speed_card)

        test_card = Card("低速测试", "仅用于调参，限速 %d mm/s" % TUNING_MAX_V)
        self.test_speed = SliderRow("速度 v", -TUNING_MAX_V, TUNING_MAX_V, 0,
                                    1, 0, " mm/s", name_width=48)
        self.test_speed.valueChanged.connect(self._on_test_velocity)
        test_card.body.addWidget(self.test_speed)

        row = QHBoxLayout()
        row.setSpacing(8)
        self.enable_btn = _button("电机使能", "toggle", checkable=True, height=34)
        self.enable_btn.toggled.connect(self._on_enable_toggled)
        self.stop_btn = _button("停车", None, height=34)
        self.stop_btn.clicked.connect(self.host.stop_motion)
        row.addWidget(self.enable_btn, 1)
        row.addWidget(self.stop_btn, 1)
        test_card.body.addLayout(row)

        self.estop_btn = _button("急停  ESTOP", "danger", height=40)
        self.estop_btn.clicked.connect(self.host.emergency_stop)
        test_card.body.addWidget(self.estop_btn)

        hint = QLabel("调参页只下发 v，ω 固定为 0；离开页面立即停车。")
        hint.setObjectName("hint")
        hint.setWordWrap(True)
        test_card.body.addWidget(hint)
        test_card.body.addStretch(1)
        layout.addWidget(test_card, 1)
        return column

    def _build_timers(self) -> None:
        self._timers: list[QTimer] = []
        for gain in range(3):
            timer = QTimer(self)
            timer.setSingleShot(True)
            timer.setInterval(SEND_DEBOUNCE_MS)
            timer.timeout.connect(lambda g=gain: self._send_gain(g))
            self._timers.append(timer)

        self.refresh_timer = QTimer(self)
        self.refresh_timer.setInterval(100)
        self.refresh_timer.timeout.connect(self._refresh)
        self.refresh_timer.start()

    # ========================================================== selection

    def _selected_wheels(self) -> list[int]:
        return [index for index, on in enumerate(self.selected) if on]

    def _on_selection_changed(self, checked: bool) -> None:
        sender = self.sender()
        for index, button in enumerate(self.wheel_buttons):
            self.selected[index] = button.isChecked()

        if not any(self.selected):
            # Keep at least one wheel selected; zero selection has no meaning.
            if isinstance(sender, QPushButton):
                sender.blockSignals(True)
                sender.setChecked(True)
                sender.blockSignals(False)
                for index, button in enumerate(self.wheel_buttons):
                    self.selected[index] = button.isChecked()

        chosen = self._selected_wheels()
        if len(chosen) == 1:
            self._load_gains(chosen[0])

        self.status.setText(
            "已选：" + "、".join(p.WHEEL_NAMES[i].split()[0] for i in chosen))

    def _load_gains(self, wheel: int) -> None:
        values = [int(round(g * GAIN_SCALE)) for g in self.gains[wheel]]
        for row, value in zip(self.gain_rows, values):
            row.slider.blockSignals(True)
            row.slider.setValue(value)
            row.slider.blockSignals(False)
            row._update_text()

    def _update_summary(self, wheel: int) -> None:
        kp, ki, kd = self.gains[wheel]
        self.summary_labels[wheel].setText(
            f"{p.WHEEL_NAMES[wheel]}   Kp {kp:.3f}   Ki {ki:.3f}   "
            f"Kd {kd:.3f}")

    # ============================================================= writing

    def _send_gain(self, gain: int) -> None:
        chosen = self._selected_wheels()
        if not chosen:
            self.status.setText("至少选择一个轮")
            return
        if not self.host.connected or self.host.estopped:
            self.status.setText("链路未连接或急停中，未写入")
            return

        value = self.gain_rows[gain].value()
        name = ("Kp", "Ki", "Kd")[gain]

        for wheel in chosen:
            param_id = p.PID_PARAM_IDS[wheel][gain]
            self.host._send(
                Cmd.SET_PARAM, p.set_param_payload(param_id, value),
                acked=True,
                on_ok=lambda w=wheel, g=gain, v=value:
                    self._gain_acked(w, g, v),
                on_fail=lambda reason, w=wheel, g=gain:
                    self._gain_failed(w, g, reason))

        self.status.setText(f"写入 {name} = {value / GAIN_SCALE:.3f} ...")

    def _gain_acked(self, wheel: int, gain: int, value: int) -> None:
        self.gains[wheel][gain] = value / GAIN_SCALE
        self._update_summary(wheel)
        self.status.setText(
            f"{p.WHEEL_NAMES[wheel]} {('Kp', 'Ki', 'Kd')[gain]} = "
            f"{value / GAIN_SCALE:.3f} 已生效")
        self.status.setStyleSheet(f"color: {GOOD};")

    def _gain_failed(self, wheel: int, gain: int, reason: str) -> None:
        self.status.setText(
            f"{p.WHEEL_NAMES[wheel]} {('Kp', 'Ki', 'Kd')[gain]} 写入失败："
            f"{reason}")
        self.status.setStyleSheet(f"color: {CRITICAL};")
        self.host._log(
            f"PID 参数写入失败 {p.WHEEL_NAMES[wheel]} "
            f"{('Kp', 'Ki', 'Kd')[gain]}: {reason}", "error")

    # =========================================================== test drive

    def _on_test_velocity(self, value: int) -> None:
        if not (self.host.connected and self.host.enabled
                and not self.host.estopped):
            return
        self.host.v_target = max(-TUNING_MAX_V, min(TUNING_MAX_V, value))
        self.host.w_target = 0

    def _on_enable_toggled(self, checked: bool) -> None:
        self.host.request_enable(checked)

    def reset_test_speed(self) -> None:
        self.test_speed.slider.blockSignals(True)
        self.test_speed.setValue(0)
        self.test_speed.slider.blockSignals(False)
        self.test_speed._update_text()

    # ================================================================ trace

    def add_wheel_sample(self, data: dict, now: float) -> None:
        """Feed one REPORT_WHEEL_SPEED frame; the target is what we command."""
        if not self.isVisible():
            return
        speeds = (data["fl_mm_s"], data["fr_mm_s"],
                  data["rl_mm_s"], data["rr_mm_s"])
        target = float(self.host.v_target) if self.host.enabled else 0.0
        self.trace.add_sample(now, speeds, target)

    # ================================================================ state

    def on_enter(self) -> None:
        self.reset_test_speed()
        self.status.setStyleSheet("")
        self.status.setText("选择轮后拖动滑条；松开 250 ms 后写入")
        self.trace.clear()
        self.trace.set_connected(self.host.connected)
        self._refresh()

    def on_leave(self) -> None:
        self.reset_test_speed()
        self.status.setStyleSheet("")
        self.trace.clear()
        for timer in self._timers:
            timer.stop()

    def _refresh(self) -> None:
        if not self.isVisible():
            return

        motion = self.host.connected and self.host.enabled \
            and not self.host.estopped
        tuning = self.host.connected and not self.host.estopped

        for row in self.gain_rows:
            row.set_live_enabled(tuning)
        self.test_speed.set_live_enabled(motion)
        self.stop_btn.setEnabled(self.host.connected and not self.host.estopped)
        self.estop_btn.setEnabled(self.host.connected and not self.host.estopped)

        self.enable_btn.blockSignals(True)
        self.enable_btn.setChecked(self.host.enabled)
        self.enable_btn.blockSignals(False)
        self.enable_btn.setEnabled(self.host.connected and not self.host.estopped)

        if not self.host.enabled and self.test_speed.value() != 0:
            self.reset_test_speed()

        self.trace.set_connected(self.host.connected)

        now = time.monotonic()
        speeds = self.host.last_wheel_speed
        fresh = speeds is not None and now - self.host.wheel_at < 0.5
        keys = ("fl_mm_s", "fr_mm_s", "rl_mm_s", "rr_mm_s")
        for tile, key in zip(self.speed_tiles, keys):
            if fresh:
                tint = (CRITICAL if abs(speeds[key]) > TUNING_MAX_V + 50
                        else None)
                tile.set_value(str(speeds[key]), tint=tint)
            else:
                tile.set_value("—", tint=None)
