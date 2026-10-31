#!/usr/bin/env python3
"""Bus-servo page for the host console.

The six joints hang off one half-duplex UART5 bus and the MCU is its only
master (protocol.md 5.6); the host never talks to a servo directly:

  * REPORT_ARM_STATUS 0xCB at 2 Hz carries each joint's angle, temperature and
    fault bits plus the servo rail voltage. It is the only live source here;
    a joint that is still being scanned shows -128 degC ("not measured yet").
  * SERVO_CONFIG 0x0E changes one address or zero point at a time. It is
    synchronous: the comm task waits for the servo to answer, up to 1 s, so
    the console waits 1.2 s and never retransmits (app.ACK_POLICY).
  * SET_PARAM 0x60..0x64 sets the protection thresholds. They take effect on
    the next 10 ms control tick, but only PARAM_SAVE makes them survive a
    power cycle.

ID and zero-point changes live in the calibration region: RAM-only until
PARAM_SAVE scope=1, which -- like every calibration write -- requires a
disabled chassis. That is also why the page greys the 0x0E controls out while
the arm is enabled, and the firmware answers 0x05 if the host races it.
"""

from __future__ import annotations

import time

from PySide6.QtCore import QTimer, Qt, Signal
from PySide6.QtWidgets import (QComboBox, QGridLayout, QHBoxLayout, QLabel,
                               QMessageBox, QPushButton, QSpinBox,
                               QVBoxLayout, QWidget)

from . import protocol as p
from .protocol import Cmd
from .widgets import CRITICAL, GOOD, MUTED, WARNING, Card

# app_robot_config.h / protocol.md 5.4 defaults. Ranges are the firmware's
# APP_PARAMS_*_MIN/MAX: writing outside them returns 0x01.
DEFAULT_OVERTEMP_C = 65
DEFAULT_UNDERVOLT_MV = 5500
DEFAULT_OVERVOLT_MV = 8400
DEFAULT_STALL_ERROR = 300       # 0.01 deg, i.e. 3.00 deg
DEFAULT_STALL_MS = 600

STALE_S = 1.5                   # 0xCB is 2 Hz; three missed frames is stale


def _button(text: str, kind: str | None = None, height: int = 32) -> QPushButton:
    button = QPushButton(text)
    if kind:
        button.setProperty("kind", kind)
    button.setMinimumHeight(height)
    button.setCursor(Qt.PointingHandCursor)
    return button


class ServoPage(QWidget):
    backRequested = Signal()

    def __init__(self, host, parent=None):
        super().__init__(parent)
        self.host = host
        self._busy = False                    # one 0x0E in flight
        self._writing = False                 # threshold queue in progress
        self._threshold_queue: list[tuple[int, int, str]] = []
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
        title = QLabel("机械臂舵机")
        title.setObjectName("appTitle")
        subtitle = QLabel("0xCB 状态回读 · 0x0E 改 ID / 标定 · 失能后可配置")
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

        left = QVBoxLayout()
        left.setSpacing(12)
        left.addWidget(self._build_joint_card(), 3)
        left.addWidget(self._build_protection_card(), 2)
        body.addLayout(left, 1)
        body.addWidget(self._build_config_card(), 0)
        layout.addLayout(body, 1)

    def _spin(self, minimum: int, maximum: int, value: int,
              step: int = 1, width: int = 104) -> QSpinBox:
        box = QSpinBox()
        box.setRange(minimum, maximum)
        box.setSingleStep(step)
        box.setValue(value)
        box.setAlignment(Qt.AlignRight)
        box.setFixedWidth(width)
        box.setKeyboardTracking(False)
        return box

    def _field(self, label: str, widget: QWidget, unit: str = "",
               name_width: int = 96) -> QHBoxLayout:
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

    def _build_joint_card(self) -> QWidget:
        card = Card("关节状态", "REPORT_ARM_STATUS 0xCB · 2 Hz")

        grid = QGridLayout()
        grid.setSpacing(6)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 1)
        grid.setColumnStretch(3, 2)
        for column, name in enumerate(("关节", "实际角", "温度", "故障")):
            label = QLabel(name)
            label.setObjectName("fieldLabel")
            grid.addWidget(label, 0, column)

        self._angle_labels: list[QLabel] = []
        self._temp_labels: list[QLabel] = []
        self._fault_labels: list[QLabel] = []
        for joint, name in enumerate(p.SERVO_JOINT_NAMES):
            row = joint + 1
            # Six rows share the card's spare height so the table does not
            # collapse into the top third of an otherwise empty card.
            grid.setRowStretch(row, 1)
            tag = QLabel(name)
            tag.setObjectName("fieldLabel")
            grid.addWidget(tag, row, 0, Qt.AlignVCenter)

            angle = QLabel("—")
            angle.setObjectName("fieldValue")
            temp = QLabel("—")
            temp.setObjectName("fieldValue")
            fault = QLabel("—")
            fault.setObjectName("fieldValue")
            fault.setWordWrap(True)
            self._angle_labels.append(angle)
            self._temp_labels.append(temp)
            self._fault_labels.append(fault)
            grid.addWidget(angle, row, 1, Qt.AlignVCenter)
            grid.addWidget(temp, row, 2, Qt.AlignVCenter)
            grid.addWidget(fault, row, 3, Qt.AlignVCenter)

        card.body.addLayout(grid, 1)

        state_row = QHBoxLayout()
        state_row.setSpacing(12)
        self.rail_label = QLabel("舵机电源 —")
        self.rail_label.setObjectName("fieldValue")
        self.state_label = QLabel("等待 0xCB 上报")
        self.state_label.setObjectName("muted")
        state_row.addWidget(self.rail_label)
        state_row.addWidget(self.state_label)
        state_row.addStretch(1)
        card.body.addLayout(state_row)

        card.body.addWidget(self._hint(
            "温度 -128 表示上电后第一轮温度扫描还没轮到这颗舵机，不代表 0 °C。"
            "角度以 0.01° 回读，单关节约 1 s 刷新一次——相邻两帧的小幅变化不是抖动。"))
        return card

    def _build_protection_card(self) -> QWidget:
        card = Card("保护阈值", "SET_PARAM 0x60–0x64 · 运行中可写")

        grid = QGridLayout()
        grid.setSpacing(8)
        self.spin_overtemp = self._spin(-40, 150, DEFAULT_OVERTEMP_C)
        self.spin_undervolt = self._spin(4000, 8999, DEFAULT_UNDERVOLT_MV, 100)
        self.spin_overvolt = self._spin(4001, 9000, DEFAULT_OVERVOLT_MV, 100)
        self.spin_stall_error = self._spin(1, 4500, DEFAULT_STALL_ERROR, 10)
        self.spin_stall_ms = self._spin(100, 5000, DEFAULT_STALL_MS, 50)
        fields = (
            ("过温", self.spin_overtemp, "°C"),
            ("欠压", self.spin_undervolt, "mV"),
            ("过压", self.spin_overvolt, "mV"),
            ("堵转误差", self.spin_stall_error, "0.01°"),
            ("堵转时间", self.spin_stall_ms, "ms"),
        )
        for index, (label, spin, unit) in enumerate(fields):
            grid.addLayout(self._field(label, spin, unit, name_width=64),
                           index // 3, index % 3)

        self.stall_degrees = QLabel("")
        self.stall_degrees.setObjectName("muted")
        self.spin_stall_error.valueChanged.connect(self._update_stall_hint)
        self._update_stall_hint(self.spin_stall_error.value())
        grid.addWidget(self.stall_degrees, 1, 2)

        card.body.addLayout(grid)

        buttons = QHBoxLayout()
        buttons.setSpacing(8)
        self.write_btn = _button("写入阈值", "primary", height=34)
        self.write_btn.clicked.connect(self._on_write_thresholds)
        self.save_param_btn = _button("保存参数区（scope=0）", None, height=34)
        self.save_param_btn.clicked.connect(
            lambda: self._on_save(p.SCOPE_PARAM))
        buttons.addWidget(self.write_btn, 1)
        buttons.addWidget(self.save_param_btn, 1)
        card.body.addLayout(buttons)

        card.body.addWidget(self._hint(
            "阈值写入后下一个 10 ms 控制节拍生效；欠压必须小于过压，否则整条"
            "请求被拒（0x01）。落盘要单独点“保存参数区”，阈值本身不随标定区走。"))
        card.body.addStretch(1)
        return card

    def _build_config_card(self) -> QWidget:
        column = QWidget()
        column.setFixedWidth(360)
        layout = QVBoxLayout(column)
        layout.setContentsMargins(0, 0, 0, 0)

        card = Card("舵机配置", "0x0E SERVO_CONFIG · 要求机械臂失能")

        self.joint_combo = QComboBox()
        for joint, name in enumerate(p.SERVO_JOINT_NAMES):
            self.joint_combo.addItem(name, joint)
        self.op_combo = QComboBox()
        for op in (p.SERVO_OP_SET_ID, p.SERVO_OP_SET_BIAS,
                   p.SERVO_OP_CALIBRATE, p.SERVO_OP_CLEAR_CALIB,
                   p.SERVO_OP_TORQUE):
            self.op_combo.addItem(p.SERVO_OP_NAMES[op], op)
        self.op_combo.currentIndexChanged.connect(self._on_op_changed)

        self.value_spin = self._spin(0, 254, 1, width=120)
        card.body.addLayout(self._field("关节", self.joint_combo))
        card.body.addLayout(self._field("操作", self.op_combo))
        card.body.addLayout(self._field("数值", self.value_spin))

        self.value_hint = QLabel("")
        self.value_hint.setObjectName("hint")
        self.value_hint.setWordWrap(True)
        card.body.addWidget(self.value_hint)

        self.exec_btn = _button("执行配置", "primary", height=38)
        self.exec_btn.clicked.connect(self._on_execute)
        card.body.addWidget(self.exec_btn)

        self.save_calib_btn = _button("保存标定区（scope=1）", None, height=34)
        self.save_calib_btn.clicked.connect(
            lambda: self._on_save(p.SCOPE_CALIB))
        card.body.addWidget(self.save_calib_btn)

        card.body.addStretch(1)
        card.body.addWidget(self._hint(
            "改 ID 与标定零点都属于标定区：ACK 0x00 只表示 RAM 已改、映射已切换，"
            "断电保存还要点“保存标定区”。掉电前没保存，重启后该关节会离线。"))

        layout.addWidget(card, 1)
        self._on_op_changed()
        return column

    def _build_timer(self) -> None:
        self.refresh_timer = QTimer(self)
        self.refresh_timer.setInterval(200)
        self.refresh_timer.timeout.connect(self._refresh)
        self.refresh_timer.start()

    # ============================================================== config

    def _update_stall_hint(self, value: int) -> None:
        self.stall_degrees.setText(f"= {value / 100:.2f}°")

    def _on_op_changed(self) -> None:
        op = self.op_combo.currentData()
        if op == p.SERVO_OP_SET_ID:
            self.value_spin.setRange(0, 254)
            self.value_spin.setValue(1)
            self.value_spin.setEnabled(True)
            self.value_hint.setText(
                "新地址必须与其它关节不重复：两颗舵机同址时一条运动命令会"
                "同时驱动它们。成功后固件的地址映射立即切到新地址。")
        elif op == p.SERVO_OP_SET_BIAS:
            self.value_spin.setRange(-100, 100)
            self.value_spin.setValue(0)
            self.value_spin.setEnabled(True)
            self.value_hint.setText(
                "偏置保存在舵机内部，不属于本板的标定区，换板子也还在。")
        elif op == p.SERVO_OP_CALIBRATE:
            self.value_spin.setEnabled(False)
            self.value_hint.setText(
                "把当前实际角当作 0.00°。先失能、用手把关节推到机械零位，"
                "再执行；要求该关节在线且 3 s 内有过角度回读。")
        elif op == p.SERVO_OP_CLEAR_CALIB:
            self.value_spin.setEnabled(False)
            self.value_hint.setText(
                "零点偏移清零，回到舵机出厂零点。改完这一轴的实际角会跳变，"
                "执行前先确认机械臂周围没有干涉。")
        else:                                     # TORQUE
            self.value_spin.setRange(0, 1)
            self.value_spin.setValue(0)
            self.value_spin.setEnabled(True)
            self.value_hint.setText(
                "0 = 松轴（可用手扳动），1 = 上力。直接作用于舵机，不落盘；"
                "正常运动由 SET_ENABLE 统一管理，这里只用于单轴调试。")
        self._update_controls()

    def _on_execute(self) -> None:
        if not self.host.connected:
            self._set_status("链路未连接", CRITICAL)
            return
        if self.host.enabled:
            self._set_status("机械臂使能中，配置被拒；请先关闭电机使能", WARNING)
            return
        if self._busy or self._writing:
            return

        joint = int(self.joint_combo.currentData())
        op = int(self.op_combo.currentData())
        value = 0 if op in (p.SERVO_OP_CALIBRATE, p.SERVO_OP_CLEAR_CALIB) \
            else int(self.value_spin.value())

        text = self._confirmation(joint, op, value)
        if text:
            answer = QMessageBox.question(
                self, "确认舵机配置", text,
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if answer != QMessageBox.Yes:
                return

        self._busy = True
        self._update_controls()
        self._set_status(
            f"{p.SERVO_JOINT_NAMES[joint]} {p.SERVO_OP_NAMES[op]} 执行中…"
            "（最长 1 s）", WARNING)
        seq = self.host._send(
            Cmd.SERVO_CONFIG, p.servo_config_payload(joint, op, value),
            acked=True,
            on_ok=lambda: self._config_ok(joint, op),
            on_fail=lambda reason: self._config_failed(joint, op, reason))
        if seq is None:
            self._busy = False
            self._set_status("链路已断开，配置未发送", CRITICAL)
            self._update_controls()

    @staticmethod
    def _confirmation(joint: int, op: int, value: int) -> str:
        name = p.SERVO_JOINT_NAMES[joint]
        if op == p.SERVO_OP_SET_ID:
            return (f"把{name}的舵机地址改为 {value}？\n\n"
                    "地址必须唯一。改完只写 RAM，需再保存标定区才会在断电后保留。")
        if op == p.SERVO_OP_CALIBRATE:
            return (f"把{name}当前的实际角当作 0.00°？\n\n"
                    "关节应已手动推到机械零位，且机械臂处于失能状态。")
        if op == p.SERVO_OP_CLEAR_CALIB:
            return (f"清除{name}的零点偏移，回到舵机出厂零点？\n\n"
                    "该轴的实际角读数会立刻跳变。")
        return ""

    def _config_ok(self, joint: int, op: int) -> None:
        self._busy = False
        self._update_controls()
        name = p.SERVO_JOINT_NAMES[joint]
        if op == p.SERVO_OP_CALIBRATE:
            text = f"{name} 零点已标定；确认下一帧 0xCB 该关节读 0.00°"
            tail = "，并保存标定区"
        elif op in (p.SERVO_OP_SET_ID, p.SERVO_OP_CLEAR_CALIB):
            text = f"{name} {p.SERVO_OP_NAMES[op]} 已执行"
            tail = "；保存标定区后断电仍有效"
        else:
            text = f"{name} {p.SERVO_OP_NAMES[op]} 已执行"
            tail = ""
        self._set_status(text + tail, GOOD)
        self.host._log(f"舵机配置 {text}{tail}", "ok")

    def _config_failed(self, joint: int, op: int, reason: str) -> None:
        self._busy = False
        self._update_controls()
        name = p.SERVO_JOINT_NAMES[joint]
        # A lost ACK does not prove the servo did nothing: the MCU may have
        # executed the write and the reply died on the way. Never let the
        # operator blindly re-issue an ID change.
        hint = ("（ACK 丢失，配置可能已生效；改 ID 前先核对）"
                if reason == "无应答" else "")
        self._set_status(f"{name} {p.SERVO_OP_NAMES[op]} 失败：{reason}{hint}",
                         CRITICAL)
        self.host._log(f"舵机配置失败 {name} "
                       f"{p.SERVO_OP_NAMES[op]}: {reason}", "error")

    # ========================================================= protection

    def _on_write_thresholds(self) -> None:
        if not self.host.connected:
            self._set_status("链路未连接", CRITICAL)
            return
        if self._writing or self._busy:
            return
        if self.spin_undervolt.value() >= self.spin_overvolt.value():
            self._set_status("欠压阈值必须小于过压阈值，固件会整条拒绝", CRITICAL)
            return

        self._threshold_queue = [
            (p.ARM_PARAM_IDS["overtemp_c"], self.spin_overtemp.value(), "过温"),
            (p.ARM_PARAM_IDS["undervolt_mv"], self.spin_undervolt.value(),
             "欠压"),
            (p.ARM_PARAM_IDS["overvolt_mv"], self.spin_overvolt.value(), "过压"),
            (p.ARM_PARAM_IDS["stall_error_cdeg"],
             self.spin_stall_error.value(), "堵转误差"),
            (p.ARM_PARAM_IDS["stall_time_ms"], self.spin_stall_ms.value(),
             "堵转时间"),
        ]
        self._writing = True
        self._update_controls()
        self._set_status("写入保护阈值…", WARNING)
        self._send_next_threshold()

    def _send_next_threshold(self) -> None:
        if not self._threshold_queue:
            self._writing = False
            self._update_controls()
            self._set_status("阈值已写入 RAM，下个控制节拍生效；"
                             "断电保存请点“保存参数区”", GOOD)
            self.host._log("机械臂保护阈值已写入 RAM，等待 PARAM_SAVE", "warn")
            return
        param_id, value, label = self._threshold_queue.pop(0)
        seq = self.host._send(
            Cmd.SET_PARAM, p.set_param_payload(param_id, value), acked=True,
            on_ok=lambda name=label, val=value: self._threshold_acked(name, val),
            on_fail=lambda reason, name=label:
                self._threshold_failed(name, reason))
        if seq is None:
            self._writing = False
            self._threshold_queue.clear()
            self._set_status("链路已断开，阈值写入中止", CRITICAL)
            self._update_controls()

    def _threshold_acked(self, label: str, value: int) -> None:
        self.host._log(f"保护阈值 {label} = {value}", "ok")
        self._send_next_threshold()

    def _threshold_failed(self, label: str, reason: str) -> None:
        self._writing = False
        self._threshold_queue.clear()
        self._set_status(f"{label}阈值写入失败：{reason}", CRITICAL)
        self.host._log(f"保护阈值 {label} 写入失败：{reason}", "error")
        self._update_controls()

    # ============================================================= saving

    def _on_save(self, scope: int) -> None:
        if not self.host.connected:
            self._set_status("链路未连接", CRITICAL)
            return
        if scope == p.SCOPE_CALIB and self.host.enabled:
            self._set_status("保存标定区要求底盘失能", WARNING)
            return
        name = "标定区" if scope == p.SCOPE_CALIB else "参数区"
        self._set_status(f"已请求保存{name}…", WARNING)
        self.host._send(
            Cmd.PARAM_SAVE, p.param_save_payload(scope), acked=True,
            on_ok=lambda: self._save_accepted(name),
            on_fail=lambda reason: self._save_failed(name, reason))

    def _save_accepted(self, name: str) -> None:
        self._set_status(f"{name}保存已受理，等待 Flash 完成（看状态栏）", WARNING)

    def _save_failed(self, name: str, reason: str) -> None:
        self._set_status(f"{name}保存被拒绝：{reason}", CRITICAL)
        self.host._log(f"{name} PARAM_SAVE 失败：{reason}", "error")

    # ============================================================== state

    def _set_status(self, text: str, colour: str) -> None:
        self.status.setText(text)
        self.status.setStyleSheet(f"color: {colour};")

    def _update_controls(self) -> None:
        connected = self.host.connected
        # 0x0E blocks the comm task for up to 1 s, so never let it overlap a
        # threshold write: the queued SET_PARAM ACKs would expire behind it.
        can_config = (connected and not self.host.enabled
                      and not self._busy and not self._writing)
        self.joint_combo.setEnabled(can_config)
        self.op_combo.setEnabled(can_config)
        op = self.op_combo.currentData()
        value_needed = op not in (p.SERVO_OP_CALIBRATE,
                                  p.SERVO_OP_CLEAR_CALIB)
        self.value_spin.setEnabled(can_config and value_needed)
        self.exec_btn.setEnabled(can_config)
        self.save_calib_btn.setEnabled(can_config)
        self.write_btn.setEnabled(connected and not self._writing
                                  and not self._busy)
        self.save_param_btn.setEnabled(connected and not self._writing
                                       and not self._busy)

    def on_enter(self) -> None:
        self.status.setStyleSheet("")
        self.status.setText("配置要求机械臂失能；改 ID / 零点后记得保存标定区")
        self._update_controls()
        self._refresh()

    def on_leave(self) -> None:
        self.status.setStyleSheet("")
        self.status.setText("")

    def _refresh(self) -> None:
        if not self.isVisible():
            return
        self._update_controls()

        data = self.host.last_arm_status
        status_at = self.host.arm_status_at
        if data is None:
            self.rail_label.setText("舵机电源 —")
            self.state_label.setText("等待 0xCB 上报")
            self.state_label.setStyleSheet(f"color: {MUTED};")
            for joint in range(p.SERVO_JOINT_COUNT):
                self._paint_joint(joint, None, None, None)
            return

        age = time.monotonic() - status_at
        stale = age > STALE_S

        rail = data["rail_mv"]
        if rail <= 0:
            self.rail_label.setText("舵机电源 无读数")
            colour = MUTED
        else:
            colour = CRITICAL if (rail < self.spin_undervolt.value()
                                  or rail > self.spin_overvolt.value()) else None
            self.rail_label.setText(f"舵机电源 {rail / 1000:.2f} V")
        self.rail_label.setStyleSheet(
            f"color: {colour};" if colour else "")

        flags = []
        flags.append("使能" if data["enabled"] else "失能")
        if data["faulted"]:
            flags.append("故障锁存")
        if data["offline"]:
            flags.append("有关节离线")
        text = " · ".join(flags)
        if stale:
            text += f" · 0xCB 已 {age:.1f} s 未更新"
        self.state_label.setText(text)
        self.state_label.setStyleSheet(
            f"color: {CRITICAL if data['faulted'] or data['offline'] else MUTED};")

        limit = self.spin_overtemp.value()
        for joint, entry in enumerate(data["joints"]):
            self._paint_joint(joint, entry, limit, stale)

    def _paint_joint(self, joint: int, entry: dict | None, limit: int | None,
                     stale: bool | None) -> None:
        angle_label = self._angle_labels[joint]
        temp_label = self._temp_labels[joint]
        fault_label = self._fault_labels[joint]

        if entry is None:
            angle_label.setText("—")
            angle_label.setStyleSheet("")
            temp_label.setText("—")
            temp_label.setStyleSheet("")
            fault_label.setText("—")
            fault_label.setStyleSheet("")
            return

        angle_label.setText(f"{entry['angle_cdeg'] / 100:.2f}°")
        angle_label.setStyleSheet(
            f"color: {MUTED};" if stale else "")

        if entry["temp_valid"]:
            temp = entry["temperature_c"]
            temp_label.setText(f"{temp} °C")
            if limit is not None and temp >= limit:
                temp_label.setStyleSheet(f"color: {CRITICAL};")
            elif limit is not None and temp >= limit - 15:
                temp_label.setStyleSheet(f"color: {WARNING};")
            else:
                temp_label.setStyleSheet(f"color: {MUTED};" if stale else "")
        else:
            temp_label.setText("—")
            temp_label.setStyleSheet(f"color: {MUTED};")

        bits = entry["fault_bits"]
        fault_label.setText(entry["faults"])
        if bits:
            fault_label.setStyleSheet(f"color: {CRITICAL};")
        else:
            fault_label.setStyleSheet(f"color: {MUTED};" if stale else "")
