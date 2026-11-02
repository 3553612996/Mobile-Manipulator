#!/usr/bin/env python3
"""Host console for the Mobile-Manipulator serial protocol.

Layout: chassis controls on the left, live state in the middle, arm on the
right, communication log across the bottom. The console speaks docs/architecture/protocol.md
v1; a built-in simulator stands in for the MCU until the firmware implements
the protocol (M2.2/M2.3).

Safety rules implemented here, not only on the MCU:
  * SET_VELOCITY is sent every 20 ms; losing the link stops the stream.
  * ESTOP is latched locally and latched again after CLEAR_FAULT.
  * Commands that require an ACK are retried 3 times at 200 ms and surface a
    failure instead of pretending they worked.
"""

from __future__ import annotations

import datetime
import html
import struct
import time
from collections.abc import Callable
from dataclasses import dataclass

import serial
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QFont, QKeySequence, QShortcut
from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox, QFrame,
                               QGridLayout, QHBoxLayout, QLabel, QMainWindow,
                               QMessageBox, QPlainTextEdit, QPushButton,
                               QStackedWidget, QStatusBar, QVBoxLayout,
                               QWidget)

from . import protocol as p
from .calibration import CalibrationPage
from .protocol import Cmd, Report, Result, describe
from .simulator import SimulatorLink
from .storage import StoragePage
from .transport import BAUDRATES, DEFAULT_BAUD, SerialLink, available_ports
from .tuning import TUNING_MAX_V, TuningPage
from .widgets import (ACCENT, BASELINE, BORDER, CRITICAL, GOOD, INK, INK_2,
                      MUTED, PAGE, SERIES_LEFT, SERIES_RIGHT, SERIOUS,
                      SURFACE, SURFACE_2, SURFACE_3, WARNING, Card, Joystick, LinkBadge,
                      SliderRow, SpeedChart, StatTile)

TX_PERIOD_MS = 20
ACK_TIMEOUT_S = 0.200
ACK_RETRIES = 3
MAX_V = 500          # mm/s, slider range
MAX_W = 1500         # mrad/s, slider range
MAX_JOINT = 9000     # centi-degrees, +-90 deg
LOG_COLORS = {"tx": INK_2, "rx": MUTED, "ok": GOOD, "warn": WARNING,
              "error": CRITICAL, "info": INK}

APP_QSS = f"""
QWidget {{ color: {INK}; font-family: "Microsoft YaHei UI", "Segoe UI"; }}
QMainWindow, QWidget#root {{ background: {PAGE}; }}
QFrame#topbar {{ background: {SURFACE_2}; border-bottom: 1px solid {BORDER}; }}
QLabel#appTitle {{ font-size: 15px; font-weight: 700; }}
QLabel#appSub {{ color: {MUTED}; font-size: 9px; letter-spacing: 2px; }}
QLabel#muted, QLabel#hint {{ color: {MUTED}; font-size: 10px; }}
QFrame#card {{ background: {SURFACE}; border: 1px solid {BORDER};
               border-radius: 12px; }}
QLabel#cardTitle {{ font-size: 13px; font-weight: 700; }}
QLabel#cardSub {{ color: {MUTED}; font-size: 9px; }}
QLabel#legend {{ font-size: 10px; }}
QFrame#tile {{ background: {SURFACE_2}; border: 1px solid {SURFACE_3};
               border-radius: 9px; }}
QLabel#tileLabel {{ color: {MUTED}; font-size: 10px; }}
QLabel#tileValue {{ font-size: 19px; font-weight: 600; }}
QLabel#tileUnit {{ color: {MUTED}; font-size: 10px; padding-bottom: 3px; }}
QLabel#tileSub {{ color: #6f6f6a; font-size: 9px; }}
QLabel#fieldLabel {{ color: {INK_2}; font-size: 11px; }}
QLabel#fieldValue {{ font-family: "Consolas", monospace; font-size: 11px; }}
QLabel#fieldActual {{ color: {MUTED}; font-family: "Consolas", monospace;
                      font-size: 10px; }}
QPushButton {{ background: {SURFACE_3}; border: 1px solid #34342f;
               border-radius: 8px; padding: 5px 12px; font-size: 11px; }}
QPushButton:hover {{ background: #2d2d2b; }}
QPushButton:pressed {{ background: {BASELINE}; }}
QPushButton:disabled {{ color: #5c5c57; background: #1b1b1a;
                        border-color: #262624; }}
QPushButton[kind="primary"] {{ background: {ACCENT}; border: 1px solid {ACCENT};
                               color: #ffffff; font-weight: 600; }}
QPushButton[kind="primary"]:hover {{ background: #4a95e9; }}
QPushButton[kind="primary"]:disabled {{ background: #24384f; color: #7f8a96; }}
QPushButton[kind="danger"] {{ background: #b83333; border: 1px solid #d03b3b;
                              color: #ffffff; font-weight: 700; }}
QPushButton[kind="danger"]:hover {{ background: #d03b3b; }}
QPushButton[kind="danger"]:disabled {{ background: #3a2222; color: #8d6a6a;
                                       border-color: #4a2b2b; }}
QPushButton[kind="toggle"]:checked {{ background: #12321f;
                                      border: 1px solid {GOOD};
                                      color: #7ee27e; font-weight: 600; }}
QPushButton[kind="sim"]:checked {{ background: #14263c; border: 1px solid {ACCENT};
                                   color: #9fc7f5; }}
QPushButton[kind="ghost"] {{ background: transparent; border: 1px solid #34342f; }}
QComboBox {{ background: {SURFACE_3}; border: 1px solid #34342f;
             border-radius: 8px; padding: 3px 10px; min-height: 22px;
             font-size: 11px; }}
QComboBox:hover {{ border-color: #464641; }}
QComboBox::drop-down {{ border: none; width: 18px; }}
QComboBox QAbstractItemView {{ background: {SURFACE_3}; border: 1px solid #34342f;
                               selection-background-color: #2f3a48; }}
QSpinBox {{ background: {SURFACE_3}; border: 1px solid #34342f;
            border-radius: 8px; padding: 3px 6px; min-height: 22px;
            font-family: "Consolas", monospace; font-size: 11px; }}
QSpinBox:hover {{ border-color: #464641; }}
QSpinBox:disabled {{ color: #5c5c57; background: #1b1b1a;
                     border-color: #262624; }}
QSpinBox::up-button, QSpinBox::down-button {{ width: 14px; border: none;
                                              background: transparent; }}
QCheckBox {{ color: {INK_2}; font-size: 10px; spacing: 6px; }}
QCheckBox::indicator {{ width: 13px; height: 13px; border-radius: 4px;
                        border: 1px solid #4a4a45; background: #1b1b1a; }}
QCheckBox::indicator:checked {{ background: {ACCENT}; border-color: {ACCENT}; }}
QPlainTextEdit#log {{ background: {SURFACE_2}; border: 1px solid {SURFACE_3};
                      border-radius: 8px; font-family: "Consolas", monospace;
                      font-size: 10px; padding: 6px;
                      selection-background-color: #2f3a48; }}
QFrame#banner {{ background: rgba(208, 59, 59, 0.14);
                 border: 1px solid {CRITICAL}; border-radius: 10px; }}
QLabel#bannerText {{ color: #ff9d9d; font-size: 12px; font-weight: 600; }}
QStatusBar {{ background: {SURFACE_2}; border-top: 1px solid {BORDER};
              color: {MUTED}; font-size: 10px; }}
QStatusBar::item {{ border: none; }}
QToolTip {{ background: {SURFACE_3}; color: {INK}; border: 1px solid #34342f;
            padding: 4px; }}
QScrollBar:vertical {{ background: transparent; width: 8px; margin: 0; }}
QScrollBar::handle:vertical {{ background: #34342f; border-radius: 4px;
                               min-height: 20px; }}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0; }}
"""


@dataclass
class _Pending:
    raw: bytes
    label: str
    deadline: float
    retries: int
    on_ok: Callable[[], None] | None
    on_fail: Callable[[str], None] | None


def _button(text: str, kind: str | None = None, checkable: bool = False,
            height: int = 32) -> QPushButton:
    button = QPushButton(text)
    if kind:
        button.setProperty("kind", kind)
    button.setCheckable(checkable)
    button.setMinimumHeight(height)
    button.setCursor(Qt.PointingHandCursor)
    return button


def _cmd_name(cmd: int) -> str:
    try:
        return Cmd(cmd).name
    except ValueError:
        return f"0x{cmd:02X}"


class HostWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("移动抓取机器人 · 上位机")
        self.setMinimumSize(1280, 860)
        self.resize(1440, 900)

        # session state
        self.link: SerialLink | SimulatorLink | None = None
        self.connected = False
        self.sim_mode = False
        self.enabled = False
        self.estopped = False
        self.tuning_active = False
        self.calib_active = False
        self.storage_active = False
        self._closing = False
        self.v_target = 0
        self.w_target = 0
        self.seq = 0
        self.pending: dict[tuple[int, int], _Pending] = {}
        self.last_rx = 0.0
        self.connected_at = 0.0
        self.rx_since = 0
        self.tx_since = 0
        self.rx_fps = 0
        self.tx_fps = 0
        self._fps_at = time.monotonic()
        self._suppress_enable = False
        self._last_velocity_log: tuple[int, int] | None = None
        self._last_report_log = 0.0

        # latest telemetry
        self.last_odom = None
        self.last_arm = None
        self.last_status = None
        self.last_beat = None
        self.last_wheel_speed = None
        self.last_storage = None
        self.odom_at = 0.0
        self.arm_at = 0.0
        self.status_at = 0.0
        self.beat_at = 0.0
        self.wheel_at = 0.0

        self._build_ui()
        self._build_shortcuts()
        self._build_timers()

        self._refresh_ports()
        self._apply_enabled_state()
        self._log("上位机就绪 · 协议 v1 · 选择串口连接，或打开“模拟器”在 PC 上试运行",
                  "info")

    # ================================================================ UI

    def _build_ui(self) -> None:
        root = QWidget()
        root.setObjectName("root")
        self.setCentralWidget(root)

        outer = QVBoxLayout(root)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)
        outer.addWidget(self._build_topbar())

        self.pages = QStackedWidget()

        self.main_page = QWidget()
        body = QVBoxLayout(self.main_page)
        body.setContentsMargins(16, 12, 16, 12)
        body.setSpacing(10)

        self.banner = self._build_banner()
        body.addWidget(self.banner)
        self.banner.hide()

        columns = QHBoxLayout()
        columns.setSpacing(14)
        columns.addWidget(self._build_left(), 0)
        columns.addWidget(self._build_middle(), 1)
        columns.addWidget(self._build_right(), 0)
        body.addLayout(columns, 1)

        body.addWidget(self._build_log(), 0)
        self.pages.addWidget(self.main_page)

        self.tuning_page = TuningPage(self)
        self.tuning_page.backRequested.connect(self._close_tuning)
        self.pages.addWidget(self.tuning_page)

        self.calib_page = CalibrationPage(self)
        self.calib_page.backRequested.connect(self._close_calibration)
        self.pages.addWidget(self.calib_page)

        self.storage_page = StoragePage(self)
        self.storage_page.backRequested.connect(self._close_storage)
        self.pages.addWidget(self.storage_page)

        outer.addWidget(self.pages, 1)

        self._build_statusbar()
        self._apply_enabled_state()

    def _build_topbar(self) -> QWidget:
        bar = QFrame()
        bar.setObjectName("topbar")
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(18, 9, 18, 9)
        layout.setSpacing(10)

        title = QLabel("移动抓取机器人")
        title.setObjectName("appTitle")
        subtitle = QLabel("HOST CONSOLE")
        subtitle.setObjectName("appSub")
        layout.addWidget(title)
        layout.addWidget(subtitle)
        layout.addStretch(1)

        port_label = QLabel("串口")
        port_label.setObjectName("fieldLabel")
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(260)
        self.port_combo.setSizeAdjustPolicy(
            QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self.port_combo.setMinimumContentsLength(18)
        self.refresh_btn = _button("刷新", "ghost")
        self.refresh_btn.setFixedWidth(52)
        self.refresh_btn.clicked.connect(self._refresh_ports)

        baud_label = QLabel("波特率")
        baud_label.setObjectName("fieldLabel")
        self.baud_combo = QComboBox()
        for baud in BAUDRATES:
            self.baud_combo.addItem(str(baud), baud)
        self.baud_combo.setCurrentIndex(BAUDRATES.index(DEFAULT_BAUD))
        self.baud_combo.setFixedWidth(92)

        self.connect_btn = _button("连接", "primary")
        self.connect_btn.setFixedWidth(88)
        self.connect_btn.clicked.connect(self._toggle_connection)

        self.sim_btn = _button("模拟器", "sim", checkable=True)
        self.sim_btn.setFixedWidth(72)
        self.sim_btn.setToolTip("PC 端虚拟 MCU：没有硬件时也能完整走一遍协议")
        self.sim_btn.toggled.connect(self._on_sim_toggled)

        self.tuning_btn = _button("PID 调参", "ghost")
        self.tuning_btn.setFixedWidth(84)
        self.tuning_btn.setToolTip("四轮速度环增益与低速测试")
        self.tuning_btn.clicked.connect(self._open_tuning)

        self.calib_btn = _button("标定", "ghost")
        self.calib_btn.setFixedWidth(60)
        self.calib_btn.setToolTip("几何、编码器比例、起转占空比与轮方向")
        self.calib_btn.clicked.connect(self._open_calibration)

        self.storage_btn = _button("存储器", "ghost")
        self.storage_btn.setFixedWidth(68)
        self.storage_btn.setToolTip("存储状态、日志导出、崩溃现场读取")
        self.storage_btn.clicked.connect(self._open_storage)

        self.badge = LinkBadge()

        layout.addWidget(port_label)
        layout.addWidget(self.port_combo)
        layout.addWidget(self.refresh_btn)
        layout.addWidget(baud_label)
        layout.addWidget(self.baud_combo)
        layout.addWidget(self.connect_btn)
        layout.addSpacing(6)
        layout.addWidget(self.sim_btn)
        layout.addWidget(self.tuning_btn)
        layout.addWidget(self.calib_btn)
        layout.addWidget(self.storage_btn)
        layout.addSpacing(8)
        layout.addWidget(self.badge)
        return bar

    def _build_banner(self) -> QWidget:
        banner = QFrame()
        banner.setObjectName("banner")
        layout = QHBoxLayout(banner)
        layout.setContentsMargins(14, 8, 10, 8)
        layout.setSpacing(10)
        self.banner_label = QLabel("")
        self.banner_label.setObjectName("bannerText")
        layout.addWidget(self.banner_label)
        layout.addStretch(1)
        self.clear_btn = _button("解除急停 / 清故障", "ghost")
        self.clear_btn.clicked.connect(self._clear_fault)
        layout.addWidget(self.clear_btn)
        return banner

    def _build_left(self) -> QWidget:
        column = QWidget()
        column.setFixedWidth(352)
        layout = QVBoxLayout(column)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(14)

        card = Card("底盘运动", "摇杆松开自动回中")
        self.joystick = Joystick()
        self.joystick.moved.connect(self._on_joystick)
        card.body.addWidget(self.joystick, 0, Qt.AlignHCenter)

        self.v_row = SliderRow("线速度 v", -MAX_V, MAX_V, 0, 1, 0, " mm/s",
                               name_width=54)
        self.w_row = SliderRow("角速度 ω", -MAX_W, MAX_W, 0, 1, 0, " mrad/s",
                               name_width=54)
        self.v_row.valueChanged.connect(self._on_v_slider)
        self.w_row.valueChanged.connect(self._on_w_slider)
        card.body.addWidget(self.v_row)
        card.body.addWidget(self.w_row)

        hint = QLabel("50 Hz 下发；链路断流 200 ms 后 MCU 自动停车。"
                      "快捷键 F2 停车，F12 急停。")
        hint.setObjectName("hint")
        hint.setWordWrap(True)
        card.body.addWidget(hint)

        row = QHBoxLayout()
        row.setSpacing(8)
        self.enable_btn = _button("电机使能", "toggle", checkable=True, height=34)
        self.enable_btn.toggled.connect(self._on_enable_toggled)
        self.stop_btn = _button("停车", None, height=34)
        self.stop_btn.setToolTip("立即下发 v=0、ω=0（F2）")
        self.stop_btn.clicked.connect(self.stop_motion)
        row.addWidget(self.enable_btn, 1)
        row.addWidget(self.stop_btn, 1)
        card.body.addLayout(row)

        self.estop_btn = _button("急停  ESTOP", "danger", height=42)
        self.estop_btn.setToolTip("下发 ESTOP 并锁存，需解除后才能再次使能（F12）")
        self.estop_btn.clicked.connect(self.emergency_stop)
        card.body.addWidget(self.estop_btn)

        layout.addWidget(card, 1)
        return column

    def _build_middle(self) -> QWidget:
        column = QWidget()
        layout = QVBoxLayout(column)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(14)

        status_card = Card("整机状态", "REPORT_STATUS · 10 Hz")
        grid = QGridLayout()
        grid.setSpacing(8)
        self.tile_state = StatTile("状态机")
        self.tile_fault = StatTile("故障码")
        self.tile_batt = StatTile("电池电压", "V")
        self.tile_temp = StatTile("温度", "°C")
        self.tile_beat = StatTile("心跳计数")
        self.tile_jitter = StatTile("调度抖动", "µs")
        self.tile_fps = StatTile("接收帧率", "fps")
        self.tile_crc = StatTile("CRC 错误")
        self.tile_state.setToolTip(
            "REPORT_STATUS 的状态字节；协议 v1 尚未定义枚举，名称是暂定映射")
        for index, tile in enumerate((self.tile_state, self.tile_fault,
                                      self.tile_batt, self.tile_temp,
                                      self.tile_beat, self.tile_jitter,
                                      self.tile_fps, self.tile_crc)):
            grid.addWidget(tile, index // 4, index % 4)
            grid.setColumnStretch(index % 4, 1)
        status_card.body.addLayout(grid)
        layout.addWidget(status_card)

        odom_card = Card("里程计", "REPORT_ODOM · 50 Hz")
        odom_row = QHBoxLayout()
        odom_row.setSpacing(8)
        self.tile_x = StatTile("X", "m")
        self.tile_y = StatTile("Y", "m")
        self.tile_theta = StatTile("航向 θ", "°")
        for tile in (self.tile_x, self.tile_y, self.tile_theta):
            odom_row.addWidget(tile, 1)
        odom_card.body.addLayout(odom_row)
        layout.addWidget(odom_card)

        chart_card = Card("轮速", "mm/s · 最近 10 秒")
        self.legend_left = QLabel("")
        self.legend_right = QLabel("")
        self.legend_left.setObjectName("legend")
        self.legend_right.setObjectName("legend")
        chart_card.add_header_widget(self.legend_left)
        chart_card.add_header_widget(self.legend_right)
        self.chart = SpeedChart()
        chart_card.body.addWidget(self.chart, 1)
        layout.addWidget(chart_card, 1)
        return column

    def _build_right(self) -> QWidget:
        column = QWidget()
        column.setFixedWidth(356)
        layout = QVBoxLayout(column)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(14)

        card = Card("机械臂", "SET_ARM_JOINTS · 按需下发")
        self.joint_rows: list[SliderRow] = []
        for index in range(6):
            row = SliderRow(f"关节 {index + 1}", -MAX_JOINT, MAX_JOINT, 0,
                            100.0, 1, "°", actual=True, name_width=48)
            self.joint_rows.append(row)
            card.body.addWidget(row)

        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setStyleSheet(f"color: {BORDER}; background: {BORDER};"
                                " max-height: 1px; border: none;")
        card.body.addWidget(separator)

        self.gripper_row = SliderRow("夹持力", 0, 100, 60, 1, 0, " %",
                                     center_fill=False, actual=True,
                                     name_width=48)
        card.body.addWidget(self.gripper_row)

        grip_row = QHBoxLayout()
        grip_row.setSpacing(8)
        self.grip_open_btn = _button("张开夹爪", None, height=32)
        self.grip_close_btn = _button("闭合夹爪", None, height=32)
        self.grip_open_btn.clicked.connect(lambda: self._send_gripper(0))
        self.grip_close_btn.clicked.connect(lambda: self._send_gripper(1))
        grip_row.addWidget(self.grip_open_btn, 1)
        grip_row.addWidget(self.grip_close_btn, 1)
        card.body.addLayout(grip_row)

        # Keep the joint sliders dense at the top and anchor the primary action
        # to the bottom edge of the column, level with the chassis controls.
        card.body.addStretch(1)
        self.send_arm_btn = _button("下发关节角度", "primary", height=36)
        self.send_arm_btn.clicked.connect(self._send_arm)
        card.body.addWidget(self.send_arm_btn)

        layout.addWidget(card, 1)
        return column

    def _build_log(self) -> QWidget:
        card = Card("通信日志", "ACK · 事件 · 错误")
        card.setFixedHeight(190)
        self.show_reports = QCheckBox("显示周期上报")
        self.show_reports.setToolTip("节流到最多每秒 10 行，避免刷屏")
        clear = _button("清空", "ghost", height=24)
        clear.setFixedWidth(52)
        clear.clicked.connect(lambda: self.log.clear())
        card.add_header_widget(self.show_reports)
        card.add_header_widget(clear)

        self.log = QPlainTextEdit()
        self.log.setObjectName("log")
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(600)
        self.log.setMinimumHeight(130)
        card.body.addWidget(self.log, 1)
        return card

    def _build_statusbar(self) -> None:
        status = QStatusBar()
        status.setSizeGripEnabled(False)
        self.setStatusBar(status)
        self.status_left = QLabel("协议 v1 · 8N1 · 下行 50 Hz · ACK 200 ms ×3")
        self.status_right = QLabel("SEQ 000")
        self.status_left.setObjectName("muted")
        self.status_right.setObjectName("muted")
        status.addWidget(self.status_left)
        status.addPermanentWidget(self.status_right)

    def _build_shortcuts(self) -> None:
        stop = QShortcut(QKeySequence("F2"), self)
        stop.activated.connect(self.stop_motion)
        estop = QShortcut(QKeySequence("F12"), self)
        estop.activated.connect(self.emergency_stop)

    def _build_timers(self) -> None:
        self.tx_timer = QTimer(self)
        self.tx_timer.setInterval(TX_PERIOD_MS)
        self.tx_timer.timeout.connect(self._tx_tick)
        self.tx_timer.start()

        self.ack_timer = QTimer(self)
        self.ack_timer.setInterval(10)
        self.ack_timer.timeout.connect(self._ack_tick)
        self.ack_timer.start()

        self.watch_timer = QTimer(self)
        self.watch_timer.setInterval(200)
        self.watch_timer.timeout.connect(self._watch)
        self.watch_timer.start()

        self.ui_timer = QTimer(self)
        self.ui_timer.setInterval(100)
        self.ui_timer.timeout.connect(self._refresh_telemetry)
        self.ui_timer.start()

        self.ports_timer = QTimer(self)
        self.ports_timer.setInterval(3000)
        self.ports_timer.timeout.connect(self._auto_refresh_ports)
        self.ports_timer.start()

    # ======================================================== connection

    def _refresh_ports(self) -> None:
        current = self.port_combo.currentData()
        ports = available_ports()
        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        if not ports:
            self.port_combo.addItem("无可用串口", None)
        else:
            for device, description in ports:
                label = device
                if description and description not in ("n/a", "N/A"):
                    label = f"{device} · {description}"
                self.port_combo.addItem(label, device)
                self.port_combo.setItemData(self.port_combo.count() - 1,
                                            description, Qt.ToolTipRole)
            if current:
                index = self.port_combo.findData(current)
                if index >= 0:
                    self.port_combo.setCurrentIndex(index)
        self.port_combo.blockSignals(False)

    def _auto_refresh_ports(self) -> None:
        if not self.connected and not self.sim_mode:
            self._refresh_ports()

    def _toggle_connection(self) -> None:
        if self.connected:
            self._close_link()
        else:
            self._open_link()

    def _open_link(self) -> None:
        if self.sim_mode:
            link: SerialLink | SimulatorLink = SimulatorLink(self)
        else:
            port = self.port_combo.currentData()
            if not port:
                self._log("没有可用串口；可打开“模拟器”在 PC 上试运行", "warn")
                return
            link = SerialLink(self)

        link.connectedChanged.connect(self._on_link_state)
        link.frameReceived.connect(self._on_frame)
        link.errorOccurred.connect(self._on_link_error)
        self.link = link

        try:
            if self.sim_mode:
                link.open()
            else:
                link.open(port, int(self.baud_combo.currentData()))
        except (serial.SerialException, OSError) as exc:
            self.link = None
            link.deleteLater()
            self._log(f"打开串口失败：{exc}", "error")
            QMessageBox.warning(self, "串口打开失败", str(exc))
            return

        if self.sim_mode:
            self._log("模拟器已启动：PC 端虚拟 MCU（50 Hz 里程计，20 Hz 关节，"
                      "10 Hz 状态/心跳）", "info")

    def _close_link(self) -> None:
        link = self.link
        self.link = None
        if link is not None:
            self._closing = True
            link.close()
            self._closing = False
            link.deleteLater()
        self._on_link_state(False)

    def _cleanup_dead_link(self) -> None:
        link = self.link
        if link is not None and not self.connected:
            self.link = None
            self._closing = True
            link.close()
            self._closing = False
            link.deleteLater()

    def _on_link_state(self, connected: bool) -> None:
        if self._closing:
            return
        self.connected = connected
        self.connect_btn.setText("断开" if connected else
                                 ("启动模拟" if self.sim_mode else "连接"))
        self._set_port_controls_enabled(not connected and not self.sim_mode)
        self.sim_btn.setEnabled(not connected)
        self.chart.connected = connected
        self.chart.update()

        if connected:
            self.connected_at = time.monotonic()
            self.last_rx = 0.0
            self.seq = 0
            self.pending.clear()
            self._last_velocity_log = None
            self._log("链路已建立", "ok")
            self._apply_enabled_state()
        else:
            self._close_tuning()
            self._close_calibration()
            self._close_storage()
            self._cleanup_dead_link()
            self.connected = False
            self.storage_page.on_link_lost()
            self.pending.clear()
            self.enabled = False
            self.v_target = self.w_target = 0
            self._suppress_enable = True
            self.enable_btn.setChecked(False)
            self._suppress_enable = False
            self.v_row.setValue(0)
            self.w_row.setValue(0)
            self.joystick.reset()
            self._clear_telemetry()
            self._apply_enabled_state()
            if self.estopped:
                self._set_banner("链路已断开 · 急停状态未知，重新连接后请确认", True)
            else:
                self.banner.hide()
            self._log("链路已断开", "warn")
        self._update_badge()

    def _on_sim_toggled(self, checked: bool) -> None:
        if self.connected:
            self._close_link()
        self.sim_mode = checked
        self._set_port_controls_enabled(not checked)
        if not self.connected:
            self.connect_btn.setText("启动模拟" if checked else "连接")
        self._log("已切换到模拟器模式" if checked else "已切回串口模式", "info")

    def _set_port_controls_enabled(self, enabled: bool) -> None:
        self.port_combo.setEnabled(enabled)
        self.baud_combo.setEnabled(enabled)
        self.refresh_btn.setEnabled(enabled)

    def _on_link_error(self, message: str) -> None:
        self._log(message, "error")

    # ============================================================ sending

    def _send(self, cmd: int, payload: bytes = b"", acked: bool = False,
              on_ok: Callable[[], None] | None = None,
              on_fail: Callable[[str], None] | None = None,
              quiet: bool = False) -> int | None:
        if not self.connected or self.link is None:
            return None
        seq = self.seq
        self.seq = (self.seq + 1) & 0xFF
        raw = p.build_frame(cmd, seq, payload)
        self.link.send(raw)
        self.tx_since += 1

        if acked:
            label = _cmd_name(cmd)
            self.pending[(int(cmd), seq)] = _Pending(
                raw=raw, label=label,
                deadline=time.monotonic() + ACK_TIMEOUT_S,
                retries=0, on_ok=on_ok, on_fail=on_fail)
        if not quiet:
            self._log_tx(cmd, seq, payload)
        return seq

    def _tx_tick(self) -> None:
        if not self.connected or self.estopped:
            return
        v = self.v_target if self.enabled else 0
        w = self.w_target if self.enabled else 0
        if self.tuning_active:
            # The tuning page is only allowed to probe at low speed, no yaw.
            v = max(-TUNING_MAX_V, min(TUNING_MAX_V, v))
            w = 0
        self._send(Cmd.SET_VELOCITY, p.set_velocity_payload(v, w), quiet=True)
        if (v, w) != self._last_velocity_log:
            self._last_velocity_log = (v, w)
            self._log(f"→ SET_VELOCITY v={v} mm/s ω={w} mrad/s (周期下发)", "tx")

    def _log_tx(self, cmd: int, seq: int, payload: bytes) -> None:
        name = _cmd_name(cmd)
        detail = ""
        if cmd == Cmd.SET_ARM_JOINTS:
            angles = struct.unpack("<6h", payload)
            detail = " ".join(f"{a / 100:.1f}°" for a in angles)
        elif cmd == Cmd.SET_ENABLE:
            detail = "使能" if payload[0] else "失能"
        elif cmd == Cmd.SET_GRIPPER:
            detail = (f"{'闭合' if payload[0] else '张开'} "
                      f"力度 {payload[1]}%")
        elif cmd == Cmd.SET_PARAM:
            param_id, value = struct.unpack("<Bi", payload)
            detail = f"{p.PARAM_NAMES.get(param_id, hex(param_id))} = {value}"
        elif cmd == Cmd.CLEAR_FAULT:
            detail = f"0x{struct.unpack('<H', payload)[0]:04X}"
        suffix = f" {detail}" if detail else ""
        self._log(f"→ {name}{suffix} seq={seq}", "tx")

    def _ack_tick(self) -> None:
        if not self.connected or not self.pending:
            return
        now = time.monotonic()
        expired: list[tuple[int, int]] = []
        for key, pending in list(self.pending.items()):
            if now < pending.deadline:
                continue
            if pending.retries < ACK_RETRIES:
                pending.retries += 1
                pending.deadline = now + ACK_TIMEOUT_S
                if self.link is not None:
                    self.link.send(pending.raw)
                self._log(f"重发 {pending.label}（第 {pending.retries} 次）", "warn")
            else:
                expired.append(key)
        for key in expired:
            pending = self.pending.pop(key)
            self._log(f"{pending.label} 未收到应答，已重试 {ACK_RETRIES} 次",
                      "error")
            if pending.on_fail:
                pending.on_fail("无应答")

    # ========================================================== receiving

    def _on_frame(self, frame) -> None:
        now = time.monotonic()
        self.last_rx = now
        self.rx_since += 1

        if frame.is_ack:
            self._handle_ack(frame)
            return

        cmd = frame.cmd
        if cmd == Report.ODOM:
            data = p.decode_odom(frame.payload)
            if data:
                self.last_odom = data
                self.odom_at = now
                self.chart.add_sample(now, data["vl_mm_s"], data["vr_mm_s"])
        elif cmd == Report.ARM:
            data = p.decode_arm(frame.payload)
            if data:
                self.last_arm = data
                self.arm_at = now
        elif cmd == Report.STATUS:
            data = p.decode_status(frame.payload)
            if data:
                self.last_status = data
                self.status_at = now
        elif cmd == Report.EVENT:
            self._handle_event(frame)
        elif cmd == Report.HEARTBEAT:
            data = p.decode_heartbeat(frame.payload)
            if data:
                self.last_beat = data
                self.beat_at = now
        elif cmd == Report.WHEEL_SPEED:
            data = p.decode_wheel_speed(frame.payload)
            if data:
                self.last_wheel_speed = data
                self.wheel_at = now
                if self.tuning_active:
                    self.tuning_page.add_wheel_sample(data, now)
        elif cmd == Report.STORAGE:
            data = p.decode_storage(frame.payload)
            if data:
                self.last_storage = data
                self.storage_page.on_storage(data)
        elif cmd == Report.LOG:
            data = p.decode_log(frame.payload)
            if data:
                self.storage_page.on_log(data)
        elif cmd == Report.CRASH:
            data = p.decode_crash(frame.payload)
            if data:
                self.storage_page.on_crash(data)

        if self.show_reports.isChecked() and now - self._last_report_log >= 0.1:
            self._last_report_log = now
            self._log("← " + describe(frame), "rx")

    def _handle_ack(self, frame) -> None:
        cmd = frame.acked_cmd
        acked = frame.acked_seq
        result = frame.result
        if cmd is None or acked is None or result is None:
            self._log("← 短 ACK，已忽略", "warn")
            return

        name = _cmd_name(cmd)
        try:
            result_name = p.RESULT_NAMES.get(Result(result),
                                             f"0x{result:02X}")
        except ValueError:
            result_name = f"0x{result:02X}"

        pending = self.pending.pop((int(cmd), int(acked)), None)
        ok = result == Result.OK
        suffix = "" if pending else "（无待确认项）"
        self._log(f"← ACK {name} seq={acked} {result_name}{suffix}",
                  "ok" if ok else "error")

        if pending is None:
            return
        if ok and pending.on_ok:
            pending.on_ok()
        elif not ok and pending.on_fail:
            pending.on_fail(result_name)

    def _handle_event(self, frame) -> None:
        event = p.decode_event(frame.payload)
        if not event:
            return
        code = event["code"]
        name = p.FAULT_NAMES.get(code, f"0x{code:04X}")
        level = "error"
        if code in (0x0201, 0x0401, 0x0402):
            level = "warn"
        elif code in (0x0202, 0x0501):
            level = "error"
        self._log(f"← REPORT_EVENT {name} t={event['timestamp_ms']} ms", level)

        # Both FATAL codes put the firmware in ESTOP; mirror that locally so
        # the enable button cannot send a request the MCU will reject.
        if code in (0x0202, 0x0501):
            self._enter_estop(f"MCU 上报 {name}")

    # ============================================================== safety

    def _on_enable_toggled(self, checked: bool) -> None:
        if self._suppress_enable:
            return
        if not self.connected:
            self._suppress_enable = True
            self.enable_btn.setChecked(False)
            self._suppress_enable = False
            return
        self._send(
            Cmd.SET_ENABLE, p.set_enable_payload(checked), acked=True,
            on_ok=lambda: self._on_enable_ack(checked, True),
            on_fail=lambda reason: self._on_enable_ack(checked, False, reason))

    def _on_enable_ack(self, requested: bool, ok: bool,
                       reason: str = "") -> None:
        if not ok:
            self._log(f"使能请求被拒绝：{reason}", "error")
            self._suppress_enable = True
            self.enable_btn.setChecked(self.enabled)
            self._suppress_enable = False
            return
        self.enabled = requested
        self._log("电机已使能" if requested else "电机已失能", "ok")
        if not requested:
            self.v_target = self.w_target = 0
            self.v_row.setValue(0)
            self.w_row.setValue(0)
            self.joystick.reset()
        self._apply_enabled_state()

    def _apply_enabled_state(self) -> None:
        motion = self.connected and self.enabled and not self.estopped
        self.joystick.setEnabled(motion)
        self.v_row.set_live_enabled(motion)
        self.w_row.set_live_enabled(motion)
        for row in self.joint_rows:
            row.set_live_enabled(motion)
        self.gripper_row.set_live_enabled(motion)
        self.grip_open_btn.setEnabled(motion)
        self.grip_close_btn.setEnabled(motion)
        self.send_arm_btn.setEnabled(motion)
        self.stop_btn.setEnabled(self.connected and not self.estopped)
        self.estop_btn.setEnabled(self.connected and not self.estopped)
        self.enable_btn.setEnabled(self.connected and not self.estopped)
        self.clear_btn.setEnabled(self.connected)
        self.tuning_btn.setEnabled(self.connected)
        self.calib_btn.setEnabled(self.connected)
        self.storage_btn.setEnabled(self.connected)

    def _on_joystick(self, x: float, y: float) -> None:
        if not (self.connected and self.enabled and not self.estopped):
            return
        v = int(round(y * MAX_V / 5.0) * 5)
        w = int(round(-x * MAX_W / 10.0) * 10)
        self.v_target = max(-MAX_V, min(MAX_V, v))
        self.w_target = max(-MAX_W, min(MAX_W, w))
        self._set_velocity_sliders(self.v_target, self.w_target)

    def _on_v_slider(self, value: int) -> None:
        self.v_target = value

    def _on_w_slider(self, value: int) -> None:
        self.w_target = value

    def _set_velocity_sliders(self, v: int, w: int) -> None:
        for row, value in ((self.v_row, v), (self.w_row, w)):
            row.slider.blockSignals(True)
            row.slider.setValue(value)
            row.slider.blockSignals(False)
            row._update_text()

    def stop_motion(self) -> None:
        self.v_target = self.w_target = 0
        self._set_velocity_sliders(0, 0)
        self.joystick.reset()
        if hasattr(self, "tuning_page"):
            self.tuning_page.reset_test_speed()
        if self.connected and not self.estopped:
            self._send(Cmd.SET_VELOCITY, p.set_velocity_payload(0, 0),
                       quiet=True)
            self._log("停车：v=0、ω=0", "warn")

    def _open_tuning(self) -> None:
        if not self.connected or self.tuning_active:
            return
        self.stop_motion()
        self.tuning_active = True
        self.pages.setCurrentWidget(self.tuning_page)
        self.tuning_page.on_enter()
        self._log("进入 PID 调参：限速 %d mm/s，ω=0" % TUNING_MAX_V, "info")

    def _close_tuning(self) -> None:
        if not self.tuning_active:
            return
        self.stop_motion()
        self.tuning_active = False
        self.tuning_page.on_leave()
        self.pages.setCurrentWidget(self.main_page)
        self._log("离开 PID 调参，已停车", "info")

    def _open_calibration(self) -> None:
        if not self.connected or self.calib_active:
            return
        self.stop_motion()
        self._close_tuning()
        self.calib_active = True
        self.pages.setCurrentWidget(self.calib_page)
        self.calib_page.on_enter()
        self._log("进入标定页：标定区写入要求底盘失能", "info")

    def _close_calibration(self) -> None:
        if not self.calib_active:
            return
        self.calib_active = False
        self.calib_page.on_leave()
        self.pages.setCurrentWidget(self.main_page)
        self._log("离开标定页", "info")

    def _open_storage(self) -> None:
        if not self.connected or self.storage_active:
            return
        self.stop_motion()
        self._close_tuning()
        self.storage_active = True
        self.pages.setCurrentWidget(self.storage_page)
        self.storage_page.on_enter()
        self._log("进入存储器页：状态查询、日志导出、崩溃现场读取", "info")

    def _close_storage(self) -> None:
        if not self.storage_active:
            return
        self.storage_active = False
        self.storage_page.on_leave()
        self.pages.setCurrentWidget(self.main_page)
        self._log("离开存储器页", "info")

    def request_enable(self, checked: bool) -> None:
        """Enable/disable from the tuning page; the main button mirrors it."""
        if not self.connected:
            return
        self._suppress_enable = True
        self.enable_btn.setChecked(checked)
        self._suppress_enable = False
        self._on_enable_toggled(checked)

    def emergency_stop(self) -> None:
        if self.estopped:
            return
        self._enter_estop(None)

        if not self.connected:
            return
        self._send(Cmd.SET_VELOCITY, p.set_velocity_payload(0, 0), quiet=True)
        self._send(
            Cmd.ESTOP, b"", acked=True,
            on_ok=lambda: self._set_banner("急停已触发 · MCU 已确认", True),
            on_fail=lambda reason: self._set_banner(
                f"急停指令未确认（{reason}）· 请立即检查链路与电源", True))
        self._set_banner("急停已触发 · 等待 MCU 确认", True)
        self._log("已下发 ESTOP", "error")

    def _enter_estop(self, source: str | None) -> None:
        if self.tuning_active:
            self._close_tuning()

        already = self.estopped
        self.estopped = True
        self.enabled = False
        self.v_target = self.w_target = 0
        self._set_velocity_sliders(0, 0)
        self.joystick.reset()
        self._suppress_enable = True
        self.enable_btn.setChecked(False)
        self._suppress_enable = False
        self._apply_enabled_state()
        text = "急停已触发"
        if source:
            text += f" · {source}"
        elif not self.connected:
            text = "急停按钮已按下，但链路未连接 —— 无法保证小车停下"
        self._set_banner(text, True)
        if not already:
            self.banner.show()

    def _clear_fault(self) -> None:
        if not self.connected:
            return
        self._send(
            Cmd.CLEAR_FAULT, p.clear_fault_payload(0xFFFF), acked=True,
            on_ok=self._on_fault_cleared,
            on_fail=lambda reason: (
                self._log(f"清除故障失败：{reason}", "error"),
                self._set_banner(f"清除故障失败：{reason}", True)))

    def _on_fault_cleared(self) -> None:
        self.estopped = False
        self.banner.hide()
        self._apply_enabled_state()
        self._log("故障已清除，急停锁存解除；请重新使能", "ok")

    def _set_banner(self, text: str, urgent: bool) -> None:
        self.banner_label.setText(text)
        self.clear_btn.setEnabled(self.connected)

    # =============================================================== arm

    def _send_arm(self) -> None:
        angles = [row.value() for row in self.joint_rows]
        self._send(Cmd.SET_ARM_JOINTS, p.set_arm_joints_payload(angles))

    def _send_gripper(self, action: int) -> None:
        force = self.gripper_row.value()
        self._send(
            Cmd.SET_GRIPPER, p.set_gripper_payload(action, force), acked=True,
            on_fail=lambda reason: self._log(f"夹爪指令失败：{reason}",
                                             "error"))

    # ============================================================= update

    def _watch(self) -> None:
        now = time.monotonic()
        if now - self._fps_at >= 1.0:
            self.rx_fps = self.rx_since
            self.tx_fps = self.tx_since
            self.rx_since = 0
            self.tx_since = 0
            self._fps_at = now
        self._update_badge()
        self.status_right.setText(
            f"SEQ {self.seq:03d} · TX {self.tx_fps} fps · RX {self.rx_fps} fps")

    def _update_badge(self) -> None:
        if not self.connected:
            self.badge.set_state("未连接", MUTED)
            return
        now = time.monotonic()
        if self.last_rx == 0.0:
            if now - self.connected_at > 0.5:
                self.badge.set_state("心跳超时", CRITICAL)
            else:
                self.badge.set_state("等待心跳", WARNING)
            return
        age = now - self.last_rx
        if age > 0.5:
            self.badge.set_state("心跳超时", CRITICAL)
        elif age > 0.25:
            self.badge.set_state("心跳变慢", WARNING)
        else:
            self.badge.set_state("在线", GOOD)

    def _clear_telemetry(self) -> None:
        for tile in (self.tile_state, self.tile_fault, self.tile_batt,
                     self.tile_temp, self.tile_beat, self.tile_jitter,
                     self.tile_fps, self.tile_crc, self.tile_x, self.tile_y,
                     self.tile_theta):
            tile.set_value("—", sub="", tint=None)
        self.last_odom = self.last_arm = None
        self.last_status = self.last_beat = None
        self.last_wheel_speed = None
        self.last_storage = None
        self.odom_at = self.arm_at = self.status_at = self.beat_at = 0.0
        self.wheel_at = 0.0
        for row in self.joint_rows:
            row.set_actual("实际 —")
        self.gripper_row.set_actual("实际 —")
        self.chart.clear()
        self._set_legend(None, None)
        self.rx_fps = self.tx_fps = 0

    def _set_legend(self, left: int | None, right: int | None) -> None:
        def html_for(colour: str, name: str, value: int | None) -> str:
            number = "—" if value is None else str(value)
            return (f'<span style="color:{colour}; font-size:13px;">●</span>'
                    f'&nbsp;<span style="color:{MUTED}">{name}</span>&nbsp;'
                    f'<span style="color:{INK}; font-family:Consolas;">'
                    f'{number}</span>')

        self.legend_left.setText(html_for(SERIES_LEFT, "左轮", left))
        self.legend_right.setText(html_for(SERIES_RIGHT, "右轮", right))

    def _refresh_telemetry(self) -> None:
        now = time.monotonic()
        if not self.connected:
            return

        # -- status ---------------------------------------------------------
        status = self.last_status
        fresh = status is not None and now - self.status_at < 1.0
        if fresh:
            state = status["state"]
            state_name = p.STATE_NAMES.get(state, f"0x{state:02X}")
            tint = CRITICAL if state == 0x04 else (
                SERIOUS if state == 0x05 else None)
            self.tile_state.set_value(state_name, sub=f"0x{state:02X}",
                                      tint=tint)

            fault = status["fault"]
            if fault:
                self.tile_fault.set_value(
                    p.FAULT_NAMES.get(fault, "故障"),
                    sub=f"0x{fault:04X}", tint=CRITICAL)
            else:
                self.tile_fault.set_value("无", sub="", tint=None)

            battery = status["battery_mv"]
            tint = CRITICAL if battery < 10200 else (
                WARNING if battery < 10800 else None)
            self.tile_batt.set_value(f"{battery / 1000:.1f}",
                                     sub=f"{battery} mV", tint=tint)

            temp = status["temp_c"]
            tint = CRITICAL if temp >= 60 else (WARNING if temp >= 45 else None)
            self.tile_temp.set_value(str(temp), sub="", tint=tint)

            jitter = status["jitter_us"]
            self.tile_jitter.set_value(
                str(jitter), tint=WARNING if jitter > 500 else None)
        else:
            for tile in (self.tile_state, self.tile_fault, self.tile_batt,
                         self.tile_temp, self.tile_jitter):
                tile.set_value("—", sub="", tint=None)

        # -- heartbeat ------------------------------------------------------
        beat = self.last_beat
        if beat is not None and now - self.beat_at < 1.0:
            self.tile_beat.set_value(str(beat["counter"]),
                                     sub=f"0x{beat['state']:02X}")
        else:
            self.tile_beat.set_value("—", sub="")

        # -- link statistics -------------------------------------------------
        self.tile_fps.set_value(str(self.rx_fps))
        crc_errors = getattr(self.link, "crc_errors", 0) if self.link else 0
        self.tile_crc.set_value(
            str(crc_errors),
            tint=CRITICAL if crc_errors >= 10 else (
                WARNING if crc_errors else None))

        # -- odometry --------------------------------------------------------
        odom = self.last_odom
        if odom is not None and now - self.odom_at < 0.5:
            self.tile_x.set_value(f"{odom['x_mm'] / 1000:.2f}")
            self.tile_y.set_value(f"{odom['y_mm'] / 1000:.2f}")
            self.tile_theta.set_value(f"{odom['theta_cdeg'] / 100:.1f}")
            self._set_legend(odom["vl_mm_s"], odom["vr_mm_s"])
        else:
            for tile in (self.tile_x, self.tile_y, self.tile_theta):
                tile.set_value("—")
            self._set_legend(None, None)

        # -- arm feedback ----------------------------------------------------
        arm = self.last_arm
        if arm is not None and now - self.arm_at < 1.0:
            for row, angle in zip(self.joint_rows, arm["angles_cdeg"]):
                row.set_actual(f"实际 {angle / 100:.1f}°")
            self.gripper_row.set_actual(
                f"实际 {p.GRIPPER_STATE_NAMES.get(arm['gripper_state'], '—')}")
        else:
            for row in self.joint_rows:
                row.set_actual("实际 —")
            self.gripper_row.set_actual("实际 —")

    # ================================================================ log

    def _log(self, text: str, kind: str = "info") -> None:
        if not hasattr(self, "log"):
            return
        stamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        colour = LOG_COLORS.get(kind, INK)
        self.log.appendHtml(
            f'<span style="color:#5f5f5a;">{stamp}</span>&nbsp;&nbsp;'
            f'<span style="color:{colour};">{html.escape(text)}</span>')
        bar = self.log.verticalScrollBar()
        bar.setValue(bar.maximum())

    # =========================================================== shutdown

    def closeEvent(self, event):  # noqa: N802
        self.shutdown()
        super().closeEvent(event)

    def shutdown(self) -> None:
        for timer in (self.tx_timer, self.ack_timer, self.watch_timer,
                      self.ui_timer, self.ports_timer):
            timer.stop()
        link = self.link
        self.link = None
        if link is not None:
            self._closing = True
            link.close()
            self._closing = False
            link.deleteLater()

    # ============================================================= demo

    def start_simulator(self, connect: bool = True) -> None:
        if not self.sim_btn.isChecked():
            self.sim_btn.setChecked(True)
        if connect and not self.connected:
            self._open_link()

    def start_demo(self) -> None:
        self.start_simulator(True)
        QTimer.singleShot(300, self._demo_step)

    def _demo_step(self) -> None:
        if not self.connected:
            return
        self.enable_btn.setChecked(True)
        self.v_row.setValue(260)
        self.w_row.setValue(180)


def create_app(argv: list[str]) -> tuple[QApplication, HostWindow]:
    app = QApplication(argv)
    app.setApplicationName("Mobile-Manipulator Host Console")
    app.setStyle("Fusion")
    app.setFont(QFont("Microsoft YaHei UI", 10))
    app.setStyleSheet(APP_QSS)
    window = HostWindow()
    return app, window
