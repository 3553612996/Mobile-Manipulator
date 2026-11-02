#!/usr/bin/env python3
"""Custom widgets for the host console.

The palette follows the project's chart design system: a near-black page plane,
graphite cards, two categorical series (blue / orange) validated against the
dark surface, and the reserved status colours for good / warning / critical.
"""

from __future__ import annotations

import math
import time
from collections import deque

from PySide6.QtCore import QPointF, QRectF, QSize, Qt, QTimer, Signal
from PySide6.QtGui import (QColor, QFont, QFontMetricsF, QPainter, QPainterPath,
                           QPen, QPolygonF)
from PySide6.QtWidgets import (QFrame, QHBoxLayout, QLabel, QSizePolicy,
                               QSlider, QVBoxLayout, QWidget)

# ------------------------------------------------------------------ palette

PAGE = "#0d0d0d"
SURFACE = "#1a1a19"
SURFACE_2 = "#141413"
SURFACE_3 = "#232322"
BORDER = "#2c2c2a"
BASELINE = "#383835"
INK = "#f0efec"
INK_2 = "#c3c2b7"
MUTED = "#898781"
ACCENT = "#3987e5"
SERIES_LEFT = "#3987e5"
SERIES_RIGHT = "#d95926"
GOOD = "#0ca30c"
WARNING = "#fab219"
SERIOUS = "#ec835a"
CRITICAL = "#d03b3b"

UI_FONT = "Microsoft YaHei UI"
MONO_FONT = "Consolas"


def color(hex_string: str, alpha: float | None = None) -> QColor:
    c = QColor(hex_string)
    if alpha is not None:
        c.setAlphaF(alpha)
    return c


class Card(QFrame):
    """Rounded surface with a title row and a body layout."""

    def __init__(self, title: str, subtitle: str = "", parent=None):
        super().__init__(parent)
        self.setObjectName("card")
        outer = QVBoxLayout(self)
        outer.setContentsMargins(16, 13, 16, 15)
        outer.setSpacing(10)

        header = QHBoxLayout()
        header.setSpacing(10)
        self.title_label = QLabel(title)
        self.title_label.setObjectName("cardTitle")
        header.addWidget(self.title_label)
        if subtitle:
            self.subtitle_label = QLabel(subtitle)
            self.subtitle_label.setObjectName("cardSub")
            header.addWidget(self.subtitle_label)
        header.addStretch(1)
        header.setContentsMargins(0, 0, 0, 0)
        self.header = header
        # A bare QBoxLayout item may swallow the card's spare height when every
        # body child has a fixed height (the arm card): the title then floats
        # down the card. Pin the header to its size hint instead.
        header_holder = QWidget()
        header_holder.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        header_holder.setLayout(header)
        outer.addWidget(header_holder)

        self.body = QVBoxLayout()
        self.body.setSpacing(8)
        outer.addLayout(self.body, 1)

    def add_header_widget(self, widget: QWidget) -> None:
        self.header.addWidget(widget)


class StatTile(QFrame):
    """Label + large value + unit, with an optional sub-line."""

    def __init__(self, label: str, unit: str = "", parent=None):
        super().__init__(parent)
        self.setObjectName("tile")
        self.setFixedHeight(74)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 8, 12, 8)
        layout.setSpacing(1)

        self.label = QLabel(label)
        self.label.setObjectName("tileLabel")
        layout.addWidget(self.label)

        value_row = QHBoxLayout()
        value_row.setSpacing(4)
        self.value = QLabel("—")
        self.value.setObjectName("tileValue")
        self.unit = QLabel(unit)
        self.unit.setObjectName("tileUnit")
        value_row.addWidget(self.value, 0, Qt.AlignBottom)
        value_row.addWidget(self.unit, 0, Qt.AlignBottom)
        value_row.addStretch(1)
        layout.addLayout(value_row)

        self.sub = QLabel("")
        self.sub.setObjectName("tileSub")
        layout.addWidget(self.sub)

    def set_value(self, text: str, unit: str | None = None,
                  sub: str | None = None, tint: str | None = None) -> None:
        self.value.setText(text)
        if unit is not None:
            self.unit.setText(unit)
        if sub is not None:
            self.sub.setText(sub)
        self.value.setStyleSheet(f"color: {tint};" if tint else "")


class ValueSlider(QSlider):
    """A slider painted from scratch: the fill grows from zero, not from the
    left end, so a symmetric range reads as a signed value."""

    def __init__(self, minimum: int, maximum: int, center_fill: bool = True,
                 parent=None):
        super().__init__(Qt.Horizontal, parent)
        self.setRange(minimum, maximum)
        self.setSingleStep(max(1, (maximum - minimum) // 200))
        self.setPageStep(max(1, (maximum - minimum) // 10))
        self.setMinimumHeight(24)
        self.setCursor(Qt.PointingHandCursor)
        self._center_fill = center_fill

    # kept as a method so subclasses/tests can reason about the geometry
    def _track(self) -> tuple[float, float, float]:
        margin = 9.0
        return margin, self.width() - margin, self.height() / 2.0

    def _x_for(self, value: int) -> float:
        x0, x1, _ = self._track()
        span = max(1.0, x1 - x0)
        low, high = self.minimum(), self.maximum()
        frac = 0.0 if high == low else (value - low) / (high - low)
        return x0 + span * frac

    def _value_for(self, x: float) -> int:
        x0, x1, _ = self._track()
        span = max(1.0, x1 - x0)
        frac = max(0.0, min(1.0, (x - x0) / span))
        low, high = self.minimum(), self.maximum()
        return int(round(low + frac * (high - low)))

    def paintEvent(self, event):  # noqa: N802 (Qt naming)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        enabled = self.isEnabled()
        _, _, cy = self._track()
        x0, x1, _ = self._track()

        groove = QPen(color(BASELINE if enabled else "#262624"), 4.0)
        groove.setCapStyle(Qt.RoundCap)
        painter.setPen(groove)
        painter.drawLine(QPointF(x0, cy), QPointF(x1, cy))

        if self._center_fill:
            center = self._x_for(0) if self.minimum() < 0 < self.maximum() \
                else self._x_for(self.minimum())
            tick = QPen(color(BASELINE if enabled else "#262624"), 1.0)
            painter.setPen(tick)
            painter.drawLine(QPointF(center, cy - 5), QPointF(center, cy + 5))
        else:
            center = x0

        xv = self._x_for(self.value())
        if abs(xv - center) > 0.5:
            fill = QPen(color(ACCENT if enabled else "#3a4a5c"), 4.0)
            fill.setCapStyle(Qt.RoundCap)
            painter.setPen(fill)
            painter.drawLine(QPointF(center, cy), QPointF(xv, cy))

        painter.setPen(QPen(color(PAGE), 2.0))
        painter.setBrush(color(INK if enabled else "#6b6b66"))
        painter.drawEllipse(QPointF(xv, cy), 6.5, 6.5)

    # -- interactions -----------------------------------------------------

    def mousePressEvent(self, event):  # noqa: N802
        if event.button() == Qt.LeftButton and self.isEnabled():
            self.setSliderDown(True)
            self.setValue(self._value_for(event.position().x()))
            self.sliderPressed.emit()
            event.accept()
        else:
            super().mousePressEvent(event)

    def mouseMoveEvent(self, event):  # noqa: N802
        if self.isSliderDown():
            self.setValue(self._value_for(event.position().x()))
            event.accept()
        else:
            super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):  # noqa: N802
        if event.button() == Qt.LeftButton and self.isSliderDown():
            self.setSliderDown(False)
            self.sliderReleased.emit()
            event.accept()
        else:
            super().mouseReleaseEvent(event)


class SliderRow(QWidget):
    """Name + live target value + slider, with an optional measured value."""

    valueChanged = Signal(int)

    def __init__(self, label: str, minimum: int, maximum: int, value: int = 0,
                 scale: float = 1.0, decimals: int = 0, suffix: str = "",
                 center_fill: bool = True, actual: bool = False,
                 name_width: int = 44, parent=None):
        super().__init__(parent)
        self._scale = scale
        self._decimals = decimals
        self._suffix = suffix
        self.setFixedHeight(40)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(1)

        top = QHBoxLayout()
        top.setSpacing(6)
        name = QLabel(label)
        name.setObjectName("fieldLabel")
        name.setFixedWidth(name_width)
        top.addWidget(name)
        top.addStretch(1)

        self.actual_label = QLabel("")
        self.actual_label.setObjectName("fieldActual")
        self.actual_label.setVisible(actual)
        top.addWidget(self.actual_label)

        self.value_label = QLabel("")
        self.value_label.setObjectName("fieldValue")
        self.value_label.setMinimumWidth(74)
        self.value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        top.addWidget(self.value_label)
        layout.addLayout(top)

        self.slider = ValueSlider(minimum, maximum, center_fill)
        self.slider.setValue(value)
        layout.addWidget(self.slider)

        self.slider.valueChanged.connect(self._on_value)
        self._update_text()

    def _format(self, value: int) -> str:
        text = f"{value / self._scale:.{self._decimals}f}"
        return f"{text}{self._suffix}" if self._suffix else text

    def _on_value(self, value: int) -> None:
        self._update_text()
        self.valueChanged.emit(value)

    def _update_text(self) -> None:
        self.value_label.setText(self._format(self.slider.value()))

    def value(self) -> int:
        return self.slider.value()

    def setValue(self, value: int) -> None:  # noqa: N802 (matches Qt)
        self.slider.setValue(int(value))

    def set_actual(self, text: str) -> None:
        self.actual_label.setText(text)

    def set_live_enabled(self, enabled: bool) -> None:
        self.slider.setEnabled(enabled)


class Joystick(QWidget):
    """Spring-loaded pad: x -> yaw, y -> forward speed."""

    moved = Signal(float, float)   # x right-positive, y up-positive, -1..1

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(184, 184)
        self.setCursor(Qt.PointingHandCursor)
        self._offset = QPointF(0.0, 0.0)
        self._dragging = False

    def _geometry(self) -> tuple[QPointF, float]:
        center = QPointF(self.width() / 2.0, self.height() / 2.0)
        radius = min(self.width(), self.height()) / 2.0 - 20.0
        return center, radius

    def _emit_offset(self) -> None:
        center, radius = self._geometry()
        self.moved.emit(self._offset.x() / radius, -self._offset.y() / radius)

    def _set_from_pos(self, pos: QPointF) -> None:
        center, radius = self._geometry()
        delta = pos - center
        distance = math.hypot(delta.x(), delta.y())
        if distance > radius:
            delta = delta * (radius / distance)
        self._offset = delta
        self.update()
        self._emit_offset()

    def paintEvent(self, event):  # noqa: N802
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        rect = QRectF(2, 2, self.width() - 4, self.height() - 4)
        path = QPainterPath()
        path.addRoundedRect(rect, 16, 16)
        painter.fillPath(path, color(SURFACE_2))
        painter.setPen(QPen(color(BASELINE), 1.0))
        painter.drawPath(path)

        center, radius = self._geometry()
        painter.setPen(QPen(color(BORDER), 1.0))
        for factor in (1.0 / 3.0, 2.0 / 3.0, 1.0):
            painter.drawEllipse(center, radius * factor, radius * factor)
        painter.setPen(QPen(color(BORDER), 1.0))
        painter.drawLine(QPointF(center.x() - radius, center.y()),
                         QPointF(center.x() + radius, center.y()))
        painter.drawLine(QPointF(center.x(), center.y() - radius),
                         QPointF(center.x(), center.y() + radius))

        label_font = QFont(UI_FONT, 8)
        painter.setFont(label_font)
        painter.setPen(color(MUTED))
        painter.drawText(QRectF(center.x() - 30, 6, 60, 14),
                         Qt.AlignCenter, "前")
        painter.drawText(QRectF(center.x() - 30, self.height() - 20, 60, 14),
                         Qt.AlignCenter, "后")
        painter.drawText(QRectF(4, center.y() - 8, 16, 16),
                         Qt.AlignCenter, "左")
        painter.drawText(QRectF(self.width() - 20, center.y() - 8, 16, 16),
                         Qt.AlignCenter, "右")

        knob = center + self._offset
        if self._dragging:
            line = QPen(color(ACCENT, 0.45), 2.0)
            line.setCapStyle(Qt.RoundCap)
            painter.setPen(line)
            painter.drawLine(center, knob)

        painter.setPen(QPen(color(SURFACE_2), 2.0))
        painter.setBrush(color(ACCENT if self.isEnabled() else "#3a4a5c"))
        painter.drawEllipse(knob, 15.0, 15.0)
        painter.setPen(Qt.NoPen)
        painter.setBrush(color("#ffffff", 0.85))
        painter.drawEllipse(knob, 4.0, 4.0)

    def mousePressEvent(self, event):  # noqa: N802
        if event.button() == Qt.LeftButton and self.isEnabled():
            self._dragging = True
            self._set_from_pos(event.position())
            event.accept()

    def mouseMoveEvent(self, event):  # noqa: N802
        if self._dragging:
            self._set_from_pos(event.position())
            event.accept()

    def mouseReleaseEvent(self, event):  # noqa: N802
        if self._dragging and event.button() == Qt.LeftButton:
            self._dragging = False
            self._offset = QPointF(0.0, 0.0)
            self.update()
            self.moved.emit(0.0, 0.0)
            event.accept()

    def reset(self) -> None:
        if self._offset != QPointF(0.0, 0.0):
            self._offset = QPointF(0.0, 0.0)
            self.update()

    def setEnabled(self, enabled: bool) -> None:  # noqa: N802
        super().setEnabled(enabled)
        if not enabled:
            self.reset()
        self.update()


class SpeedChart(QWidget):
    """Live line chart for the two wheel speeds, last 10 seconds."""

    WINDOW_S = 10.0
    TICK_S = 2.0

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(170)
        self.setMouseTracking(True)
        self._samples: deque[tuple[float, float, float]] = deque(maxlen=800)
        self._hover: QPointF | None = None
        self._dirty = False
        self._timer = QTimer(self)
        self._timer.setInterval(33)
        self._timer.timeout.connect(self._tick)
        self._timer.start()
        self.connected = False

    # -- data -------------------------------------------------------------

    def add_sample(self, t: float, left: float, right: float) -> None:
        self._samples.append((t, left, right))
        cutoff = t - self.WINDOW_S - 1.0
        while self._samples and self._samples[0][0] < cutoff:
            self._samples.popleft()
        self._dirty = True

    def clear(self) -> None:
        self._samples.clear()
        self._dirty = True
        self.update()

    def _tick(self) -> None:
        if self._dirty:
            self._dirty = False
            self.update()

    # -- geometry ---------------------------------------------------------

    def _plot_rect(self) -> QRectF:
        return QRectF(46, 10, max(10.0, self.width() - 46 - 14),
                      max(10.0, self.height() - 10 - 20))

    def _range(self) -> float:
        peak = 0.0
        for _, left, right in self._samples:
            peak = max(peak, abs(left), abs(right))
        return max(200.0, math.ceil(peak * 1.15 / 100.0) * 100.0)

    def _x_for(self, t: float, now: float, rect: QRectF) -> float:
        return rect.right() - (now - t) / self.WINDOW_S * rect.width()

    def _y_for(self, value: float, limit: float, rect: QRectF) -> float:
        return rect.center().y() - value / limit * (rect.height() / 2.0)

    def mouseMoveEvent(self, event):  # noqa: N802
        rect = self._plot_rect()
        pos = event.position()
        self._hover = pos if rect.contains(pos) else None
        self.update()

    def leaveEvent(self, event):  # noqa: N802
        self._hover = None
        self.update()

    # -- painting ---------------------------------------------------------

    def paintEvent(self, event):  # noqa: N802
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        rect = self._plot_rect()
        now = time.monotonic()
        limit = self._range()

        tick_font = QFont(MONO_FONT, 8)
        painter.setFont(tick_font)
        metrics = QFontMetricsF(tick_font)

        # Grid and y labels.
        for step in (-2, -1, 0, 1, 2):
            value = limit * step / 2.0
            y = self._y_for(value, limit, rect)
            painter.setPen(QPen(color(BASELINE if step == 0 else BORDER), 1.0))
            painter.drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y))
            painter.setPen(color(MUTED))
            text = f"{value:.0f}"
            painter.drawText(QRectF(0, y - 7, rect.left() - 8, 14),
                             Qt.AlignRight | Qt.AlignVCenter, text)

        # Time labels.
        for offset in range(0, int(self.WINDOW_S) + 1, int(self.TICK_S)):
            x = rect.right() - offset / self.WINDOW_S * rect.width()
            painter.setPen(QPen(color(BORDER), 1.0))
            painter.drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()))
            painter.setPen(color(MUTED))
            text = "现在" if offset == 0 else f"-{offset}s"
            width = metrics.horizontalAdvance(text) + 6
            align = Qt.AlignRight if offset == 0 else Qt.AlignHCenter
            box = QRectF(x - width / 2, rect.bottom() + 3, width, 14)
            if offset == 0:
                box = QRectF(x - width, rect.bottom() + 3, width, 14)
                align = Qt.AlignRight
            painter.drawText(box, align | Qt.AlignVCenter, text)

        if not self._samples:
            painter.setFont(QFont(UI_FONT, 9))
            painter.setPen(color(MUTED))
            painter.drawText(rect, Qt.AlignCenter,
                             "等待里程计上报…" if self.connected else "未连接")
            return

        # Series.
        left_points: list[QPointF] = []
        right_points: list[QPointF] = []
        for t, left, right in self._samples:
            if t < now - self.WINDOW_S:
                continue
            x = self._x_for(t, now, rect)
            left_points.append(QPointF(x, self._y_for(left, limit, rect)))
            right_points.append(QPointF(x, self._y_for(right, limit, rect)))

        painter.save()
        painter.setClipRect(rect.adjusted(-1, -1, 1, 1))
        for points, colour in ((left_points, SERIES_LEFT),
                               (right_points, SERIES_RIGHT)):
            if len(points) < 2:
                continue
            pen = QPen(color(colour), 2.0)
            pen.setCapStyle(Qt.RoundCap)
            pen.setJoinStyle(Qt.RoundJoin)
            painter.setPen(pen)
            painter.drawPolyline(QPolygonF(points))
        painter.restore()

        # End markers carry a surface ring so the two stay legible when they
        # cross; the inset keeps the ring inside the clip.
        last = self._samples[-1]
        for value, colour in ((last[1], SERIES_LEFT), (last[2], SERIES_RIGHT)):
            x = min(rect.right() - 1.0, self._x_for(last[0], now, rect))
            y = self._y_for(value, limit, rect)
            painter.setPen(QPen(color(SURFACE), 2.0))
            painter.setBrush(color(colour))
            painter.drawEllipse(QPointF(x, y), 4.0, 4.0)

        self._paint_hover(painter, rect, now, limit)

    def _paint_hover(self, painter: QPainter, rect: QRectF, now: float,
                     limit: float) -> None:
        if self._hover is None or not self._samples:
            return
        # Nearest sample to the cursor by x, within the window.
        target = self._hover.x()
        best = None
        best_dx = 1e9
        for sample in self._samples:
            if sample[0] < now - self.WINDOW_S:
                continue
            dx = abs(self._x_for(sample[0], now, rect) - target)
            if dx < best_dx:
                best, best_dx = sample, dx
        if best is None or best_dx > 18:
            return

        t, left, right = best
        x = self._x_for(t, now, rect)
        if x < rect.left() or x > rect.right():
            return

        painter.setPen(QPen(color(INK_2, 0.28), 1.0))
        painter.drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()))

        for value, colour in ((left, SERIES_LEFT), (right, SERIES_RIGHT)):
            y = self._y_for(value, limit, rect)
            painter.setPen(QPen(color(SURFACE), 2.0))
            painter.setBrush(color(colour))
            painter.drawEllipse(QPointF(x, y), 4.0, 4.0)

        text = (f"-{now - t:.1f}s   左 {left:>4.0f}   右 {right:>4.0f} mm/s")
        font = QFont(MONO_FONT, 8)
        painter.setFont(font)
        metrics = QFontMetricsF(font)
        width = metrics.horizontalAdvance(text) + 16
        height = 22.0
        box_x = x + 10
        if box_x + width > rect.right():
            box_x = x - 10 - width
        box_y = max(rect.top(), min(self._hover.y() - height - 8,
                                    rect.bottom() - height))
        box = QRectF(box_x, box_y, width, height)
        path = QPainterPath()
        path.addRoundedRect(box, 6, 6)
        painter.fillPath(path, color(SURFACE_3))
        # drawPath strokes *and* fills; drop the marker brush first.
        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(color(BASELINE), 1.0))
        painter.drawPath(path)
        painter.setPen(color(INK_2))
        painter.drawText(box, Qt.AlignCenter, text)


class WheelTraceChart(QWidget):
    """Wheel speed against the commanded target, last 6 seconds.

    One small multiple per wheel instead of four lines in one plot: the
    categorical palette only validates three slots for all-pairs separation,
    and four wheel traces cross each other constantly. Each panel carries one
    series plus the shared dashed target, so wheel identity comes from the
    panel title rather than from hue.
    """

    WINDOW_S = 6.0
    MIN_LIMIT = 50.0

    def __init__(self, names, parent=None):
        super().__init__(parent)
        self._names = tuple(names)
        self.setMinimumHeight(150)
        self.setMouseTracking(True)
        self._samples: deque[tuple[float, float, float, float, float, float]] = \
            deque(maxlen=1200)
        self._hover: QPointF | None = None
        self._dirty = False
        self._timer = QTimer(self)
        self._timer.setInterval(33)
        self._timer.timeout.connect(self._tick)
        self._timer.start()
        self.connected = False

    # -- data -------------------------------------------------------------

    def add_sample(self, t: float, speeds, target: float) -> None:
        self._samples.append((t, float(speeds[0]), float(speeds[1]),
                              float(speeds[2]), float(speeds[3]),
                              float(target)))
        cutoff = t - self.WINDOW_S - 1.0
        while self._samples and self._samples[0][0] < cutoff:
            self._samples.popleft()
        self._dirty = True

    def clear(self) -> None:
        self._samples.clear()
        self._dirty = True
        self.update()

    def set_connected(self, connected: bool) -> None:
        if self.connected != connected:
            self.connected = connected
            self.update()

    def _tick(self) -> None:
        if self._dirty:
            self._dirty = False
            self.update()

    # -- geometry ---------------------------------------------------------

    def _plot_rect(self) -> QRectF:
        return QRectF(40, 24, max(10.0, self.width() - 40 - 8),
                      max(10.0, self.height() - 24 - 20))

    def _panels(self) -> list[QRectF]:
        rect = self._plot_rect()
        gap = 10.0
        width = max(10.0, (rect.width() - gap * 3) / 4.0)
        return [QRectF(rect.left() + index * (width + gap), rect.top(),
                       width, rect.height()) for index in range(4)]

    def _range(self) -> float:
        peak = 0.0
        for sample in self._samples:
            for value in sample[1:]:
                peak = max(peak, abs(value))
        return max(self.MIN_LIMIT, math.ceil(peak * 1.15 / 50.0) * 50.0)

    def _x_for(self, t: float, now: float, rect: QRectF) -> float:
        return rect.right() - (now - t) / self.WINDOW_S * rect.width()

    def _y_for(self, value: float, limit: float, rect: QRectF) -> float:
        return rect.center().y() - value / limit * (rect.height() / 2.0)

    def _panel_at(self, x: float) -> int | None:
        for index, rect in enumerate(self._panels()):
            if rect.left() <= x <= rect.right():
                return index
        return None

    def mouseMoveEvent(self, event):  # noqa: N802
        self._hover = event.position() if self._panel_at(
            event.position().x()) is not None else None
        self.update()

    def leaveEvent(self, event):  # noqa: N802
        self._hover = None
        self.update()

    # -- painting ---------------------------------------------------------

    def paintEvent(self, event):  # noqa: N802
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        panels = self._panels()
        now = time.monotonic()
        limit = self._range()

        tick_font = QFont(MONO_FONT, 8)
        painter.setFont(tick_font)

        # Shared y grid and labels: four panels on one scale stay comparable.
        for step in (-2, -1, 0, 1, 2):
            value = limit * step / 2.0
            y = self._y_for(value, limit, panels[0])
            painter.setPen(QPen(color(BASELINE if step == 0 else BORDER), 1.0))
            for rect in panels:
                painter.drawLine(QPointF(rect.left(), y),
                                 QPointF(rect.right(), y))
            painter.setPen(color(MUTED))
            painter.drawText(QRectF(0, y - 7, panels[0].left() - 6, 14),
                             Qt.AlignRight | Qt.AlignVCenter, f"{value:.0f}")

        painter.setFont(QFont(UI_FONT, 9))
        painter.setPen(color(INK_2))
        for rect, name in zip(panels, self._names):
            painter.drawText(QRectF(rect.left(), 2, rect.width(), 18),
                             Qt.AlignLeft | Qt.AlignVCenter, name)

        painter.setFont(tick_font)
        painter.setPen(color(MUTED))
        painter.drawText(QRectF(panels[0].left(), panels[0].bottom() + 3,
                                90, 14),
                         Qt.AlignLeft | Qt.AlignVCenter,
                         f"-{int(self.WINDOW_S)}s")
        painter.drawText(QRectF(panels[-1].right() - 90,
                                panels[-1].bottom() + 3, 90, 14),
                         Qt.AlignRight | Qt.AlignVCenter, "现在")

        if not self._samples:
            painter.setFont(QFont(UI_FONT, 9))
            painter.setPen(color(MUTED))
            empty = QRectF(panels[0].left(), panels[0].top(),
                           panels[-1].right() - panels[0].left(),
                           panels[0].height())
            painter.drawText(empty, Qt.AlignCenter,
                             "等待轮速上报…" if self.connected else "未连接")
            return

        target_pen = QPen(color(INK_2, 0.72), 1.3, Qt.DashLine)
        target_pen.setDashPattern([4.0, 3.0])
        actual_pen = QPen(color(SERIES_LEFT), 2.0)
        actual_pen.setCapStyle(Qt.RoundCap)
        actual_pen.setJoinStyle(Qt.RoundJoin)
        last = self._samples[-1]

        painter.save()
        for index, rect in enumerate(panels):
            painter.setClipRect(rect.adjusted(-1, -1, 1, 1))

            target_points: list[QPointF] = []
            actual_points: list[QPointF] = []
            for t, fl, fr, rl, rr, target in self._samples:
                if t < now - self.WINDOW_S:
                    continue
                x = self._x_for(t, now, rect)
                target_points.append(
                    QPointF(x, self._y_for(target, limit, rect)))
                actual_points.append(
                    QPointF(x, self._y_for((fl, fr, rl, rr)[index],
                                           limit, rect)))

            if len(actual_points) >= 2:
                painter.setPen(actual_pen)
                painter.drawPolyline(QPolygonF(actual_points))
            if len(target_points) >= 2:
                painter.setPen(target_pen)
                painter.drawPolyline(QPolygonF(target_points))

            value = (last[1], last[2], last[3], last[4])[index]
            x = min(rect.right() - 1.0, self._x_for(last[0], now, rect))
            if x >= rect.left():
                painter.setPen(QPen(color(SURFACE), 2.0))
                painter.setBrush(color(SERIES_LEFT))
                painter.drawEllipse(
                    QPointF(x, self._y_for(value, limit, rect)), 4.0, 4.0)
        painter.restore()

        self._paint_hover(painter, panels, now, limit)

    def _paint_hover(self, painter: QPainter, panels: list[QRectF],
                     now: float, limit: float) -> None:
        if self._hover is None or not self._samples:
            return

        hovered = self._panel_at(self._hover.x())
        if hovered is None:
            return

        rect = panels[hovered]
        best = None
        best_dx = 1e9
        for sample in self._samples:
            if sample[0] < now - self.WINDOW_S:
                continue
            dx = abs(self._x_for(sample[0], now, rect) - self._hover.x())
            if dx < best_dx:
                best, best_dx = sample, dx
        if best is None or best_dx > 18:
            return

        t, fl, fr, rl, rr, target = best
        for panel in panels:
            x = self._x_for(t, now, panel)
            painter.setPen(QPen(color(INK_2, 0.28), 1.0))
            painter.drawLine(QPointF(x, panel.top()), QPointF(x, panel.bottom()))

        for panel, value in zip(panels, (fl, fr, rl, rr)):
            x = self._x_for(t, now, panel)
            y = self._y_for(value, limit, panel)
            painter.setPen(QPen(color(SURFACE), 2.0))
            painter.setBrush(color(SERIES_LEFT))
            painter.drawEllipse(QPointF(x, y), 4.0, 4.0)

        text = (f"-{now - t:.1f}s  目标 {target:>4.0f}   "
                f"A {fl:>4.0f}  B {fr:>4.0f}  C {rl:>4.0f}  "
                f"D {rr:>4.0f} mm/s")
        font = QFont(MONO_FONT, 8)
        painter.setFont(font)
        metrics = QFontMetricsF(font)
        width = metrics.horizontalAdvance(text) + 16
        height = 22.0
        box_x = self._hover.x() + 10
        if box_x + width > panels[-1].right():
            box_x = self._hover.x() - 10 - width
        box_y = max(panels[0].top(),
                    min(self._hover.y() - height - 8,
                        panels[0].bottom() - height))
        box = QRectF(box_x, box_y, width, height)
        path = QPainterPath()
        path.addRoundedRect(box, 6, 6)
        painter.fillPath(path, color(SURFACE_3))
        # drawPath strokes *and* fills, so clear the marker brush or it
        # repaints the tooltip in the series colour.
        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(color(BASELINE), 1.0))
        painter.drawPath(path)
        painter.setPen(color(INK_2))
        painter.drawText(box, Qt.AlignCenter, text)


class LinkBadge(QWidget):
    """Small dot + status text used in the title bar."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._text = "未连接"
        self._colour = MUTED
        self._font = QFont(UI_FONT, 9)
        self.setFixedHeight(22)

    def set_state(self, text: str, colour: str) -> None:
        if text != self._text or colour != self._colour:
            self._text = text
            self._colour = colour
            self.updateGeometry()
            self.update()

    def sizeHint(self) -> QSize:  # noqa: N802
        width = QFontMetricsF(self._font).horizontalAdvance(self._text)
        return QSize(int(width) + 26, 22)

    def paintEvent(self, event):  # noqa: N802
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setBrush(color(self._colour))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(QPointF(7, self.height() / 2.0), 4.0, 4.0)
        painter.setFont(self._font)
        painter.setPen(color(INK_2))
        painter.drawText(QRectF(17, 0, self.width() - 17, self.height()),
                         Qt.AlignLeft | Qt.AlignVCenter, self._text)
