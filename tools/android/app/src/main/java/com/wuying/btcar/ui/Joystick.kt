package com.wuying.btcar.ui

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.gestures.detectDragGestures
import androidx.compose.foundation.layout.aspectRatio
import androidx.compose.material3.MaterialTheme
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.unit.dp
import kotlin.math.sqrt

private const val DEADZONE = 0.12f

/**
 * 单摇杆：x ∈ [-1, 1] 右正，y ∈ [-1, 1] 前正。
 *
 * 死区内的微小抖动直接归零，避免手指放置时车自己走；
 * 越过死区后重新从 0 线性放大到 1，手感不会在 12% 处跳变。
 */
@Composable
fun Joystick(
    x: Float,
    y: Float,
    active: Boolean,
    onChange: (Float, Float) -> Unit,
    onRelease: () -> Unit,
    modifier: Modifier = Modifier
) {
    val palette = MaterialTheme.colorScheme
    Canvas(
        modifier
            .aspectRatio(1f)
            .then(
                if (active) {
                    Modifier.pointerInput(Unit) {
                        fun emit(position: Offset) {
                            val radius = minOf(size.width, size.height) / 2f
                            val center = Offset(size.width / 2f, size.height / 2f)
                            val dx = (position.x - center.x) / radius
                            val dy = (center.y - position.y) / radius
                            val magnitude = sqrt(dx * dx + dy * dy)
                            if (magnitude < DEADZONE) {
                                onChange(0f, 0f)
                                return
                            }
                            val scaled = ((magnitude - DEADZONE) / (1f - DEADZONE)).coerceAtMost(1f)
                            val k = scaled / magnitude
                            onChange(
                                (dx * k).coerceIn(-1f, 1f),
                                (dy * k).coerceIn(-1f, 1f)
                            )
                        }

                        detectDragGestures(
                            onDragStart = { emit(it) },
                            onDrag = { change, _ ->
                                change.consume()
                                emit(change.position)
                            },
                            onDragEnd = onRelease,
                            onDragCancel = onRelease
                        )
                    }
                } else {
                    Modifier
                }
            )
    ) {
        val radius = size.minDimension / 2f
        val center = Offset(size.width / 2f, size.height / 2f)

        drawCircle(color = palette.surfaceVariant, radius = radius)
        drawCircle(
            color = palette.outline,
            radius = radius,
            style = Stroke(width = 2.dp.toPx())
        )
        val cross = palette.outline.copy(alpha = 0.45f)
        drawLine(
            color = cross,
            start = Offset(center.x - radius, center.y),
            end = Offset(center.x + radius, center.y),
            strokeWidth = 1.dp.toPx()
        )
        drawLine(
            color = cross,
            start = Offset(center.x, center.y - radius),
            end = Offset(center.x, center.y + radius),
            strokeWidth = 1.dp.toPx()
        )
        drawCircle(
            color = palette.outline.copy(alpha = 0.6f),
            radius = radius * DEADZONE,
            center = center,
            style = Stroke(width = 1.dp.toPx())
        )

        val knobCenter = Offset(
            center.x + x * radius * 0.78f,
            center.y - y * radius * 0.78f
        )
        val knobColor = if (active) palette.primary else palette.outline
        drawCircle(color = knobColor.copy(alpha = 0.20f), radius = radius * 0.30f, center = knobCenter)
        drawCircle(color = knobColor, radius = radius * 0.24f, center = knobCenter)
    }
}
