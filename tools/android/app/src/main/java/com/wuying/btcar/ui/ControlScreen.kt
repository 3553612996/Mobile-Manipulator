package com.wuying.btcar.ui

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.systemBarsPadding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Slider
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.dp
import com.wuying.btcar.UiState
import com.wuying.btcar.link.SppLink
import com.wuying.btcar.protocol.Protocol
import kotlinx.coroutines.delay
import kotlin.math.roundToInt

@Composable
fun ControlScreen(
    state: UiState,
    onToggleLink: () -> Unit,
    onStick: (Float, Float) -> Unit,
    onStickRelease: () -> Unit,
    onMaxSpeed: (Int) -> Unit,
    onToggleEnable: () -> Unit,
    onEmergencyStop: () -> Unit,
    onClearFault: () -> Unit,
    onDismissMessage: () -> Unit
) {
    LaunchedEffect(state.message) {
        if (state.message == null) return@LaunchedEffect
        delay(6000)
        onDismissMessage()
    }

    Column(
        modifier = Modifier
            .fillMaxSize()
            .background(MaterialTheme.colorScheme.background)
            .systemBarsPadding()
            .verticalScroll(rememberScrollState())
            .padding(horizontal = 16.dp, vertical = 12.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp)
    ) {
        Header(state = state, onToggleLink = onToggleLink)
        state.message?.let { text ->
            MessageBanner(text = text, onDismiss = onDismissMessage)
        }
        StatusCard(state)
        SpeedCard(state.maxSpeed, onMaxSpeed)
        DriveCard(
            state = state,
            onStick = onStick,
            onStickRelease = onStickRelease,
            onToggleEnable = onToggleEnable,
            onEmergencyStop = onEmergencyStop,
            onClearFault = onClearFault
        )
        MotionCard(state)
        ArmPlaceholder()
        Spacer(Modifier.height(8.dp))
    }
}

// ---------------------------------------------------------------- 头部

@Composable
private fun Header(state: UiState, onToggleLink: () -> Unit) {
    val connecting = state.link is SppLink.LinkState.Connecting
    Row(verticalAlignment = Alignment.CenterVertically) {
        Column(Modifier.weight(1f)) {
            Text("蓝牙小车", style = MaterialTheme.typography.titleLarge)
            Spacer(Modifier.height(4.dp))
            Row(verticalAlignment = Alignment.CenterVertically) {
                LinkChip(state)
                Spacer(Modifier.width(8.dp))
                Text(
                    text = linkDetail(state),
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                    maxLines = 1,
                    overflow = TextOverflow.Ellipsis
                )
            }
        }
        Button(onClick = onToggleLink, enabled = !connecting) {
            Text(
                when {
                    connecting -> "连接中…"
                    state.connected -> "断开"
                    state.link is SppLink.LinkState.Failed -> "重连"
                    else -> "连接"
                }
            )
        }
    }
}

@Composable
private fun LinkChip(state: UiState) {
    val (color, label) = when (state.link) {
        is SppLink.LinkState.Idle -> Accents.muted to "未连接"
        is SppLink.LinkState.Connecting -> Accents.warn to "连接中"
        is SppLink.LinkState.Connected -> Accents.ok to "已连接"
        is SppLink.LinkState.Failed -> Accents.danger to "链路异常"
    }
    Row(verticalAlignment = Alignment.CenterVertically) {
        Box(Modifier.size(8.dp).background(color, CircleShape))
        Spacer(Modifier.width(6.dp))
        Text(label, color = color, style = MaterialTheme.typography.labelMedium)
    }
}

private fun linkDetail(state: UiState): String = when (val link = state.link) {
    is SppLink.LinkState.Idle -> "选择一个已配对的 HC-05 / HC-06 模块"
    is SppLink.LinkState.Connecting -> link.name
    is SppLink.LinkState.Connected -> "${link.name} · ${link.address}"
    is SppLink.LinkState.Failed -> link.message
}

@Composable
private fun MessageBanner(text: String, onDismiss: () -> Unit) {
    Card(colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surfaceVariant)) {
        Row(
            modifier = Modifier.fillMaxWidth().padding(start = 14.dp, top = 6.dp, bottom = 6.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            Text(
                text = text,
                modifier = Modifier.weight(1f),
                style = MaterialTheme.typography.bodyMedium,
                color = MaterialTheme.colorScheme.onSurfaceVariant
            )
            TextButton(onClick = onDismiss) { Text("关闭") }
        }
    }
}

// ---------------------------------------------------------------- 状态

@Composable
private fun StatusCard(state: UiState) {
    val status = state.status
    val fault = status?.fault ?: 0
    Card(colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surface)) {
        Column(Modifier.fillMaxWidth().padding(16.dp), verticalArrangement = Arrangement.spacedBy(4.dp)) {
            Text("实时状态", style = MaterialTheme.typography.titleSmall)
            Spacer(Modifier.height(4.dp))

            Row(horizontalArrangement = Arrangement.spacedBy(16.dp)) {
                Column(Modifier.weight(1f)) {
                    InfoRow("状态", Protocol.stateName(status?.state))
                    InfoRow(
                        "故障",
                        Protocol.faultName(fault),
                        if (fault != 0) Accents.danger else MaterialTheme.colorScheme.onSurface
                    )
                    InfoRow(
                        "电量",
                        status?.let { "%.2f V".format(it.batteryMv / 1000f) } ?: "—",
                        when {
                            status == null -> MaterialTheme.colorScheme.onSurface
                            status.batteryMv < 6800 -> Accents.danger
                            status.batteryMv < 7200 -> Accents.warn
                            else -> Accents.ok
                        }
                    )
                }
                Column(Modifier.weight(1f)) {
                    InfoRow("板温", status?.let { "${it.temperatureC} ℃" } ?: "—")
                    InfoRow(
                        "数据",
                        when {
                            !state.connected -> "—"
                            state.rxAgeMs < 0 -> "等待上报"
                            else -> "${state.rxAgeMs} ms 前"
                        },
                        if (state.connected && (state.rxAgeMs < 0 || state.rxAgeMs > 1000)) {
                            Accents.warn
                        } else {
                            MaterialTheme.colorScheme.onSurface
                        }
                    )
                    InfoRow("帧", "${state.framesOk} / 错 ${state.crcErrors}")
                }
            }

            state.lastEvent?.let { event ->
                Spacer(Modifier.height(4.dp))
                Text(
                    text = "事件 0x%04X @ %d ms".format(event.code, event.timestampMs),
                    style = MaterialTheme.typography.bodySmall,
                    color = Accents.warn
                )
            }
        }
    }
}

@Composable
private fun InfoRow(label: String, value: String, color: Color = MaterialTheme.colorScheme.onSurface) {
    Row(
        modifier = Modifier.fillMaxWidth().padding(vertical = 3.dp),
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text(
            text = label,
            style = MaterialTheme.typography.bodySmall,
            color = MaterialTheme.colorScheme.onSurfaceVariant
        )
        Spacer(Modifier.width(8.dp))
        Text(
            text = value,
            modifier = Modifier.weight(1f),
            style = MaterialTheme.typography.bodyMedium.copy(fontFamily = FontFamily.Monospace),
            color = color,
            maxLines = 1,
            overflow = TextOverflow.Ellipsis,
            textAlign = TextAlign.End
        )
    }
}

// ---------------------------------------------------------------- 速度

@Composable
private fun SpeedCard(maxSpeed: Int, onMaxSpeed: (Int) -> Unit) {
    Card(colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surface)) {
        Column(Modifier.fillMaxWidth().padding(16.dp)) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Text("最大速度", style = MaterialTheme.typography.titleSmall)
                Spacer(Modifier.weight(1f))
                Text(
                    "$maxSpeed mm/s",
                    style = MaterialTheme.typography.bodyMedium.copy(fontFamily = FontFamily.Monospace),
                    color = MaterialTheme.colorScheme.primary
                )
            }
            Slider(
                value = maxSpeed.toFloat(),
                onValueChange = { onMaxSpeed(it.roundToInt()) },
                valueRange = 100f..500f,
                steps = 7
            )
            Text(
                "只限制前进/后退的线速度；转向按比例缩放，满速 300 时对应 900 mrad/s。",
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant
            )
        }
    }
}

// ---------------------------------------------------------------- 行驶

@Composable
private fun DriveCard(
    state: UiState,
    onStick: (Float, Float) -> Unit,
    onStickRelease: () -> Unit,
    onToggleEnable: () -> Unit,
    onEmergencyStop: () -> Unit,
    onClearFault: () -> Unit
) {
    val active = state.connected && state.enabled
    Card(colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surface)) {
        Column(
            Modifier.fillMaxWidth().padding(16.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Row(Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically) {
                Text("行驶控制", style = MaterialTheme.typography.titleSmall)
                Spacer(Modifier.weight(1f))
                Text(
                    if (active) "已使能" else "未使能",
                    style = MaterialTheme.typography.labelMedium,
                    color = if (active) Accents.ok else Accents.muted
                )
            }
            Spacer(Modifier.height(12.dp))
            Joystick(
                x = state.stickX,
                y = state.stickY,
                active = active,
                onChange = onStick,
                onRelease = onStickRelease,
                modifier = Modifier.fillMaxWidth(0.72f)
            )
            Spacer(Modifier.height(8.dp))
            Text(
                text = when {
                    !state.connected -> "连接蓝牙后可用"
                    !state.enabled -> "点击「使能」后才能行驶"
                    else -> "上/下：前进后退 · 左/右：转向 · 松手立即归零"
                },
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
                textAlign = TextAlign.Center
            )
            Spacer(Modifier.height(12.dp))
            Row(horizontalArrangement = Arrangement.spacedBy(12.dp)) {
                Button(
                    onClick = onToggleEnable,
                    modifier = Modifier.weight(1f),
                    enabled = state.connected
                ) {
                    Text(if (state.enabled) "失能" else "使能")
                }
                Button(
                    onClick = onEmergencyStop,
                    modifier = Modifier.weight(1f),
                    enabled = state.connected,
                    colors = ButtonDefaults.buttonColors(
                        containerColor = MaterialTheme.colorScheme.error,
                        contentColor = MaterialTheme.colorScheme.onError
                    )
                ) {
                    Text("急停")
                }
            }
            Spacer(Modifier.height(8.dp))
            OutlinedButton(
                onClick = onClearFault,
                modifier = Modifier.fillMaxWidth(),
                enabled = state.connected
            ) {
                Text("清除故障")
            }
        }
    }
}

// ---------------------------------------------------------------- 运动反馈

@Composable
private fun MotionCard(state: UiState) {
    val odom = state.odom
    Card(colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surface)) {
        Column(Modifier.fillMaxWidth().padding(16.dp), verticalArrangement = Arrangement.spacedBy(4.dp)) {
            Text("运动反馈", style = MaterialTheme.typography.titleSmall)
            Spacer(Modifier.height(4.dp))
            Row(horizontalArrangement = Arrangement.spacedBy(16.dp)) {
                Column(Modifier.weight(1f)) {
                    InfoRow("左轮", odom?.let { "${it.vlMmS} mm/s" } ?: "—")
                    InfoRow("X", odom?.let { "%.2f m".format(it.xMm / 1000f) } ?: "—")
                }
                Column(Modifier.weight(1f)) {
                    InfoRow("右轮", odom?.let { "${it.vrMmS} mm/s" } ?: "—")
                    InfoRow("Y", odom?.let { "%.2f m".format(it.yMm / 1000f) } ?: "—")
                }
            }
            InfoRow("航向", odom?.let { "%.2f°".format(it.headingCentiDeg / 100f) } ?: "—")
        }
    }
}

// ---------------------------------------------------------------- 机械臂占位

@Composable
private fun ArmPlaceholder() {
    Card(
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surfaceVariant)
    ) {
        Column(Modifier.fillMaxWidth().padding(16.dp)) {
            Text(
                "机械臂（预留）",
                style = MaterialTheme.typography.titleSmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant
            )
            Spacer(Modifier.height(6.dp))
            Text(
                "协议里 SET_ARM_JOINTS(0x02) / SET_GRIPPER(0x03) 与 ARM(0xC2) 上报已经定义好，" +
                    "接上机械臂后在这里加六个关节滑条、夹爪按钮和角度回读。",
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant
            )
        }
    }
}
