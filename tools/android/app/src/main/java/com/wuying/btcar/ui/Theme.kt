package com.wuying.btcar.ui

import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Typography
import androidx.compose.material3.darkColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.ui.graphics.Color

private val BtCarColors = darkColorScheme(
    primary = Color(0xFF6FD3FF),
    onPrimary = Color(0xFF00283A),
    primaryContainer = Color(0xFF123B52),
    onPrimaryContainer = Color(0xFFCCEEFF),
    secondary = Color(0xFF9CD67E),
    onSecondary = Color(0xFF14300A),
    error = Color(0xFFFF6B6B),
    onError = Color(0xFF3A0000),
    errorContainer = Color(0xFF4A1515),
    onErrorContainer = Color(0xFFFFD9D9),
    background = Color(0xFF0B0F14),
    onBackground = Color(0xFFE6EDF3),
    surface = Color(0xFF121820),
    onSurface = Color(0xFFE6EDF3),
    surfaceVariant = Color(0xFF1B2430),
    onSurfaceVariant = Color(0xFFA7B4C2),
    outline = Color(0xFF34404E),
    outlineVariant = Color(0xFF222C38)
)

/** 状态色不参与主题配色，只用于链路 / 故障提示。 */
object Accents {
    val ok = Color(0xFF64D68A)
    val warn = Color(0xFFFFC857)
    val danger = Color(0xFFFF5D5D)
    val muted = Color(0xFF8A97A6)
}

@Composable
fun BtCarTheme(content: @Composable () -> Unit) {
    MaterialTheme(colorScheme = BtCarColors, typography = Typography(), content = content)
}
