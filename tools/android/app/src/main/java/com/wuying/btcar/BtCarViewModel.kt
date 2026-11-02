package com.wuying.btcar

import android.bluetooth.BluetoothDevice
import android.os.SystemClock
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.wuying.btcar.link.SppLink
import com.wuying.btcar.protocol.ArmFeedback
import com.wuying.btcar.protocol.EventMessage
import com.wuying.btcar.protocol.Frame
import com.wuying.btcar.protocol.Heartbeat
import com.wuying.btcar.protocol.Messages
import com.wuying.btcar.protocol.Odom
import com.wuying.btcar.protocol.Protocol
import com.wuying.btcar.protocol.RobotStatus
import com.wuying.btcar.protocol.WheelDuty
import com.wuying.btcar.protocol.WheelSpeeds
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.collect
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlin.math.roundToInt

data class UiState(
    val link: SppLink.LinkState = SppLink.LinkState.Idle,
    val enabled: Boolean = false,
    val maxSpeed: Int = 300,
    val stickX: Float = 0f,
    val stickY: Float = 0f,
    val odom: Odom? = null,
    val status: RobotStatus? = null,
    val heartbeat: Heartbeat? = null,
    val wheels: WheelSpeeds? = null,
    val duty: WheelDuty? = null,
    val arm: ArmFeedback? = null,
    val lastEvent: EventMessage? = null,
    val message: String? = null,
    val nowMs: Long = SystemClock.elapsedRealtime(),
    val lastRxMs: Long = 0L,
    val framesOk: Long = 0L,
    val crcErrors: Long = 0L
) {
    val connected: Boolean get() = link is SppLink.LinkState.Connected
    val rxAgeMs: Long get() = if (lastRxMs == 0L) -1 else (nowMs - lastRxMs).coerceAtLeast(0)
}

/**
 * 一整台车的会话状态。
 *
 * 关键时序（与 base_bridge 相同）：
 *   - SET_VELOCITY 每 20 ms 发一次，不管车在不在动；固件 200 ms 收不到就停车。
 *     所以"连接后立刻开始发零速"是必要的，否则一使能就会被判指令超时。
 *   - 只有使能后才把摇杆值放进速度帧；未使能时恒发 0，避免使能瞬间窜车。
 *   - 断开前先发零速 + 失能，避免留下 0x0202 故障锁存。
 */
class BtCarViewModel : ViewModel() {

    private val link = SppLink(viewModelScope)
    private val _ui = MutableStateFlow(UiState())
    val ui: StateFlow<UiState> = _ui.asStateFlow()

    private var streamJob: Job? = null

    init {
        link.onFrame = ::handleFrame
        link.onNotice = { text -> _ui.update { it.copy(message = text) } }

        viewModelScope.launch {
            link.state.collect { state ->
                _ui.update {
                    it.copy(
                        link = state,
                        enabled = if (state is SppLink.LinkState.Connected) it.enabled else false,
                        framesOk = link.parser.framesOk,
                        crcErrors = link.parser.crcErrors
                    )
                }
                if (state is SppLink.LinkState.Connected) startStream() else stopStream()
            }
        }

        // 链路年龄和统计每秒刷两次，心跳停了也能看出"多久没数据"。
        viewModelScope.launch {
            while (isActive) {
                delay(500)
                _ui.update {
                    it.copy(
                        nowMs = SystemClock.elapsedRealtime(),
                        framesOk = link.parser.framesOk,
                        crcErrors = link.parser.crcErrors
                    )
                }
            }
        }
    }

    // ------------------------------------------------------------ 连接

    fun connect(device: BluetoothDevice) {
        viewModelScope.launch { link.connect(device) }
    }

    fun disconnect() {
        viewModelScope.launch(Dispatchers.IO) {
            val zero = Protocol.i16Bytes(0) + Protocol.i16Bytes(0)
            link.send(Protocol.Cmd.SET_VELOCITY, zero, needAck = false)
            link.send(Protocol.Cmd.SET_ENABLE, byteArrayOf(0), needAck = false)
            link.disconnect()
            _ui.update { it.copy(enabled = false, stickX = 0f, stickY = 0f) }
        }
    }

    // ------------------------------------------------------------ 控制

    fun setStick(x: Float, y: Float) {
        _ui.update {
            it.copy(
                stickX = x.coerceIn(-1f, 1f),
                stickY = y.coerceIn(-1f, 1f)
            )
        }
    }

    fun releaseStick() {
        _ui.update { it.copy(stickX = 0f, stickY = 0f) }
    }

    fun setMaxSpeed(value: Int) {
        val snapped = ((value.coerceIn(100, 500) + 25) / 50) * 50
        _ui.update { it.copy(maxSpeed = snapped.coerceIn(100, 500)) }
    }

    fun setEnabled(on: Boolean) {
        if (!_ui.value.connected) return
        viewModelScope.launch(Dispatchers.IO) {
            val result = link.send(Protocol.Cmd.SET_ENABLE, byteArrayOf(if (on) 1 else 0))
            _ui.update {
                when {
                    result == null -> it.copy(message = "使能命令无应答，检查蓝牙连接")
                    result == Protocol.Result.OK -> it.copy(
                        enabled = on,
                        message = if (on) "底盘已使能" else "底盘已失能"
                    )
                    else -> it.copy(
                        enabled = false,
                        message = "使能被拒绝：${Protocol.resultName(result)}"
                    )
                }
            }
        }
    }

    fun emergencyStop() {
        viewModelScope.launch(Dispatchers.IO) {
            val result = link.send(Protocol.Cmd.ESTOP)
            _ui.update {
                it.copy(
                    enabled = false,
                    stickX = 0f,
                    stickY = 0f,
                    message = when (result) {
                        Protocol.Result.OK -> "急停已锁存；清除故障后需要重新使能"
                        null -> "急停无应答——立即断开连接"
                        else -> "急停被拒绝：${Protocol.resultName(result)}"
                    }
                )
            }
        }
    }

    fun clearFault() {
        viewModelScope.launch(Dispatchers.IO) {
            val result = link.send(
                Protocol.Cmd.CLEAR_FAULT,
                Protocol.u16Bytes(Protocol.CLEAR_ALL_FAULTS)
            )
            _ui.update {
                it.copy(
                    message = when (result) {
                        Protocol.Result.OK -> "故障已清除，需要重新使能"
                        null -> "清除命令无应答"
                        else -> "清除失败：${Protocol.resultName(result)}"
                    }
                )
            }
        }
    }

    fun clearMessage() {
        _ui.update { it.copy(message = null) }
    }

    /** 切后台时调用：停住、失能，但保留连接和 20 ms 零速流，回来能直接用。 */
    fun onBackground() {
        _ui.update { it.copy(stickX = 0f, stickY = 0f) }
        if (!_ui.value.connected) return
        viewModelScope.launch(Dispatchers.IO) {
            link.send(
                Protocol.Cmd.SET_VELOCITY,
                Protocol.i16Bytes(0) + Protocol.i16Bytes(0),
                needAck = false
            )
            if (_ui.value.enabled) {
                link.send(Protocol.Cmd.SET_ENABLE, byteArrayOf(0), needAck = false)
            }
            _ui.update { it.copy(enabled = false) }
        }
    }

    // ------------------------------------------------------------ 内部

    private fun startStream() {
        if (streamJob?.isActive == true) return
        streamJob = viewModelScope.launch(Dispatchers.IO) {
            while (isActive) {
                val snapshot = _ui.value
                link.send(
                    Protocol.Cmd.SET_VELOCITY,
                    velocityPayload(snapshot),
                    needAck = false
                )
                delay(20)
            }
        }
    }

    private fun stopStream() {
        streamJob?.cancel()
        streamJob = null
    }

    private fun velocityPayload(state: UiState): ByteArray {
        var vx = 0
        var wz = 0
        if (state.connected && state.enabled) {
            vx = (state.stickY * state.maxSpeed).roundToInt()
                .coerceIn(-state.maxSpeed, state.maxSpeed)
            val maxWz = (1500f * state.maxSpeed / 500f).roundToInt()
            wz = (-state.stickX * maxWz).roundToInt().coerceIn(-1500, 1500)
        }
        return Protocol.i16Bytes(vx) + Protocol.i16Bytes(wz)
    }

    private fun handleFrame(frame: Frame) {
        val now = SystemClock.elapsedRealtime()
        when (frame.cmd) {
            Protocol.Report.ODOM -> Messages.odom(frame.payload)?.let {
                _ui.update { s -> s.copy(odom = it, lastRxMs = now) }
            }
            Protocol.Report.STATUS -> Messages.status(frame.payload)?.let {
                _ui.update { s -> s.copy(status = it, lastRxMs = now) }
            }
            Protocol.Report.HEARTBEAT -> Messages.heartbeat(frame.payload)?.let {
                _ui.update { s -> s.copy(heartbeat = it, lastRxMs = now) }
            }
            Protocol.Report.EVENT -> Messages.event(frame.payload)?.let {
                _ui.update { s -> s.copy(lastEvent = it, lastRxMs = now) }
            }
            Protocol.Report.WHEEL_SPEED -> Messages.wheelSpeeds(frame.payload)?.let {
                _ui.update { s -> s.copy(wheels = it, lastRxMs = now) }
            }
            Protocol.Report.WHEEL_DUTY -> Messages.wheelDuty(frame.payload)?.let {
                _ui.update { s -> s.copy(duty = it, lastRxMs = now) }
            }
            Protocol.Report.ARM -> Messages.arm(frame.payload)?.let {
                _ui.update { s -> s.copy(arm = it, lastRxMs = now) }
            }
            else -> _ui.update { s -> s.copy(lastRxMs = now) }
        }
    }

    override fun onCleared() {
        streamJob?.cancel()
        link.disconnect()
        super.onCleared()
    }
}
