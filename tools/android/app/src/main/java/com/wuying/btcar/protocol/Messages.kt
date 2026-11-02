package com.wuying.btcar.protocol

/** 0xC1：里程计与左右轮速。 */
data class Odom(
    val xMm: Int,
    val yMm: Int,
    val headingCentiDeg: Int,
    val vlMmS: Int,
    val vrMmS: Int
)

/** 0xC3：状态、故障码、电池、板温、控制周期抖动。 */
data class RobotStatus(
    val state: Int,
    val fault: Int,
    val batteryMv: Int,
    val temperatureC: Int,
    val jitterUs: Int
)

/** 0xC5：心跳，固件每 100 ms 发一次。 */
data class Heartbeat(val state: Int, val counter: Int)

/** 0xC4：故障/事件发生的那一拍。 */
data class EventMessage(val code: Int, val timestampMs: Long)

/** 0xC6：四轮实测轮速，FL/FR/RL/RR。 */
data class WheelSpeeds(val fl: Int, val fr: Int, val rl: Int, val rr: Int)

/** 0xC7：四轮实际占空比，千分比，带符号。 */
data class WheelDuty(val fl: Int, val fr: Int, val rl: Int, val rr: Int)

/** 0xC2：六关节角度（0.01°）、夹爪位置与状态。占位，机械臂接入后再用。 */
data class ArmFeedback(
    val anglesCentiDeg: IntArray,
    val gripperPosition: Int,
    val gripperState: Int
) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is ArmFeedback) return false
        return anglesCentiDeg.contentEquals(other.anglesCentiDeg) &&
            gripperPosition == other.gripperPosition &&
            gripperState == other.gripperState
    }

    override fun hashCode(): Int =
        anglesCentiDeg.contentHashCode() * 31 + gripperPosition * 17 + gripperState
}

/**
 * 上报解码。长度不对返回 null —— 宁可少显示一帧，也不按猜的偏移解析。
 * 偏移量见 docs/architecture/protocol.md 第 5 节。
 */
object Messages {

    fun odom(payload: ByteArray): Odom? = if (payload.size == 14) {
        Odom(
            xMm = Protocol.i32(payload, 0),
            yMm = Protocol.i32(payload, 4),
            headingCentiDeg = Protocol.i16(payload, 8),
            vlMmS = Protocol.i16(payload, 10),
            vrMmS = Protocol.i16(payload, 12)
        )
    } else {
        null
    }

    fun status(payload: ByteArray): RobotStatus? = if (payload.size == 8) {
        RobotStatus(
            state = Protocol.u8(payload, 0),
            fault = Protocol.u16(payload, 1),
            batteryMv = Protocol.u16(payload, 3),
            temperatureC = payload[5].toInt(),          // int8，有符号
            jitterUs = Protocol.u16(payload, 6)
        )
    } else {
        null
    }

    fun heartbeat(payload: ByteArray): Heartbeat? = if (payload.size == 3) {
        Heartbeat(state = Protocol.u8(payload, 0), counter = Protocol.u16(payload, 1))
    } else {
        null
    }

    fun event(payload: ByteArray): EventMessage? = if (payload.size == 6) {
        EventMessage(
            code = Protocol.u16(payload, 0),
            timestampMs = Protocol.u32(payload, 2)
        )
    } else {
        null
    }

    fun wheelSpeeds(payload: ByteArray): WheelSpeeds? = if (payload.size == 8) {
        WheelSpeeds(
            fl = Protocol.i16(payload, 0),
            fr = Protocol.i16(payload, 2),
            rl = Protocol.i16(payload, 4),
            rr = Protocol.i16(payload, 6)
        )
    } else {
        null
    }

    fun wheelDuty(payload: ByteArray): WheelDuty? = if (payload.size == 8) {
        WheelDuty(
            fl = Protocol.i16(payload, 0),
            fr = Protocol.i16(payload, 2),
            rl = Protocol.i16(payload, 4),
            rr = Protocol.i16(payload, 6)
        )
    } else {
        null
    }

    fun arm(payload: ByteArray): ArmFeedback? = if (payload.size == 15) {
        ArmFeedback(
            anglesCentiDeg = IntArray(6) { Protocol.i16(payload, it * 2) },
            gripperPosition = Protocol.i16(payload, 12),
            gripperState = Protocol.u8(payload, 14)
        )
    } else {
        null
    }
}
