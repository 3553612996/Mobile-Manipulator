package com.wuying.btcar.protocol

/**
 * 与 firmware/app/05_Middlewares/protocol/protocol_def.h 逐项对应。
 *
 * 帧格式（多字节小端）：
 *   AA 55 | LEN | CMD | SEQ | PAYLOAD(LEN-2 字节) | CRC_LO CRC_HI
 *   LEN = 2 + 负载长度，合法范围 2..66
 *   CRC-16/CCITT-FALSE（poly 0x1021，init 0xFFFF），覆盖 LEN..PAYLOAD
 *   ACK 帧：CMD = 原命令 | 0x80，负载 = [被回的原 SEQ, 结果码]
 *
 * 这里的常量与 ROS 侧 robot_base/protocol.hpp、固件 protocol_def.h 是三份
 * 同一协议的实现；改协议时三处一起改，并用 docs/architecture/protocol.md
 * 的测试向量验证。
 */
object Protocol {

    const val HEAD1 = 0xAA
    const val HEAD2 = 0x55
    const val LEN_MIN = 2
    const val LEN_MAX = 66
    const val PAYLOAD_MAX = LEN_MAX - 2
    const val ACK_FLAG = 0x80
    const val CLEAR_ALL_FAULTS = 0xFFFF

    /** 经典蓝牙串口（SPP）固定 UUID，HC-05/HC-06/JDY-31 都用它。 */
    const val SPP_UUID = "00001101-0000-1000-8000-00805F9B34FB"

    object Cmd {
        const val SET_VELOCITY = 0x01
        const val SET_ARM_JOINTS = 0x02
        const val SET_GRIPPER = 0x03
        const val SET_ENABLE = 0x04
        const val SET_PARAM = 0x05
        const val ESTOP = 0x06
        const val CLEAR_FAULT = 0x07
        const val PARAM_SAVE = 0x08
        const val PARAM_RESET = 0x09
        const val STORAGE_QUERY = 0x0A
        const val LOG_EXPORT = 0x0B
        const val CRASH_READ = 0x0C
        const val LOG_CLEAR = 0x0D
    }

    object Report {
        const val ODOM = 0xC1
        const val ARM = 0xC2
        const val STATUS = 0xC3
        const val EVENT = 0xC4
        const val HEARTBEAT = 0xC5
        const val WHEEL_SPEED = 0xC6
        const val WHEEL_DUTY = 0xC7
        const val STORAGE = 0xC8
        const val LOG = 0xC9
        const val CRASH = 0xCA
    }

    object Result {
        const val OK = 0
        const val RANGE = 1
        const val STATE = 2
        const val UNKNOWN_CMD = 3
        const val FAILED = 4
        const val BUSY = 5
        const val STORAGE_UNAVAILABLE = 6
    }

    object State {
        const val INIT = 0
        const val IDLE = 1
        const val MANUAL = 2
        const val AUTO = 3
        const val ESTOP = 4
        const val FAULT = 5
    }

    // ------------------------------------------------------------ 编解码

    /** CRC-16/CCITT-FALSE，逐位实现，和固件 platform_crc.c 等价。 */
    fun crc16(buf: ByteArray, from: Int, until: Int): Int {
        var crc = 0xFFFF
        for (i in from until until) {
            crc = crc xor ((buf[i].toInt() and 0xFF) shl 8)
            repeat(8) {
                crc = if (crc and 0x8000 != 0) {
                    ((crc shl 1) xor 0x1021)
                } else {
                    crc shl 1
                }
                crc = crc and 0xFFFF
            }
        }
        return crc and 0xFFFF
    }

    /** 组一帧。负载超过 64 字节直接抛异常，避免把坏帧发出去。 */
    fun build(cmd: Int, seq: Int, payload: ByteArray = ByteArray(0)): ByteArray {
        require(payload.size <= PAYLOAD_MAX) {
            "payload ${payload.size} B 超过上限 $PAYLOAD_MAX B"
        }
        val len = payload.size + 2
        val out = ByteArray(3 + len + 2)
        out[0] = HEAD1.toByte()
        out[1] = HEAD2.toByte()
        out[2] = len.toByte()
        out[3] = cmd.toByte()
        out[4] = seq.toByte()
        payload.copyInto(out, 5)
        val crc = crc16(out, 2, 3 + len)
        out[3 + len] = (crc and 0xFF).toByte()
        out[4 + len] = ((crc shr 8) and 0xFF).toByte()
        return out
    }

    internal fun u8(buf: ByteArray, offset: Int): Int = buf[offset].toInt() and 0xFF

    internal fun u16(buf: ByteArray, offset: Int): Int =
        u8(buf, offset) or (u8(buf, offset + 1) shl 8)

    internal fun i16(buf: ByteArray, offset: Int): Int =
        u16(buf, offset).toShort().toInt()

    internal fun i32(buf: ByteArray, offset: Int): Int =
        u8(buf, offset) or
            (u8(buf, offset + 1) shl 8) or
            (u8(buf, offset + 2) shl 16) or
            (u8(buf, offset + 3) shl 24)

    internal fun u32(buf: ByteArray, offset: Int): Long =
        i32(buf, offset).toLong() and 0xFFFFFFFFL

    fun i16Bytes(value: Int): ByteArray = byteArrayOf(
        (value and 0xFF).toByte(),
        ((value shr 8) and 0xFF).toByte()
    )

    fun u16Bytes(value: Int): ByteArray = i16Bytes(value)

    // ------------------------------------------------------------ 可读名字

    fun stateName(state: Int?): String = when (state) {
        State.INIT -> "INIT"
        State.IDLE -> "IDLE"
        State.MANUAL -> "MANUAL"
        State.AUTO -> "AUTO"
        State.ESTOP -> "ESTOP"
        State.FAULT -> "FAULT"
        null -> "—"
        else -> "0x%02X".format(state)
    }

    fun resultName(result: Int?): String = when (result) {
        null -> "无应答"
        Result.OK -> "OK"
        Result.RANGE -> "参数越界"
        Result.STATE -> "当前状态不允许"
        Result.UNKNOWN_CMD -> "未知命令"
        Result.FAILED -> "执行失败"
        Result.BUSY -> "存储忙"
        Result.STORAGE_UNAVAILABLE -> "存储不可用"
        else -> "0x%02X".format(result)
    }

    fun faultName(code: Int?): String = when (code) {
        null, 0 -> "无"
        0x0101 -> "驱动过流"
        0x0102 -> "编码器异常"
        0x0201 -> "连续 CRC 错"
        0x0202 -> "指令超时"
        0x0301 -> "关节越限"
        0x0302 -> "关节堵转"
        0x0303 -> "轨迹异常"
        0x0401 -> "电量低"
        0x0402 -> "过温"
        0x0501 -> "急停"
        0x0601 -> "参数无效"
        0x0602 -> "标定无效"
        0x0603 -> "存储写入失败"
        0x0604 -> "分区异常"
        else -> "0x%04X".format(code)
    }
}

/** 一帧已通过 CRC 的数据。payload 已去掉 LEN/CMD/SEQ 与 CRC。 */
class Frame(val cmd: Int, val seq: Int, val payload: ByteArray) {
    /**
     * ACK 是命令码 | 0x80，落在 0x80..0xBF；上报码在 0xC0..0xFF。
     * 只看 0x80 位会把 0xC1..0xCA 的上报误判成 ACK，必须连 0x40 一起看。
     */
    val isAck: Boolean get() = (cmd and 0xC0) == 0x80
    val ackedSeq: Int? get() = if (isAck && payload.isNotEmpty()) payload[0].toInt() and 0xFF else null
    val result: Int? get() = if (isAck && payload.size >= 2) payload[1].toInt() and 0xFF else null
}

/**
 * 增量解析器。feed() 接受任意切分，包括一次一个字节。
 * LEN 或 CRC 不合法时丢掉一个字节重新找帧头，坏帧不会连锁。
 */
class FrameParser {

    private var buffer: ByteArray = ByteArray(0)

    var framesOk: Long = 0
        private set
    var crcErrors: Long = 0
        private set
    var lenErrors: Long = 0
        private set
    var resyncBytes: Long = 0
        private set

    fun reset() {
        buffer = ByteArray(0)
    }

    fun feed(data: ByteArray, length: Int = data.size, onFrame: (Frame) -> Unit) {
        if (length <= 0) return
        buffer += data.copyOf(length)

        while (true) {
            var start = -1
            var i = 0
            while (i + 1 < buffer.size) {
                if (buffer[i] == Protocol.HEAD1.toByte() &&
                    buffer[i + 1] == Protocol.HEAD2.toByte()
                ) {
                    start = i
                    break
                }
                i++
            }

            if (start < 0) {
                // 结尾的单个 0xAA 可能是下一帧的帧头，留着；其余全丢。
                buffer = if (buffer.isNotEmpty() &&
                    buffer[buffer.size - 1] == Protocol.HEAD1.toByte()
                ) {
                    byteArrayOf(Protocol.HEAD1.toByte())
                } else {
                    ByteArray(0)
                }
                return
            }

            if (start > 0) {
                resyncBytes += start
                buffer = buffer.copyOfRange(start, buffer.size)
            }

            if (buffer.size < 3) return

            val len = Protocol.u8(buffer, 2)
            if (len < Protocol.LEN_MIN || len > Protocol.LEN_MAX) {
                lenErrors++
                buffer = buffer.copyOfRange(1, buffer.size)
                continue
            }

            val total = len + 5
            if (buffer.size < total) return

            val expected = Protocol.u16(buffer, total - 2)
            if (Protocol.crc16(buffer, 2, total - 2) != expected) {
                crcErrors++
                buffer = buffer.copyOfRange(1, buffer.size)
                continue
            }

            framesOk++
            val frame = Frame(
                cmd = Protocol.u8(buffer, 3),
                seq = Protocol.u8(buffer, 4),
                payload = buffer.copyOfRange(5, total - 2)
            )
            buffer = buffer.copyOfRange(total, buffer.size)
            onFrame(frame)
        }
    }
}
