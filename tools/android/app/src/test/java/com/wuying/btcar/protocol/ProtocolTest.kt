package com.wuying.btcar.protocol

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * 用 docs/architecture/protocol.md 里的测试向量锁住编解码。
 * 这三帧和固件 protocol.c、ROS 侧 robot_base/protocol.hpp 必须字节一致。
 */
class ProtocolTest {

    private val velocityVector = byteArrayOf(
        0xAA.toByte(), 0x55, 0x06, 0x01, 0x01,
        0x2C, 0x01, 0xF4.toByte(), 0x01, 0x72, 0x3F
    )
    private val enableVector = byteArrayOf(
        0xAA.toByte(), 0x55, 0x03, 0x04, 0x02, 0x01, 0x9F.toByte(), 0xB5.toByte()
    )
    private val estopVector = byteArrayOf(
        0xAA.toByte(), 0x55, 0x02, 0x06, 0x03, 0x39, 0x38
    )

    private fun ByteArray.hex(): String = joinToString(" ") { "%02X".format(it) }

    @Test
    fun `set velocity 编码与固件一致`() {
        val frame = Protocol.build(
            Protocol.Cmd.SET_VELOCITY,
            1,
            Protocol.i16Bytes(300) + Protocol.i16Bytes(500)
        )
        assertEquals(velocityVector.hex(), frame.hex())
    }

    @Test
    fun `set enable 编码与固件一致`() {
        val frame = Protocol.build(Protocol.Cmd.SET_ENABLE, 2, byteArrayOf(1))
        assertEquals(enableVector.hex(), frame.hex())
    }

    @Test
    fun `estop 编码与固件一致`() {
        val frame = Protocol.build(Protocol.Cmd.ESTOP, 3)
        assertEquals(estopVector.hex(), frame.hex())
    }

    @Test
    fun `按任意切分喂入都能还原三帧`() {
        val stream = velocityVector + enableVector + estopVector
        val frames = mutableListOf<Frame>()
        val parser = FrameParser()

        // 3 字节一段，模拟蓝牙一次读回半帧的情况。
        var offset = 0
        while (offset < stream.size) {
            val end = minOf(offset + 3, stream.size)
            parser.feed(stream.copyOfRange(offset, end)) { frames += it }
            offset = end
        }

        assertEquals(3, frames.size)
        assertEquals(Protocol.Cmd.SET_VELOCITY, frames[0].cmd)
        assertEquals(1, frames[0].seq)
        assertEquals(4, frames[0].payload.size)
        assertEquals(300, Protocol.i16(frames[0].payload, 0))
        assertEquals(500, Protocol.i16(frames[0].payload, 2))
        assertEquals(Protocol.Cmd.SET_ENABLE, frames[1].cmd)
        assertEquals(Protocol.Cmd.ESTOP, frames[2].cmd)
        assertEquals(0, frames[2].payload.size)
    }

    @Test
    fun `CRC 错误的帧被丢弃且计数`() {
        val broken = velocityVector.copyOf()
        broken[broken.size - 1] = (broken[broken.size - 1].toInt() xor 0xFF).toByte()

        val parser = FrameParser()
        val frames = mutableListOf<Frame>()
        parser.feed(broken) { frames += it }

        assertTrue(frames.isEmpty())
        assertTrue(parser.crcErrors >= 1)
    }

    @Test
    fun `ACK 帧能取出原序号和结果码`() {
        val ack = Protocol.build(Protocol.Cmd.SET_ENABLE or Protocol.ACK_FLAG, 7, byteArrayOf(5, 0))
        val parser = FrameParser()
        var frame: Frame? = null
        parser.feed(ack) { frame = it }

        val parsed = frame
        assertNotNull(parsed)
        assertTrue(parsed!!.isAck)
        assertEquals(5, parsed.ackedSeq)
        assertEquals(Protocol.Result.OK, parsed.result)
    }

    @Test
    fun `长度非法的帧头会被跳过`() {
        // LEN = 0x01 非法，解析器丢掉第一个字节重新找帧头。
        val bad = byteArrayOf(0xAA.toByte(), 0x55, 0x01, 0x04, 0x00) + enableVector
        val parser = FrameParser()
        val frames = mutableListOf<Frame>()
        parser.feed(bad) { frames += it }

        assertEquals(1, frames.size)
        assertEquals(Protocol.Cmd.SET_ENABLE, frames[0].cmd)
        assertTrue(parser.lenErrors >= 1)
    }

    @Test
    fun `未知上报码不会被当成 ACK`() {
        val frame = Frame(Protocol.Report.HEARTBEAT, 9, byteArrayOf(1, 0, 0))
        assertNull(frame.ackedSeq)
        assertNull(frame.result)
        assertEquals(false, frame.isAck)
    }
}
