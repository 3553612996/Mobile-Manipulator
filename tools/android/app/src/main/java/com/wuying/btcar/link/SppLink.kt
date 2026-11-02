package com.wuying.btcar.link

import android.annotation.SuppressLint
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothSocket
import com.wuying.btcar.protocol.Frame
import com.wuying.btcar.protocol.FrameParser
import com.wuying.btcar.protocol.Protocol
import kotlinx.coroutines.CompletableDeferred
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import kotlinx.coroutines.withContext
import kotlinx.coroutines.withTimeoutOrNull
import java.io.IOException
import java.io.OutputStream
import java.util.UUID
import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.atomic.AtomicInteger

/**
 * 经典蓝牙 SPP 链路：连接、收帧、发帧、等 ACK。
 *
 * 线程约定：
 *   - 读循环跑在 IO 线程，只做"拆包 + 分发"，不碰 UI；
 *   - 所有写操作串行化在一把协程锁后面，一帧不会被另一帧插队；
 *   - 等待 ACK 用 CompletableDeferred，读线程收到 ACK 时唤醒发送方。
 */
class SppLink(private val scope: CoroutineScope) {

    sealed interface LinkState {
        data object Idle : LinkState
        data class Connecting(val name: String) : LinkState
        data class Connected(val name: String, val address: String) : LinkState
        data class Failed(val message: String) : LinkState
    }

    private val _state = MutableStateFlow<LinkState>(LinkState.Idle)
    val state: StateFlow<LinkState> = _state.asStateFlow()

    /** 非 ACK 帧回调，在 IO 线程执行，只更新状态，不做重活。 */
    var onFrame: ((Frame) -> Unit)? = null

    /** 连接级别提示（断开、写失败），在 IO 线程执行。 */
    var onNotice: ((String) -> Unit)? = null

    val parser = FrameParser()

    private val waiters = ConcurrentHashMap<Int, CompletableDeferred<Int>>()
    private val seqCounter = AtomicInteger(0)
    private val writeLock = Mutex()

    @Volatile
    private var socket: BluetoothSocket? = null

    @Volatile
    private var output: OutputStream? = null

    private var readJob: Job? = null

    val isConnected: Boolean get() = socket?.isConnected == true

    @SuppressLint("MissingPermission")
    suspend fun connect(device: BluetoothDevice) = withContext(Dispatchers.IO) {
        closeSocket()
        parser.reset()

        val name = runCatching { device.name }.getOrNull() ?: device.address
        _state.value = LinkState.Connecting(name)

        val uuid = UUID.fromString(Protocol.SPP_UUID)
        var sock: BluetoothSocket? = null
        var lastError = "未知错误"

        try {
            sock = device.createRfcommSocketToServiceRecord(uuid)
            sock.connect()
        } catch (first: Exception) {
            lastError = first.message ?: first.javaClass.simpleName
            runCatching { sock?.close() }
            sock = null

            /*
             * HC-05/HC-06 的 SDP 记录常常不完整，安全套接字握手失败时
             * 退回不安全套接字：已经配对过的模块走这条基本都能连上。
             */
            delay(300)
            var fallback: BluetoothSocket? = null
            try {
                fallback = device.createInsecureRfcommSocketToServiceRecord(uuid)
                fallback.connect()
                sock = fallback
            } catch (second: Exception) {
                lastError = second.message ?: second.javaClass.simpleName
                runCatching { fallback?.close() }
                sock = null
            }
        }

        if (sock == null) {
            _state.value = LinkState.Failed("连接失败：$lastError")
            return@withContext
        }

        socket = sock
        output = runCatching { sock.outputStream }.getOrNull()
        if (output == null) {
            runCatching { sock.close() }
            socket = null
            _state.value = LinkState.Failed("无法打开输出流")
            return@withContext
        }

        _state.value = LinkState.Connected(name, device.address)
        readJob = scope.launch(Dispatchers.IO) { readLoop(sock) }
    }

    fun disconnect() {
        val wasActive = socket != null || _state.value is LinkState.Connected
        closeSocket()
        failWaiters()
        if (wasActive) _state.value = LinkState.Idle
    }

    /**
     * 发一帧。needAck 时等 ACK 的结果码，超时按 retries 重发。
     * 返回 null 表示链路断了或一直没有 ACK；调用方必须把 null 当成失败。
     */
    suspend fun send(
        cmd: Int,
        payload: ByteArray = ByteArray(0),
        needAck: Boolean = true,
        timeoutMs: Long = 400,
        retries: Int = 2
    ): Int? {
        val out = output ?: return null
        var attempt = 0

        while (true) {
            val seq = nextSeq()
            val waiter = if (needAck) CompletableDeferred<Int>() else null
            if (waiter != null) waiters[seq] = waiter

            try {
                writeLock.withLock {
                    out.write(Protocol.build(cmd, seq, payload))
                    out.flush()
                }
            } catch (e: IOException) {
                if (waiter != null) waiters.remove(seq)
                onLinkLost("发送失败：${e.message}")
                return null
            }

            if (waiter == null) return Protocol.Result.OK

            val result = withTimeoutOrNull(timeoutMs) { waiter.await() }
            waiters.remove(seq)
            if (result != null) return result
            if (attempt >= retries) return null
            attempt++
        }
    }

    // ------------------------------------------------------------ 内部

    private fun readLoop(sock: BluetoothSocket) {
        val input = try {
            sock.inputStream
        } catch (e: IOException) {
            onLinkLost("无法打开输入流：${e.message}")
            return
        }

        val buffer = ByteArray(256)
        try {
            // closeSocket() 会把 socket 置空，这里用它判断读循环该不该退出。
            while (socket === sock) {
                val n = input.read(buffer)
                if (n < 0) {
                    if (socket === sock) onLinkLost("对端关闭了连接")
                    return
                }
                if (n > 0) {
                    parser.feed(buffer, n) { frame -> dispatch(frame) }
                }
            }
        } catch (e: IOException) {
            if (socket === sock) onLinkLost("连接中断：${e.message}")
        }
    }

    private fun dispatch(frame: Frame) {
        if (frame.isAck) {
            val seq = frame.ackedSeq ?: return
            waiters.remove(seq)?.complete(frame.result ?: 0)
            return
        }
        onFrame?.invoke(frame)
    }

    private fun nextSeq(): Int {
        // 跳过还有等待者的序号，避免迟到的 ACK 叫醒错误的发送方。
        repeat(256) {
            val candidate = seqCounter.getAndIncrement() and 0xFF
            if (!waiters.containsKey(candidate)) return candidate
        }
        return seqCounter.getAndIncrement() and 0xFF
    }

    private fun onLinkLost(message: String) {
        val wasActive = socket != null
        closeSocket()
        failWaiters()
        if (wasActive && _state.value !is LinkState.Failed) {
            _state.value = LinkState.Failed(message)
            onNotice?.invoke(message)
        }
    }

    private fun closeSocket() {
        readJob?.cancel()
        readJob = null
        runCatching { socket?.close() }
        socket = null
        output = null
    }

    private fun failWaiters() {
        waiters.keys.toList().forEach { key ->
            waiters.remove(key)?.let { if (!it.isCompleted) it.complete(-1) }
        }
    }
}
