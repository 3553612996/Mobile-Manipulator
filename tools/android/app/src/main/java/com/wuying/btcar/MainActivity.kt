package com.wuying.btcar

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.compose.setContent
import androidx.activity.result.ActivityResultLauncher
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.heightIn
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp
import androidx.core.content.ContextCompat
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.compose.LifecycleEventEffect
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.lifecycle.viewmodel.compose.viewModel
import com.wuying.btcar.ui.BtCarTheme
import com.wuying.btcar.ui.ControlScreen

class MainActivity : ComponentActivity() {

    private lateinit var enableBluetooth: ActivityResultLauncher<Intent>
    private var adapter: BluetoothAdapter? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        adapter = getSystemService(BluetoothManager::class.java)?.adapter
        enableBluetooth = registerForActivityResult(ActivityResultContracts.StartActivityForResult()) { }

        setContent {
            BtCarTheme {
                val vm: BtCarViewModel = viewModel()
                AppRoot(viewModel = vm, adapter = adapter, requestEnable = ::requestEnable)
            }
        }
    }

    private fun requestEnable() {
        runCatching { enableBluetooth.launch(Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE)) }
    }
}

@Composable
private fun AppRoot(
    viewModel: BtCarViewModel,
    adapter: BluetoothAdapter?,
    requestEnable: () -> Unit
) {
    val context = LocalContext.current
    val state by viewModel.ui.collectAsStateWithLifecycle()
    var showPicker by remember { mutableStateOf(false) }
    var notice by remember { mutableStateOf<String?>(null) }

    val permissionLauncher = rememberLauncherForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { granted ->
        if (granted.values.all { it }) {
            if (adapter.isEnabledSafe()) {
                showPicker = true
            } else {
                requestEnable()
            }
        } else {
            notice = "缺少蓝牙权限，无法连接小车"
        }
    }

    val openPicker: () -> Unit = {
        val local = adapter
        if (local == null) {
            notice = "本机没有蓝牙适配器"
        } else if (!local.isEnabledSafe()) {
            requestEnable()
        } else {
            val missing = requiredPermissions().filter {
                ContextCompat.checkSelfPermission(context, it) != PackageManager.PERMISSION_GRANTED
            }
            if (missing.isEmpty()) {
                showPicker = true
            } else {
                permissionLauncher.launch(missing.toTypedArray())
            }
        }
    }

    // 切到后台就失能，避免手机锁屏后车还在按最后一帧速度跑。
    LifecycleEventEffect(Lifecycle.Event.ON_STOP) {
        viewModel.onBackground()
    }

    ControlScreen(
        state = state,
        onToggleLink = { if (state.connected) viewModel.disconnect() else openPicker() },
        onStick = viewModel::setStick,
        onStickRelease = viewModel::releaseStick,
        onMaxSpeed = viewModel::setMaxSpeed,
        onToggleEnable = { viewModel.setEnabled(!state.enabled) },
        onEmergencyStop = viewModel::emergencyStop,
        onClearFault = viewModel::clearFault,
        onDismissMessage = viewModel::clearMessage
    )

    notice?.let { text ->
        AlertDialog(
            onDismissRequest = { notice = null },
            title = { Text("提示") },
            text = { Text(text) },
            confirmButton = {
                TextButton(onClick = { notice = null }) { Text("知道了") }
            }
        )
    }

    if (showPicker) {
        DevicePickerDialog(
            adapter = adapter,
            onSelect = { device ->
                showPicker = false
                viewModel.connect(device)
            },
            onDismiss = { showPicker = false }
        )
    }
}

@SuppressLint("MissingPermission")
@Composable
private fun DevicePickerDialog(
    adapter: BluetoothAdapter?,
    onSelect: (BluetoothDevice) -> Unit,
    onDismiss: () -> Unit
) {
    val context = LocalContext.current
    val paired = remember(adapter) { adapter?.bondedDevices?.toList().orEmpty() }
    var found by remember { mutableStateOf<List<BluetoothDevice>>(emptyList()) }
    var discovering by remember { mutableStateOf(false) }

    DisposableEffect(adapter) {
        val receiver = object : BroadcastReceiver() {
            override fun onReceive(ctx: Context?, intent: Intent?) {
                when (intent?.action) {
                    BluetoothDevice.ACTION_FOUND -> {
                        val device = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                            intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE, BluetoothDevice::class.java)
                        } else {
                            @Suppress("DEPRECATION")
                            intent.getParcelableExtra<BluetoothDevice>(BluetoothDevice.EXTRA_DEVICE)
                        }
                        if (device != null) {
                            found = (found + device).distinctBy { it.address }
                        }
                    }
                    BluetoothAdapter.ACTION_DISCOVERY_FINISHED -> discovering = false
                }
            }
        }
        ContextCompat.registerReceiver(
            context,
            receiver,
            IntentFilter().apply {
                addAction(BluetoothDevice.ACTION_FOUND)
                addAction(BluetoothAdapter.ACTION_DISCOVERY_FINISHED)
            },
            ContextCompat.RECEIVER_NOT_EXPORTED
        )
        onDispose {
            runCatching { context.unregisterReceiver(receiver) }
            runCatching { adapter?.cancelDiscovery() }
        }
    }

    val devices = (paired + found).distinctBy { it.address }

    AlertDialog(
        onDismissRequest = onDismiss,
        title = { Text("选择蓝牙模块") },
        text = {
            Column(Modifier.heightIn(max = 380.dp)) {
                Text(
                    "先在系统蓝牙设置里配对 HC-05 / HC-06，再从下面选择。",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant
                )
                Spacer(Modifier.height(8.dp))
                if (devices.isEmpty()) {
                    Text(
                        text = if (discovering) "正在搜索…" else "没有找到设备，点右下角搜索",
                        style = MaterialTheme.typography.bodySmall
                    )
                } else {
                    LazyColumn {
                        items(devices, key = { it.address }) { device ->
                            Column(
                                Modifier
                                    .fillMaxWidth()
                                    .clickable { onSelect(device) }
                                    .padding(vertical = 10.dp)
                            ) {
                                Text(device.displayName(), style = MaterialTheme.typography.bodyLarge)
                                Text(
                                    text = device.address + if (device.isBondedPair()) " · 已配对" else "",
                                    style = MaterialTheme.typography.bodySmall,
                                    color = MaterialTheme.colorScheme.onSurfaceVariant
                                )
                            }
                        }
                    }
                }
            }
        },
        confirmButton = {
            TextButton(
                onClick = {
                    val local = adapter ?: return@TextButton
                    if (!runCatching { local.isDiscovering }.getOrDefault(false)) {
                        discovering = true
                        runCatching { local.startDiscovery() }
                            .onFailure { discovering = false }
                    }
                }
            ) {
                Text(if (discovering) "搜索中…" else "搜索设备")
            }
        },
        dismissButton = {
            TextButton(onClick = onDismiss) { Text("取消") }
        }
    )
}

@SuppressLint("MissingPermission")
private fun BluetoothAdapter?.isEnabledSafe(): Boolean =
    this != null && runCatching { isEnabled }.getOrDefault(false)

@SuppressLint("MissingPermission")
private fun BluetoothDevice.displayName(): String =
    runCatching { name }.getOrNull()?.takeIf { it.isNotBlank() } ?: "未知设备"

@SuppressLint("MissingPermission")
private fun BluetoothDevice.isBondedPair(): Boolean =
    runCatching { bondState == BluetoothDevice.BOND_BONDED }.getOrDefault(false)

private fun requiredPermissions(): List<String> =
    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
        listOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT)
    } else {
        listOf(Manifest.permission.ACCESS_FINE_LOCATION)
    }
