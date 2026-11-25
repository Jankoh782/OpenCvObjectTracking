package io.github.chayanforyou.tracking

import android.Manifest
import android.annotation.SuppressLint
import android.app.Activity
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothSocket
import android.content.Context
import android.content.pm.PackageManager
import android.os.Build
import android.widget.Toast
import androidx.core.app.ActivityCompat
import com.google.android.material.dialog.MaterialAlertDialogBuilder
import java.io.OutputStream
import java.util.Locale
import java.util.UUID

class BluetoothHelper(
    private val activity: Activity,
    private val onStatusChanged: (connected: Boolean, deviceName: String) -> Unit
) {
    private val sppUuid: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")
    private var btSocket: BluetoothSocket? = null
    private var btOutputStream: OutputStream? = null
    var isConnected = false
        private set
    private var lastSendTime = 0L

    fun sendCoordinates(outX: Int, outY: Int, outA: Int) {
        if (!isConnected || btOutputStream == null) return
        val now = System.currentTimeMillis()
        if (now - lastSendTime < 50) return
        lastSendTime = now

        val msg = String.format(Locale.US, "X%dY%dA%d\n", outX, outY, outA)
        try {
            btOutputStream?.write(msg.toByteArray())
        } catch (_: Exception) {
            isConnected = false
            activity.runOnUiThread { onStatusChanged(false, "") }
        }
    }
        fun sendCommand(cmd: String) {
        if (!isConnected || btOutputStream == null) return
        try {
            btOutputStream?.write(cmd.toByteArray())
            btOutputStream?.flush()
        } catch (_: Exception) {
            isConnected = false
            activity.runOnUiThread { onStatusChanged(false, "") }
        }
    }

    @SuppressLint("MissingPermission")
    fun showDevicesDialog(onRequestPermissions: () -> Unit) {
        val btManager = activity.getSystemService(Context.BLUETOOTH_SERVICE) as? BluetoothManager
        val adapter: BluetoothAdapter? = btManager?.adapter
        if (adapter == null || !adapter.isEnabled) {
            Toast.makeText(activity, "Zapnite Bluetooth v telefóne!", Toast.LENGTH_LONG).show()
            return
        }

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S &&
            ActivityCompat.checkSelfPermission(activity, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED
        ) {
            onRequestPermissions()
            return
        }

        val pairedDevices: Set<BluetoothDevice> = adapter.bondedDevices
        if (pairedDevices.isEmpty()) {
            Toast.makeText(activity, "Najprv spárujte ESP32 / HC-05 v nastaveniach mobilu!", Toast.LENGTH_LONG).show()
            return
        }

        val devicesList = pairedDevices.toList()
        val deviceNames = devicesList.map { "${it.name} (${it.address})" }.toTypedArray()

        MaterialAlertDialogBuilder(activity)
            .setTitle("Vyberte Arduino Bluetooth")
            .setItems(deviceNames) { _, which ->
                connectToDevice(devicesList.get(which))
            }
            .show()
    }

    @SuppressLint("MissingPermission")
    private fun connectToDevice(device: BluetoothDevice) {
        Toast.makeText(activity, "Pripájam k ${device.name}...", Toast.LENGTH_SHORT).show()
        Thread {
            try {
                btSocket?.close()
                val socket = device.createRfcommSocketToServiceRecord(sppUuid)
                socket.connect()
                btSocket = socket
                btOutputStream = socket.outputStream
                isConnected = true
                activity.runOnUiThread {
                    onStatusChanged(true, device.name ?: "Arduino")
                    Toast.makeText(activity, "Pripojené k Arduinu!", Toast.LENGTH_SHORT).show()
                }
            } catch (e: Exception) {
                isConnected = false
                activity.runOnUiThread {
                    Toast.makeText(activity, "Zlyhalo pripojenie BT: ${e.message}", Toast.LENGTH_LONG).show()
                }
            }
        }.start()
    }

    fun close() {
        try {
            btSocket?.close()
        } catch (_: Exception) {
        }
    }
}
