package io.github.chayanforyou.tracking

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothSocket
import android.content.ContentValues
import android.content.pm.PackageManager
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Point
import android.media.Image
import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.os.Build
import android.os.Bundle
import android.os.Environment
import android.provider.MediaStore
import android.view.Gravity
import android.view.MotionEvent
import android.view.Surface
import android.view.WindowManager
import android.widget.Button
import android.widget.LinearLayout
import android.widget.RadioButton
import android.widget.RadioGroup
import android.widget.TextView
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import com.google.android.material.dialog.MaterialAlertDialogBuilder
import com.google.android.material.floatingactionbutton.FloatingActionButton
import com.google.ar.core.ArCoreApk
import com.google.ar.core.Config
import com.google.ar.core.Frame
import com.google.ar.core.Session
import com.google.ar.core.TrackingState
import org.opencv.android.OpenCVLoader
import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.Rect2d
import org.opencv.imgproc.Imgproc
import org.opencv.tracking.legacy_Tracker
import org.opencv.tracking.legacy_TrackerCSRT
import org.opencv.tracking.legacy_TrackerKCF
import org.opencv.tracking.legacy_TrackerMIL
import org.opencv.tracking.legacy_TrackerMOSSE
import org.opencv.tracking.legacy_TrackerMedianFlow
import org.opencv.tracking.legacy_TrackerTLD
import java.io.OutputStream
import java.nio.ByteOrder
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.UUID
import java.util.concurrent.CopyOnWriteArrayList
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10

class MainActivity : AppCompatActivity(), GLSurfaceView.Renderer {

    private lateinit var surfaceView: GLSurfaceView
    private lateinit var trackingOverlay: OverlayView
    private lateinit var selectTracker: FloatingActionButton
    private lateinit var tvCoordinates: TextView
    private lateinit var tvScanStatus: TextView
    private lateinit var btnBluetooth: Button
    private lateinit var btnScanToggle: Button
    private lateinit var btnExportPly: Button

    // ARCore
    private var arSession: Session? = null
    private val backgroundRenderer = BackgroundRenderer()
    private var installRequested = false

    // OpenCV Tracking
    private var imageGrab: Mat? = null
    private var tracker: legacy_Tracker? = null
    private var drawing = Drawing.DRAWING
    private var selectedTracker = TrackerType.CSRT
    private val pointStart = Point(0, 0)
    private val pointEnd = Point(0, 0)
    private var isTargetLocked = false

    // 3D Mapovanie (.ply)
    private var isScanning3D = false
    private val pointCloudList = CopyOnWriteArrayList<FloatArray>()
    private var frameCounter = 0

    // Bluetooth (HC-05 / HC-06 / ESP32 SPP)
    private val sppUuid: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")
    private var btSocket: BluetoothSocket? = null
    private var btOutputStream: OutputStream? = null
    private var isBtConnected = false
    private var lastBtSendTime = 0L

    private enum class Drawing { DRAWING, TRACKING, CLEAR }
    private enum class TrackerType { MEDIAN_FLOW, CSRT, KCF, MOSSE, TLD, MIL }

    private val permissionLauncher =
        registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { permissions ->
            if (permissions[Manifest.permission.CAMERA] != true) {
                Toast.makeText(this, "Aplikácia vyžaduje povolenie pre kameru!", Toast.LENGTH_LONG).show()
            }
        }

    @SuppressLint("ClickableViewAccessibility")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        if (!OpenCVLoader.initLocal()) {
            Toast.makeText(this, "Chyba inicializácie OpenCV!", Toast.LENGTH_LONG).show()
            return
        }
        setContentView(R.layout.activity_main)

        surfaceView = findViewById(R.id.surfaceview)
        trackingOverlay = findViewById(R.id.tracking_overlay)
        selectTracker = findViewById(R.id.select_tracker)
        tvCoordinates = findViewById(R.id.tv_coordinates)
        tvScanStatus = findViewById(R.id.tv_scan_status)
        btnBluetooth = findViewById(R.id.btn_bluetooth)
        btnScanToggle = findViewById(R.id.btn_scan_toggle)
        btnExportPly = findViewById(R.id.btn_export_ply)

        surfaceView.preserveEGLContextOnPause = true
        surfaceView.setEGLContextClientVersion(2)
        surfaceView.setEGLConfigChooser(8, 8, 8, 8, 16, 0)
        surfaceView.setRenderer(this)
        surfaceView.renderMode = GLSurfaceView.RENDERMODE_CONTINUOUSLY

        setupOverlayCallbacks()

        selectTracker.setOnClickListener { selectTracker() }
        btnBluetooth.setOnClickListener { showBluetoothDevicesDialog() }
        btnScanToggle.setOnClickListener { toggle3DScanning() }
        btnExportPly.setOnClickListener { exportPointCloudToPly() }

        requestAppPermissions()
    }

    private fun requestAppPermissions() {
        val perms = mutableListOf(Manifest.permission.CAMERA)
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            perms.add(Manifest.permission.BLUETOOTH_CONNECT)
            perms.add(Manifest.permission.BLUETOOTH_SCAN)
        }
        permissionLauncher.launch(perms.toTypedArray())
    }

    @SuppressLint("ClickableViewAccessibility")
    private fun setupOverlayCallbacks() {
        trackingOverlay.addCallback(object : OverlayView.DrawCallback {
            override fun drawCallback(canvas: Canvas) {
                if (drawing != Drawing.CLEAR) {
                    val paint = Paint().apply {
                        color = if (drawing == Drawing.TRACKING) Color.GREEN else Color.BLUE
                        strokeWidth = 8f
                        style = Paint.Style.STROKE
                    }
                    canvas.drawRect(
                        pointStart.x.toFloat(), pointStart.y.toFloat(),
                        pointEnd.x.toFloat(), pointEnd.y.toFloat(), paint
                    )
                    if (drawing == Drawing.TRACKING) {
                        val cx = (pointStart.x + pointEnd.x) / 2f
                        val cy = (pointStart.y + pointEnd.y) / 2f
                        canvas.drawLine(cx, 0f, cx, trackingOverlay.height.toFloat(), paint)
                        canvas.drawLine(0f, cy, trackingOverlay.width.toFloat(), cy, paint)
                    }
                }
            }
        })

        trackingOverlay.setOnTouchListener { _, event ->
            val x = event.x.toInt()
            val y = event.y.toInt()
            when (event.action and MotionEvent.ACTION_MASK) {
                MotionEvent.ACTION_DOWN -> if (!isTargetLocked) {
                    drawing = Drawing.DRAWING
                    pointStart.set(x, y)
                    pointEnd.set(x, y)
                    trackingOverlay.invalidate()
                }
                MotionEvent.ACTION_MOVE -> if (!isTargetLocked) {
                    pointEnd.set(x, y)
                    trackingOverlay.invalidate()
                }
                MotionEvent.ACTION_UP -> if (!isTargetLocked) {
                    drawing = Drawing.CLEAR
                    trackingOverlay.invalidate()
                }
                MotionEvent.ACTION_POINTER_DOWN -> {
                    isTargetLocked = !isTargetLocked
                    drawing = Drawing.DRAWING
                    trackingOverlay.invalidate()
                    if (isTargetLocked) {
                        selectTracker.hide()
                        Toast.makeText(this, "Cieľ ZAMKNUTÝ (2 prsty = odomknúť)", Toast.LENGTH_SHORT).show()
                    } else {
                        selectTracker.show()
                        tvCoordinates.text = "3D Poloha: Čakám na výber objektu..."
                        Toast.makeText(this, "Cieľ ODOMKNUTÝ", Toast.LENGTH_SHORT).show()
                    }
                }
            }
            true
        }
    }

    override fun onResume() {
        super.onResume()
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            return
        }
        if (arSession == null) {
            try {
                when (ArCoreApk.getInstance().requestInstall(this, !installRequested)) {
                    ArCoreApk.InstallStatus.INSTALL_REQUESTED -> {
                        installRequested = true
                        return
                    }
                    ArCoreApk.InstallStatus.INSTALLED -> {}
                }
                arSession = Session(this).apply {
                    val config = Config(this)
                    if (isDepthModeSupported(Config.DepthMode.AUTOMATIC)) {
                        config.depthMode = Config.DepthMode.AUTOMATIC
                    }
                    config.updateMode = Config.UpdateMode.LATEST_CAMERA_IMAGE
                    configure(config)
                }
            } catch (e: Exception) {
                Toast.makeText(this, "ARCore chyba: ${e.message}", Toast.LENGTH_LONG).show()
                return
            }
        }
        try {
            arSession?.resume()
            surfaceView.onResume()
        } catch (e: Exception) {
            e.printStackTrace()
        }
