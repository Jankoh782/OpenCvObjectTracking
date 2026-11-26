package io.github.chayanforyou.tracking

import android.Manifest
import android.annotation.SuppressLint
import android.content.pm.PackageManager
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Point
import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.os.Build
import android.os.Bundle
import android.view.MotionEvent
import android.view.Surface
import android.view.WindowManager
import android.widget.Button
import android.widget.TextView
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import com.google.android.material.floatingactionbutton.FloatingActionButton
import com.google.ar.core.ArCoreApk
import com.google.ar.core.Config
import com.google.ar.core.Session
import com.google.ar.core.TrackingState
import org.opencv.android.OpenCVLoader
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
    private lateinit var btnClearMap: Button

    private var arSession: Session? = null
    private val backgroundRenderer = BackgroundRenderer()
    private var installRequested = false

    private var drawing = Drawing.DRAWING
    private val pointStart = Point(0, 0)
    private val pointEnd = Point(0, 0)
    private var isTargetLocked = false
    private var lastGamepadCmd: String = ""
    private var isScanning3D = false
    private var frameCounter = 0
    private val plyExporter = PlyExporter()
    private val pointCloudRenderer = PointCloudRenderer()
    private lateinit var btHelper: BluetoothHelper
    private lateinit var trackerEngine: ArTrackerEngine

    private enum class Drawing { DRAWING, TRACKING, CLEAR }

    private val permissionLauncher =
        registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { permissions ->
            if (permissions.get(Manifest.permission.CAMERA) != true) {
                Toast.makeText(this, "Aplikácia vyžaduje povolenie pre kameru!", Toast.LENGTH_LONG).show()
            }
        }

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
        btnClearMap = findViewById(R.id.btnClearMap)
        btnClearMap.setOnClickListener {
            clearEverything()
        }

        btHelper = BluetoothHelper(this) { connected, deviceName ->
            btnBluetooth.text = if (connected) "BT: $deviceName" else "Pripojiť BT (Arduino)"
            updateStatusText()
        }
        trackerEngine = ArTrackerEngine(this, trackingOverlay, tvCoordinates, btHelper, plyExporter)

        surfaceView.preserveEGLContextOnPause = true
        surfaceView.setEGLContextClientVersion(2)
        surfaceView.setEGLConfigChooser(8, 8, 8, 8, 16, 0)
        surfaceView.setRenderer(this)
        surfaceView.renderMode = GLSurfaceView.RENDERMODE_CONTINUOUSLY

        setupOverlayCallbacks()

        selectTracker.setOnClickListener { trackerEngine.showTrackerDialog() }
        btnBluetooth.setOnClickListener { btHelper.showDevicesDialog { requestAppPermissions() } }
        btnScanToggle.setOnClickListener { toggle3DScanning() }
        btnExportPly.setOnClickListener {
            isScanning3D = false
            btnScanToggle.text = "Štart 3D skenu"
            plyExporter.exportToDownloads(this)
        }

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

                    if (drawing == Drawing.TRACKING && pointEnd.x > pointStart.x) {
                        val cx = (pointStart.x + pointEnd.x) / 2f
                        val cy = (pointStart.y + pointEnd.y) / 2f
                        val redPaint = Paint().apply {
                            color = Color.RED
                            style = Paint.Style.FILL
                        }
                        canvas.drawCircle(cx, cy, 18f, redPaint)
                    }
                }
            }
        })


        var lastTapTime = 0L

        trackingOverlay.setOnTouchListener { _, event ->
            val x = event.x.toInt()
            val y = event.y.toInt()
            when (event.action and MotionEvent.ACTION_MASK) {
                MotionEvent.ACTION_DOWN -> {
                    val now = System.currentTimeMillis()
                    if (isTargetLocked && (now - lastTapTime < 350)) {
                        // Dvojité ťuknutie jedným prstom zruší výber
                        cancelTargetSelection()
                    } else if (!isTargetLocked) {
                        drawing = Drawing.DRAWING
                        pointStart.set(x, y)
                        pointEnd.set(x, y)
                        trackingOverlay.invalidate()
                    }
                    lastTapTime = now
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
                    if (!isTargetLocked) {
                        // Zamknutie cieľa druhým prstom
                        isTargetLocked = true
                        drawing = Drawing.DRAWING
                        trackingOverlay.invalidate()
                        selectTracker.hide()
                        Toast.makeText(this, "Cieľ ZAMKNUTÝ (2 prsty / dvojklik = zrušiť)", Toast.LENGTH_SHORT).show()
                    } else {
                        // Zrušenie výberu druhým prstom
                        cancelTargetSelection()
                      }
                }
            }
            true
        }
    }

    private fun cancelTargetSelection() {
        isTargetLocked = false
        drawing = Drawing.CLEAR
        pointStart.set(0, 0)
        pointEnd.set(0, 0)
        trackingOverlay.invalidate()
        selectTracker.show()
        tvCoordinates.text = "3D Poloha: Výber zrušený"
        // Pošleme robotu príkaz na zastavenie (stred X=120, plocha A=90 -> stopMotors())
        btHelper.sendCoordinates(120, 120, 90)
        Toast.makeText(this, "Výber ZRUŠENÝ", Toast.LENGTH_SHORT).show()
    }

                          private fun clearEverything() {
        isScanning3D = false
        btnScanToggle.text = "Štart 3D skenu"
        plyExporter.clear()
        trackerEngine.resetTracker()
        isTargetLocked = false
        drawing = Drawing.CLEAR
        pointStart.set(0, 0)
        pointEnd.set(0, 0)
        trackingOverlay.invalidate()
        selectTracker.show()
        tvCoordinates.text = "3D Poloha: Obrazovka vyčistená"
        updateStatusText(true)
        btHelper.sendCoordinates(120, 120, 90)
        Toast.makeText(this, "Mapa a obrazovka vyčistené!", Toast.LENGTH_SHORT).show()
    }
    override fun onResume() {
        super.onResume()
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) return
        if (arSession == null) {
            try {
                if (ArCoreApk.getInstance().requestInstall(this, !installRequested) == ArCoreApk.InstallStatus.INSTALL_REQUESTED) {
                    installRequested = true
                    return
                }
                arSession = Session(this).apply {
                                    val filter = com.google.ar.core.CameraConfigFilter(this).apply {
                    targetFps = java.util.EnumSet.of(com.google.ar.core.CameraConfig.TargetFps.TARGET_FPS_30)
                }
                val cameraConfigList = getSupportedCameraConfigs(filter)
                if (cameraConfigList.isNotEmpty()) {
                    cameraConfig = cameraConfigList[0]
                }

                    val config = Config(this)
                   if (isDepthModeSupported(Config.DepthMode.AUTOMATIC)) config.depthMode = Config.DepthMode.AUTOMATIC
                    config.updateMode = Config.UpdateMode.LATEST_CAMERA_IMAGE
                    config.focusMode = Config.FocusMode.AUTO
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
        } catch (_: Exception) {
        }
    }

    override fun onPause() {
        super.onPause()
        surfaceView.onPause()
        arSession?.pause()
    }

    override fun onDestroy() {
        arSession?.close()
        arSession = null
        trackerEngine.release()
        btHelper.close()
        super.onDestroy()
    }

    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        GLES20.glClearColor(0.1f, 0.1f, 0.1f, 1.0f)
        backgroundRenderer.createOnGlThread()
        pointCloudRenderer.createOnGlThread()
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
        GLES20.glViewport(0, 0, width, height)
        val rotation = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) display?.rotation ?: Surface.ROTATION_0
        else @Suppress("DEPRECATION") (getSystemService(WINDOW_SERVICE) as WindowManager).defaultDisplay.rotation
        arSession?.setDisplayGeometry(rotation, width, height)
    }

    override fun onDrawFrame(gl: GL10?) {
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)
        val session = arSession ?: return
        try {
            session.setCameraTextureName(backgroundRenderer.textureId)
                      val frame = session.update()
            backgroundRenderer.draw(frame)

                    pointCloudRenderer.draw(frame.camera, plyExporter.pointCloudList)
         frameCounter++
                 if (isScanning3D && frameCounter % 3 == 0) {
            val isTracking = frame.camera.trackingState == TrackingState.TRACKING
            if (isTracking) {
                plyExporter.collectFromFrame(frame)
            } else {
              plyExporter.debugInfo = "Stav AR: ${frame.camera.trackingState} (${frame.camera.trackingFailureReason})"
            }
            runOnUiThread { updateStatusText(isTracking) }
        }

            if (isTargetLocked) {
                trackerEngine.processFrame(frame, pointStart, pointEnd, drawing == Drawing.DRAWING) {
                    drawing = Drawing.TRACKING
                }
            } else {
                trackerEngine.resetTracker()
            }

                } catch (_: Exception) {
        }
         }
    
    private fun toggle3DScanning() {
        isScanning3D = !isScanning3D
        if (isScanning3D) {
            plyExporter.clear()
            btnScanToggle.text = "Stop 3D sken"
            Toast.makeText(this, "Skenovanie spustené – hýbte mobilom", Toast.LENGTH_SHORT).show()
        } else {
            btnScanToggle.text = "Štart 3D skenu"
            Toast.makeText(this, "Skenovanie zastavené (${plyExporter.pointCloudList.size} bodov)", Toast.LENGTH_SHORT).show()
        }
        updateStatusText()
    }

 private fun updateStatusText(arReady: Boolean = true) {
     val btText = if (btHelper.isConnected) "Pripojené" else "Odpojené"
     val arText = if (arReady) "" else " (Hýbte mobilom!)"
            tvScanStatus.text = "3D Mapa: ${plyExporter.debugInfo}$arText | BT: $btText"
 }
    override fun onGenericMotionEvent(event: MotionEvent): Boolean {
        if ((event.source and android.view.InputDevice.SOURCE_JOYSTICK) == android.view.InputDevice.SOURCE_JOYSTICK &&
            event.action == MotionEvent.ACTION_MOVE
        ) {
            val x = event.getAxisValue(MotionEvent.AXIS_X) + event.getAxisValue(MotionEvent.AXIS_HAT_X)
            val y = event.getAxisValue(MotionEvent.AXIS_Y) + event.getAxisValue(MotionEvent.AXIS_HAT_Y)

            val cmd = when {
                y < -0.5f -> "g" // vpred()
                y > 0.5f  -> "b" // vzad()
                x < -0.5f -> "L" // vlavo()
                x > 0.5f  -> "R" // right()
                else      -> "s" // stop()
            }

            if (cmd != lastGamepadCmd) {
                lastGamepadCmd = cmd
                btHelper.sendCommand(cmd)
            }
            return true
        }
        return super.onGenericMotionEvent(event)
    }

    override fun onKeyDown(keyCode: Int, event: android.view.KeyEvent): Boolean {
        if ((event.source and android.view.InputDevice.SOURCE_GAMEPAD) == android.view.InputDevice.SOURCE_GAMEPAD) {
            val cmd = when (keyCode) {
                android.view.KeyEvent.KEYCODE_BUTTON_R1 -> "h" // lift() - R1
                android.view.KeyEvent.KEYCODE_BUTTON_L1 -> "d" // dole() - L1
                android.view.KeyEvent.KEYCODE_BUTTON_Y  -> "l" // LEDS_ON() - Trojuholník
                android.view.KeyEvent.KEYCODE_BUTTON_B  -> "t" // LEDS_OFF() - Krúžok
                android.view.KeyEvent.KEYCODE_BUTTON_A  -> "s" // stop() - Krížik
                else -> null
            }
            if (cmd != null) {
                lastGamepadCmd = cmd
                btHelper.sendCommand(cmd)
                return true
            }
        }
        return super.onKeyDown(keyCode, event)
    }

}

