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
import kotlin.math.abs

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
            if (permissions.get(Manifest.permission.CAMERA) != true) {
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
### Krok 3: Hneď pod ČASŤ 1 vložte ČASŤ 2 (dolná polovica súboru)
Kliknite na úplný koniec vloženého textu v editore (za poslednú `}`), stlačte **Enter** a prilepte túto druhú polovicu:

```kotlin
    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        GLES20.glClearColor(0.1f, 0.1f, 0.1f, 1.0f)
        backgroundRenderer.createOnGlThread()
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
        GLES20.glViewport(0, 0, width, height)
        val rotation = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
            display?.rotation ?: Surface.ROTATION_0
        } else {
            @Suppress("DEPRECATION")
            (getSystemService(WINDOW_SERVICE) as WindowManager).defaultDisplay.rotation
        }
        arSession?.setDisplayGeometry(rotation, width, height)
    }

    override fun onDrawFrame(gl: GL10?) {
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)
        val session = arSession ?: return

        try {
            session.setCameraTextureName(backgroundRenderer.textureId)
            val frame = session.update()
            backgroundRenderer.draw(frame)

            if (frame.camera.trackingState != TrackingState.TRACKING) return

            frameCounter++

            if (isScanning3D && frameCounter % 3 == 0) {
                collectPointCloud(frame)
            }

            if (isTargetLocked) {
                processOpenCvAndArCore(frame)
            } else {
                tracker?.clear()
                tracker = null
            }
        } catch (_: Exception) {
        }
    }

    private fun processOpenCvAndArCore(frame: Frame) {
        var camImage: Image? = null
        try {
            camImage = frame.acquireCameraImage()
            val mat = yuvToGrayMat(camImage)
            Core.transpose(mat, mat)
            Core.flip(mat, mat, 1)
            Imgproc.resize(mat, mat, org.opencv.core.Size(240.0, 320.0))

            imageGrab?.release()
            imageGrab = mat

            if (drawing == Drawing.DRAWING) {
                val minX = (minOf(pointStart.x.toFloat(), pointEnd.x.toFloat()) / trackingOverlay.width * mat.cols()).toInt().coerceIn(0, mat.cols() - 2)
                val minY = (minOf(pointStart.y.toFloat(), pointEnd.y.toFloat()) / trackingOverlay.height * mat.rows()).toInt().coerceIn(0, mat.rows() - 2)
                val maxX = (maxOf(pointStart.x.toFloat(), pointEnd.x.toFloat()) / trackingOverlay.width * mat.cols()).toInt().coerceIn(minX + 1, mat.cols() - 1)
                val maxY = (maxOf(pointStart.y.toFloat(), pointEnd.y.toFloat()) / trackingOverlay.height * mat.rows()).toInt().coerceIn(minY + 1, mat.rows() - 1)

                val initRect = Rect2d(minX.toDouble(), minY.toDouble(), (maxX - minX).toDouble(), (maxY - minY).toDouble())
                tracker = when (selectedTracker) {
                    TrackerType.MEDIAN_FLOW -> legacy_TrackerMedianFlow.create()
                    TrackerType.CSRT -> legacy_TrackerCSRT.create()
                    TrackerType.KCF -> legacy_TrackerKCF.create()
                    TrackerType.MOSSE -> legacy_TrackerMOSSE.create()
                    TrackerType.TLD -> legacy_TrackerTLD.create()
                    TrackerType.MIL -> legacy_TrackerMIL.create()
                }.apply {
                    init(mat, initRect)
                }
                drawing = Drawing.TRACKING
            } else {
                val rect = Rect2d()
                if (tracker?.update(mat, rect) == true) {
                    pointStart.x = (rect.x * trackingOverlay.width / mat.cols()).toInt()
                    pointStart.y = (rect.y * trackingOverlay.height / mat.rows()).toInt()
                    pointEnd.x = pointStart.x + (rect.width * trackingOverlay.width / mat.cols()).toInt()
                    pointEnd.y = pointStart.y + (rect.height * trackingOverlay.height / mat.rows()).toInt()
                    trackingOverlay.postInvalidate()

                    val centerX = (pointStart.x + pointEnd.x) / 2f
                    val centerY = (pointStart.y + pointEnd.y) / 2f

                    var posX = 0f
                    var posY = 0f
                    var posZ = getDepthAtScreenPoint(frame, centerX, centerY)

                    val hits = frame.hitTest(centerX, centerY)
                    if (hits.isNotEmpty()) {
                        val pose = hits.first().hitPose
                        posX = pose.tx()
                        posY = pose.ty()
                        if (posZ <= 0f) posZ = -pose.tz()
                    }

                    val outX = ((centerX / trackingOverlay.width) * 240f).toInt().coerceIn(0, 240)
                    val outY = ((centerY / trackingOverlay.height) * 240f).toInt().coerceIn(0, 240)
                    val outA = if (posZ > 0.05f) {
                        (80f / posZ).toInt().coerceIn(10, 250)
                    } else {
                        val boxWidth = abs(pointEnd.x - pointStart.x).toFloat()
                        ((boxWidth / trackingOverlay.width) * 200f).toInt().coerceIn(10, 250)
                    }

                    runOnUiThread {
                        tvCoordinates.text = String.format(
                            Locale.US,
                            "3D: X=%.2fm Y=%.2fm Z=%.2fm | BT: X%dY%dA%d",
                            posX, posY, posZ, outX, outY, outA
                        )
                    }

                    sendCoordinatesToArduino(outX, outY, outA)
                }
            }
        } catch (_: Exception) {
        } finally {
            camImage?.close()
        }
    }

    private fun yuvToGrayMat(image: Image): Mat {
        val yPlane = image.planes.first()
        val yBuffer = yPlane.buffer
        val rowStride = yPlane.rowStride
        val width = image.width
        val height = image.height
        val data = ByteArray(yBuffer.remaining())
        yBuffer.get(data)
        val fullMat = Mat(height, rowStride, CvType.CV_8UC1)
        fullMat.put(0, 0, data)
        val cropped = fullMat.submat(0, height, 0, width).clone()
        fullMat.release()
        return cropped
    }

    private fun getDepthAtScreenPoint(frame: Frame, screenX: Float, screenY: Float): Float {
        try {
            frame.acquireDepthImage16Bits().use { depthImage ->
                val normX = (screenX / trackingOverlay.width).coerceIn(0f, 0.99f)
                val normY = (screenY / trackingOverlay.height).coerceIn(0f, 0.99f)
                val imgX = (normY * depthImage.width).toInt().coerceIn(0, depthImage.width - 1)
                val imgY = ((1f - normX) * depthImage.height).toInt().coerceIn(0, depthImage.height - 1)

                val plane = depthImage.planes.first()
                val byteIndex = imgY * plane.rowStride + imgX * plane.pixelStride
                val buffer = plane.buffer.order(ByteOrder.nativeOrder())
                val depthMm = buffer.getShort(byteIndex).toInt() and 0xFFFF
                if (depthMm > 0) return depthMm / 1000f
            }
        } catch (_: Exception) {
        }
        return 0f
    }

    private fun collectPointCloud(frame: Frame) {
        try {
            frame.acquirePointCloud().use { pointCloud ->
                val pointsBuffer = pointCloud.points
                while (pointsBuffer.remaining() >= 4) {
                    val x = pointsBuffer.get()
                    val y = pointsBuffer.get()
                    val z = pointsBuffer.get()
                    val confidence = pointsBuffer.get()
                    if (confidence > 0.35f && pointCloudList.size < 150000) {
                        pointCloudList.add(floatArrayOf(x, y, z))
                    }
                }
                runOnUiThread { updateStatusText() }
            }
        } catch (_: Exception) {
        }
    }

    private fun toggle3DScanning() {
        isScanning3D = !isScanning3D
        if (isScanning3D) {
            pointCloudList.clear()
            btnScanToggle.text = "Stop 3D sken"
            Toast.makeText(this, "Skenovanie spustené – hýbte mobilom", Toast.LENGTH_SHORT).show()
        } else {
            btnScanToggle.text = "Štart 3D skenu"
            Toast.makeText(this, "Skenovanie zastavené (${pointCloudList.size} bodov)", Toast.LENGTH_SHORT).show()
        }
        updateStatusText()
    }

    private fun exportPointCloudToPly() {
        if (pointCloudList.isEmpty()) {
            Toast.makeText(this, "Žiadne 3D body! Najprv kliknite na 'Štart 3D skenu'.", Toast.LENGTH_LONG).show()
            return
        }
        isScanning3D = false
        btnScanToggle.text = "Štart 3D skenu"

        val timeStamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
        val fileName = "ARCore_Scan_$timeStamp.ply"

        try {
            val sb = StringBuilder()
            sb.append("ply\n")
            sb.append("format ascii 1.0\n")
            sb.append("element vertex ${pointCloudList.size}\n")
            sb.append("property float x\n")
            sb.append("property float y\n")
            sb.append("property float z\n")
            sb.append("end_header\n")

            for (pt in pointCloudList) {
                sb.append(String.format(Locale.US, "%.4f %.4f %.4f\n", pt.get(0), pt.get(1), pt.get(2)))
            }

            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
                val values = ContentValues().apply {
                    put(MediaStore.Downloads.DISPLAY_NAME, fileName)
                    put(MediaStore.Downloads.MIME_TYPE, "application/octet-stream")
                    put(MediaStore.Downloads.RELATIVE_PATH, Environment.DIRECTORY_DOWNLOADS)
                }
                val uri = contentResolver.insert(MediaStore.Downloads.EXTERNAL_CONTENT_URI, values)
                uri?.let {
                    contentResolver.openOutputStream(it)?.use { out ->
                        out.write(sb.toString().toByteArray())
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
    }

    override fun onPause() {
        super.onPause()
        surfaceView.onPause()
        arSession?.pause()
    }

    override fun onDestroy() {
        arSession?.close()
        arSession = null
        imageGrab?.release()
        tracker?.clear()
        try {
            btSocket?.close()
        } catch (_: Exception) {
        }
        super.onDestroy()
    }

    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        GLES20.glClearColor(0.1f, 0.1f, 0.1f, 1.0f)
        backgroundRenderer.createOnGlThread()
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
        GLES20.glViewport(0, 0, width, height)
        val rotation = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
            display?.rotation ?: Surface.ROTATION_0
        } else {
            @Suppress("DEPRECATION")
            (getSystemService(WINDOW_SERVICE) as WindowManager).defaultDisplay.rotation
        }
        arSession?.setDisplayGeometry(rotation, width, height)
    }

    override fun onDrawFrame(gl: GL10?) {
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)
        val session = arSession ?: return

        try {
            session.setCameraTextureName(backgroundRenderer.textureId)
            val frame = session.update()
            backgroundRenderer.draw(frame)

            if (frame.camera.trackingState != TrackingState.TRACKING) return

            frameCounter++

            if (isScanning3D && frameCounter % 3 == 0) {
                collectPointCloud(frame)
            }

            if (isTargetLocked) {
                processOpenCvAndArCore(frame)
            } else {
                tracker?.clear()
                tracker = null
            }
        } catch (_: Exception) {
        }
    }

    private fun processOpenCvAndArCore(frame: Frame) {
        var camImage: Image? = null
        try {
            camImage = frame.acquireCameraImage()
            val mat = yuvToGrayMat(camImage)
            Core.transpose(mat, mat)
            Core.flip(mat, mat, 1)
            Imgproc.resize(mat, mat, org.opencv.core.Size(240.0, 320.0))

            imageGrab?.release()
            imageGrab = mat

            if (drawing == Drawing.DRAWING) {
                val minX = (minOf(pointStart.x.toFloat(), pointEnd.x.toFloat()) / trackingOverlay.width * mat.cols()).toInt().coerceIn(0, mat.cols() - 2)
                val minY = (minOf(pointStart.y.toFloat(), pointEnd.y.toFloat()) / trackingOverlay.height * mat.rows()).toInt().coerceIn(0, mat.rows() - 2)
                val maxX = (maxOf(pointStart.x.toFloat(), pointEnd.x.toFloat()) / trackingOverlay.width * mat.cols()).toInt().coerceIn(minX + 1, mat.cols() - 1)
                val maxY = (maxOf(pointStart.y.toFloat(), pointEnd.y.toFloat()) / trackingOverlay.height * mat.rows()).toInt().coerceIn(minY + 1, mat.rows() - 1)

                val initRect = Rect2d(minX.toDouble(), minY.toDouble(), (maxX - minX).toDouble(), (maxY - minY).toDouble())
                tracker = when (selectedTracker) {
                    TrackerType.MEDIAN_FLOW -> legacy_TrackerMedianFlow.create()
                    TrackerType.CSRT -> legacy_TrackerCSRT.create()
                    TrackerType.KCF -> legacy_TrackerKCF.create()
                    TrackerType.MOSSE -> legacy_TrackerMOSSE.create()
                    TrackerType.TLD -> legacy_TrackerTLD.create()
                    TrackerType.MIL -> legacy_TrackerMIL.create()
                }.apply {
                    init(mat, initRect)
                }
                drawing = Drawing.TRACKING
            } else {
                val rect = Rect2d()
                if (tracker?.update(mat, rect) == true) {
                    pointStart.x = (rect.x * trackingOverlay.width / mat.cols()).toInt()
                    pointStart.y = (rect.y * trackingOverlay.height / mat.rows()).toInt()
                    pointEnd.x = pointStart.x + (rect.width * trackingOverlay.width / mat.cols()).toInt()
                    pointEnd.y = pointStart.y + (rect.height * trackingOverlay.height / mat.rows()).toInt()
                    trackingOverlay.postInvalidate()

                    val centerX = (pointStart.x + pointEnd.x) / 2f
                    val centerY = (pointStart.y + pointEnd.y) / 2f

                    var posX = 0f
                    var posY = 0f
                    var posZ = getDepthAtScreenPoint(frame, centerX, centerY)

                    val hits = frame.hitTest(centerX, centerY)
                    if (hits.isNotEmpty()) {
                        val pose = hits.first().hitPose
                        posX = pose.tx()
                        posY = pose.ty()
                        if (posZ <= 0f) posZ = -pose.tz()
                    }

                    val outX = ((centerX / trackingOverlay.width) * 240f).toInt().coerceIn(0, 240)
                    val outY = ((centerY / trackingOverlay.height) * 240f).toInt().coerceIn(0, 240)
                    val outA = if (posZ > 0.05f) {
                        (80f / posZ).toInt().coerceIn(10, 250)
                    } else {
                        val boxWidth = abs(pointEnd.x - pointStart.x).toFloat()
                        ((boxWidth / trackingOverlay.width) * 200f).toInt().coerceIn(10, 250)
                    }

                    runOnUiThread {
                        tvCoordinates.text = String.format(
                            Locale.US,
                            "3D: X=%.2fm Y=%.2fm Z=%.2fm | BT: X%dY%dA%d",
                            posX, posY, posZ, outX, outY, outA
                        )
                    }

                    sendCoordinatesToArduino(outX, outY, outA)
                }
            }
        } catch (_: Exception) {
        } finally {
            camImage?.close()
        }
    }

    private fun yuvToGrayMat(image: Image): Mat {
        val yPlane = image.planes.first()
        val yBuffer = yPlane.buffer
        val rowStride = yPlane.rowStride
        val width = image.width
        val height = image.height
        val data = ByteArray(yBuffer.remaining())
        yBuffer.get(data)
        val fullMat = Mat(height, rowStride, CvType.CV_8UC1)
        fullMat.put(0, 0, data)
        val cropped = fullMat.submat(0, height, 0, width).clone()
        fullMat.release()
        return cropped
    }

    private fun getDepthAtScreenPoint(frame: Frame, screenX: Float, screenY: Float): Float {
        try {
            frame.acquireDepthImage16Bits().use { depthImage ->
                val normX = (screenX / trackingOverlay.width).coerceIn(0f, 0.99f)
                val normY = (screenY / trackingOverlay.height).coerceIn(0f, 0.99f)
                val imgX = (normY * depthImage.width).toInt().coerceIn(0, depthImage.width - 1)
                val imgY = ((1f - normX) * depthImage.height).toInt().coerceIn(0, depthImage.height - 1)

                val plane = depthImage.planes.first()
                val byteIndex = imgY * plane.rowStride + imgX * plane.pixelStride
                val buffer = plane.buffer.order(ByteOrder.nativeOrder())
                val depthMm = buffer.getShort(byteIndex).toInt() and 0xFFFF
                if (depthMm > 0) return depthMm / 1000f
            }
        } catch (_: Exception) {
        }
        return 0f
    }

    private fun collectPointCloud(frame: Frame) {
        try {
            frame.acquirePointCloud().use { pointCloud ->
                val pointsBuffer = pointCloud.points
                while (pointsBuffer.remaining() >= 4) {
                    val x = pointsBuffer.get()
                    val y = pointsBuffer.get()
                    val z = pointsBuffer.get()
                    val confidence = pointsBuffer.get()
                    if (confidence > 0.35f && pointCloudList.size < 150000) {
                        pointCloudList.add(floatArrayOf(x, y, z))
                    }
                }
                runOnUiThread { updateStatusText() }
            }
        } catch (_: Exception) {
