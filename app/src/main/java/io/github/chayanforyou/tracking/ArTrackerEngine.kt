package io.github.chayanforyou.tracking

import android.app.Activity
import android.graphics.Point
import android.media.Image
import android.view.Gravity
import android.widget.LinearLayout
import android.widget.RadioButton
import android.widget.RadioGroup
import android.widget.TextView
import com.google.android.material.dialog.MaterialAlertDialogBuilder
import com.google.ar.core.Frame
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
import java.nio.ByteOrder
import java.util.Locale
import kotlin.math.abs

class ArTrackerEngine(
    private val activity: Activity,
    private val trackingOverlay: OverlayView,
    private val tvCoordinates: TextView,
    private val btHelper: BluetoothHelper
) {
    private var imageGrab: Mat? = null
    private var tracker: legacy_Tracker? = null
    var selectedTracker = TrackerType.CSRT

    enum class TrackerType { MEDIAN_FLOW, CSRT, KCF, MOSSE, TLD, MIL }

    fun resetTracker() {
        tracker?.clear()
        tracker = null
    }

    fun release() {
        imageGrab?.release()
        resetTracker()
    }

    fun processFrame(
        frame: Frame,
        pointStart: Point,
        pointEnd: Point,
        isInitDrawing: Boolean,
        onInitDone: () -> Unit
    ) {
        var camImage: Image? = null
        try {
            camImage = frame.acquireCameraImage()
            val mat = yuvToGrayMat(camImage)
            Core.transpose(mat, mat)
            Core.flip(mat, mat, 1)
            Imgproc.resize(mat, mat, org.opencv.core.Size(240.0, 320.0))

            imageGrab?.release()
            imageGrab = mat

            if (isInitDrawing) {
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
                onInitDone()
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

                                       try {
                        val hits = frame.hitTest(centerX, centerY)
                        if (hits.isNotEmpty()) {
                            val pose = hits.first().hitPose
                            posX = pose.tx()
                            posY = pose.ty()
                            if (posZ <= 0f) posZ = -pose.tz()
                        }
                    } catch (_: Exception) {
                    }


                    val outX = ((centerX / trackingOverlay.width) * 240f).toInt().coerceIn(0, 240)
                    val outY = ((centerY / trackingOverlay.height) * 240f).toInt().coerceIn(0, 240)
                    val outA = if (posZ > 0.05f) {
                        (80f / posZ).toInt().coerceIn(10, 250)
                    } else {
                        val boxWidth = abs(pointEnd.x - pointStart.x).toFloat()
                        ((boxWidth / trackingOverlay.width) * 200f).toInt().coerceIn(10, 250)
                    }

                    activity.runOnUiThread {
                        tvCoordinates.text = String.format(
                            Locale.US,
                            "3D: X=%.2fm Y=%.2fm Z=%.2fm | BT: X%dY%dA%d",
                            posX, posY, posZ, outX, outY, outA
                        )
                    }

                    btHelper.sendCoordinates(outX, outY, outA)
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

    fun showTrackerDialog() {
        val builder = MaterialAlertDialogBuilder(activity)
        builder.setTitle("Výber sledovacieho algoritmu")

        val trackerMap = mapOf(
            TrackerType.CSRT to "TrackerCSRT (Najpresnejší)",
            TrackerType.KCF to "TrackerKCF (Rýchly a presný)",
            TrackerType.MEDIAN_FLOW to "TrackerMedianFlow (Rýchly)",
            TrackerType.MOSSE to "TrackerMOSSE (Ultra rýchly)",
            TrackerType.TLD to "TrackerTLD (Dlhodobý)",
            TrackerType.MIL to "TrackerMIL (Odolný)"
        )

        val radioGroup = RadioGroup(activity).apply {
            orientation = RadioGroup.VERTICAL
            setPadding(48, 32, 48, 32)
        }

        for ((type, label) in trackerMap) {
            val rb = RadioButton(activity).apply {
                text = label
                isChecked = (type == selectedTracker)
                setOnClickListener { selectedTracker = type }
            }
            radioGroup.addView(rb)
        }

        val layout = LinearLayout(activity).apply {
            gravity = Gravity.CENTER_VERTICAL
            addView(radioGroup)
        }

        builder.setView(layout)
        builder.setPositiveButton("OK") { dialog, _ -> dialog.dismiss() }
        builder.show()
    }
}
