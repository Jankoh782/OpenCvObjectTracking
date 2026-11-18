package io.github.chayanforyou.tracking

import android.app.Activity
import android.graphics.Point
import android.media.Image
import android.widget.TextView
import android.widget.Toast
import com.google.ar.core.Frame
import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfDMatch
import org.opencv.core.MatOfKeyPoint
import org.opencv.core.Rect
import org.opencv.features2d.BFMatcher
import org.opencv.features2d.ORB
import org.opencv.imgproc.Imgproc
import java.util.Locale
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

class ArTrackerEngine(
    private val activity: Activity,
    private val trackingOverlay: OverlayView,
    private val tvCoordinates: TextView,
    private val btHelper: BluetoothHelper,
    private val plyExporter: PlyExporter
) {
    private val orb = ORB.create(800)
    private val matcher = BFMatcher.create(Core.NORM_HAMMING, true)

    private var targetKeypoints: MatOfKeyPoint? = null
    private var targetDescriptors: Mat? = null
    private var lastMarkerTime = 0L

    fun resetTracker() {
        targetKeypoints?.release()
        targetKeypoints = null
        targetDescriptors?.release()
        targetDescriptors = null
    }

    fun release() {
        resetTracker()
    }

    fun showTrackerDialog() {
        Toast.makeText(activity, "Aktívny režim: ORB vyhľadávanie potlače a vzorov", Toast.LENGTH_SHORT).show()
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
            Imgproc.resize(mat, mat, org.opencv.core.Size(480.0, 640.0))

            if (isInitDrawing) {
                val minX = (min(pointStart.x.toFloat(), pointEnd.x.toFloat()) / trackingOverlay.width * mat.cols()).toInt().coerceIn(0, mat.cols() - 10)
                val minY = (min(pointStart.y.toFloat(), pointEnd.y.toFloat()) / trackingOverlay.height * mat.rows()).toInt().coerceIn(0, mat.rows() - 10)
                val maxX = (max(pointStart.x.toFloat(), pointEnd.x.toFloat()) / trackingOverlay.width * mat.cols()).toInt().coerceIn(minX + 10, mat.cols())
                val maxY = (max(pointStart.y.toFloat(), pointEnd.y.toFloat()) / trackingOverlay.height * mat.rows()).toInt().coerceIn(minY + 10, mat.rows())

                val roi = mat.submat(Rect(minX, minY, maxX - minX, maxY - minY))
                resetTracker()

                val kp = MatOfKeyPoint()
                val desc = Mat()
                orb.detectAndCompute(roi, Mat(), kp, desc)
                roi.release()

                if (!desc.empty() && kp.toArray().size >= 8) {
                    targetKeypoints = kp
                    targetDescriptors = desc
                    activity.runOnUiThread {
                        Toast.makeText(activity, "Vzor uložený (${kp.toArray().size} bodov potlače)", Toast.LENGTH_SHORT).show()
                    }
                } else {
                    kp.release()
                    desc.release()
                    activity.runOnUiThread {
                        Toast.makeText(activity, "Málo detailov! Vyberte oblasť s potlačou alebo textom.", Toast.LENGTH_LONG).show()
                    }
                }
                onInitDone()
            } else {
                val tDesc = targetDescriptors ?: run {
                    mat.release()
                    return
                }

                val sceneKp = MatOfKeyPoint()
                val sceneDesc = Mat()
                orb.detectAndCompute(mat, Mat(), sceneKp, sceneDesc)

                var found = false
                if (!sceneDesc.empty()) {
                    val matches = MatOfDMatch()
                    matcher.match(tDesc, sceneDesc, matches)

                    val goodMatches = matches.toList().filter { it.distance < 45f }

                    if (goodMatches.size >= 8) {
                        val kpArray = sceneKp.toArray()
                        var minMatchX = mat.cols().toDouble()
                        var minMatchY = mat.rows().toDouble()
                        var maxMatchX = 0.0
                        var maxMatchY = 0.0

                        for (m in goodMatches) {
                            val pt = kpArray.get(m.trainIdx).pt
                            if (pt.x < minMatchX) minMatchX = pt.x
                            if (pt.y < minMatchY) minMatchY = pt.y
                            if (pt.x > maxMatchX) maxMatchX = pt.x
                            if (pt.y > maxMatchY) maxMatchY = pt.y
                        }

                        val pad = 20.0
                        val rx1 = (minMatchX - pad).coerceIn(0.0, mat.cols().toDouble())
                        val ry1 = (minMatchY - pad).coerceIn(0.0, mat.rows().toDouble())
                        val rx2 = (maxMatchX + pad).coerceIn(0.0, mat.cols().toDouble())
                        val ry2 = (maxMatchY + pad).coerceIn(0.0, mat.rows().toDouble())

                        pointStart.x = (rx1 * trackingOverlay.width / mat.cols()).toInt()
                        pointStart.y = (ry1 * trackingOverlay.height / mat.rows()).toInt()
                        pointEnd.x = (rx2 * trackingOverlay.width / mat.cols()).toInt()
                        pointEnd.y = (ry2 * trackingOverlay.height / mat.rows()).toInt()
                        trackingOverlay.postInvalidate()

                        val centerX = (pointStart.x + pointEnd.x) / 2f
                        val centerY = (pointStart.y + pointEnd.y) / 2f

                        var posX = 0f
                        var posY = 0f
                        var posZ = 0f
                        var hasWorldPose = false

                        try {
                            val hits = frame.hitTest(centerX, centerY)
                            if (hits.isNotEmpty()) {
                                val pose = hits.first().hitPose
                                posX = pose.tx()
                                posY = pose.ty()
                                posZ = pose.tz()
                                hasWorldPose = true
                            }
                        } catch (_: Exception) {
                        }

                        val now = System.currentTimeMillis()
                        if (hasWorldPose && (now - lastMarkerTime > 500)) {
                            lastMarkerTime = now
                            plyExporter.addTargetMarker(posX, posY, posZ)
                        }

                        val outX = ((centerX / trackingOverlay.width) * 240f).toInt().coerceIn(0, 240)
                        val outY = ((centerY / trackingOverlay.height) * 240f).toInt().coerceIn(0, 240)
                        val boxWidth = abs(pointEnd.x - pointStart.x).toFloat()
                        val outA = ((boxWidth / trackingOverlay.width) * 200f).toInt().coerceIn(10, 250)

                        activity.runOnUiThread {
                            tvCoordinates.text = String.format(
                                Locale.US,
                                "NÁJDENÉ (%d zhôd) | Mapa: X=%.2f Y=%.2f Z=%.2f | BT: X%dY%dA%d",
                                goodMatches.size, posX, posY, posZ, outX, outY, outA
                            )
                        }
                        btHelper.sendCoordinates(outX, outY, outA)
                        found = true
                    }
                    matches.release()
                }

                if (!found) {
                    pointStart.set(0, 0)
                    pointEnd.set(0, 0)
                    trackingOverlay.postInvalidate()
                    activity.runOnUiThread {
                        tvCoordinates.text = "Hľadám uložený vzor v okolí..."
                    }
                }

                sceneKp.release()
                sceneDesc.release()
            }
            mat.release()
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
}
