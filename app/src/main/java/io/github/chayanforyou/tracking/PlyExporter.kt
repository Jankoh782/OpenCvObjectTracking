package io.github.chayanforyou.tracking

import android.content.ContentValues
import android.content.Context
import android.os.Build
import android.os.Environment
import android.provider.MediaStore
import android.widget.Toast
import com.google.ar.core.Frame
import com.google.ar.core.Pose
import com.google.ar.core.Session
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfByte
import org.opencv.core.MatOfFloat
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.imgproc.Imgproc
import org.opencv.video.Video
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.concurrent.CopyOnWriteArrayList
import kotlin.math.sqrt

class PlyExporter {
    val pointCloudList = CopyOnWriteArrayList<FloatArray>()
    private val facesList = CopyOnWriteArrayList<IntArray>()
    var debugInfo: String = "Pripravený"
    private val voxelFilter = VoxelGridFilter(0.03f)

    private var prevGray: Mat? = null
    private var prevPts: MatOfPoint2f? = null
    private var prevPose: Pose? = null

    fun addTargetMarker(x: Float, y: Float, z: Float) {
        val offsets = floatArrayOf(0f, 0.02f, -0.02f)
        for (ox in offsets) {
            pointCloudList.add(floatArrayOf(x + ox, y, z, 255f, 0f, 0f))
            pointCloudList.add(floatArrayOf(x, y + ox, z, 255f, 0f, 0f))
        }
    }

    fun collectFromFrame(frame: Frame, session: Session? = null) {
        if (pointCloudList.size >= 300_000) return

        var addedNow = 0

        try {
            val pose = frame.camera.pose
            val cxWorld = pose.tx()
            val cyWorld = pose.ty()
            val czWorld = pose.tz()

            // 1. Zaznamenáme aj vlastnú trasu robota/mobilu (modré body)
            if (voxelFilter.checkOrInsert(cxWorld, cyWorld, czWorld, pointCloudList.size) == -1) {
                pointCloudList.add(floatArrayOf(cxWorld, cyWorld, czWorld, 0f, 170f, 255f))
                addedNow++
            }

            // 2. Získame obraz z kamery pre OpenCV 3D trianguláciu stien a prekážok
            val currGray = acquireGrayMat(frame) ?: return

            val pGray = prevGray
            val pPts = prevPts
            val pPose = prevPose

            if (pGray == null || pPts == null || pPose == null || pPts.empty()) {
                val corners = MatOfPoint()
                Imgproc.goodFeaturesToTrack(currGray, corners, 220, 0.02, 12.0)
                val pts2f = MatOfPoint2f()
                corners.convertTo(pts2f, CvType.CV_32FC2)
                corners.release()

                prevGray?.release()
                prevPts?.release()
                prevGray = currGray
                prevPts = pts2f
                prevPose = pose
                debugInfo = "3D Mapa: ${pointCloudList.size} bodov (inicializujem...)"
                return
            }

            // Vzdialenosť pohybu mobilu od referenčnej snímky (minimálne 3.5 cm pre presnú trianguláciu)
            val dxCam = cxWorld - pPose.tx()
            val dyCam = cyWorld - pPose.ty()
            val dzCam = czWorld - pPose.tz()
            val baseline = sqrt(dxCam * dxCam + dyCam * dyCam + dzCam * dzCam)

            if (baseline < 0.035f) {
                currGray.release()
                debugInfo = "3D Mapa: ${pointCloudList.size} bodov (pohnite mobilom)"
                return
            }

            // Sledovanie bodov medzi snímkami cez OpenCV Optical Flow
            val nextPts = MatOfPoint2f()
            val status = MatOfByte()
            val err = MatOfFloat()
            Video.calcOpticalFlowPyrLK(pGray, currGray, pPts, nextPts, status, err)

            val prevArray = pPts.toArray()
            val nextArray = nextPts.toArray()
            val statusArray = status.toArray()

            val intrinsics = frame.camera.imageIntrinsics
            val fx = intrinsics.focalLength.get(0)
            val fy = intrinsics.focalLength.get(1)
            val ppx = intrinsics.principalPoint.get(0)
            val ppy = intrinsics.principalPoint.get(1)

            val c1 = floatArrayOf(pPose.tx(), pPose.ty(), pPose.tz())
            val c2 = floatArrayOf(cxWorld, cyWorld, czWorld)

            val count = minOf(prevArray.size, nextArray.size, statusArray.size)
            for (i in 0 until count) {
                if (statusArray.get(i).toInt() != 1) continue

                val pt1 = prevArray.get(i)
                val pt2 = nextArray.get(i)

                // Lúč 1 vo svetových súradniciach
                val dir1Local = floatArrayOf(
                    ((pt1.x - ppx) / fx).toFloat(),
                    ((ppy - pt1.y) / fy).toFloat(),
                    -1.0f
                )
                val v1 = normalize(pPose.rotateVector(dir1Local))

                // Lúč 2 vo svetových súradniciach
                val dir2Local = floatArrayOf(
                    ((pt2.x - ppx) / fx).toFloat(),
                    ((ppy - pt2.y) / fy).toFloat(),
                    -1.0f
                )
                val v2 = normalize(pose.rotateVector(dir2Local))

                // 3D triangulácia priesečníka oboch lúčov
                val w0x = c1.get(0) - c2.get(0)
                val w0y = c1.get(1) - c2.get(1)
                val w0z = c1.get(2) - c2.get(2)

                val b = v1.get(0) * v2.get(0) + v1.get(1) * v2.get(1) + v1.get(2) * v2.get(2)
                val denom = 1.0f - b * b
                if (denom < 0.0004f) continue

                val d = v1.get(0) * w0x + v1.get(1) * w0y + v1.get(2) * w0z
                val e = v2.get(0) * w0x + v2.get(1) * w0y + v2.get(2) * w0z

                val s = (b * e - d) / denom
                val t = (e - b * d) / denom

                // Platné prekážky a steny vo vzdialenosti 25 cm až 4.5 m
                if (s in 0.25f..4.5f && t in 0.25f..4.5f) {
                    val p1x = c1.get(0) + s * v1.get(0)
                    val p1y = c1.get(1) + s * v1.get(1)
                    val p1z = c1.get(2) + s * v1.get(2)

                    val p2x = c2.get(0) + t * v2.get(0)
                    val p2y = c2.get(1) + t * v2.get(1)
                    val p2z = c2.get(2) + t * v2.get(2)

                    val errDist = sqrt(
                        (p1x - p2x) * (p1x - p2x) +
                        (p1y - p2y) * (p1y - p2y) +
                        (p1z - p2z) * (p1z - p2z)
                    )

                    if (errDist < 0.08f) {
                        val wx = (p1x + p2x) * 0.5f
                        val wy = (p1y + p2y) * 0.5f
                        val wz = (p1z + p2z) * 0.5f

                        if (voxelFilter.checkOrInsert(wx, wy, wz, pointCloudList.size) == -1) {
                            pointCloudList.add(floatArrayOf(wx, wy, wz, 0f, 255f, 140f))
                            addedNow++
                        }
                    }
                }
            }

            nextPts.release()
            status.release()
            err.release()

            // Nájdeme nové body v aktuálnej snímke pre ďalší krok pohybu
            val newCorners = MatOfPoint()
            Imgproc.goodFeaturesToTrack(currGray, newCorners, 220, 0.02, 12.0)
            val newPts2f = MatOfPoint2f()
            newCorners.convertTo(newPts2f, CvType.CV_32FC2)
            newCorners.release()

            pGray.release()
            pPts.release()
            prevGray = currGray
            prevPts = newPts2f
            prevPose = pose

            debugInfo = "3D Mapa: ${pointCloudList.size} bodov (+$addedNow)"
        } catch (e: Exception) {
            debugInfo = "3D Mapa: ${pointCloudList.size} bodov"
        }
    }

    private fun acquireGrayMat(frame: Frame): Mat? {
        return try {
            frame.acquireCameraImage().use { image ->
                val width = image.width
                val height = image.height
                val yPlane = image.planes.get(0)
                val yBuffer = yPlane.buffer
                val rowStride = yPlane.rowStride

                val data = ByteArray(width * height)
                if (rowStride == width) {
                    yBuffer.get(data)
                } else {
                    val rowBytes = ByteArray(rowStride)
                    for (r in 0 until height) {
                        yBuffer.position(r * rowStride)
                        yBuffer.get(rowBytes, 0, width)
                        System.arraycopy(rowBytes, 0, data, r * width, width)
                    }
                }
                val mat = Mat(height, width, CvType.CV_8UC1)
                mat.put(0, 0, data)
                mat
            }
        } catch (_: Exception) {
            null
        }
    }

    private fun normalize(v: FloatArray): FloatArray {
        val len = sqrt(v.get(0) * v.get(0) + v.get(1) * v.get(1) + v.get(2) * v.get(2))
        return if (len > 1e-6f) {
            floatArrayOf(v.get(0) / len, v.get(1) / len, v.get(2) / len)
        } else {
            floatArrayOf(0f, 0f, -1f)
        }
    }

    fun clear() {
        pointCloudList.clear()
        facesList.clear()
        voxelFilter.clear()
        prevGray?.release()
        prevPts?.release()
        prevGray = null
        prevPts = null
        prevPose = null
    }

    fun exportToDownloads(context: Context) {
        if (pointCloudList.isEmpty()) {
            Toast.makeText(context, "0 bodov ($debugInfo)", Toast.LENGTH_LONG).show()
            return
        }
        val timeStamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
        val fileName = "Room_3D_Scan_$timeStamp.ply"
        try {
            val sb = StringBuilder()
            sb.append("ply\n")
            sb.append("format ascii 1.0\n")
            sb.append("element vertex ${pointCloudList.size}\n")
            sb.append("property float x\n")
            sb.append("property float y\n")
