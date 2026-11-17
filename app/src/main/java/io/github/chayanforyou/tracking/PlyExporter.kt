package io.github.chayanforyou.tracking

import android.content.ContentValues
import android.content.Context
import android.media.Image
import android.os.Build
import android.os.Environment
import android.provider.MediaStore
import android.widget.Toast
import com.google.ar.core.Frame
import com.google.ar.core.Plane
import com.google.ar.core.Point
import com.google.ar.core.Pose
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
import kotlin.math.abs
import kotlin.math.sqrt

class PlyExporter {
    val pointCloudList = CopyOnWriteArrayList<FloatArray>()
    var lastError: String = ""

    private var prevGray: Mat? = null
    private var prevPose: Pose? = null

    fun collectFromFrame(frame: Frame) {
        val camPose = frame.camera.pose

        // 1. Klasický ARCore PointCloud + HitTest
        try {
            frame.acquirePointCloud().use { pointCloud ->
                val pointsBuffer = pointCloud.points
                val totalFloats = pointsBuffer.limit()
                var i = 0
                while (i + 3 < totalFloats && pointCloudList.size < 300000) {
                    val x = pointsBuffer.get(i)
                    val y = pointsBuffer.get(i + 1)
                    val z = pointsBuffer.get(i + 2)
                    i += 4
                    if (x != 0f || y != 0f || z != 0f) {
                        val dist = distance(camPose, x, y, z)
                        addColoredPoint(x, y, z, dist)
                    }
                }
            }
        } catch (_: Exception) {
        }

        try {
            for (gx in 1..9 step 2) {
                for (gy in 1..9 step 2) {
                    if (pointCloudList.size >= 300000) break
                    val hits = frame.hitTest(gx * 100f, gy * 150f)
                    for (hit in hits) {
                        if (hit.trackable is Plane || hit.trackable is Point) {
                            val p = hit.hitPose
                            addColoredPoint(p.tx(), p.ty(), p.tz(), distance(camPose, p.tx(), p.ty(), p.tz()))
                            break
                        }
                    }
                }
            }
        } catch (_: Exception) {
        }

        // 2. Zaručený 3D LiDAR výpočet z pohybu kamery (ARCore Pose + OpenCV Optical Flow)
        var camImage: Image? = null
        try {
            camImage = frame.acquireCameraImage()
            val currGray = yuvToSmallGray(camImage)

            val pGray = prevGray
            val pPose = prevPose

            if (pGray != null && pPose != null) {
                val baseDist = distance(camPose, pPose.tx(), pPose.ty(), pPose.tz())
                // Počítame 3D body, keď sa mobil pohol aspoň o 2 cm
                if (baseDist > 0.02f && baseDist < 0.5f) {
                    val corners = MatOfPoint()
                    Imgproc.goodFeaturesToTrack(pGray, corners, 150, 0.02, 10.0)
                    val prevPts = MatOfPoint2f(*corners.toArray())

                    if (prevPts.total() > 5) {
                        val nextPts = MatOfPoint2f()
                        val status = MatOfByte()
                        val err = MatOfFloat()
                        Video.calcOpticalFlowPyrLK(pGray, currGray, prevPts, nextPts, status, err)

                        val pArr = prevPts.toArray()
                        val nArr = nextPts.toArray()
                        val stArr = status.toArray()

                        val intrinsics = frame.camera.imageIntrinsics
                        val scale = 320f / intrinsics.imageDimensions.get(0).toFloat()
                        val fx = intrinsics.focalLength.get(0) * scale
                        val cx = intrinsics.principalPoint.get(0) * scale
                        val cy = intrinsics.principalPoint.get(1) * scale

                        for (k in pArr.indices) {
                            if (stArr.get(k).toInt() == 1 && pointCloudList.size < 300000) {
                                val dxPx = nArr.get(k).x - pArr.get(k).x
                                val dyPx = nArr.get(k).y - pArr.get(k).y
                                val pixelShift = sqrt(dxPx * dxPx + dyPx * dyPx).toFloat()

                                if (pixelShift > 1.5f) {
                                    val depthZ = ((fx * baseDist) / pixelShift).coerceIn(0.3f, 4.5f)
                                    val localX = ((pArr.get(k).x.toFloat() - cx) * depthZ) / fx
                                    val localY = -((pArr.get(k).y.toFloat() - cy) * depthZ) / fx
                                    val worldPt = pPose.transformPoint(floatArrayOf(localX, localY, -depthZ))
                                    addColoredPoint(worldPt.get(0), worldPt.get(1), worldPt.get(2), depthZ)
                                }
                            }
                        }
                        nextPts.release()
                        status.release()
                        err.release()
                    }
                    corners.release()
                    prevPts.release()
                }
            }

                  prevGray?.release()
            prevGray = currGray
            prevPose = camPose
     } catch (e: Exception) {

            lastError = e.javaClass.simpleName
        } finally {
            camImage?.close()
        }
    }

    private fun yuvToSmallGray(image: Image): Mat {
        val yPlane = image.planes.first()
        val yBuffer = yPlane.buffer
        val rowStride = yPlane.rowStride
        val width = image.width
        val height = image.height
        val data = ByteArray(yBuffer.remaining())
        yBuffer.get(data)
        val fullMat = Mat(height, rowStride, CvType.CV_8UC1)
        fullMat.put(0, 0, data)
        val cropped = fullMat.submat(0, height, 0, width)
        val small = Mat()
        Imgproc.resize(cropped, small, org.opencv.core.Size(320.0, 240.0))
        cropped.release()
        fullMat.release()
        return small
    }

    private fun distance(pose: Pose, x: Float, y: Float, z: Float): Float {
        val dx = x - pose.tx()
        val dy = y - pose.ty()
        val dz = z - pose.tz()
        return sqrt(dx * dx + dy * dy + dz * dz)
    }

    private fun addColoredPoint(x: Float, y: Float, z: Float, distMeters: Float) {
        val normDist = ((distMeters - 0.3f) / 4.2f).coerceIn(0f, 1f)
        val r = (1f - normDist) * 255f
        val g = (1f - abs(normDist - 0.5f) * 2f) * 255f
        val b = normDist * 255f
        pointCloudList.add(floatArrayOf(x, y, z, r, g, b))
    }

    fun clear() {
        pointCloudList.clear()
        prevGray?.release()
        prevGray = null
        prevPose = null
        lastError = ""
    }

    fun exportToDownloads(context: Context) {
        if (pointCloudList.isEmpty()) {
            val errInfo = if (lastError.isNotEmpty()) " ($lastError)" else ""
            Toast.makeText(context, "0 bodov$errInfo – pohnite mobilom do strany", Toast.LENGTH_LONG).show()
            return
        }

        val timeStamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
        val fileName = "LiDAR_3D_Map_$timeStamp.ply"

        try {
            val sb = StringBuilder()
            sb.append("ply\n")
            sb.append("format ascii 1.0\n")
            sb.append("element vertex ${pointCloudList.size}\n")
            sb.append("property float x\n")
            sb.append("property float y\n")
            sb.append("property float z\n")
            sb.append("property uchar red\n")
            sb.append("property uchar green\n")
            sb.append("property uchar blue\n")
            sb.append("end_header\n")

            for (pt in pointCloudList) {
                sb.append(
                    String.format(
                        Locale.US,
                        "%.4f %.4f %.4f %d %d %d\n",
                        pt.get(0), pt.get(1), pt.get(2),
                        pt.get(3).toInt(), pt.get(4).toInt(), pt.get(5).toInt()
                    )
                )
            }

            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
                val values = ContentValues().apply {
                    put(MediaStore.Downloads.DISPLAY_NAME, fileName)
                    put(MediaStore.Downloads.MIME_TYPE, "application/octet-stream")
                    put(MediaStore.Downloads.RELATIVE_PATH, Environment.DIRECTORY_DOWNLOADS)
                }
                val uri = context.contentResolver.insert(MediaStore.Downloads.EXTERNAL_CONTENT_URI, values)
                uri?.let {
                    context.contentResolver.openOutputStream(it)?.use { out ->
                        out.write(sb.toString().toByteArray())
                    }
                }
            }
            Toast.makeText(context, "LiDAR 3D mapa uložená: $fileName", Toast.LENGTH_LONG).show()
        } catch (e: Exception) {
            Toast.makeText(context, "Chyba exportu: ${e.message}", Toast.LENGTH_LONG).show()
        }
    }
}
