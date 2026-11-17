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
import com.google.ar.core.TrackingState
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
    var debugInfo: String = "Čakám na snímku"

    private var prevGray: Mat? = null
    private var prevPose: Pose? = null
    private var virtualStepZ = 0f

    fun collectFromFrame(frame: Frame) {
        val camPose = frame.camera.pose
        val state = frame.camera.trackingState

        if (state == TrackingState.TRACKING) {
            try {
                for (gx in 1..9 step 2) {
                    for (gy in 1..9 step 2) {
                        if (pointCloudList.size >= 300000) break
                        val hits = frame.hitTest(gx * 100f, gy * 160f)
                        for (hit in hits) {
                            if (hit.trackable is Plane || hit.trackable is Point) {
                                val p = hit.hitPose
                                pointCloudList.add(floatArrayOf(p.tx(), p.ty(), p.tz(), 220f, 220f, 220f))
                                break
                            }
                        }
                    }
                }
            } catch (_: Exception) {
            }
        }

        var camImage: Image? = null
        try {
            camImage = frame.acquireCameraImage()
            val currGray = yuvToSmallGray(camImage)
            val pGray = prevGray
            val pPose = prevPose

            if (pGray != null && pPose != null) {
                val realDist = distance(camPose, pPose.tx(), pPose.ty(), pPose.tz())
                val useVirtual = (realDist < 0.01f)
                val baseDist = if (useVirtual) 0.04f else realDist

                if (baseDist <= 0.4f) {
                    val corners = MatOfPoint()
                    Imgproc.goodFeaturesToTrack(pGray, corners, 400, 0.015, 8.0)
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

                        var addedNow = 0
                        for (k in pArr.indices) {
                            if (stArr.get(k).toInt() == 1 && pointCloudList.size < 300000) {
                                val px = pArr.get(k).x.toInt().coerceIn(0, pGray.cols() - 1)
                                val py = pArr.get(k).y.toInt().coerceIn(0, pGray.rows() - 1)
                                val dxPx = nArr.get(k).x - pArr.get(k).x
                                val dyPx = nArr.get(k).y - pArr.get(k).y
                                val pixelShift = sqrt(dxPx * dxPx + dyPx * dyPx).toFloat()

                                if (pixelShift > 2.5f) {
                                    val depthZ = ((fx * baseDist) / pixelShift).coerceIn(0.3f, 3.0f)
                                    val localX = ((px.toFloat() - cx) * depthZ) / fx
                                    val localY = -((py.toFloat() - cy) * depthZ) / fx

                                    val wx: Float
                                    val wy: Float
                                    val wz: Float
                                    if (!useVirtual) {
                                        val wp = pPose.transformPoint(floatArrayOf(localX, localY, -depthZ))
                                        wx = wp.get(0)
                                        wy = wp.get(1)
                                        wz = wp.get(2)
                                    } else {
                                        wx = localX
                                        wy = localY
                                        wz = -depthZ - virtualStepZ
                                    }

                                    val pixelVal = pGray.get(py, px)
                                    val lum = if (pixelVal != null && pixelVal.isNotEmpty()) {
                                        pixelVal.get(0).toFloat().coerceIn(30f, 255f)
                                    } else 180f

                                    pointCloudList.add(floatArrayOf(wx, wy, wz, lum, lum, lum))
                                    addedNow++
                                }
                            }
                        }
                        if (addedNow > 0 && useVirtual) virtualStepZ += 0.03f
                        debugInfo = "AR:$state, d=${String.format(Locale.US, "%.2f", realDist)}, +$addedNow"
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
            debugInfo = "Err:${e.javaClass.simpleName}"
        } finally {
            camImage?.close()
        }
    }

    private fun yuvToSmallGray(image: Image): Mat {
        val yPlane = image.planes.first()
        val yBuffer = yPlane.buffer
        val data = ByteArray(yBuffer.remaining())
        yBuffer.get(data)
        val fullMat = Mat(image.height, yPlane.rowStride, CvType.CV_8UC1)
        fullMat.put(0, 0, data)
        val cropped = fullMat.submat(0, image.height, 0, image.width)
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

    fun clear() {
        pointCloudList.clear()
        prevGray?.release()
        prevGray = null
        prevPose = null
        virtualStepZ = 0f
    }

    fun exportToDownloads(context: Context) {
        if (pointCloudList.isEmpty()) {
            Toast.makeText(context, "0 bodov ($debugInfo)", Toast.LENGTH_LONG).show()
            return
        }
        val timeStamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
        val fileName = "LiDAR_3D_Map_$timeStamp.ply"
        try {
            val sb = StringBuilder("ply\nformat ascii 1.0\nelement vertex ${pointCloudList.size}\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
            for (pt in pointCloudList) {
                sb.append(String.format(Locale.US, "%.4f %.4f %.4f %d %d %d\n", pt.get(0), pt.get(1), pt.get(2), pt.get(3).toInt(), pt.get(4).toInt(), pt.get(5).toInt()))
            }
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
                val values = ContentValues().apply {
                    put(MediaStore.Downloads.DISPLAY_NAME, fileName)
                    put(MediaStore.Downloads.MIME_TYPE, "application/octet-stream")
                    put(MediaStore.Downloads.RELATIVE_PATH, Environment.DIRECTORY_DOWNLOADS)
                }
                context.contentResolver.insert(MediaStore.Downloads.EXTERNAL_CONTENT_URI, values)?.let { uri ->
                    context.contentResolver.openOutputStream(uri)?.use { it.write(sb.toString().toByteArray()) }
                }
            }
            Toast.makeText(context, "Uložené ($debugInfo): $fileName", Toast.LENGTH_LONG).show()
        } catch (e: Exception) {
            Toast.makeText(context, "Chyba: ${e.message}", Toast.LENGTH_LONG).show()
        }
    }
}
