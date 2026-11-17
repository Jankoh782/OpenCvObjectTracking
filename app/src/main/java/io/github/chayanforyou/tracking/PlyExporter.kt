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
    var lastError: String = ""

    private var prevGray: Mat? = null
    private var prevPose: Pose? = null

    fun collectFromFrame(frame: Frame) {
        val camPose = frame.camera.pose
        val isTracking = (frame.camera.trackingState == TrackingState.TRACKING)

        // 1. Zber 3D bodov z detekovaných plôch (stôl, steny, podlaha) hustou mriežkou
        if (isTracking) {
            try {
                for (gx in 1..9) {
                    for (gy in 1..9) {
                        if (pointCloudList.size >= 300000) break
                        val hits = frame.hitTest(gx * 100f, gy * 160f)
                        for (hit in hits) {
                            if (hit.trackable is Plane || hit.trackable is Point) {
                                val p = hit.hitPose
                                // Plochy zafarbíme neutrálnou svetlosivou pre jasnú štruktúru miestnosti
                                pointCloudList.add(floatArrayOf(p.tx(), p.ty(), p.tz(), 210f, 210f, 210f))
                                break
                            }
                        }
                    }
                }
            } catch (_: Exception) {
            }
        }

        // 2. Hustá 3D triangulácia s reálnym jasom z kamery (iba pri skutočnom posune mobilu)
        var camImage: Image? = null
        try {
            camImage = frame.acquireCameraImage()
            val currGray = yuvToSmallGray(camImage)

            val pGray = prevGray
            val pPose = prevPose

            if (pGray != null && pPose != null) {
                val baseDist = distance(camPose, pPose.tx(), pPose.ty(), pPose.tz())

                // Počítame iba pri reálnom kroku/posune ruky (3 cm až 35 cm) - eliminuje modrý šum!
                if (baseDist in 0.03f..0.35f) {
                    val corners = MatOfPoint()
                    Imgproc.goodFeaturesToTrack(pGray, corners, 600, 0.01, 6.0)
                    val prevPts = MatOfPoint2f(*corners.toArray())

                    if (prevPts.total() > 10) {
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
                                val px = pArr.get(k).x.toInt().coerceIn(0, pGray.cols() - 1)
                                val py = pArr.get(k).y.toInt().coerceIn(0, pGray.rows() - 1)

                                val dxPx = nArr.get(k).x - pArr.get(k).x
                                val dyPx = nArr.get(k).y - pArr.get(k).y
                                val pixelShift = sqrt(dxPx * dxPx + dyPx * dyPx).toFloat()

                                // Ignorujeme príliš malé posuny (šum v diaľke)
                                if (pixelShift > 3.0f) {
                                    val depthZ = (fx * baseDist) / pixelShift
                                    // Berieme iba reálne predmety v miestnosti (0.25m až 3.5m)
                                    if (depthZ in 0.25f..3.5f) {
                                        val localX = ((px.toFloat() - cx) * depthZ) / fx
                                        val localY = -((py.toFloat() - cy) * depthZ) / fx
                                        val worldPt = pPose.transformPoint(floatArrayOf(localX, localY, -depthZ))

                                        // Reálna farba/jas pixelu z kamery namiesto modrej!
                                        val pixelVal = pGray.get(py, px)
                                        val lum = if (pixelVal != null && pixelVal.isNotEmpty()) {
                                            pixelVal.get(0).toFloat().coerceIn(0f, 255f)
                                        } else {
                                            180f
                                        }

                                        pointCloudList.add(
                                            floatArrayOf(
                                                worldPt.get(0), worldPt.get(1), worldPt.get(2),
                                                lum, lum, lum
                                            )
                                        )
                                    }
                                }
                            }
                        }
                        nextPts.release()
                        status.release()
                        err.release()
                    }
                    corners.release()
                    prevPts.release()

                    // Aktualizujeme referenčnú snímku až po úspešnom posune o 3 cm
                    prevGray?.release()
                    prevGray = currGray
                    prevPose = camPose
                } else if (baseDist > 0.35f) {
                    prevGray?.release()
                    prevGray = currGray
                    prevPose = camPose
                } else {
                    currGray.release()
                }
            } else {
                prevGray?.release()
                prevGray = currGray
                prevPose = camPose
            }
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

    fun clear() {
        pointCloudList.clear()
        prevGray?.release()
        prevGray = null
        prevPose = null
        lastError = ""
    }

    fun exportToDownloads(context: Context) {
        if (pointCloudList.isEmpty()) {
            Toast.makeText(context, "0 bodov – urobte krok do strany s mobilom", Toast.LENGTH_LONG).show()
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
