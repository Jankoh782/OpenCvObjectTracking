package io.github.chayanforyou.tracking

import android.content.ContentValues
import android.content.Context
import android.media.Image
import android.os.Build
import android.os.Environment
import android.provider.MediaStore
import android.widget.Toast
import com.google.ar.core.Frame
import com.google.ar.core.Pose
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfByte
import org.opencv.core.MatOfFloat
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
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
    private val voxelFilter = VoxelGridFilter(0.05f)
    fun addTargetMarker(x: Float, y: Float, z: Float) {
        // Pridá zhluk 5 jasne červených bodov (255, 0, 0), aby bol cieľ na mape neprehliadnuteľný
        val offsets = floatArrayOf(0f, 0.02f, -0.02f)
        for (ox in offsets) {
            pointCloudList.add(floatArrayOf(x + ox, y, z, 255f, 0f, 0f))
            pointCloudList.add(floatArrayOf(x, y + ox, z, 255f, 0f, 0f))
        }
    }

    private var prevGray: Mat? = null
    private var prevPose: Pose? = null
    private var virtualStepZ = 0f

    fun collectFromFrame(frame: Frame) {
        val camPose = frame.camera.pose
        var camImage: Image? = null
        try {
            camImage = frame.acquireCameraImage()
            val currGray = yuvToSmallGray(camImage)
            val pGray = prevGray
            val pPose = prevPose

            if (pGray != null && pPose != null) {
                val realDist = distance(camPose, pPose.tx(), pPose.ty(), pPose.tz())
                val useVirtual = (realDist < 0.015f)
                val baseDist = if (useVirtual) 0.04f else realDist

                if (baseDist <= 0.45f) {
                    val cols = 32
                    val rows = 24
                    val stepX = 320f / (cols + 1)
                    val stepY = 240f / (rows + 1)

                    val gridPoints = ArrayList<Point>(cols * rows)
                    for (r in 1..rows) {
                        for (c in 1..cols) {
                            gridPoints.add(Point((c * stepX).toDouble(), (r * stepY).toDouble()))
                        }
                    }

                    val prevPts = MatOfPoint2f(*gridPoints.toTypedArray())
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

                    val gridIndices = IntArray(cols * rows) { -1 }
                    var addedNow = 0

                    for (idx in pArr.indices) {
                        if (stArr.get(idx).toInt() == 1 && pointCloudList.size < 300000) {
                            val px = pArr.get(idx).x.toInt().coerceIn(0, pGray.cols() - 1)
                            val py = pArr.get(idx).y.toInt().coerceIn(0, pGray.rows() - 1)
                            val dxPx = nArr.get(idx).x - pArr.get(idx).x
                            val dyPx = nArr.get(idx).y - pArr.get(idx).y
                            val pixelShift = sqrt(dxPx * dxPx + dyPx * dyPx).toFloat()

                            val depthZ = if (pixelShift > 1.2f) {
                                ((fx * baseDist) / pixelShift).coerceIn(0.4f, 3.5f)
                            } else {
                                2.5f
                            }

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

                           val existingIdx = voxelFilter.checkOrInsert(wx, wy, wz, pointCloudList.size)
if (existingIdx == -1) {
    gridIndices[idx] = pointCloudList.size
    pointCloudList.add(floatArrayOf(wx, wy, wz, lum, lum, lum))
    addedNow++
} else {
    gridIndices[idx] = existingIdx
}

                        }
                    }

                    // Spojenie susedných bodov mriežky do súvislých 3D plôch
                    for (r in 0 until rows - 1) {
                        for (c in 0 until cols - 1) {
                            val i00 = gridIndices[r * cols + c]
                            val i10 = gridIndices[r * cols + (c + 1)]
                            val i01 = gridIndices[(r + 1) * cols + c]
                            val i11 = gridIndices[(r + 1) * cols + (c + 1)]

                            if (i00 != -1 && i10 != -1 && i01 != -1) {
                                if (isClose(i00, i10) && isClose(i00, i01)) {
                                    facesList.add(intArrayOf(i00, i01, i10))
                                }
                            }
                            if (i10 != -1 && i01 != -1 && i11 != -1) {
                                if (isClose(i11, i10) && isClose(i11, i01)) {
                                    facesList.add(intArrayOf(i10, i01, i11))
                                }
                            }
                        }
                    }

                    if (addedNow > 0 && useVirtual) virtualStepZ += 0.03f
                    debugInfo = "Plochy: ${facesList.size}, Body: ${pointCloudList.size}"

                    prevPts.release()
                    nextPts.release()
                    status.release()
                    err.release()
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

    private fun isClose(idxA: Int, idxB: Int): Boolean {
        val a = pointCloudList.get(idxA)
        val b = pointCloudList.get(idxB)
        val dx = a.get(0) - b.get(0)
        val dy = a.get(1) - b.get(1)
        val dz = a.get(2) - b.get(2)
        return (dx * dx + dy * dy + dz * dz) < 0.16f // Max 40 cm medzera medzi susednými bodmi
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
        facesList.clear()
        prevGray?.release()
        prevGray = null
        prevPose = null
        virtualStepZ = 0f
        voxelFilter.clear()
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
            sb.append("property float z\n")
            sb.append("property uchar red\n")
            sb.append("property uchar green\n")
            sb.append("property uchar blue\n")
            sb.append("element face ${facesList.size}\n")
            sb.append("property list uchar int vertex_indices\n")
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

            for (f in facesList) {
                sb.append("3 ${f.get(0)} ${f.get(1)} ${f.get(2)}\n")
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
