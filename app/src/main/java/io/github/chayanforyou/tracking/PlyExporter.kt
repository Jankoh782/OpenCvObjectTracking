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
       fun collectFromFrame(frame: Frame) {
        if (pointCloudList.size >= 300_000) return

        var addedNow = 0
        var usedDepth = false
        var depthErr = ""

        try {
            // 1. HUSTÁ 3D MAPA STIEN A PREKÁŽOK (ARCore Depth API)
            frame.acquireDepthImage16Bits().use { depthImage ->
                val width = depthImage.width
                val height = depthImage.height
                val plane = depthImage.planes[0]
                // DÔLEŽITÉ: Little Endian poradie bajtov pre správne milimetre!
                val buffer = plane.buffer.order(java.nio.ByteOrder.LITTLE_ENDIAN).asShortBuffer()
                val rowStride = plane.rowStride / 2

                val camera = frame.camera
                val intrinsics = camera.imageIntrinsics
                val focalLength = intrinsics.focalLength
                val principalPoint = intrinsics.principalPoint
                val imgDims = intrinsics.imageDimensions

                val scaleX = width.toFloat() / imgDims[0]
                val scaleY = height.toFloat() / imgDims[1]
                val fx = focalLength[0] * scaleX
                val fy = focalLength[1] * scaleY
                val cx = principalPoint[0] * scaleX
                val cy = principalPoint[1] * scaleY

                val pose = camera.pose

                for (y in 0 until height step 2) {
                    for (x in 0 until width step 2) {
                       val rawDepth = buffer.get(y * rowStride + x).toInt() and 0x1FFF
                        // Platná vzdialenosť od 25 cm do 4.5 metra
                        if (rawDepth in 250..4500) {
                            val zMeters = rawDepth / 1000f
                            val localX = (x - cx) * zMeters / fx
                            val localY = (cy - y) * zMeters / fy
                            val localZ = -zMeters

                            val worldPt = pose.transformPoint(floatArrayOf(localX, localY, localZ))
                            val wx = worldPt[0]
                            val wy = worldPt[1]
                            val wz = worldPt[2]

                            if (voxelFilter.checkOrInsert(wx, wy, wz, pointCloudList.size) == -1) {
                                val g = ((wy + 1.0f) * 110f).coerceIn(20f, 180f)
                                pointCloudList.add(floatArrayOf(wx, wy, wz, 15f, g, 210f))
                                addedNow++
                            }
                        }
                    }
                }
                usedDepth = true
            }
              } catch (e: Exception) {
            depthErr = e.javaClass.simpleName
        }

        // 2. ZÁLOŽNÝ REŽIM (zbiera všetky body z ARCore bez obmedzenia)
        var pcErr = ""
        if (!usedDepth) {
            try {
                frame.acquirePointCloud().use { pointCloud ->
                    val points = pointCloud.points
                    val numPoints = points.remaining() / 4
                                      for (i in 0 until numPoints) {
                        val x = points.get(i * 4)
                        val y = points.get(i * 4 + 1)
                        val z = points.get(i * 4 + 2)
                        val conf = points.get(i * 4 + 3)

                                                // Odfiltrujeme iba chybné ďaleké body (> 4 m) a NaN bez obmedzenia istoty (conf)
                        if (x.isNaN() || y.isNaN() || z.isNaN() || x * x + y * y + z * z > 16.0f) continue

                        if (voxelFilter.checkOrInsert(x, y, z, pointCloudList.size) == -1) {
                            // Žiarivá tyrkysovo-zelená farba
                            pointCloudList.add(floatArrayOf(x, y, z, 0f, 255f, 180f))
                            addedNow++
                        }
                    }
                }
            } catch (e: Exception) {
                pcErr = e.javaClass.simpleName
            }
        }

              debugInfo = if (usedDepth) {
            "Depth OK: ${pointCloudList.size} (+$addedNow)"
        } else if (pcErr.isNotEmpty()) {
            "Pohnite na hrany/nábytok | Body: ${pointCloudList.size}"
        } else {
            "Skenujem: ${pointCloudList.size} (+$addedNow)"
        }
    }

      fun clear() {

        pointCloudList.clear()
        facesList.clear()
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
