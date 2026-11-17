package io.github.chayanforyou.tracking

import android.content.ContentValues
import android.content.Context
import android.os.Build
import android.os.Environment
import android.provider.MediaStore
import android.widget.Toast
import com.google.ar.core.Frame
import java.nio.ByteOrder
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.concurrent.CopyOnWriteArrayList
import kotlin.math.abs
import kotlin.math.sqrt

class PlyExporter {
    val pointCloudList = CopyOnWriteArrayList<FloatArray>()

    fun collectFromFrame(frame: Frame) {
        var addedFromDepth = false
        try {
            frame.acquireDepthImage16Bits().use { depthImage ->
                val dWidth = depthImage.width
                val dHeight = depthImage.height
                val plane = depthImage.planes.first()
                val buffer = plane.buffer.order(ByteOrder.nativeOrder())

                val camera = frame.camera
                val pose = camera.pose
                val intrinsics = camera.imageIntrinsics
                val focalLength = intrinsics.focalLength
                val principalPoint = intrinsics.principalPoint
                val imgDims = intrinsics.imageDimensions

                val scaleX = dWidth.toFloat() / imgDims.get(0)
                val scaleY = dHeight.toFloat() / imgDims.get(1)
                val fx = focalLength.get(0) * scaleX
                val fy = focalLength.get(1) * scaleY
                val cx = principalPoint.get(0) * scaleX
                val cy = principalPoint.get(1) * scaleY

                val step = 4
                for (y in 0 until dHeight step step) {
                    for (x in 0 until dWidth step step) {
                        if (pointCloudList.size >= 300000) return

                        val byteIndex = y * plane.rowStride + x * plane.pixelStride
                        val depthMm = buffer.getShort(byteIndex).toInt() and 0xFFFF
                        val zMeters = depthMm / 1000f

                        if (zMeters in 0.2f..5.0f) {
                            val camX = (x - cx) * zMeters / fx
                            val camY = -(y - cy) * zMeters / fy
                            val camZ = -zMeters

                            val worldPt = pose.transformPoint(floatArrayOf(camX, camY, camZ))
                            addColoredPoint(worldPt.get(0), worldPt.get(1), worldPt.get(2), zMeters)
                            addedFromDepth = true
                        }
                    }
                }
            }
        } catch (_: Exception) {
        }

        // Univerzálny ARCore PointCloud bez filtra confidence
        if (!addedFromDepth) {
            try {
                val camPose = frame.camera.pose
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
                            val dx = x - camPose.tx()
                            val dy = y - camPose.ty()
                            val dz = z - camPose.tz()
                            val dist = sqrt(dx * dx + dy * dy + dz * dz)
                            addColoredPoint(x, y, z, dist)
                        }
                    }
                }
            } catch (_: Exception) {
            }
        }
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
    }

    fun exportToDownloads(context: Context) {
        if (pointCloudList.isEmpty()) {
            Toast.makeText(context, "Žiadne 3D body! Najprv kliknite na 'Štart 3D skenu'.", Toast.LENGTH_LONG).show()
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
