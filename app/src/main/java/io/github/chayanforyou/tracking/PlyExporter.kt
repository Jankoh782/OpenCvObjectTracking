package io.github.chayanforyou.tracking

import android.content.ContentValues
import android.content.Context
import android.os.Build
import android.os.Environment
import android.provider.MediaStore
import android.widget.Toast
import com.google.ar.core.Frame
import com.google.ar.core.Plane
import java.nio.ByteOrder
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.concurrent.CopyOnWriteArrayList
import kotlin.math.abs
import kotlin.math.sqrt

class PlyExporter {
    val pointCloudList = CopyOnWriteArrayList<FloatArray>()
    var lastError: String = ""

    fun collectFromFrame(frame: Frame) {
        val startCount = pointCloudList.size

        // 1. Pokus o ARCore PointCloud (surové čítanie buffera)
        try {
            val camPose = frame.camera.pose
            frame.acquirePointCloud().use { pointCloud ->
                val pointsBuffer = pointCloud.points
                val count = pointsBuffer.remaining() / 4
                for (i in 0 until count) {
                    if (pointCloudList.size >= 300000) break
                    val x = pointsBuffer.get()
                    val y = pointsBuffer.get()
                    val z = pointsBuffer.get()
                    pointsBuffer.get() // ID / confidence

                    if (x != 0f || y != 0f || z != 0f) {
                        val dx = x - camPose.tx()
                        val dy = y - camPose.ty()
                        val dz = z - camPose.tz()
                        val dist = sqrt(dx * dx + dy * dy + dz * dz)
                        addColoredPoint(x, y, z, dist)
                    }
                }
            }
        } catch (e: Exception) {
            lastError = "PC: ${e.javaClass.simpleName}"
        }

        // 2. Zaručený zber zo všetkých nájdených 3D plôch a mriežky na obrazovke (HitTest)
        if (pointCloudList.size == startCount) {
            try {
                val camPose = frame.camera.pose
                // Skenujeme mriežku 5x5 bodov v strede obrazu pomocou ARCore HitTest
                for (gx in 2..8 step 2) {
                    for (gy in 2..8 step 2) {
                        if (pointCloudList.size >= 300000) break
                        val sx = gx * 100f
                        val sy = gy * 150f
                        val hits = frame.hitTest(sx, sy)
                        if (hits.isNotEmpty()) {
                            val pose = hits.first().hitPose
                            val x = pose.tx()
                            val y = pose.ty()
                            val z = pose.tz()
                            val dx = x - camPose.tx()
                            val dy = y - camPose.ty()
                            val dz = z - camPose.tz()
                            val dist = sqrt(dx * dx + dy * dy + dz * dz)
                            addColoredPoint(x, y, z, dist)
                        }
                    }
                }
            } catch (e: Exception) {
                lastError = "HT: ${e.javaClass.simpleName}"
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
        lastError = ""
    }

    fun exportToDownloads(context: Context) {
        if (pointCloudList.isEmpty()) {
            val errInfo = if (lastError.isNotEmpty()) " ($lastError)" else ""
            Toast.makeText(context, "0 bodov$errInfo – mierte na stôl/podlahu so vzorom", Toast.LENGTH_LONG).show()
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
