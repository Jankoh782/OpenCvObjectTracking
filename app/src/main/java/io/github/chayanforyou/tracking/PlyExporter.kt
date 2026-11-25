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

        try {
            // Získame skutočné 3D body priestoru z ARCore
            frame.acquirePointCloud().use { pointCloud ->
                val points = pointCloud.points // Buffer: [x, y, z, confidence, x, y, z, confidence...]
                val numPoints = points.remaining() / 4
                var addedNow = 0

                for (i in 0 until numPoints) {
                    val x = points.get(i * 4)
                    val y = points.get(i * 4 + 1)
                    val z = points.get(i * 4 + 2)
                    val confidence = points.get(i * 4 + 3)

                    // Ignorujeme neisté body (šum) a body príliš ďaleko (> 5 metrov)
                    if (confidence < 0.35f) continue
                    if (x * x + y * y + z * z > 25f) continue

                    // Preženieme cez tvoj VoxelGridFilter (kocky 5x5x5 cm)
                    val existingIdx = voxelFilter.checkOrInsert(x, y, z, pointCloudList.size)
                    if (existingIdx == -1) {
                        // Nový reálny 3D bod -> uložíme (biela/svetlosivá farba)
                        pointCloudList.add(floatArrayOf(x, y, z, 220f, 220f, 220f))
                        addedNow++
                    }
                }

                debugInfo = "Reálne 3D body: ${pointCloudList.size} (+$addedNow)"
            }
        } catch (e: Exception) {
            debugInfo = "Err: ${e.javaClass.simpleName}"
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
