package io.github.chayanforyou.tracking

import android.content.ContentValues
import android.content.Context
import android.os.Build
import android.os.Environment
import android.provider.MediaStore
import android.widget.Toast
import com.google.ar.core.Frame
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.concurrent.CopyOnWriteArrayList

class PlyExporter {
    val pointCloudList = CopyOnWriteArrayList<FloatArray>()

    fun collectFromFrame(frame: Frame) {
        try {
            frame.acquirePointCloud().use { pointCloud ->
                val pointsBuffer = pointCloud.points
                while (pointsBuffer.remaining() >= 4) {
                    val x = pointsBuffer.get()
                    val y = pointsBuffer.get()
                    val z = pointsBuffer.get()
                    val confidence = pointsBuffer.get()
                    if (confidence > 0.35f && pointCloudList.size < 150000) {
                        pointCloudList.add(floatArrayOf(x, y, z))
                    }
                }
            }
        } catch (_: Exception) {
        }
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
        val fileName = "ARCore_Scan_$timeStamp.ply"

        try {
            val sb = StringBuilder()
            sb.append("ply\n")
            sb.append("format ascii 1.0\n")
            sb.append("element vertex ${pointCloudList.size}\n")
            sb.append("property float x\n")
            sb.append("property float y\n")
            sb.append("property float z\n")
            sb.append("end_header\n")

            for (pt in pointCloudList) {
                sb.append(String.format(Locale.US, "%.4f %.4f %.4f\n", pt.get(0), pt.get(1), pt.get(2)))
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
            Toast.makeText(context, "Uložené do Stiahnuté (Downloads): $fileName", Toast.LENGTH_LONG).show()
        } catch (e: Exception) {
            Toast.makeText(context, "Chyba exportu: ${e.message}", Toast.LENGTH_LONG).show()
        }
    }
}
