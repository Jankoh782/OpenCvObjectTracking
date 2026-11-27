package io.github.chayanforyou.tracking

import android.content.ContentValues
import android.content.Context
import android.os.Build
import android.os.Environment
import android.provider.MediaStore
import android.widget.Toast
import com.google.ar.core.Frame
import com.google.ar.core.Plane
import com.google.ar.core.TrackingState
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.concurrent.CopyOnWriteArrayList
import kotlin.math.ceil
import kotlin.math.hypot

class PlyExporter {
    val pointCloudList = CopyOnWriteArrayList<FloatArray>()
    private val facesList = CopyOnWriteArrayList<IntArray>()
    var debugInfo: String = "Pripravený"
    private val voxelFilter = VoxelGridFilter(0.03f)

    fun addTargetMarker(x: Float, y: Float, z: Float) {
        val offsets = floatArrayOf(0f, 0.02f, -0.02f)
        for (ox in offsets) {
            pointCloudList.add(floatArrayOf(x + ox, y, z, 255f, 0f, 0f))
            pointCloudList.add(floatArrayOf(x, y + ox, z, 255f, 0f, 0f))
        }
    }

    fun collectFromFrame(frame: Frame) {
        if (pointCloudList.size >= 300_000) return

        var addedNow = 0
        var planeCount = 0

        // 1. ZBER 3D GEOMETRIE Z ROZPOZNANÝCH PLÔCH (PODLAHA, STENY, NÁBYTOK)
        try {
            val planes = frame.getUpdatedTrackables(Plane::class.java)
            for (plane in planes) {
                if (plane.trackingState != TrackingState.TRACKING) continue
                if (plane.subsumedBy != null) continue
                planeCount++

                val poly = plane.polygon
                poly.rewind()
                val numVerts = poly.limit() / 2
                if (numVerts < 3) continue

                val pose = plane.centerPose
                val worldVerts = ArrayList<FloatArray>(numVerts)

                for (i in 0 until numVerts) {
                    val lx = poly.get(i * 2)
                    val lz = poly.get(i * 2 + 1)
                    val wp = pose.transformPoint(floatArrayOf(lx, 0f, lz))
                    worldVerts.add(wp)
                }

                val centerWorld = pose.transformPoint(floatArrayOf(0f, 0f, 0f))

                // Vykreslíme hustý obrys plochy aj lúče zo stredu (mriežka po 4 cm)
                for (i in 0 until numVerts) {
                    val p1 = worldVerts.get(i)
                    val p2 = worldVerts.get((i + 1) % numVerts)

                    // Obrysová hrana
                    val edgeLen = hypot((p2[0] - p1[0]).toDouble(), (p2[2] - p1[2]).toDouble()).toFloat()
                    val edgeSteps = ceil(edgeLen / 0.04f).toInt().coerceAtLeast(1)
                    for (s in 0..edgeSteps) {
                        val t = s.toFloat() / edgeSteps
                        val wx = p1[0] + (p2[0] - p1[0]) * t
                        val wy = p1[1] + (p2[1] - p1[1]) * t
                        val wz = p1[2] + (p2[2] - p1[2]) * t

                        if (voxelFilter.checkOrInsert(wx, wy, wz, pointCloudList.size) == -1) {
                            pointCloudList.add(floatArrayOf(wx, wy, wz, 0f, 255f, 180f))
                            addedNow++
                        }

                        // Výplň plochy smerom do stredu
                        val radLen = hypot((wx - centerWorld[0]).toDouble(), (wz - centerWorld[2]).toDouble()).toFloat()
                        val radSteps = ceil(radLen / 0.06f).toInt().coerceAtLeast(1)
                        for (r in 1 until radSteps) {
                            val rt = r.toFloat() / radSteps
                            val ix = centerWorld[0] + (wx - centerWorld[0]) * rt
                            val iy = centerWorld[1] + (wy - centerWorld[1]) * rt
                            val iz = centerWorld[2] + (wz - centerWorld[2]) * rt

                            if (voxelFilter.checkOrInsert(ix, iy, iz, pointCloudList.size) == -1) {
                                pointCloudList.add(floatArrayOf(ix, iy, iz, 0f, 210f, 150f))
                                addedNow++
                            }
                        }
                    }
                }
            }
        } catch (_: Exception) {
        }

        // 2. DOPLNENIE SUROVÝCH BODOV Z POINTCLOUD (AK SÚ DOSTUPNÉ)
        try {
            frame.acquirePointCloud().use { pointCloud ->
                val points = pointCloud.points
                points.rewind()
                val numPoints = points.limit() / 4
                for (i in 0 until numPoints) {
                    val x = points.get(i * 4)
                    val y = points.get(i * 4 + 1)
                    val z = points.get(i * 4 + 2)
                    if (x.isNaN() || y.isNaN() || z.isNaN() || x * x + y * y + z * z > 16.0f) continue

                    if (voxelFilter.checkOrInsert(x, y, z, pointCloudList.size) == -1) {
                        pointCloudList.add(floatArrayOf(x, y, z, 0f, 180f, 255f))
                        addedNow++
                    }
                }
            }
        } catch (_: Exception) {
        }

        debugInfo = "Plochy: $planeCount | Body: ${pointCloudList.size} (+$addedNow)"
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
