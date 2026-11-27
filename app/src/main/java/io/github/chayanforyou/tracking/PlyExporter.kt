package io.github.chayanforyou.tracking

import android.content.ContentValues
import android.content.Context
import android.os.Build
import android.os.Environment
import android.provider.MediaStore
import android.widget.Toast
import com.google.ar.core.Frame
import com.google.ar.core.Plane
import com.google.ar.core.Session
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
    private val voxelFilter = VoxelGridFilter(0.04f)

    fun addTargetMarker(x: Float, y: Float, z: Float) {
        val offsets = floatArrayOf(0f, 0.02f, -0.02f)
        for (ox in offsets) {
            pointCloudList.add(floatArrayOf(x + ox, y, z, 255f, 0f, 0f))
            pointCloudList.add(floatArrayOf(x, y + ox, z, 255f, 0f, 0f))
        }
    }

    fun collectFromFrame(frame: Frame, session: Session? = null) {
        if (pointCloudList.size >= 300_000) return

        var addedNow = 0
        var planeCount = 0

        // 1. ZBER 3D GEOMETRIE ZO VŠETKÝCH NÁJDENÝCH PLÔCH (PODLAHA, STENY, STÔL)
        try {
            val planes = session?.getAllTrackables(Plane::class.java)
                ?: frame.getUpdatedTrackables(Plane::class.java)

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

                for (i in 0 until numVerts) {
                    val p1 = worldVerts.get(i)
                    val p2 = worldVerts.get((i + 1) % numVerts)

                    val dx = p2.get(0) - p1.get(0)
                    val dy = p2.get(1) - p1.get(1)
                    val dz = p2.get(2) - p1.get(2)
                    val edgeLen = hypot(hypot(dx.toDouble(), dy.toDouble()), dz.toDouble()).toFloat()
                    val edgeSteps = ceil(edgeLen / 0.04f).toInt().coerceIn(1, 100)

                    for (s in 0..edgeSteps) {
                        val t = s.toFloat() / edgeSteps
                        val wx = p1.get(0) + dx * t
                        val wy = p1.get(1) + dy * t
                        val wz = p1.get(2) + dz * t

                        if (voxelFilter.checkOrInsert(wx, wy, wz, pointCloudList.size) == -1) {
                            pointCloudList.add(floatArrayOf(wx, wy, wz, 0f, 255f, 180f))
                            addedNow++
                        }

                        val rx = wx - centerWorld.get(0)
                        val ry = wy - centerWorld.get(1)
                        val rz = wz - centerWorld.get(2)
                        val radLen = hypot(hypot(rx.toDouble(), ry.toDouble()), rz.toDouble()).toFloat()
                        val radSteps = ceil(radLen / 0.08f).toInt().coerceIn(1, 50)

                        for (r in 1 until radSteps) {
                            val rt = r.toFloat() / radSteps
                            val ix = centerWorld.get(0) + rx * rt
                            val iy = centerWorld.get(1) + ry * rt
                            val iz = centerWorld.get(2) + rz * rt

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

        // 2. DOPLNENIE SUROVÝCH BODOV Z POINTCLOUD
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
