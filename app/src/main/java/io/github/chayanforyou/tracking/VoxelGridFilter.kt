package io.github.chayanforyou.tracking

import kotlin.math.floor

data class VoxelKey(val x: Int, val y: Int, val z: Int)

class VoxelGridFilter(
    // Veľkosť jednej kocky v metroch (0.05f = 5 cm)
    private val voxelSize: Float = 0.05f
) {
    // Pamätá si obsadený voxel a index bodu v pointCloudList
    private val occupiedVoxels = HashMap<VoxelKey, Int>()

    /**
     * Vráti existujúci index bodu, ak už v tomto voxeli bod je.
     * Ak je voxel prázdny, uloží nový index (newIndex) a vráti -1 (znak, že ide o NOVÝ bod).
     */
    fun checkOrInsert(x: Float, y: Float, z: Float, newIndex: Int): Int {
        val vx = floor(x / voxelSize).toInt()
        val vy = floor(y / voxelSize).toInt()
        val vz = floor(z / voxelSize).toInt()

        val key = VoxelKey(vx, vy, vz)
        val existingIndex = occupiedVoxels[key]
        return if (existingIndex != null) {
            existingIndex // Bod už existuje, vrátime jeho index pre plochy (faces)
        } else {
            occupiedVoxels[key] = newIndex
            -1 // Nový voxel!
        }
    }

    fun clear() {
        occupiedVoxels.clear()
    }
}
