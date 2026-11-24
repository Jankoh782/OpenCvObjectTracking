package io.github.chayanforyou.tracking

import kotlin.math.floor

data class VoxelKey(val x: Int, val y: Int, val z: Int)

class VoxelGridFilter(
    // Veľkosť jednej kocky v metroch (0.05f = 5 cm; pre jemnejšiu mapu dajte napr. 0.02f)
    private val voxelSize: Float = 0.05f
) {
    // Množina už navštívených voxelov
    private val occupiedVoxels = HashSet<VoxelKey>()

    /**
     * Vráti true, ak je bod v novom voxeli (ešte nebol vykreslený).
     * Vráti false, ak už na tomto mieste bod existuje (treba ho ignorovať).
     */
    fun isNewPoint(x: Float, y: Float, z: Float): Boolean {
        val vx = floor(x / voxelSize).toInt()
        val vy = floor(y / voxelSize).toInt()
        val vz = floor(z / voxelSize).toInt()

        val key = VoxelKey(vx, vy, vz)
        return occupiedVoxels.add(key) // vráti true iba ak kľúč v sete ešte nebol
    }

    // Vyčistí pamäť voxelov (zavoláte pri stlačení tlačidla Clear Map)
    fun clear() {
        occupiedVoxels.clear()
    }
}
