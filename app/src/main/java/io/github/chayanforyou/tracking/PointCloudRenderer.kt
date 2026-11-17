package io.github.chayanforyou.tracking

import android.opengl.GLES20
import android.opengl.Matrix
import com.google.ar.core.Camera
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer

class PointCloudRenderer {
    private var program = 0
    private var positionAttrib = 0
    private var colorAttrib = 0
    private var mvpMatrixUniform = 0
    private var pointSizeUniform = 0

    private val viewMatrix = FloatArray(16)
    private val projMatrix = FloatArray(16)
    private val vpMatrix = FloatArray(16)

    private val maxRenderPoints = 60000
    private val vertexBuffer: FloatBuffer = ByteBuffer.allocateDirect(maxRenderPoints * 6 * 4)
        .order(ByteOrder.nativeOrder())
        .asFloatBuffer()

    fun createOnGlThread() {
        val vShader = loadShader(GLES20.GL_VERTEX_SHADER, """
            uniform mat4 u_ModelViewProjection;
            uniform float u_PointSize;
            attribute vec3 a_Position;
            attribute vec3 a_Color;
            varying vec3 v_Color;
            void main() {
                gl_Position = u_ModelViewProjection * vec4(a_Position, 1.0);
                gl_PointSize = u_PointSize;
                v_Color = a_Color;
            }
        """.trimIndent())

        val fShader = loadShader(GLES20.GL_FRAGMENT_SHADER, """
            precision mediump float;
            varying vec3 v_Color;
            void main() {
                gl_FragColor = vec4(v_Color, 0.85);
            }
        """.trimIndent())

        program = GLES20.glCreateProgram().also {
            GLES20.glAttachShader(it, vShader)
            GLES20.glAttachShader(it, fShader)
            GLES20.glLinkProgram(it)
            positionAttrib = GLES20.glGetAttribLocation(it, "a_Position")
            colorAttrib = GLES20.glGetAttribLocation(it, "a_Color")
            mvpMatrixUniform = GLES20.glGetUniformLocation(it, "u_ModelViewProjection")
            pointSizeUniform = GLES20.glGetUniformLocation(it, "u_PointSize")
        }
    }

    fun draw(camera: Camera, points: List<FloatArray>) {
        if (points.isEmpty()) return

        camera.getViewMatrix(viewMatrix, 0)
        camera.getProjectionMatrix(projMatrix, 0, 0.1f, 100.0f)
        Matrix.multiplyMM(vpMatrix, 0, projMatrix, 0, viewMatrix, 0)

        vertexBuffer.clear()
        val total = points.size
        val step = if (total > maxRenderPoints) (total / maxRenderPoints) + 1 else 1
        var count = 0

        var i = 0
        while (i < total && count < maxRenderPoints) {
            val pt = points.get(i)
            vertexBuffer.put(pt.get(0))
            vertexBuffer.put(pt.get(1))
            vertexBuffer.put(pt.get(2))
            vertexBuffer.put(pt.get(3) / 255f)
            vertexBuffer.put(pt.get(4) / 255f)
            vertexBuffer.put(pt.get(5) / 255f)
            count++
            i += step
        }
        vertexBuffer.flip()

        GLES20.glUseProgram(program)
        GLES20.glUniformMatrix4fv(mvpMatrixUniform, 1, false, vpMatrix, 0)
        GLES20.glUniform1f(pointSizeUniform, 12.0f)

        val stride = 6 * 4
        vertexBuffer.position(0)
        GLES20.glVertexAttribPointer(positionAttrib, 3, GLES20.GL_FLOAT, false, stride, vertexBuffer)
        GLES20.glEnableVertexAttribArray(positionAttrib)

        vertexBuffer.position(3)
        GLES20.glVertexAttribPointer(colorAttrib, 3, GLES20.GL_FLOAT, false, stride, vertexBuffer)
        GLES20.glEnableVertexAttribArray(colorAttrib)

        GLES20.glDrawArrays(GLES20.GL_POINTS, 0, count)

        GLES20.glDisableVertexAttribArray(positionAttrib)
        GLES20.glDisableVertexAttribArray(colorAttrib)
    }

    private fun loadShader(type: Int, code: String): Int {
        return GLES20.glCreateShader(type).also { shader ->
            GLES20.glShaderSource(shader, code)
            GLES20.glCompileShader(shader)
        }
    }
}
