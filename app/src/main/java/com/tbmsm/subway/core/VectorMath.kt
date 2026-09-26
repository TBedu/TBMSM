package com.tbmsm.subway.core

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

const val TWO_PI = 2.0 * Math.PI

/** 把角度归一化到 (-pi, pi]，用于航向差。 */
fun wrapPi(a: Double): Double {
    var r = a % TWO_PI
    if (r > Math.PI) r -= TWO_PI
    if (r <= -Math.PI) r += TWO_PI
    return r
}

/**
 * 三维向量。状态量里使用可变版本以减少 GC 压力。
 * 坐标约定：导航系为 ENU（x=东, y=北, z=天），机体系为 FLU（x=前, y=左, z=上）。
 */
class Vec3(var x: Double = 0.0, var y: Double = 0.0, var z: Double = 0.0) {

    fun copy(): Vec3 = Vec3(x, y, z)

    fun set(o: Vec3): Vec3 {
        x = o.x; y = o.y; z = o.z
        return this
    }

    fun set(a: Double, b: Double, c: Double): Vec3 {
        x = a; y = b; z = c
        return this
    }

    fun zero(): Vec3 = set(0.0, 0.0, 0.0)

    operator fun plus(o: Vec3): Vec3 = Vec3(x + o.x, y + o.y, z + o.z)

    operator fun minus(o: Vec3): Vec3 = Vec3(x - o.x, y - o.y, z - o.z)

    operator fun unaryMinus(): Vec3 = Vec3(-x, -y, -z)

    operator fun times(s: Double): Vec3 = Vec3(x * s, y * s, z * s)

    operator fun div(s: Double): Vec3 = Vec3(x / s, y / s, z / s)

    fun dot(o: Vec3): Double = x * o.x + y * o.y + z * o.z

    fun cross(o: Vec3): Vec3 = Vec3(
        y * o.z - z * o.y,
        z * o.x - x * o.z,
        x * o.y - y * o.x
    )

    fun norm(): Double = sqrt(x * x + y * y + z * z)

    fun norm2(): Double = x * x + y * y + z * z

    fun normalized(): Vec3 {
        val n = norm()
        return if (n < 1e-15) Vec3(0.0, 0.0, 0.0) else Vec3(x / n, y / n, z / n)
    }

    override fun toString(): String = String.format("(%.4f, %.4f, %.4f)", x, y, z)

    companion object {
        fun zero(): Vec3 = Vec3(0.0, 0.0, 0.0)
    }
}

/**
 * 3x3 矩阵，行主序。
 */
class Mat3(
    val a00: Double, val a01: Double, val a02: Double,
    val a10: Double, val a11: Double, val a12: Double,
    val a20: Double, val a21: Double, val a22: Double
) {

    fun row(i: Int): Vec3 = when (i) {
        0 -> Vec3(a00, a01, a02)
        1 -> Vec3(a10, a11, a12)
        else -> Vec3(a20, a21, a22)
    }

    fun col(j: Int): Vec3 = when (j) {
        0 -> Vec3(a00, a10, a20)
        1 -> Vec3(a01, a11, a21)
        else -> Vec3(a02, a12, a22)
    }

    operator fun times(v: Vec3): Vec3 = Vec3(
        a00 * v.x + a01 * v.y + a02 * v.z,
        a10 * v.x + a11 * v.y + a12 * v.z,
        a20 * v.x + a21 * v.y + a22 * v.z
    )

    /** 返回 this * o。 */
    operator fun times(o: Mat3): Mat3 = Mat3(
        a00 * o.a00 + a01 * o.a10 + a02 * o.a20,
        a00 * o.a01 + a01 * o.a11 + a02 * o.a21,
        a00 * o.a02 + a01 * o.a12 + a02 * o.a22,

        a10 * o.a00 + a11 * o.a10 + a12 * o.a20,
        a10 * o.a01 + a11 * o.a11 + a12 * o.a21,
        a10 * o.a02 + a11 * o.a12 + a12 * o.a22,

        a20 * o.a00 + a21 * o.a10 + a22 * o.a20,
        a20 * o.a01 + a21 * o.a11 + a22 * o.a21,
        a20 * o.a02 + a21 * o.a12 + a22 * o.a22
    )

    fun transpose(): Mat3 = Mat3(
        a00, a10, a20,
        a01, a11, a21,
        a02, a12, a22
    )

    operator fun get(i: Int, j: Int): Double = when (i) {
        0 -> when (j) { 0 -> a00; 1 -> a01; else -> a02 }
        1 -> when (j) { 0 -> a10; 1 -> a11; else -> a12 }
        else -> when (j) { 0 -> a20; 1 -> a21; else -> a22 }
    }

    override fun toString(): String =
        "[%.6f %.6f %.6f]\n[%.6f %.6f %.6f]\n[%.6f %.6f %.6f]"
            .format(a00, a01, a02, a10, a11, a12, a20, a21, a22)

    companion object {
        fun identity(): Mat3 = Mat3(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)

        /** 反对称矩阵 [v]x，满足 [v]x * w = v x w。 */
        fun skew(v: Vec3): Mat3 = Mat3(
            0.0, -v.z, v.y,
            v.z, 0.0, -v.x,
            -v.y, v.x, 0.0
        )
    }
}

/**
 * 单位四元数（Hamilton 约定，w 为实部），表示机体系 b -> 导航系 n 的旋转。
 */
class Quat(
    val w: Double,
    val x: Double,
    val y: Double,
    val z: Double
) {

    fun normalized(): Quat {
        val n = sqrt(w * w + x * x + y * y + z * z)
        return if (n < 1e-15) Quat(1.0, 0.0, 0.0, 0.0) else Quat(w / n, x / n, y / n, z / n)
    }

    /** 四元数乘法，this * o。 */
    operator fun times(o: Quat): Quat = Quat(
        w * o.w - x * o.x - y * o.y - z * o.z,
        w * o.x + x * o.w + y * o.z - z * o.y,
        w * o.y - x * o.z + y * o.w + z * o.x,
        w * o.z + x * o.y - y * o.x + z * o.w
    )

    fun conjugate(): Quat = Quat(w, -x, -y, -z)

    fun rotate(v: Vec3): Vec3 = toMat3() * v

    fun toMat3(): Mat3 {
        val xx = x * x; val yy = y * y; val zz = z * z
        val xy = x * y; val xz = x * z; val yz = y * z
        val wx = w * x; val wy = w * y; val wz = w * z
        return Mat3(
            1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy),
            2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
            2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)
        )
    }

    /** 旋转向量指数映射（小角度亦可稳定使用）。 */
    fun toRotationVector(): Vec3 {
        val n = sqrt(x * x + y * y + z * z)
        if (n < 1e-12) return Vec3(2.0 * x, 2.0 * y, 2.0 * z)
        val angle = 2.0 * atan2(n, w)
        val s = angle / n
        return Vec3(x * s, y * s, z * s)
    }

    override fun toString(): String = String.format("[%.5f, %.5f, %.5f, %.5f]", w, x, y, z)

    companion object {
        fun identity(): Quat = Quat(1.0, 0.0, 0.0, 0.0)

        /** 由旋转向量做指数映射。 */
        fun fromRotationVector(v: Vec3): Quat {
            val angle = v.norm()
            if (angle < 1e-9) {
                return Quat(1.0, 0.5 * v.x, 0.5 * v.y, 0.5 * v.z).normalized()
            }
            val half = 0.5 * angle
            val s = sin(half) / angle
            return Quat(cos(half), v.x * s, v.y * s, v.z * s)
        }

        fun fromAxisAngle(axis: Vec3, angle: Double): Quat {
            val u = axis.normalized()
            val half = 0.5 * angle
            val s = sin(half)
            return Quat(cos(half), u.x * s, u.y * s, u.z * s)
        }

        /** 由航向/俯仰/横滚构造 C_b^n（导航系 ENU，机体系 FLU）。 */
        fun fromYawPitchRoll(yaw: Double, pitch: Double, roll: Double): Quat {
            val qz = Quat(cos(0.5 * yaw), 0.0, 0.0, sin(0.5 * yaw))
            val qy = Quat(cos(0.5 * pitch), 0.0, sin(0.5 * pitch), 0.0)
            val qx = Quat(cos(0.5 * roll), sin(0.5 * roll), 0.0, 0.0)
            return (qz * qy * qx).normalized()
        }
    }
}

/** 由 C_b^n 提取欧拉角：返回 [heading, pitch, roll]，单位 rad。 */
fun eulerFromMatrix(c: Mat3): DoubleArray {
    // 机体系 x 轴（前进方向）在导航系中的表示
    val forward = Vec3(c.a00, c.a10, c.a20)
    // 导航系天向在机体系中的表示 = C_b^n 转置的第三列
    val upInBody = Vec3(c.a02, c.a12, c.a22)
    val heading = atan2(forward.x, forward.y)
    val pitch = atan2(-upInBody.x, sqrt(upInBody.y * upInBody.y + upInBody.z * upInBody.z))
    val roll = atan2(upInBody.y, upInBody.z)
    return doubleArrayOf(heading, pitch, roll)
}

/** 判断数值是否有限，用于拦截 NaN 污染滤波器。 */
fun isFinite(v: Double): Boolean = !v.isNaN() && !v.isInfinite()

fun isFinite(v: Vec3): Boolean = isFinite(v.x) && isFinite(v.y) && isFinite(v.z)
