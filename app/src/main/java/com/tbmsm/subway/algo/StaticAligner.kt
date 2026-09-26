package com.tbmsm.subway.algo

import com.tbmsm.subway.core.Mat3
import com.tbmsm.subway.core.Quat
import com.tbmsm.subway.core.Vec3
import com.tbmsm.subway.core.isFinite
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.sqrt

enum class AlignState { COLLECTING, READY, REJECTED }

/**
 * 粗对准结果。
 *
 * @param mounting C_m^b：手机坐标系 -> 机体系。由重力方向与「手机顶部朝前」的假设构造，
 *                  因此天然包含了手机没有完全放平时的安装横滚/俯仰。
 * @param initialAttitude 初始 C_b^n，其中偏航仍不可观测（由外部按轨道/磁力计给定或置 0）。
 * @param gyroBiasBody 机体系陀螺零偏初值。
 * @param isFlat 手机是否接近平放；若为 false，前向假设不可靠，会有较大安装偏航误差。
 * @param warnings 需要提示用户的告警。
 */
data class AlignResult(
    val mounting: Mat3,
    val initialAttitude: Quat,
    val gyroBiasBody: Vec3,
    val isFlat: Boolean,
    val projectionSlope: Double,
    val meanAcc: Vec3,
    val meanGyro: Vec3,
    val accVarMax: Double,
    val gyroVarMax: Double,
    val warnings: List<String>
)

/**
 * 静止粗对准：估计安装矩阵、陀螺零偏与初始横滚/俯仰。
 *
 * 关键点：静止时无法观测绕重力轴的安装偏航（例如手机在水平面内转了 10 度），
 * 该误差在数学上与初始航向误差等价，由 ESKF 的偏航状态吸收，
 * 因此初始偏航方差必须给足（见 NavParams.initAttSigma）。
 */
class StaticAligner(private val params: NavParams) {

    private val targetSamples: Int = (params.alignDurationS * 100.0).toInt().coerceAtLeast(50)

    private var n = 0
    private val sumAcc = Vec3.zero()
    private val sumGyro = Vec3.zero()
    private val sumAccSq = Vec3.zero()
    private val sumGyroSq = Vec3.zero()

    var rejectedBatches: Int = 0
        private set

    val progress: Float get() = (n.toFloat() / targetSamples).coerceIn(0f, 1f)
    val sampleCount: Int get() = n
    val requiredSamples: Int get() = targetSamples

    fun reset() {
        n = 0
        sumAcc.zero(); sumGyro.zero(); sumAccSq.zero(); sumGyroSq.zero()
        rejectedBatches = 0
    }

    /** 输入手机坐标系（Android 原始轴）的加计与陀螺样本。 */
    fun add(accPhone: Vec3, gyroPhone: Vec3): AlignState {
        sumAcc.x += accPhone.x; sumAcc.y += accPhone.y; sumAcc.z += accPhone.z
        sumGyro.x += gyroPhone.x; sumGyro.y += gyroPhone.y; sumGyro.z += gyroPhone.z
        sumAccSq.x += accPhone.x * accPhone.x
        sumAccSq.y += accPhone.y * accPhone.y
        sumAccSq.z += accPhone.z * accPhone.z
        sumGyroSq.x += gyroPhone.x * gyroPhone.x
        sumGyroSq.y += gyroPhone.y * gyroPhone.y
        sumGyroSq.z += gyroPhone.z * gyroPhone.z
        n++
        if (n < targetSamples) return AlignState.COLLECTING

        val inv = 1.0 / n
        val meanAcc = Vec3(sumAcc.x * inv, sumAcc.y * inv, sumAcc.z * inv)
        val meanGyro = Vec3(sumGyro.x * inv, sumGyro.y * inv, sumGyro.z * inv)
        val accVar = maxOf(
            sumAccSq.x * inv - meanAcc.x * meanAcc.x,
            sumAccSq.y * inv - meanAcc.y * meanAcc.y,
            sumAccSq.z * inv - meanAcc.z * meanAcc.z
        )
        val gyroVar = maxOf(
            sumGyroSq.x * inv - meanGyro.x * meanGyro.x,
            sumGyroSq.y * inv - meanGyro.y * meanGyro.y,
            sumGyroSq.z * inv - meanGyro.z * meanGyro.z
        )

        val ok = abs(meanAcc.norm() - params.g) < params.alignAccTolMps2 &&
            meanGyro.norm() < params.alignGyroTolRad &&
            accVar < params.alignAccVarMax &&
            gyroVar < params.alignGyroVarMax

        if (!ok) {
            rejectedBatches++
            reset()
            return AlignState.REJECTED
        }
        return AlignState.READY
    }

    /** 生成对准结果。仅在 add() 返回 READY 后调用。 */
    fun build(initialYawRad: Double, extraWarnings: List<String> = emptyList()): AlignResult {
        val inv = 1.0 / n.coerceAtLeast(1)
        val meanAcc = Vec3(sumAcc.x * inv, sumAcc.y * inv, sumAcc.z * inv)
        val meanGyro = Vec3(sumGyro.x * inv, sumGyro.y * inv, sumGyro.z * inv)
        val accVar = maxOf(0.0, sumAccSq.x * inv - meanAcc.x * meanAcc.x,
            sumAccSq.y * inv - meanAcc.y * meanAcc.y, sumAccSq.z * inv - meanAcc.z * meanAcc.z)
        val gyroVar = maxOf(0.0, sumGyroSq.x * inv - meanGyro.x * meanGyro.x,
            sumGyroSq.y * inv - meanGyro.y * meanGyro.y, sumGyroSq.z * inv - meanGyro.z * meanGyro.z)

        val warnings = ArrayList<String>()
        warnings.addAll(extraWarnings)

        // 手机系下的「天」方向：静止时加计输出即为比力方向 = 天向
        val upInPhone = meanAcc.normalized()
        // 假设手机顶部朝前：把手机 +y 轴投影到水平面
        val phoneY = Vec3(0.0, 1.0, 0.0)
        val proj = phoneY - upInPhone * phoneY.dot(upInPhone)
        val slope = proj.norm()

        val isFlat = slope > 0.5
        val forwardInPhone = if (isFlat) {
            proj.normalized()
        } else {
            // 手机接近竖放：无法从重力确定前向，退回标称安装矩阵并告警
            warnings.add("手机可能未平放（顶面未朝上），前向假设不可靠，安装偏航误差可能很大")
            Vec3(0.0, 1.0, 0.0)
        }

        val up = upInPhone
        var forward = forwardInPhone
        // 正交化，避免 up 与 forward 不完全垂直
        forward = (forward - up * forward.dot(up)).normalized()
        val left = up.cross(forward)

        val mounting = Mat3(
            forward.x, forward.y, forward.z,
            left.x, left.y, left.z,
            up.x, up.y, up.z
        )

        // 机体系下的重力方向 -> 初始横滚/俯仰
        val accBody = mounting * meanAcc
        val upInBody = if (accBody.norm() > 1e-6) accBody.normalized() else Vec3(0.0, 0.0, 1.0)
        val pitch = atan2(-upInBody.x, hypot(upInBody.y, upInBody.z))
        val roll = atan2(upInBody.y, upInBody.z)
        val initialAttitude = Quat.fromYawPitchRoll(initialYawRad, pitch, roll)

        val gyroBiasBody = mounting * meanGyro

        if (accVar > 0.002) {
            warnings.add("静止期间振动偏大，零偏估计可能不准，建议把手机放在地板或固定支架上")
        }

        return AlignResult(
            mounting = mounting,
            initialAttitude = initialAttitude,
            gyroBiasBody = gyroBiasBody,
            isFlat = isFlat,
            projectionSlope = slope,
            meanAcc = meanAcc,
            meanGyro = meanGyro,
            accVarMax = accVar,
            gyroVarMax = gyroVar,
            warnings = warnings
        )
    }

    /** 强制生成结果（超时或用户强制开始时使用，quality 会变差）。 */
    fun buildForced(initialYawRad: Double, extraWarnings: List<String>): AlignResult {
        val list = ArrayList<String>()
        list.add("未获得理想的静止窗口，对准结果可能不准")
        list.addAll(extraWarnings)
        return build(initialYawRad, list)
    }

    fun hasSamples(): Boolean = n > 0

    fun qualityText(): String {
        if (n == 0) return "无样本"
        val inv = 1.0 / n
        val meanAcc = Vec3(sumAcc.x * inv, sumAcc.y * inv, sumAcc.z * inv)
        val meanGyro = Vec3(sumGyro.x * inv, sumGyro.y * inv, sumGyro.z * inv)
        val accVar = maxOf(0.0, sumAccSq.x * inv - meanAcc.x * meanAcc.x,
            sumAccSq.y * inv - meanAcc.y * meanAcc.y, sumAccSq.z * inv - meanAcc.z * meanAcc.z)
        val gyroVar = maxOf(0.0, sumGyroSq.x * inv - meanGyro.x * meanGyro.x,
            sumGyroSq.y * inv - meanGyro.y * meanGyro.y, sumGyroSq.z * inv - meanGyro.z * meanGyro.z)
        return String.format(
            "样本 %d/%d  |a|=%.3f  |w|=%.4f  σa=%.4f  σw=%.4f  (有限值=%b)",
            n, targetSamples, meanAcc.norm(), meanGyro.norm(),
            sqrt(accVar), sqrt(gyroVar), isFinite(meanAcc)
        )
    }
}
