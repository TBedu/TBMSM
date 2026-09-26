package com.tbmsm.subway.algo

import com.tbmsm.subway.core.Vec3
import kotlin.math.sqrt

/**
 * 零速检测器：Skog 等人的 GLRT（广义似然比检验）+ 进入/退出迟滞。
 *
 * 检验统计量（窗口 W 内取平均）：
 *
 *   T_k = (1/W) * Σ [ ||a_j - g * â_k||^2 / σa^2  +  ||ω_j||^2 / σg^2 ]
 *
 * 其中 â_k 为窗口内加计均值方向。静止时 T_k 服从 6 自由度卡方分布（期望 6），
 * 运动时（尤其是列车振动）T_k 会显著增大，因此门限可设得比较宽松也不会误判。
 *
 * 迟滞策略：
 *  - 进入静止：连续 zuptEnterHold 个采样满足 T < γ（默认 1.0s）
 *  - 退出静止：连续 zuptExitHold 个采样不满足（默认 0.05s，防止停站期间误退出）
 *
 * 安全阀：若外部给出的估计速度超过 zuptMaxSpeedGuard，强制拒绝 ZUPT，
 * 用于防御「列车匀速平顺运行时被误判为静止」这一最危险的失效模式。
 */
class ZuptDetector(private val params: NavParams) {

    private val window = params.zuptWindow.coerceAtLeast(5)
    private val accX = DoubleArray(window)
    private val accY = DoubleArray(window)
    private val accZ = DoubleArray(window)
    private val gyroX = DoubleArray(window)
    private val gyroY = DoubleArray(window)
    private val gyroZ = DoubleArray(window)
    private var count = 0
    private var head = 0

    var isStill: Boolean = false
        private set

    /** 最近一次 GLRT 统计量，用于界面展示与调试。 */
    var lastStatistic: Double = Double.MAX_VALUE
        private set

    private var belowCount = 0
    private var aboveCount = 0

    /** 累计判定为静止的采样点数。 */
    var stillSampleCount: Long = 0
        private set

    /** 累计 ZUPT 被安全阀拦截的次数。 */
    var guardRejectCount: Long = 0
        private set

    /** 是否已积累满一个完整窗口，未满之前统计量不可信。 */
    val warmedUp: Boolean get() = count >= window

    fun reset() {
        count = 0
        head = 0
        isStill = false
        lastStatistic = Double.MAX_VALUE
        belowCount = 0
        aboveCount = 0
        stillSampleCount = 0
        guardRejectCount = 0
    }

    /**
     * 输入一帧机体系加计与陀螺（已完成安装矩阵转换），返回本帧是否判定为静止。
     *
     * @param estimatedSpeed 滤波器当前速度模长，用于安全阀；不确定时传 0.0。
     */
    fun update(accBody: Vec3, gyroBody: Vec3, estimatedSpeed: Double): Boolean {
        // 写入环形缓冲
        accX[head] = accBody.x
        accY[head] = accBody.y
        accZ[head] = accBody.z
        gyroX[head] = gyroBody.x
        gyroY[head] = gyroBody.y
        gyroZ[head] = gyroBody.z
        head = (head + 1) % window
        if (count < window) count++

        if (count < window) {
            lastStatistic = Double.MAX_VALUE
            return isStill
        }

        // 窗口内加计均值，用于估计重力方向
        var sx = 0.0
        var sy = 0.0
        var sz = 0.0
        for (i in 0 until window) {
            sx += accX[i]; sy += accY[i]; sz += accZ[i]
        }
        val inv = 1.0 / window
        var mx = sx * inv
        var my = sy * inv
        var mz = sz * inv
        val norm = sqrt(mx * mx + my * my + mz * mz)
        if (norm < 1e-6) {
            lastStatistic = Double.MAX_VALUE
            return isStill
        }
        val scale = params.g / norm
        mx *= scale; my *= scale; mz *= scale

        val invSa2 = 1.0 / (params.zuptSigmaAcc * params.zuptSigmaAcc)
        val invSg2 = 1.0 / (params.zuptSigmaGyro * params.zuptSigmaGyro)
        var t = 0.0
        for (i in 0 until window) {
            val dx = accX[i] - mx
            val dy = accY[i] - my
            val dz = accZ[i] - mz
            t += (dx * dx + dy * dy + dz * dz) * invSa2
            t += (gyroX[i] * gyroX[i] + gyroY[i] * gyroY[i] + gyroZ[i] * gyroZ[i]) * invSg2
        }
        t *= inv
        lastStatistic = t

        val stillNow = t < params.zuptThreshold

        if (stillNow) {
            aboveCount++
            belowCount = 0
            if (!isStill && aboveCount >= params.zuptEnterHold) {
                isStill = true
            }
        } else {
            belowCount++
            aboveCount = 0
            if (isStill && belowCount >= params.zuptExitHold) {
                isStill = false
            }
        }

        // 安全阀：已经在动（估计速度明显非零）时不接受 ZUPT
        if (isStill && estimatedSpeed > params.zuptMaxSpeedGuard) {
            guardRejectCount++
            return false
        }

        if (isStill) stillSampleCount++
        return isStill
    }

    /** 当前是否处于「窗口未满」的预热阶段。 */
    fun statisticText(): String =
        if (lastStatistic == Double.MAX_VALUE) "--" else String.format("%.1f", lastStatistic)
}
