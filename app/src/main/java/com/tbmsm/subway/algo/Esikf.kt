package com.tbmsm.subway.algo

import com.tbmsm.subway.core.Mat3
import com.tbmsm.subway.core.MatN
import com.tbmsm.subway.core.Quat
import com.tbmsm.subway.core.Vec3
import com.tbmsm.subway.core.choleskyFactor
import com.tbmsm.subway.core.choleskySolve
import com.tbmsm.subway.core.choleskySolveMatrix
import com.tbmsm.subway.core.chi2Quantile99
import com.tbmsm.subway.core.eulerFromMatrix
import com.tbmsm.subway.core.wrapPi
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.sqrt

/**
 * 15 维误差状态卡尔曼滤波（ESKF）。
 *
 * 名义状态   x  = [p, v, q, b_g, b_a]
 * 误差状态   dx = [δp, δv, δθ, δb_g, δb_a]
 *
 * 姿态误差采用「全局误差」定义（导航系）：
 *
 *      C_true = Exp(δθ) · C_nom
 *
 * 该约定下位置/速度/航向类观测的 H 矩阵都是常系数，无需携带姿态旋转，
 * 能显著降低实现出错概率。
 *
 * 误差状态索引：
 *      0..2    δp       3..5   δv       6..8   δθ
 *      9..11   δb_g     12..14 δb_a
 */
class Esikf(private val params: NavParams) {

    // ---------------- 名义状态 ----------------
    var p: Vec3 = Vec3.zero()
        private set
    var v: Vec3 = Vec3.zero()
        private set
    var q: Quat = Quat.identity()
        private set
    var bg: Vec3 = Vec3.zero()
        private set
    var ba: Vec3 = Vec3.zero()
        private set

    // ---------------- 协方差 ----------------
    var cov: MatN = MatN.zeros(N, N)
        private set

    /** 累计成功执行的观测更新次数。 */
    var acceptedUpdates: Int = 0
        private set

    /** 累计因 NIS 门限被拒绝的观测次数。 */
    var nisRejected: Int = 0
        private set

    /** 累计因矩阵非正定等原因失败的观测次数。 */
    var numericRejected: Int = 0
        private set

    /** 状态是否出现非有限值（出现后引擎会重启滤波器）。 */
    var diverged: Boolean = false
        private set

    /**
     * 最近一次预测使用的离散转移矩阵与过程噪声矩阵。
     * RTS 平滑需要它们做反向递推，因此在这里暴露出来，避免重复计算。
     */
    var lastF: MatN = MatN.identity(N)
        private set
    var lastQd: MatN = MatN.zeros(N, N)
        private set

    fun reset(q0: Quat, gyroBias: Vec3, accBias: Vec3 = Vec3.zero()) {
        p.zero()
        v.zero()
        q = q0.normalized()
        bg = gyroBias.copy()
        ba = accBias.copy()
        cov = MatN.zeros(N, N)
        val attVar = params.initAttSigma * params.initAttSigma
        val gyroBiasVar = params.initGyroBiasSigma * params.initGyroBiasSigma
        val accBiasVar = params.initAccBiasSigma * params.initAccBiasSigma
        val posVar = params.initPosSigma * params.initPosSigma
        val velVar = params.initVelSigma * params.initVelSigma
        for (i in 0..2) {
            cov[i, i] = posVar
            cov[3 + i, 3 + i] = velVar
            cov[6 + i, 6 + i] = attVar
            cov[9 + i, 9 + i] = gyroBiasVar
            cov[12 + i, 12 + i] = accBiasVar
        }
        acceptedUpdates = 0
        nisRejected = 0
        numericRejected = 0
        diverged = false
    }

    /** 外部（如轨道先验）给出了绝对位置时使用。 */
    fun setPosition(pos: Vec3) {
        p = pos.copy()
    }

    /** 外部给出绝对航向时，绕天向旋转姿态。 */
    fun setHeading(headingRad: Double) {
        val delta = wrapPi(headingRad - heading())
        val up = Vec3(0.0, 0.0, 1.0)
        val corr = Quat.fromAxisAngle(up, delta)
        q = (corr * q).normalized()
    }

    val attitude: Mat3 get() = q.toMat3()

    /** 航向角（相对真北，顺时针为正，ENU 下为 atan2(E, N)）。 */
    fun heading(): Double {
        val c = q.toMat3()
        return atan2(c.a00, c.a10)
    }

    /** 机体系 x 轴（前进方向）在导航系中的表示。 */
    fun forwardAxis(): Vec3 {
        val c = q.toMat3()
        return Vec3(c.a00, c.a10, c.a20)
    }

    /** 机体系 z 轴（地板法向，向上）在导航系中的表示。 */
    fun upAxis(): Vec3 {
        val c = q.toMat3()
        return Vec3(c.a02, c.a12, c.a22)
    }

    /** 返回 [heading, pitch, roll]，单位 rad。 */
    fun euler(): DoubleArray = eulerFromMatrix(q.toMat3())

    fun positionSigma(): Double = sqrt(abs(cov[0, 0]) + abs(cov[1, 1]))
    fun velocitySigma(): Double = sqrt(abs(cov[3, 3]) + abs(cov[4, 4]))
    fun headingSigma(): Double = sqrt(abs(cov[8, 8]))
    fun gyroBiasSigma(): Double = sqrt(
        abs(cov[9, 9]) + abs(cov[10, 10]) + abs(cov[11, 11])
    )

    // ==================================================================
    //  预测：IMU 机械编排
    // ==================================================================

    fun predict(accBody: Vec3, gyroBody: Vec3, dt: Double) {
        if (dt <= 0.0 || dt > 0.5) return
        if (diverged) return

        val c = q.toMat3()
        val fMinusBa = accBody - ba
        val aNav = c * fMinusBa + Vec3(0.0, 0.0, -params.g)

        // ---- 名义状态递推 ----
        p = p + v * dt + aNav * (0.5 * dt * dt)
        v = v + aNav * dt
        val wMinusBg = gyroBody - bg
        q = (q * Quat.fromRotationVector(wMinusBg * dt)).normalized()

        // ---- 误差状态转移矩阵 F ----
        val f = MatN.zeros(N, N)
        for (i in 0..2) f[i, 3 + i] = 1.0

        val sf = Mat3.skew(c * fMinusBa)
        for (i in 0..2) {
            for (j in 0..2) {
                f[3 + i, 6 + j] = -sf[i, j]
                f[3 + i, 12 + j] = -c[i, j]
            }
        }
        val sw = Mat3.skew(wMinusBg)
        for (i in 0..2) {
            for (j in 0..2) f[6 + i, 6 + j] = -sw[i, j]
            f[6 + i, 9 + i] = -1.0
        }

        // Fd = I + F*dt （100Hz 下的一阶离散化误差可忽略）
        val fd = MatN.identity(N)
        for (i in 0 until N) {
            for (j in 0 until N) {
                val vij = f[i, j]
                if (vij != 0.0) fd[i, j] += vij * dt
            }
        }

        // ---- 过程噪声 Qd = G Qc G^T dt ----
        // 由于 C 正交、噪声各向同性，可化简为对角阵
        val qd = MatN.zeros(N, N)
        val qv = params.sigmaAccNoise * params.sigmaAccNoise * dt
        val qt = params.sigmaGyroNoise * params.sigmaGyroNoise * dt
        val qbg = params.sigmaGyroBiasRw * params.sigmaGyroBiasRw * dt
        val qba = params.sigmaAccBiasRw * params.sigmaAccBiasRw * dt
        for (i in 0..2) {
            qd[3 + i, 3 + i] = qv
            qd[6 + i, 6 + i] = qt
            qd[9 + i, 9 + i] = qbg
            qd[12 + i, 12 + i] = qba
        }

        cov = fd.multiply(cov).multiply(fd.transpose()).plus(qd)
        cov.symmetrize()

        lastF = fd
        lastQd = qd

        if (!isStateFinite()) diverged = true
    }

    private fun isStateFinite(): Boolean {
        if (!p.isFinite() || !v.isFinite() || !bg.isFinite() || !ba.isFinite()) return false
        if (!cov.isFinite()) return false
        val qn = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z
        return qn.isFinite() && qn > 1e-6
    }

    // ==================================================================
    //  观测更新
    // ==================================================================

    /**
     * 通用更新。rows 为观测矩阵的每一行（长度 15），z 为创新，rDiag 为观测噪声方差对角。
     * 返回是否被接受。
     */
    private fun apply(rows: Array<DoubleArray>, z: DoubleArray, rDiag: DoubleArray): Boolean {
        if (diverged) return false
        val dof = z.size
        require(rows.size == dof && rDiag.size == dof)

        val h = MatN.zeros(dof, N)
        for (i in 0 until dof) {
            for (j in 0 until N) h[i, j] = rows[i][j]
        }
        val pht = cov.multiply(h.transpose())      // 15 x dof
        val s = h.multiply(pht)                    // dof x dof
        for (i in 0 until dof) s[i, i] += rDiag[i]

        var l = choleskyFactor(s)
        if (l == null) {
            for (i in 0 until dof) s[i, i] += 1e-9 + 1e-6 * abs(s[i, i])
            l = choleskyFactor(s)
            if (l == null) {
                numericRejected++
                return false
            }
        }

        // NIS 卡方检验
        val sz = choleskySolve(l, dof, z)
        var nis = 0.0
        for (i in 0 until dof) nis += z[i] * sz[i]
        if (nis.isNaN() || nis > chi2Quantile99(dof)) {
            nisRejected++
            return false
        }

        val dx = pht.multiply(sz)                        // 15
        val sInvH = choleskySolveMatrix(l, dof, h)       // dof x 15
        val kh = pht.multiply(sInvH)                     // 15 x 15
        cov = cov.minus(kh.multiply(cov))
        cov.symmetrize()

        applyErrorToNominal(dx)
        acceptedUpdates++
        if (!isStateFinite()) diverged = true
        return true
    }

    private fun applyErrorToNominal(dx: DoubleArray) {
        p.x += dx[0]; p.y += dx[1]; p.z += dx[2]
        v.x += dx[3]; v.y += dx[4]; v.z += dx[5]
        val dTheta = Vec3(dx[6], dx[7], dx[8])
        if (dTheta.norm() > 1e-14) {
            q = (Quat.fromRotationVector(dTheta) * q).normalized()
        }
        bg = bg + Vec3(dx[9], dx[10], dx[11])
        ba = ba + Vec3(dx[12], dx[13], dx[14])
    }

    /** 零速修正：速度观测为 0。 */
    fun updateZupt(sigma: Double = params.zuptVelSigma): Boolean {
        val z = doubleArrayOf(-v.x, -v.y, -v.z)
        val rows = Array(3) { i ->
            DoubleArray(N).also { it[3 + i] = 1.0 }
        }
        return apply(rows, z, doubleArrayOf(sigma * sigma, sigma * sigma, sigma * sigma))
    }

    /**
     * 重力调平：静止时加计输出方向应与名义姿态给出的天向一致。
     *
     * 观测：u_meas = a_b/|a_b|，预测：u_pred = C_n^b * e_U
     * 灵敏度 H = -C_n^b * [e_U]x（仅 δθ 块）
     *
     * 该观测同时使加计零偏的水平分量可观，这正是静止段能估出 b_a 的原因。
     */
    fun updateLevel(accBody: Vec3, sigma: Double = params.levelSigma): Boolean {
        val n = accBody.norm()
        if (n < 1e-3) return false
        val c = q.toMat3()
        val upInBody = c.transpose() * Vec3(0.0, 0.0, 1.0)
        val uMeas = accBody / n
        val z = doubleArrayOf(uMeas.x - upInBody.x, uMeas.y - upInBody.y, uMeas.z - upInBody.z)
        val hTheta = c.transpose() * Mat3.skew(Vec3(0.0, 0.0, 1.0))
        val rows = Array(3) { i ->
            DoubleArray(N).also { row ->
                for (j in 0..2) row[6 + j] = -hTheta[i, j]
            }
        }
        val v = sigma * sigma
        return apply(rows, z, doubleArrayOf(v, v, v))
    }

    /** GNSS 位置更新（ENU，米）。 */
    fun updatePosition(
        measuredEnu: Vec3,
        sigmaHorizontal: Double = params.gnssPosSigma,
        sigmaVertical: Double = params.gnssPosSigma
    ): Boolean {
        val z = doubleArrayOf(measuredEnu.x - p.x, measuredEnu.y - p.y, measuredEnu.z - p.z)
        val rows = Array(3) { i -> DoubleArray(N).also { it[i] = 1.0 } }
        return apply(
            rows, z,
            doubleArrayOf(sigmaHorizontal * sigmaHorizontal,
                sigmaHorizontal * sigmaHorizontal,
                sigmaVertical * sigmaVertical)
        )
    }

    /** GNSS 速度更新（ENU，m/s）。 */
    fun updateVelocity(measuredEnu: Vec3, sigma: Double = params.gnssVelSigma): Boolean {
        val z = doubleArrayOf(measuredEnu.x - v.x, measuredEnu.y - v.y, measuredEnu.z - v.z)
        val rows = Array(3) { i -> DoubleArray(N).also { it[3 + i] = 1.0 } }
        val s2 = sigma * sigma
        // 竖直方向多普勒精度较差，给更大的方差
        return apply(rows, z, doubleArrayOf(s2, s2, s2 * 4.0))
    }

    /**
     * 航向观测（航迹角或轨道方向）。
     * 观测模型：ψ_true = ψ_est + δθ_z  =>  H 在 δθ_z 处为 +1。
     */
    fun updateHeading(measuredHeading: Double, sigma: Double): Boolean {
        val z = doubleArrayOf(wrapPi(measuredHeading - heading()))
        val rows = arrayOf(DoubleArray(N).also { it[8] = 1.0 })
        return apply(rows, z, doubleArrayOf(sigma * sigma))
    }

    /**
     * 横向位置约束：真实位置应落在轨道中心线上。
     * errLateral = u_lat · (p_track - p_nom)
     */
    fun updateLateral(uLat: Vec3, errLateral: Double, sigma: Double): Boolean {
        val row = DoubleArray(N)
        row[0] = uLat.x; row[1] = uLat.y; row[2] = uLat.z
        return apply(arrayOf(row), doubleArrayOf(errLateral), doubleArrayOf(sigma * sigma))
    }

    /** 沿轨里程约束：errAlong = 期望沿轨位置 - 当前沿轨位置。 */
    fun updateAlongTrack(uAlong: Vec3, errAlong: Double, sigma: Double): Boolean {
        val row = DoubleArray(N)
        row[0] = uAlong.x; row[1] = uAlong.y; row[2] = uAlong.z
        return apply(arrayOf(row), doubleArrayOf(errAlong), doubleArrayOf(sigma * sigma))
    }

    /**
     * 磁力计航向约束（只用水平两分量，避免与重力调平互相打架）。
     *
     * 估计的导航系磁场：m_nom = C_b^n * m_b
     * 观测模型：m_nom - m_ref ≈ [m_ref]x * δθ
     *
     * @param magBody 机体系磁场（uT）
     * @param magRefEnu 导航系理论磁场（uT，ENU）
     * @param sigmaField 观测噪声（uT），建议取 0.1~0.2 倍水平场强
     */
    fun updateMagnetometer(magBody: Vec3, magRefEnu: Vec3, sigmaField: Double): Boolean {
        val mNom = q.toMat3() * magBody
        val z = doubleArrayOf(mNom.x - magRefEnu.x, mNom.y - magRefEnu.y)
        val sk = Mat3.skew(magRefEnu)
        val rows = Array(2) { i ->
            DoubleArray(N).also { row ->
                for (j in 0..2) row[6 + j] = sk[i, j]
            }
        }
        val s2 = sigmaField * sigmaField
        return apply(rows, z, doubleArrayOf(s2, s2))
    }

    companion object {
        const val N = 15

        // 误差状态分块索引，便于外部构造 H
        const val IDX_P = 0
        const val IDX_V = 3
        const val IDX_THETA = 6
        const val IDX_BG = 9
        const val IDX_BA = 12
    }
}
