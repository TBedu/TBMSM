package com.tbmsm.subway.algo

import com.tbmsm.subway.core.Mat3
import com.tbmsm.subway.core.MatN
import com.tbmsm.subway.core.Quat
import com.tbmsm.subway.core.Vec3
import com.tbmsm.subway.core.choleskyFactor
import com.tbmsm.subway.core.choleskySolveMatrix
import com.tbmsm.subway.core.wrapPi
import com.tbmsm.subway.model.TrajSample
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.sqrt

/**
 * 后向 RTS（Rauch–Tung–Striebel）平滑。
 *
 * 正向 ESKF 只用了「过去」的信息，而地铁航段短、停站锚点密集，
 * 反向平滑能显著改善航向与零偏的可观测性，是提升精度的性价比最高的一步。
 *
 * 实现要点：
 *  - 正向每步保存 (x_k, P_k, F_k, Q_k)，其中 F_k 为 k-1 -> k 的转移矩阵；
 *  - 反向递推增益 C_k = P_k F_{k+1}^T (P_{k+1}^-)^{-1}；
 *  - 误差状态平滑后注入名义状态，得到平滑轨迹。
 *
 * 内存：100Hz、10 分钟约 6 万个节点，每个节点 15x15 协方差（1800 字节）约 108 MB，
 * 因此对节点做抽稀保存（默认每 10 个采样保存一次，即 10Hz），
 * 平滑时按时间就近取节点。对本项目关心的低频量（速度、加速度、里程）足够。
 */
class RtsSmoother(private val params: NavParams) {

    private class Node(
        val tNs: Long,
        val p: Vec3,
        val v: Vec3,
        val q: Quat,
        val bg: Vec3,
        val ba: Vec3,
        val cov: MatN,
        val f: MatN,
        val qd: MatN
    )

    private val nodes = ArrayList<Node>(4096)
    private var strideCounter = 0
    private val stride = 10

    fun reset() {
        nodes.clear()
        strideCounter = 0
    }

    val nodeCount: Int get() = nodes.size

    /**
     * 记录一个正向滤波节点。由引擎在每个 IMU 步之后调用。
     *
     * @param f 该步的误差状态转移矩阵（离散）
     * @param qd 该步的过程噪声矩阵
     */
    fun record(
        tNs: Long,
        p: Vec3, v: Vec3, q: Quat, bg: Vec3, ba: Vec3,
        cov: MatN, f: MatN, qd: MatN,
        force: Boolean = false
    ) {
        strideCounter++
        if (!force && strideCounter % stride != 0) return
        nodes.add(
            Node(
                tNs = tNs,
                p = p.copy(), v = v.copy(), q = q,
                bg = bg.copy(), ba = ba.copy(),
                cov = cov.copy(), f = f.copy(), qd = qd.copy()
            )
        )
    }

    /**
     * 执行反向平滑，返回平滑后的轨迹样本。
     *
     * @param rawSamples 正向输出的原始样本（用于取时间戳与静止标志）
     */
    fun smooth(rawSamples: List<TrajSample>): List<TrajSample> {
        if (nodes.size < 3 || rawSamples.isEmpty()) return rawSamples

        val n = nodes.size
        val smoothedCov = arrayOfNulls<MatN>(n)
        val smoothedErr = arrayOfNulls<DoubleArray>(n)
        smoothedCov[n - 1] = nodes[n - 1].cov
        smoothedErr[n - 1] = DoubleArray(Esikf.N)

        for (k in n - 2 downTo 0) {
            val nk = nodes[k]
            val nk1 = nodes[k + 1]
            // P_{k+1}^- = F P_k F^T + Q
            val pPred = nk1.f.multiply(nk.cov).multiply(nk1.f.transpose()).plus(nk1.qd)
            pPred.symmetrize()

            val c = solveRtsGain(nk.cov, nk1.f, pPred)
            if (c == null) {
                smoothedCov[k] = nk.cov
                smoothedErr[k] = DoubleArray(Esikf.N)
                continue
            }
            val errK1 = smoothedErr[k + 1]!!
            smoothedErr[k] = c.multiply(errK1)
            val covK = nk.cov.plus(
                c.multiply(smoothedCov[k + 1]!!.minus(pPred)).multiply(c.transpose())
            )
            covK.symmetrize()
            smoothedCov[k] = covK
        }

        val out = ArrayList<TrajSample>(rawSamples.size)
        var nodeIdx = 0
        for (s in rawSamples) {
            while (nodeIdx < n - 2 && nodes[nodeIdx + 1].tNs <= s.tNs) nodeIdx++
            val dx = smoothedErr[nodeIdx] ?: DoubleArray(Esikf.N)
            val node = nodes[nodeIdx]

            val pS = Vec3(node.p.x + dx[0], node.p.y + dx[1], node.p.z + dx[2])
            val vS = Vec3(node.v.x + dx[3], node.v.y + dx[4], node.v.z + dx[5])
            val dTheta = Vec3(dx[6], dx[7], dx[8])
            val qS = if (dTheta.norm() > 1e-14) {
                (Quat.fromRotationVector(dTheta) * node.q).normalized()
            } else {
                node.q
            }
            val c = qS.toMat3()
            val speed = sqrt(vS.x * vS.x + vS.y * vS.y + vS.z * vS.z)
            val forward = Vec3(c.a00, c.a10, c.a20)
            val up = Vec3(c.a02, c.a12, c.a22)
            val heading = atan2(forward.x, forward.y)
            val pitch = atan2(-up.x, sqrt(up.y * up.y + up.z * up.z))
            val roll = atan2(up.y, up.z)

            out.add(
                s.copy(
                    east = pS.x, north = pS.y, up = pS.z,
                    vEast = vS.x, vNorth = vS.y, vUp = vS.z,
                    speedMps = speed,
                    headingDeg = Math.toDegrees(heading),
                    pitchDeg = Math.toDegrees(pitch),
                    rollDeg = Math.toDegrees(roll),
                    smoothed = true
                )
            )
        }
        return out
    }

    /** C = P_k F^T (P_{k+1}^-)^{-1}，用 Cholesky 求解。 */
    private fun solveRtsGain(pk: MatN, f: MatN, pPred: MatN): MatN? {
        val n = Esikf.N
        val pft = pk.multiply(f.transpose())
        val l = choleskyFactor(pPred) ?: return null
        val x = choleskySolveMatrix(l, n, pft.transpose())
        return x.transpose()
    }

    /**
     * 平滑后航向与正向航向的差异统计（度），用于评估平滑收益。
     *
     * @param forwardHeadings 与节点一一对应的正向航向（弧度）；长度不足时返回 0。
     */
    fun headingCorrectionStats(forwardHeadings: DoubleArray): Pair<Double, Double> {
        if (nodes.size < 2 || forwardHeadings.size != nodes.size) return 0.0 to 0.0
        var maxAbs = 0.0
        var sumAbs = 0.0
        for (i in nodes.indices) {
            val c = nodes[i].q.toMat3()
            val h = atan2(c.a00, c.a10)
            val d = abs(wrapPi(h - forwardHeadings[i]))
            maxAbs = maxOf(maxAbs, d)
            sumAbs += d
        }
        return Math.toDegrees(maxAbs) to Math.toDegrees(sumAbs / nodes.size)
    }

    /** 各节点的正向航向（弧度），供 headingCorrectionStats 使用。 */
    fun forwardHeadings(): DoubleArray {
        val arr = DoubleArray(nodes.size)
        for (i in nodes.indices) {
            val c = nodes[i].q.toMat3()
            arr[i] = atan2(c.a00, c.a10)
        }
        return arr
    }
}