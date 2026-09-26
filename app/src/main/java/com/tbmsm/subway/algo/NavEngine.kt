package com.tbmsm.subway.algo

import com.tbmsm.subway.core.GeoPoint
import com.tbmsm.subway.core.Geodesy
import com.tbmsm.subway.core.GeomagneticField
import com.tbmsm.subway.core.Mat3
import com.tbmsm.subway.core.Quat
import com.tbmsm.subway.core.Vec3
import com.tbmsm.subway.core.wrapPi
import com.tbmsm.subway.model.EngineState
import com.tbmsm.subway.model.GnssFix
import com.tbmsm.subway.model.ImuSample
import com.tbmsm.subway.model.LiveState
import com.tbmsm.subway.model.MagSample
import com.tbmsm.subway.model.RunSegment
import com.tbmsm.subway.model.TrajSample
import com.tbmsm.subway.track.TrackModel
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * 导航引擎：把传感器流、ESKF、各类观测更新与状态机串起来。
 *
 * 设计约束：
 *  - 本类不依赖任何 Android API，可在纯 JVM 上做单元测试与离线回放；
 *  - 所有输入都带单调时间戳（纳秒），内部按时间戳排序处理；
 *  - 输出为 TrajSample 列表 + LiveState 快照。
 *
 * 处理顺序（每个 IMU 采样）：
 *   1. 用安装矩阵把手机系数据转到机体系
 *   2. ESKF 预测
 *   3. 零速检测 -> ZUPT + 重力调平
 *   4. 轨道约束（航向 + 横向）
 *   5. 磁力计门控更新（默认关闭）
 *   6. 输出低通滤波与抽稀记录
 */
class NavEngine(
    val params: NavParams = NavParams(),
    private val track: TrackModel? = null,
    private val origin: GeoPoint? = null
) {

    // ---------------- 子模块 ----------------
    private val zupt = ZuptDetector(params)
    private val aligner = StaticAligner(params)
    private val eskf = Esikf(params)
    private val smoother = RtsSmoother(params)

    // ---------------- 会话状态 ----------------
    var state: EngineState = EngineState.IDLE
        private set

    private var sessionId: String = ""
    private var startTNs: Long = 0L
    private var lastImuTNs: Long = 0L
    private var lastGnssTNs: Long = 0L
    private var lastMagTNs: Long = 0L

    private var mounting: Mat3 = Mat3.identity()
    private var alignResult: AlignResult? = null

    private val samples = ArrayList<TrajSample>(8192)
    private val segments = ArrayList<RunSegment>(32)
    private val warnings = ArrayList<String>()

    private var imuCount: Long = 0
    private var gnssCount: Int = 0
    private var gnssRejected: Int = 0
    private var divergenceCount: Int = 0
    private var stillSamples: Long = 0
    private var totalSamples: Long = 0

    // ---------------- 输出低通 ----------------
    private var lpfAccelAlong = 0.0
    private var lpfAccelVertical = 0.0
    private var lpfAccelLateral = 0.0
    private var lpfSpeed = 0.0
    private var lpfInitialized = false

    // ---------------- 站间段统计 ----------------
    private var currentSegmentIndex = 0
    private var segmentDepartTNs = 0L
    private var segmentDepartS = 0.0
    private var segmentMaxSpeed = 0.0
    private var segmentMaxAccel = 0.0
    private var segmentMaxDecel = 0.0
    private var wasStill = true
    private var lastStationIdx: Int? = null

    // ---------------- 轨道投影 ----------------
    private var trackHintS: Double? = null
    private var trackActive = false

    // ---------------- GNSS ----------------
    private var gnssActive = false
    private var lastGnssAccuracy = 0.0
    private var gnssOrigin: GeoPoint? = null

    // ---------------- 磁力计 ----------------
    private var magOk = false
    private var magRefEnu: Vec3? = null
    private var magRefTotal = 0.0
    private var magRefIncl = 0.0

    // ---------------- 输出抽稀 ----------------
    private var lastRecordTNs = 0L

    /** 最近一帧机体系比力，供输出加速度计算使用。 */
    private var lastAccBody: Vec3 = Vec3(0.0, 0.0, 9.80665)

    /** 供界面读取的实时快照。 */
    var live: LiveState = LiveState()
        private set

    // ==================================================================
    //  生命周期
    // ==================================================================

    fun start(sessionId: String, startTNs: Long) {
        this.sessionId = sessionId
        this.startTNs = startTNs
        this.lastImuTNs = startTNs
        this.lastGnssTNs = 0L
        this.lastMagTNs = 0L
        state = EngineState.ALIGNING
        zupt.reset()
        aligner.reset()
        smoother.reset()
        samples.clear()
        segments.clear()
        warnings.clear()
        imuCount = 0
        gnssCount = 0
        gnssRejected = 0
        divergenceCount = 0
        stillSamples = 0
        totalSamples = 0
        lpfInitialized = false
        currentSegmentIndex = 0
        segmentDepartTNs = startTNs
        segmentDepartS = 0.0
        segmentMaxSpeed = 0.0
        segmentMaxAccel = 0.0
        segmentMaxDecel = 0.0
        wasStill = true
        lastStationIdx = null
        trackHintS = null
        trackActive = false
        gnssActive = false
        magOk = false
        lastRecordTNs = startTNs
        live = LiveState(state = EngineState.ALIGNING, sessionId = sessionId)
    }

    /** 用户强制开始（跳过对准，质量会下降）。 */
    fun forceStart() {
        if (state != EngineState.ALIGNING) return
        if (!aligner.hasSamples()) {
            warnings.add("没有任何静止样本，无法估计安装角与零偏，结果不可信")
            return
        }
        val res = aligner.buildForced(0.0, emptyList())
        finishAlignment(res)
    }

    fun stop() {
        if (state == EngineState.RUNNING || state == EngineState.ALIGNING) {
            state = EngineState.FINISHED
        }
    }

    // ==================================================================
    //  输入
    // ==================================================================

    fun onImu(s: ImuSample) {
        if (state == EngineState.IDLE || state == EngineState.FINISHED || state == EngineState.ERROR) return
        imuCount++
        totalSamples++

        when (state) {
            EngineState.ALIGNING -> handleAligning(s)
            EngineState.RUNNING -> handleRunning(s)
            else -> Unit
        }
    }

    fun onMag(s: MagSample) {
        if (state != EngineState.RUNNING) return
        lastMagTNs = s.tNs
        val magPhone = Vec3(s.mx, s.my, s.mz)
        val magBody = mounting * magPhone
        val ref = magRefEnu ?: return

        // 三项门控：模长、倾角、与滤波器航向的一致性
        val total = magBody.norm()
        if (magRefTotal <= 1e-6) return
        val normOk = abs(total - magRefTotal) / magRefTotal < params.magNormTolRel
        val incl = atan2(-magBody.z, sqrt(magBody.x * magBody.x + magBody.y * magBody.y))
        val inclOk = abs(wrapPi(incl - magRefIncl)) < params.magInclTolRad
        val magHeading = atan2(magBody.y, magBody.x)
        val consistOk = abs(wrapPi(magHeading - eskf.heading())) < params.magConsistTolRad
        magOk = normOk && inclOk && consistOk

        if (magOk && params.useMagYaw) {
            val sigmaField = 0.15 * sqrt(ref.x * ref.x + ref.y * ref.y)
            eskf.updateMagnetometer(magBody, ref, sigmaField)
        }
    }

    fun onGnss(fix: GnssFix) {
        if (state != EngineState.RUNNING) return
        // 精度门限：超过 10 m 整帧丢弃，位置/速度/航迹角都不用
        if (fix.accuracyM > params.gnssMaxAccuracyM) {
            gnssRejected++
            gnssActive = false
            lastGnssAccuracy = fix.accuracyM
            return
        }
        if (fix.tNs <= lastGnssTNs) return
        lastGnssTNs = fix.tNs
        gnssCount++
        gnssActive = true
        lastGnssAccuracy = fix.accuracyM

        val ref = gnssOrigin ?: return
        val enu = Geodesy.enuDelta(ref, GeoPoint(fix.lat, fix.lon, fix.alt))

        // 位置：弱约束，且丢弃高程
        if (params.useGnssPosition) {
            eskf.updatePosition(enu, fix.accuracyM, 1e3)
        }

        // 速度：多普勒速度是最可靠的 GNSS 观测量
        if (fix.hasSpeed) {
            val headingRad = if (fix.hasBearing) Math.toRadians(fix.bearingDeg) else 0.0
            val vEnu = Vec3(
                fix.speedMps * sin(headingRad),
                fix.speedMps * cos(headingRad),
                0.0
            )
            eskf.updateVelocity(vEnu, params.gnssVelSigma)

            // 航迹角：地上段获得绝对航向的最佳途径
            if (fix.hasBearing && fix.speedMps > params.gnssCourseMinSpeed) {
                val sigma = maxOf(
                    params.gnssCourseSigma,
                    params.gnssVelSigma / fix.speedMps
                )
                eskf.updateHeading(headingRad, sigma)
            }
        }
    }

    // ==================================================================
    //  对准阶段
    // ==================================================================

    private fun handleAligning(s: ImuSample) {
        val accPhone = Vec3(s.ax, s.ay, s.az)
        val gyroPhone = Vec3(s.gx, s.gy, s.gz)
        when (aligner.add(accPhone, gyroPhone)) {
            AlignState.COLLECTING -> Unit
            AlignState.REJECTED -> {
                warnings.add("检测到车辆未静止，已重新开始对准")
            }
            AlignState.READY -> {
                val res = aligner.build(initialYawFromMag(accPhone))
                finishAlignment(res)
            }
        }
        live = live.copy(
            state = EngineState.ALIGNING,
            alignProgress = aligner.progress,
            imuCount = imuCount,
            warning = warnings.lastOrNull() ?: ""
        )
    }

    /** 用磁力计估计初始航向；不可信时返回 0（相对航向）。 */
    private fun initialYawFromMag(accPhone: Vec3): Double {
        if (!params.useMagHeadingInit) return 0.0
        val ref = origin ?: return 0.0
        val field = GeomagneticField.field(ref)
        magRefEnu = field.toEnu().div(1000.0)   // nT -> uT
        magRefTotal = field.total / 1000.0
        magRefIncl = field.inclinationRad
        // 对准阶段没有磁力计样本时无法定航向，交由 GNSS/轨道修正
        return 0.0
    }

    private fun finishAlignment(res: AlignResult) {
        alignResult = res
        mounting = res.mounting
        warnings.addAll(res.warnings)
        eskf.reset(res.initialAttitude, res.gyroBiasBody)
        state = EngineState.RUNNING
        segmentDepartTNs = lastImuTNs
        segmentDepartS = 0.0
        smoother.record(
            lastImuTNs, eskf.p, eskf.v, eskf.q, eskf.bg, eskf.ba,
            eskf.cov, eskf.lastF, eskf.lastQd,
            force = true
        )
        live = live.copy(state = EngineState.RUNNING, alignProgress = 1f)
    }

    // ==================================================================
    //  运行阶段
    // ==================================================================

    private fun handleRunning(s: ImuSample) {
        val dt = if (lastImuTNs == 0L) 0.01 else (s.tNs - lastImuTNs) / 1e9
        lastImuTNs = s.tNs
        if (dt <= 0.0 || dt > 0.5) return

        val accBody = mounting * Vec3(s.ax, s.ay, s.az)
        val gyroBody = mounting * Vec3(s.gx, s.gy, s.gz)
        lastAccBody = accBody

        // ---- 1. 预测 ----
        eskf.predict(accBody, gyroBody, dt)

        // ---- 2. 零速检测与 ZUPT ----
        val still = zupt.update(accBody, gyroBody, eskf.v.norm())
        if (still) {
            stillSamples++
            eskf.updateZupt()
            eskf.updateLevel(accBody)
        }

        // ---- 3. 轨道约束 ----
        applyTrackConstraints()

        // ---- 4. 站间段统计 ----
        updateSegments(s.tNs, still)

        // ---- 5. 记录平滑节点 ----
        smoother.record(
            s.tNs, eskf.p, eskf.v, eskf.q, eskf.bg, eskf.ba,
            eskf.cov, eskf.lastF, eskf.lastQd
        )

        // ---- 6. 输出 ----
        emitSample(s.tNs, still)

        if (eskf.diverged) {
            divergenceCount++
            warnings.add("滤波器数值发散，已重置姿态与协方差")
            eskf.reset(eskf.q, eskf.bg, eskf.ba)
        }

        live = live.copy(
            state = EngineState.RUNNING,
            elapsedS = (s.tNs - startTNs) / 1e9,
            speedMps = lpfSpeed,
            distanceM = eskf.p.norm(),
            headingDeg = Math.toDegrees(eskf.heading()),
            pitchDeg = Math.toDegrees(eskf.euler()[1]),
            rollDeg = Math.toDegrees(eskf.euler()[2]),
            accelAlong = lpfAccelAlong,
            accelVertical = lpfAccelVertical,
            accelLateral = lpfAccelLateral,
            still = still,
            zuptStatistic = zupt.lastStatistic,
            posSigmaM = eskf.positionSigma(),
            headingSigmaDeg = Math.toDegrees(eskf.headingSigma()),
            gyroBiasDegPerS = Math.toDegrees(eskf.bg.norm()),
            gnssActive = gnssActive,
            gnssAccuracyM = lastGnssAccuracy,
            gnssCount = gnssCount,
            gnssRejected = gnssRejected,
            magOk = magOk,
            trackActive = trackActive,
            segmentCount = segments.size,
            lastSegmentRunTimeS = segments.lastOrNull()?.runTimeS ?: 0.0,
            lastSegmentDistanceM = segments.lastOrNull()?.distanceM ?: 0.0,
            imuCount = imuCount,
            imuRateHz = if (dt > 0) 1.0 / dt else 0.0,
            warning = warnings.lastOrNull() ?: ""
        )
    }

    /**
     * 轨道约束：航向 + 横向位置。
     *
     * 这是地下段航向的唯一可靠来源，也是把「三维自由惯性推算」降维成
     * 「沿轨一维里程估计」的关键，直接决定方案是否可行。
     */
    private fun applyTrackConstraints() {
        val t = track ?: return
        if (!t.valid) return
        val proj = t.project(eskf.p, trackHintS) ?: return
        if (proj.distance > params.trackMaxLateralM) {
            trackActive = false
            return
        }
        trackActive = true
        trackHintS = proj.s

        // 航向约束：直线段收紧，曲线段放宽
        val sigmaYaw = if (t.isCurve(proj.s)) {
            params.trackYawSigmaCurve
        } else {
            params.trackYawSigmaStraight
        }
        eskf.updateHeading(proj.heading, sigmaYaw)

        // 横向位置约束：真实位置应落在中心线上
        val errLateral = (proj.point.x - eskf.p.x) * proj.left.x +
            (proj.point.y - eskf.p.y) * proj.left.y
        eskf.updateLateral(proj.left, errLateral, params.trackLateralSigma)
    }

    /** 站间运行段识别：静止 -> 运动 为出发，运动 -> 静止 为到达。 */
    private fun updateSegments(tNs: Long, still: Boolean) {
        if (still && !wasStill) {
            // 到达
            val runTime = (tNs - segmentDepartTNs) / 1e9
            val distance = eskf.p.norm() - segmentDepartS
            if (runTime > 5.0 && distance > 20.0) {
                var prior = 0.0
                val t = track
                if (t != null && t.stationS.isNotEmpty()) {
                    val near = t.nearestStation(trackHintS ?: 0.0)
                    if (near != null && near.second < 150.0) {
                        val idx = near.first
                        val prev = lastStationIdx
                        if (prev != null) {
                            val p = t.stationPrior(prev, idx)
                            if (p != null) {
                                prior = p
                                // 站间距先验作为一次沿轨观测，抑制累计里程漂移
                                val uAlong = t.tangentAt(trackHintS ?: 0.0)
                                eskf.updateAlongTrack(
                                    uAlong, prior - distance,
                                    params.stationDistanceSigma
                                )
                            }
                        }
                        lastStationIdx = idx
                    }
                }
                // 用修正后的位置重算里程，保证与滤波器状态一致
                val distanceCorrected = eskf.p.norm() - segmentDepartS
                segments.add(
                    RunSegment(
                        index = currentSegmentIndex++,
                        departTNs = segmentDepartTNs,
                        arriveTNs = tNs,
                        runTimeS = runTime,
                        distanceM = distanceCorrected,
                        meanSpeedMps = if (runTime > 0) distanceCorrected / runTime else 0.0,
                        maxSpeedMps = segmentMaxSpeed,
                        maxAccelMps2 = segmentMaxAccel,
                        maxDecelMps2 = segmentMaxDecel,
                        stationDistancePriorM = prior,
                        stationDistanceErrorM = if (prior > 0) distanceCorrected - prior else 0.0
                    )
                )
            }
            segmentMaxSpeed = 0.0
            segmentMaxAccel = 0.0
            segmentMaxDecel = 0.0
        } else if (!still && wasStill) {
            // 出发
            segmentDepartTNs = tNs
            segmentDepartS = eskf.p.norm()
            segmentMaxSpeed = 0.0
            segmentMaxAccel = 0.0
            segmentMaxDecel = 0.0
        }
        wasStill = still
    }

    /** 输出低通滤波 + 抽稀记录。 */
    private fun emitSample(tNs: Long, still: Boolean) {
        val c = eskf.attitude
        val forward = Vec3(c.a00, c.a10, c.a20)
        val up = Vec3(c.a02, c.a12, c.a22)
        val left = Vec3(c.a01, c.a11, c.a21)

        // 去重力、去零偏后的导航系加速度
        val aNav = c * (lastAccBody - eskf.ba) + Vec3(0.0, 0.0, -params.g)
        val aAlong = aNav.dot(forward)
        val aVert = aNav.dot(up)
        val aLat = aNav.dot(left)

        val dt = if (lastRecordTNs == 0L) 0.01 else (tNs - lastRecordTNs) / 1e9
        val alpha = if (params.outputLpfTauS <= 0.0) 1.0 else {
            (dt / (params.outputLpfTauS + dt)).coerceIn(0.0, 1.0)
        }
        // 静止时速度直接置零，避免低通滤波把残余速度拖尾带进输出
        val speedNow = if (still) 0.0 else eskf.v.norm()
        if (!lpfInitialized) {
            lpfAccelAlong = aAlong
            lpfAccelVertical = aVert
            lpfAccelLateral = aLat
            lpfSpeed = speedNow
            lpfInitialized = true
        } else {
            lpfAccelAlong += alpha * (aAlong - lpfAccelAlong)
            lpfAccelVertical += alpha * (aVert - lpfAccelVertical)
            lpfAccelLateral += alpha * (aLat - lpfAccelLateral)
            lpfSpeed += alpha * (speedNow - lpfSpeed)
        }

        if (lpfSpeed > segmentMaxSpeed) segmentMaxSpeed = lpfSpeed
        if (lpfAccelAlong > segmentMaxAccel) segmentMaxAccel = lpfAccelAlong
        if (lpfAccelAlong < segmentMaxDecel) segmentMaxDecel = lpfAccelAlong

        if ((tNs - lastRecordTNs) / 1e9 < params.outputStrideS) return
        lastRecordTNs = tNs

        val euler = eskf.euler()
        samples.add(
            TrajSample(
                tNs = tNs,
                tRelS = (tNs - startTNs) / 1e9,
                east = eskf.p.x, north = eskf.p.y, up = eskf.p.z,
                vEast = eskf.v.x, vNorth = eskf.v.y, vUp = eskf.v.z,
                speedMps = if (still) 0.0 else lpfSpeed,
                accelAlong = lpfAccelAlong,
                accelVertical = lpfAccelVertical,
                accelLateral = lpfAccelLateral,
                headingDeg = Math.toDegrees(euler[0]),
                pitchDeg = Math.toDegrees(euler[1]),
                rollDeg = Math.toDegrees(euler[2]),
                still = still,
                posSigmaM = eskf.positionSigma(),
                smoothed = false
            )
        )
    }

    // ==================================================================
    //  结果
    // ==================================================================

    fun rawSamples(): List<TrajSample> = samples

    fun runSegments(): List<RunSegment> = segments

    fun warningList(): List<String> = warnings

    fun divergenceCountValue(): Int = divergenceCount

    fun stillRatio(): Double = if (totalSamples == 0L) 0.0 else stillSamples.toDouble() / totalSamples

    fun nisRejectedCount(): Int = eskf.nisRejected

    /** 因精度超过门限被丢弃的 GNSS 帧数。 */
    fun gnssRejectedCount(): Int = gnssRejected

    fun mountingMatrix(): Mat3 = mounting

    fun alignInfo(): AlignResult? = alignResult

    /** 执行 RTS 平滑，返回平滑轨迹。 */
    fun smooth(): List<TrajSample> {
        val fwd = smoother.forwardHeadings()
        val stats = smoother.headingCorrectionStats(fwd)
        if (stats.first > 0.05) {
            warnings.add(
                String.format("RTS 平滑对航向的最大修正 %.2f 度，说明正向航向存在可观测性不足", stats.first)
            )
        }
        return smoother.smooth(samples)
    }

    fun smootherNodeCount(): Int = smoother.nodeCount

    /** 会话参考原点（用于把 ENU 转回经纬度）。 */
    fun referenceOrigin(): GeoPoint? = gnssOrigin ?: origin

    /** 由第一帧 GNSS 设定参考原点。 */
    fun setGnssOrigin(fix: GnssFix) {
        if (gnssOrigin == null) {
            gnssOrigin = GeoPoint(fix.lat, fix.lon, fix.alt)
            val field = GeomagneticField.field(gnssOrigin!!)
            magRefEnu = field.toEnu().div(1000.0)
            magRefTotal = field.total / 1000.0
            magRefIncl = field.inclinationRad
        }
    }

    fun hasGnssOrigin(): Boolean = gnssOrigin != null

    /** 把 ENU 位置转成经纬度，供导出使用。 */
    fun toGeo(p: Vec3): GeoPoint? {
        val ref = referenceOrigin() ?: return null
        return Geodesy.enuToGeodetic(ref, p)
    }
}
