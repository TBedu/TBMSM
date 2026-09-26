package com.tbmsm.subway.track

import com.tbmsm.subway.core.GeoPoint
import com.tbmsm.subway.core.Geodesy
import com.tbmsm.subway.core.Vec3
import com.tbmsm.subway.core.wrapPi
import com.tbmsm.subway.model.TrajSample
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * 由传感器解算结果自动生成轨道中心线。
 *
 * 背景：地下段没有 GNSS，无法直接测出轨道走向。但地铁线路有两个可利用的先验：
 *   1. 列车始终沿轨道中心线行驶，所以「列车走过的轨迹」本身就是中心线的一个采样；
 *   2. 站台位置固定，站间距是稳定的物理量。
 *
 * 因此做法是：
 *   1. 取一次（或多次）解算轨迹，按里程等间隔重采样；
 *   2. 用滑动窗口做角度平滑，抑制航向噪声；
 *   3. 用 Douglas–Peucker 简化成折线，得到「直线段 + 曲线段」的紧凑表示；
 *   4. 把停站位置识别为站点，站间距取多次会话的中位数。
 *
 * 精度说明（必须清楚）：
 *   - 生成的中心线**继承了生成它的那次解算的误差**。若那次解算没有轨道约束，
 *     航向会漂移，生成的中心线也会跟着弯——用它去约束下一次解算，
 *     相当于把误差固化下来。所以：
 *   - 首次生成应尽量用「地上段 + 短地下段」的数据，或多次会话取平均；
 *   - 生成后应人工核对（结果页会画出中心线，可直接目视检查是否与线路走向一致）；
 *   - 有条件时优先用运营方的实测中心线替换。
 *
 * 本类不依赖 Android，可在纯 JVM 上测试。
 */
object TrackBuilder {

    /** 生成参数。 */
    data class Options(
        /** 重采样间隔（米）。太小会保留噪声，太大会抹掉曲线。 */
        val resampleStepM: Double = 10.0,
        /** 航向平滑窗口（米）。应大于一个「曲线段」的尺度才能抹平噪声，又不能大到吃掉真实转弯。 */
        val smoothWindowM: Double = 60.0,
        /** Douglas–Peucker 简化容差（米）。 */
        val simplifyToleranceM: Double = 3.0,
        /** 判定为停站的最小静止时长（秒）。 */
        val stationMinStillS: Double = 8.0,
        /** 相邻停站合并距离（米），避免同一站被拆成两个。 */
        val stationMergeM: Double = 120.0,
        /** 轨迹最短长度（米），低于此值不生成。 */
        val minTrackLengthM: Double = 200.0,
        /** 单段轨迹最大长度（米），超过则截断，避免把多次运行拼在一起。 */
        val maxTrackLengthM: Double = 60000.0
    )

    /** 生成结果。 */
    data class Result(
        val track: TrackModel,
        val stationS: List<Double>,
        val stationDistancePriors: List<Double>,
        val sourceSessions: List<String>,
        val rawLengthM: Double,
        val simplifiedPoints: Int,
        val warnings: List<String>
    )

    /**
     * 由单次会话的轨迹生成轨道。
     *
     * @param samples 解算轨迹（ENU，原点为会话参考点）
     * @param origin 会话参考原点，用于把 ENU 转回经纬度
     * @param name 生成的轨道名称
     */
    fun buildFromSession(
        samples: List<TrajSample>,
        origin: GeoPoint,
        name: String,
        options: Options = Options()
    ): Result? = buildFromSessions(listOf(samples to origin), name, options)

    /**
     * 由多次会话的轨迹生成轨道（推荐）。
     *
     * 多次会话的轨迹会先各自重采样，再按「起点对齐 + 长度加权」融合：
     * 由于每次运行的起点都是同一个站台，起点对齐是合理的；
     * 融合能显著削弱单次解算的随机漂移。
     */
    fun buildFromSessions(
        sessions: List<Pair<List<TrajSample>, GeoPoint>>,
        name: String,
        options: Options = Options()
    ): Result? {
        val warnings = ArrayList<String>()
        val usable = sessions.filter { it.first.size >= 10 }
        if (usable.isEmpty()) return null

        // 1. 每条轨迹重采样成等间隔折线
        val polylines = ArrayList<List<Vec3>>(usable.size)
        val stationCandidates = ArrayList<List<Double>>(usable.size)
        for ((samples, _) in usable) {
            val pl = resample(samples, options)
            if (pl.size < 3) continue
            val len = polylineLength(pl)
            if (len < options.minTrackLengthM) continue
            polylines.add(pl)
            stationCandidates.add(detectStations(samples, options))
        }
        if (polylines.isEmpty()) return null

        // 2. 融合：以最长的一条为基准，其余按起点对齐后做加权平均
        val base = polylines.maxByOrNull { polylineLength(it) }!!
        val baseLen = polylineLength(base)
        val fused = if (polylines.size == 1) {
            base
        } else {
            fusePolylines(polylines, baseLen, options)
        }

        // 3. 航向平滑 + 简化
        val smoothed = smoothPolyline(fused, options)
        val simplified = douglasPeucker(smoothed, options.simplifyToleranceM)
        if (simplified.size < 2) return null

        // 4. 站点：多次会话取中位数
        val stationS = mergeStations(stationCandidates, options)
        val priors = ArrayList<Double>()
        for (i in 0 until stationS.size - 1) {
            priors.add(stationS[i + 1] - stationS[i])
        }

        // 5. 转回经纬度
        val origin = usable.first().second
        val geoPoints = simplified.map { Geodesy.enuToGeodetic(origin, it) }

        val model = TrackModel.fromGeoPolyline(name, geoPoints, stationS, priors) ?: return null

        if (polylines.size == 1) {
            warnings.add(
                "轨道由单次解算生成，精度受该次解算误差影响。" +
                    "建议用多次会话生成，或人工核对后再使用。"
            )
        }
        if (stationS.size < 2) {
            warnings.add("未识别到足够的停站位置，站间距先验不可用（不影响航向与横向约束）")
        }
        if (baseLen > options.maxTrackLengthM) {
            warnings.add("轨迹长度超过 ${options.maxTrackLengthM.toInt()} m，可能包含多次运行，建议分段生成")
        }

        return Result(
            track = model,
            stationS = stationS,
            stationDistancePriors = priors,
            sourceSessions = usable.map { it.first.firstOrNull()?.tNs?.toString() ?: "" },
            rawLengthM = baseLen,
            simplifiedPoints = simplified.size,
            warnings = warnings
        )
    }

    // ==================================================================
    //  重采样
    // ==================================================================

    /**
     * 把轨迹按里程等间隔重采样。
     *
     * 用里程而不是时间做参数，是因为列车在站台静止时时间在走、里程不走，
     * 按时间采样会在站台堆出一串重复点，干扰后续的角度平滑。
     */
    private fun resample(samples: List<TrajSample>, options: Options): List<Vec3> {
        val out = ArrayList<Vec3>(1024)
        var acc = 0.0
        var nextS = 0.0
        var prev = Vec3(samples[0].east, samples[0].north, 0.0)
        out.add(prev.copy())

        for (i in 1 until samples.size) {
            val cur = Vec3(samples[i].east, samples[i].north, 0.0)
            val d = hypot(cur.x - prev.x, cur.y - prev.y)
            if (d < 1e-6) continue
            // 单步位移异常（解算跳变）直接跳过，避免污染中心线
            if (d > 50.0) {
                prev = cur
                continue
            }
            var travelled = 0.0
            while (acc + d >= nextS + options.resampleStepM) {
                val need = nextS + options.resampleStepM - acc
                val t = need / d
                out.add(
                    Vec3(
                        prev.x + (cur.x - prev.x) * t,
                        prev.y + (cur.y - prev.y) * t,
                        0.0
                    )
                )
                nextS += options.resampleStepM
                travelled += need
                if (out.size > 20000) return out
            }
            acc += d
            prev = cur
        }
        return out
    }

    private fun polylineLength(pl: List<Vec3>): Double {
        var s = 0.0
        for (i in 0 until pl.size - 1) {
            s += hypot(pl[i + 1].x - pl[i].x, pl[i + 1].y - pl[i].y)
        }
        return s
    }

    // ==================================================================
    //  融合
    // ==================================================================

    /**
     * 多条轨迹融合。
     *
     * 起点对齐（同一条线路的同一站台），按弧长参数做加权平均。
     * 权重取「该轨迹在当前位置是否有效」——超出自身长度的轨迹不参与。
     */
    private fun fusePolylines(
        polylines: List<List<Vec3>>,
        targetLength: Double,
        options: Options
    ): List<Vec3> {
        val step = options.resampleStepM
        val n = (targetLength / step).toInt().coerceAtLeast(2)
        val out = ArrayList<Vec3>(n + 1)
        for (k in 0..n) {
            val s = k * step
            var sx = 0.0
            var sy = 0.0
            var w = 0.0
            for (pl in polylines) {
                val p = pointAtArcLength(pl, s) ?: continue
                sx += p.x
                sy += p.y
                w += 1.0
            }
            if (w > 0) out.add(Vec3(sx / w, sy / w, 0.0))
        }
        return if (out.size >= 2) out else polylines.first()
    }

    /** 按弧长取折线上的点，超出长度返回 null。 */
    private fun pointAtArcLength(pl: List<Vec3>, s: Double): Vec3? {
        if (pl.size < 2) return null
        var acc = 0.0
        for (i in 0 until pl.size - 1) {
            val d = hypot(pl[i + 1].x - pl[i].x, pl[i + 1].y - pl[i].y)
            if (acc + d >= s) {
                val t = if (d < 1e-9) 0.0 else (s - acc) / d
                return Vec3(
                    pl[i].x + (pl[i + 1].x - pl[i].x) * t,
                    pl[i].y + (pl[i + 1].y - pl[i].y) * t,
                    0.0
                )
            }
            acc += d
        }
        return null
    }

    // ==================================================================
    //  航向平滑
    // ==================================================================

    /**
     * 滑动窗口航向平滑。
     *
     * 直接对坐标做低通会把曲线段「拉直」，所以改为：
     * 先算每点的局部航向，对航向做圆周平均（用 sin/cos 分量，避免 ±π 跳变），
     * 再按平滑后的航向重新积分出坐标。
     */
    private fun smoothPolyline(pl: List<Vec3>, options: Options): List<Vec3> {
        if (pl.size < 3) return pl
        val step = options.resampleStepM
        val half = (options.smoothWindowM / step / 2).toInt().coerceAtLeast(1)

        // 逐点局部航向
        val headings = DoubleArray(pl.size)
        for (i in pl.indices) {
            val a = pl[maxOf(0, i - 1)]
            val b = pl[minOf(pl.size - 1, i + 1)]
            headings[i] = atan2(b.x - a.x, b.y - a.y)
        }

        // 圆周平均
        val smoothed = DoubleArray(pl.size)
        for (i in pl.indices) {
            var sc = 0.0
            var ss = 0.0
            var w = 0.0
            for (k in -half..half) {
                val j = i + k
                if (j < 0 || j >= pl.size) continue
                // 三角权重，中心权重最大
                val weight = 1.0 - abs(k).toDouble() / (half + 1)
                sc += cos(headings[j]) * weight
                ss += sin(headings[j]) * weight
                w += weight
            }
            smoothed[i] = if (w > 0) atan2(ss / w, sc / w) else headings[i]
        }

        // 按平滑航向重新积分
        val out = ArrayList<Vec3>(pl.size)
        out.add(pl[0].copy())
        for (i in 1 until pl.size) {
            val d = hypot(pl[i].x - pl[i - 1].x, pl[i].y - pl[i - 1].y)
            val h = smoothed[i]
            val prev = out[i - 1]
            out.add(Vec3(prev.x + sin(h) * d, prev.y + cos(h) * d, 0.0))
        }
        return out
    }

    // ==================================================================
    //  折线简化
    // ==================================================================

    /** Douglas–Peucker 简化，保留转弯特征点。 */
    private fun douglasPeucker(pl: List<Vec3>, tolerance: Double): List<Vec3> {
        if (pl.size < 3) return pl
        val keep = BooleanArray(pl.size)
        keep[0] = true
        keep[pl.size - 1] = true
        simplifyRecursive(pl, 0, pl.size - 1, tolerance, keep)
        val out = ArrayList<Vec3>()
        for (i in pl.indices) if (keep[i]) out.add(pl[i])
        return out
    }

    private fun simplifyRecursive(
        pl: List<Vec3>,
        start: Int,
        end: Int,
        tolerance: Double,
        keep: BooleanArray
    ) {
        if (end <= start + 1) return
        val a = pl[start]
        val b = pl[end]
        val dx = b.x - a.x
        val dy = b.y - a.y
        val len = hypot(dx, dy)
        var maxDist = 0.0
        var maxIdx = -1
        for (i in start + 1 until end) {
            val p = pl[i]
            val dist = if (len < 1e-9) {
                hypot(p.x - a.x, p.y - a.y)
            } else {
                abs(dy * (p.x - a.x) - dx * (p.y - a.y)) / len
            }
            if (dist > maxDist) {
                maxDist = dist
                maxIdx = i
            }
        }
        if (maxDist > tolerance && maxIdx > 0) {
            keep[maxIdx] = true
            simplifyRecursive(pl, start, maxIdx, tolerance, keep)
            simplifyRecursive(pl, maxIdx, end, tolerance, keep)
        }
    }

    // ==================================================================
    //  站点识别
    // ==================================================================

    /**
     * 从轨迹中识别停站位置。
     *
     * 判据：连续静止时长超过阈值，且该处累计里程与上一个站点相距足够远。
     * 返回各站点的沿轨里程（米）。
     */
    private fun detectStations(samples: List<TrajSample>, options: Options): List<Double> {
        val stations = ArrayList<Double>()
        var stillStartT = -1.0
        var stillStartS = 0.0
        var s = 0.0

        for (i in samples.indices) {
            val cur = samples[i]
            if (i > 0) {
                val prev = samples[i - 1]
                s += hypot(cur.east - prev.east, cur.north - prev.north)
            }
            if (cur.still) {
                if (stillStartT < 0) {
                    stillStartT = cur.tRelS
                    stillStartS = s
                }
            } else {
                if (stillStartT >= 0) {
                    val duration = samples[i - 1].tRelS - stillStartT
                    if (duration >= options.stationMinStillS) {
                        val center = (stillStartS + s) / 2.0
                        if (stations.isEmpty() ||
                            center - stations.last() > options.stationMergeM
                        ) {
                            stations.add(center)
                        }
                    }
                    stillStartT = -1.0
                }
            }
        }
        // 收尾：轨迹结束时仍在静止
        if (stillStartT >= 0) {
            val duration = samples.last().tRelS - stillStartT
            if (duration >= options.stationMinStillS) {
                val center = (stillStartS + s) / 2.0
                if (stations.isEmpty() || center - stations.last() > options.stationMergeM) {
                    stations.add(center)
                }
            }
        }
        return stations
    }

    /** 多次会话的站点里程取中位数，并做单调性修正。 */
    private fun mergeStations(candidates: List<List<Double>>, options: Options): List<Double> {
        val valid = candidates.filter { it.size >= 2 }
        if (valid.isEmpty()) return emptyList()
        val maxCount = valid.maxOf { it.size }
        val merged = ArrayList<Double>(maxCount)
        for (k in 0 until maxCount) {
            val values = valid.mapNotNull { it.getOrNull(k) }.sorted()
            if (values.isEmpty()) continue
            val median = values[values.size / 2]
            if (merged.isEmpty() || median - merged.last() > options.stationMergeM) {
                merged.add(median)
            }
        }
        return merged
    }

    // ==================================================================
    //  导出
    // ==================================================================

    /** 生成可直接写入 assets/tracks/ 的 JSON 文本。 */
    fun toJson(result: Result, name: String): String {
        val t = result.track
        val sb = StringBuilder()
        sb.append("{\n")
        sb.append("  \"name\": \"").append(name).append("\",\n")
        sb.append("  \"origin\": { \"lat\": ").append(t.origin.latDeg)
            .append(", \"lon\": ").append(t.origin.lonDeg)
            .append(", \"alt\": ").append(t.origin.altM).append(" },\n")
        sb.append("  \"points\": [\n")
        for (i in t.points.indices) {
            val geo = Geodesy.enuToGeodetic(t.origin, t.points[i])
            sb.append("    { \"lat\": ").append(fmt(geo.latDeg, 7))
                .append(", \"lon\": ").append(fmt(geo.lonDeg, 7)).append(" }")
            if (i < t.points.size - 1) sb.append(",")
            sb.append("\n")
        }
        sb.append("  ],\n")
        sb.append("  \"stations\": [\n")
        for (i in result.stationS.indices) {
            sb.append("    { \"name\": \"站").append(i + 1).append("\", \"s\": ")
                .append(fmt(result.stationS[i], 1)).append(" }")
            if (i < result.stationS.size - 1) sb.append(",")
            sb.append("\n")
        }
        sb.append("  ],\n")
        sb.append("  \"stationDistancePriors\": [")
        sb.append(result.stationDistancePriors.joinToString(", ") { fmt(it, 1) })
        sb.append("]\n")
        sb.append("}\n")
        return sb.toString()
    }

    private fun fmt(v: Double, digits: Int): String =
        String.format(java.util.Locale.US, "%.${digits}f", v)

    /** 供界面预览：返回中心线在 ENU 下的点列。 */
    fun previewPoints(result: Result): List<Pair<Double, Double>> =
        result.track.points.map { it.x to it.y }

    /** 计算两条中心线的平均偏差（米），用于评估生成质量。 */
    fun deviationFrom(reference: TrackModel, candidate: TrackModel): Double {
        if (!reference.valid || !candidate.valid) return Double.NaN
        var sum = 0.0
        var count = 0
        for (p in candidate.points) {
            val proj = reference.project(p) ?: continue
            sum += proj.distance
            count++
        }
        return if (count == 0) Double.NaN else sum / count
    }

    /** 航向一致性检查：返回候选中心线与参考中心线的最大航向差（度）。 */
    fun headingDeviationDeg(reference: TrackModel, candidate: TrackModel): Double {
        if (!reference.valid || !candidate.valid) return Double.NaN
        var maxDev = 0.0
        for (p in candidate.points) {
            val proj = reference.project(p) ?: continue
            val dev = abs(wrapPi(proj.heading - candidate.headingAt(0.0)))
            if (dev > maxDev) maxDev = dev
        }
        return Math.toDegrees(maxDev)
    }

    /** 由 ENU 点列直接构造（供离线工具使用）。 */
    fun fromEnuPoints(
        name: String,
        origin: GeoPoint,
        points: List<Vec3>,
        stationS: List<Double> = emptyList(),
        priors: List<Double> = emptyList()
    ): TrackModel? {
        if (points.size < 2) return null
        return TrackModel(name, origin, points, stationS, priors)
    }

    /** 计算折线总长（对外暴露，供界面显示）。 */
    fun lengthOf(points: List<Vec3>): Double = polylineLength(points)

    /** 点到折线的最近距离（对外暴露）。 */
    fun distanceToPolyline(points: List<Vec3>, p: Vec3): Double {
        var best = Double.MAX_VALUE
        for (i in 0 until points.size - 1) {
            val a = points[i]
            val b = points[i + 1]
            val dx = b.x - a.x
            val dy = b.y - a.y
            val len2 = dx * dx + dy * dy
            val t = if (len2 < 1e-9) 0.0 else {
                (((p.x - a.x) * dx + (p.y - a.y) * dy) / len2).coerceIn(0.0, 1.0)
            }
            val d = hypot(p.x - (a.x + dx * t), p.y - (a.y + dy * t))
            if (d < best) best = d
        }
        return if (best == Double.MAX_VALUE) Double.NaN else best
    }

    /** 折线自交检测的粗略指标：返回最小非相邻段间距（米）。 */
    fun minSelfDistance(points: List<Vec3>): Double {
        if (points.size < 4) return Double.MAX_VALUE
        var best = Double.MAX_VALUE
        for (i in 0 until points.size - 1) {
            for (j in i + 2 until points.size - 1) {
                val d = segmentDistance(points[i], points[i + 1], points[j], points[j + 1])
                if (d < best) best = d
            }
        }
        return best
    }

    private fun segmentDistance(a1: Vec3, a2: Vec3, b1: Vec3, b2: Vec3): Double {
        // 采样法近似，够用
        var best = Double.MAX_VALUE
        val n = 8
        for (i in 0..n) {
            val t = i.toDouble() / n
            val p = Vec3(a1.x + (a2.x - a1.x) * t, a1.y + (a2.y - a1.y) * t, 0.0)
            for (k in 0..n) {
                val u = k.toDouble() / n
                val q = Vec3(b1.x + (b2.x - b1.x) * u, b1.y + (b2.y - b1.y) * u, 0.0)
                val d = hypot(p.x - q.x, p.y - q.y)
                if (d < best) best = d
            }
        }
        return best
    }

    /** 曲率统计，用于判断生成的中心线是否合理（地铁最小曲线半径通常 > 300 m）。 */
    fun curvatureStats(points: List<Vec3>): Pair<Double, Double> {
        if (points.size < 3) return 0.0 to 0.0
        var maxK = 0.0
        var sumK = 0.0
        var count = 0
        for (i in 1 until points.size - 1) {
            val a = points[i - 1]
            val b = points[i]
            val c = points[i + 1]
            val h1 = atan2(b.x - a.x, b.y - a.y)
            val h2 = atan2(c.x - b.x, c.y - b.y)
            val dPsi = abs(wrapPi(h2 - h1))
            val ds = (hypot(b.x - a.x, b.y - a.y) + hypot(c.x - b.x, c.y - b.y)) / 2.0
            if (ds > 1e-6) {
                val k = dPsi / ds
                if (k > maxK) maxK = k
                sumK += k
                count++
            }
        }
        return maxK to (if (count > 0) sumK / count else 0.0)
    }

    /** 最小曲线半径（米），用于合理性检查。 */
    fun minCurveRadius(points: List<Vec3>): Double {
        val (maxK, _) = curvatureStats(points)
        return if (maxK < 1e-9) Double.MAX_VALUE else 1.0 / maxK
    }

    /** 判断生成的中心线是否可信。 */
    fun sanityCheck(result: Result): List<String> {
        val issues = ArrayList<String>()
        val pts = result.track.points
        val radius = minCurveRadius(pts)
        if (radius < 150.0) {
            issues.add(
                String.format("最小曲线半径仅 %.0f m，小于地铁常见下限（约 300 m），" +
                    "可能是解算航向漂移导致的虚假转弯", radius
            )
        }
        val selfDist = minSelfDistance(pts)
        if (selfDist < 30.0) {
            issues.add(
                String.format("中心线存在自交或近自交（最小间距 %.0f m），" +
                    "说明轨迹被解算误差扭曲", selfDist
            )
        }
        if (result.simplifiedPoints < 3) {
            issues.add("简化后只剩一条直线，可能丢失了真实转弯")
        }
        return issues
    }

    /** 计算折线在给定里程处的切向（供外部校验）。 */
    fun tangentAt(points: List<Vec3>, s: Double): Vec3 {
        var acc = 0.0
        for (i in 0 until points.size - 1) {
            val d = hypot(points[i + 1].x - points[i].x, points[i + 1].y - points[i].y)
            if (acc + d >= s) {
                val dx = points[i + 1].x - points[i].x
                val dy = points[i + 1].y - points[i].y
                val n = sqrt(dx * dx + dy * dy)
                return if (n < 1e-9) Vec3(0.0, 1.0, 0.0) else Vec3(dx / n, dy / n, 0.0)
            }
            acc += d
        }
        return Vec3(0.0, 1.0, 0.0)
    }
}
