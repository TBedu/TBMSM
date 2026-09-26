package com.tbmsm.subway.track

import com.tbmsm.subway.core.GeoPoint
import com.tbmsm.subway.core.Geodesy
import com.tbmsm.subway.core.Vec3
import com.tbmsm.subway.core.wrapPi
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * 轨道中心线：以折线形式给出，内部预计算累计里程、切向、法向与曲率。
 *
 * 坐标全部为以 origin 为原点的 ENU 米制坐标，避免在解算循环里反复做大地坐标换算。
 */
class TrackModel(
    val name: String,
    val origin: GeoPoint,
    val points: List<Vec3>,
    /** 站台中心点对应的里程（米），可为空。 */
    val stationS: List<Double> = emptyList(),
    /** 站间距先验（米），长度应为 stationS.size - 1。 */
    val stationDistancePriors: List<Double> = emptyList()
) {

    val size: Int get() = points.size
    val valid: Boolean get() = points.size >= 2

    /** 各顶点累计里程。 */
    val cumulativeS: DoubleArray = DoubleArray(points.size)

    /** 各段切向单位向量（ENU 水平面内）。 */
    private val segTangent = Array(points.size - 1) { Vec3.zero() }

    /** 各段长度。 */
    private val segLength = DoubleArray(points.size - 1)

    /** 各段航向（弧度，ENU 下 atan2(E, N)）。 */
    private val segHeading = DoubleArray(points.size - 1)

    /** 各段曲率（1/m，左转为正）。 */
    private val segCurvature = DoubleArray(points.size - 1)

    /** 各段横向单位向量（切向左转 90 度）。 */
    private val segLeft = Array(points.size - 1) { Vec3.zero() }

    val totalLength: Double

    init {
        var s = 0.0
        for (i in 0 until points.size - 1) {
            val d = points[i + 1] - points[i]
            val len = sqrt(d.x * d.x + d.y * d.y)
            segLength[i] = len
            cumulativeS[i] = s
            if (len > 1e-6) {
                val t = Vec3(d.x / len, d.y / len, 0.0)
                segTangent[i] = t
                segHeading[i] = atan2(t.x, t.y)
                segLeft[i] = Vec3(-t.y, t.x, 0.0)
            } else {
                segTangent[i] = Vec3(0.0, 1.0, 0.0)
                segHeading[i] = 0.0
                segLeft[i] = Vec3(-1.0, 0.0, 0.0)
            }
            s += len
        }
        cumulativeS[points.size - 1] = s
        totalLength = s

        // 曲率用相邻段航向差 / 平均段长估计
        for (i in 0 until points.size - 1) {
            val prev = if (i > 0) segHeading[i - 1] else segHeading[i]
            val next = if (i < points.size - 2) segHeading[i + 1] else segHeading[i]
            val dPsi = wrapPi(next - prev)
            val ds = if (i > 0 && i < points.size - 2) {
                0.5 * (segLength[i - 1] + segLength[i + 1])
            } else {
                segLength[i]
            }
            segCurvature[i] = if (ds > 1e-6) dPsi / ds else 0.0
        }
    }

    /** 最近点投影结果。 */
    data class Projection(
        val s: Double,
        val lateral: Double,
        val heading: Double,
        val curvature: Double,
        val left: Vec3,
        val tangent: Vec3,
        val point: Vec3,
        val distance: Double
    )

    /**
     * 把 ENU 位置投影到轨道中心线。
     *
     * @param hintS 上一帧里程，用于局部搜索；传 null 则全局搜索。
     * @param searchWindowM 局部搜索窗口（米）。
     */
    fun project(p: Vec3, hintS: Double? = null, searchWindowM: Double = 200.0): Projection? {
        if (!valid) return null
        var best = Double.MAX_VALUE
        var bestSeg = -1
        var bestT = 0.0

        val i0: Int
        val i1: Int
        if (hintS != null) {
            i0 = segmentIndexAt((hintS - searchWindowM).coerceAtLeast(0.0))
            i1 = segmentIndexAt((hintS + searchWindowM).coerceAtMost(totalLength))
        } else {
            i0 = 0
            i1 = segLength.size - 1
        }

        for (i in i0..i1) {
            val a = points[i]
            val b = points[i + 1]
            val abx = b.x - a.x
            val aby = b.y - a.y
            val len2 = abx * abx + aby * aby
            if (len2 < 1e-9) continue
            var t = ((p.x - a.x) * abx + (p.y - a.y) * aby) / len2
            t = t.coerceIn(0.0, 1.0)
            val cx = a.x + abx * t
            val cy = a.y + aby * t
            val dx = p.x - cx
            val dy = p.y - cy
            val d2 = dx * dx + dy * dy
            if (d2 < best) {
                best = d2
                bestSeg = i
                bestT = t
            }
        }
        if (bestSeg < 0) return null

        val a = points[bestSeg]
        val b = points[bestSeg + 1]
        val point = Vec3(a.x + (b.x - a.x) * bestT, a.y + (b.y - a.y) * bestT, 0.0)
        val tangent = segTangent[bestSeg]
        val left = segLeft[bestSeg]
        val s = cumulativeS[bestSeg] + segLength[bestSeg] * bestT
        val lateral = (p.x - point.x) * left.x + (p.y - point.y) * left.y

        return Projection(
            s = s,
            lateral = lateral,
            heading = segHeading[bestSeg],
            curvature = segCurvature[bestSeg],
            left = left,
            tangent = tangent,
            point = point,
            distance = sqrt(best)
        )
    }

    fun segmentIndexAt(s: Double): Int {
        if (segLength.isEmpty()) return 0
        val sc = s.coerceIn(0.0, totalLength)
        var lo = 0
        var hi = segLength.size - 1
        while (lo < hi) {
            val mid = (lo + hi + 1) / 2
            if (cumulativeS[mid] <= sc) lo = mid else hi = mid - 1
        }
        return lo
    }

    /** 里程 s 处的切向。 */
    fun tangentAt(s: Double): Vec3 {
        if (!valid) return Vec3(0.0, 1.0, 0.0)
        return segTangent[segmentIndexAt(s)]
    }

    /** 里程 s 处的航向。 */
    fun headingAt(s: Double): Double {
        if (!valid) return 0.0
        return segHeading[segmentIndexAt(s)]
    }

    /** 里程 s 处的曲率。 */
    fun curvatureAt(s: Double): Double {
        if (!valid) return 0.0
        return segCurvature[segmentIndexAt(s)]
    }

    /** 里程 s 处的横向单位向量（左）。 */
    fun leftAt(s: Double): Vec3 {
        if (!valid) return Vec3(-1.0, 0.0, 0.0)
        return segLeft[segmentIndexAt(s)]
    }

    /** 里程 s 处的位置。 */
    fun pointAt(s: Double): Vec3 {
        if (!valid) return Vec3.zero()
        val i = segmentIndexAt(s)
        val ds = (s - cumulativeS[i]).coerceIn(0.0, segLength[i])
        val a = points[i]
        val t = segTangent[i]
        return Vec3(a.x + t.x * ds, a.y + t.y * ds, 0.0)
    }

    /** 里程 s 处是否处于曲线段（用于放宽航向约束）。 */
    fun isCurve(s: Double, threshold: Double = 1.0 / 800.0): Boolean =
        abs(curvatureAt(s)) > threshold

    /** 最近的站台里程与站序号。 */
    fun nearestStation(s: Double): Pair<Int, Double>? {
        if (stationS.isEmpty()) return null
        var bestIdx = 0
        var bestD = Double.MAX_VALUE
        for (i in stationS.indices) {
            val d = abs(stationS[i] - s)
            if (d < bestD) {
                bestD = d
                bestIdx = i
            }
        }
        return bestIdx to bestD
    }

    /** 站间距先验（米），越界返回 null。 */
    fun stationPrior(fromIdx: Int, toIdx: Int): Double? {
        if (fromIdx < 0 || toIdx < 0) return null
        val lo = minOf(fromIdx, toIdx)
        val hi = maxOf(fromIdx, toIdx)
        if (hi - lo != 1) return null
        if (lo >= stationDistancePriors.size) return null
        return stationDistancePriors[lo]
    }

    /** 把 ENU 位置转回经纬度，用于导出。 */
    fun toGeo(p: Vec3): GeoPoint = Geodesy.enuToGeodetic(origin, p)

    companion object {

        /**
         * 由经纬度折线构造。origin 取第一个点。
         */
        fun fromGeoPolyline(
            name: String,
            geoPoints: List<GeoPoint>,
            stationS: List<Double> = emptyList(),
            stationDistancePriors: List<Double> = emptyList()
        ): TrackModel? {
            if (geoPoints.size < 2) return null
            val origin = geoPoints.first()
            val enu = geoPoints.map { Geodesy.enuDelta(origin, it) }
            return TrackModel(name, origin, enu, stationS, stationDistancePriors)
        }

        /**
         * 由「起点 + 方位角/长度序列」构造，适合没有实测轨道数据时手工录入。
         *
         * @param legs 每段 (方位角弧度, 长度米)
         */
        fun fromLegs(
            name: String,
            origin: GeoPoint,
            legs: List<Pair<Double, Double>>,
            stationS: List<Double> = emptyList(),
            stationDistancePriors: List<Double> = emptyList()
        ): TrackModel? {
            if (legs.isEmpty()) return null
            val pts = ArrayList<Vec3>(legs.size + 1)
            var e = 0.0
            var n = 0.0
            pts.add(Vec3(0.0, 0.0, 0.0))
            for ((bearing, len) in legs) {
                e += sin(bearing) * len
                n += cos(bearing) * len
                pts.add(Vec3(e, n, 0.0))
            }
            return TrackModel(name, origin, pts, stationS, stationDistancePriors)
        }
    }
}
