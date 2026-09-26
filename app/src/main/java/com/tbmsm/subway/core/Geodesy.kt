package com.tbmsm.subway.core

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/** WGS84 椭球参数。 */
object Wgs84 {
    const val A = 6378137.0
    const val F = 1.0 / 298.257223563
    const val E2 = F * (2.0 - F)
    const val B = A * (1.0 - F)
    const val EP2 = (A * A - B * B) / (B * B)
}

data class GeoPoint(val latDeg: Double, val lonDeg: Double, val altM: Double = 0.0)

object Geodesy {

    fun geodeticToEcef(p: GeoPoint): Vec3 {
        val lat = Math.toRadians(p.latDeg)
        val lon = Math.toRadians(p.lonDeg)
        val sinLat = sin(lat)
        val cosLat = cos(lat)
        val n = Wgs84.A / sqrt(1.0 - Wgs84.E2 * sinLat * sinLat)
        val x = (n + p.altM) * cosLat * cos(lon)
        val y = (n + p.altM) * cosLat * sin(lon)
        val z = (n * (1.0 - Wgs84.E2) + p.altM) * sinLat
        return Vec3(x, y, z)
    }

    /** Bowring 迭代法，稳定且收敛快。 */
    fun ecefToGeodetic(e: Vec3): GeoPoint {
        val lon = atan2(e.y, e.x)
        val p = sqrt(e.x * e.x + e.y * e.y)
        if (p < 1e-9) {
            val lat = if (e.z >= 0) 90.0 else -90.0
            return GeoPoint(lat, Math.toDegrees(lon), e.z - Wgs84.B)
        }
        var lat = atan2(e.z, p * (1.0 - Wgs84.E2))
        var n = Wgs84.A
        repeat(8) {
            val sinLat = sin(lat)
            n = Wgs84.A / sqrt(1.0 - Wgs84.E2 * sinLat * sinLat)
            val h = p / cos(lat) - n
            val theta = atan2(e.z * Wgs84.A, p * Wgs84.B) * (Wgs84.B / Wgs84.A)
            val newLat = atan2(
                e.z + Wgs84.EP2 * Wgs84.B * sin(theta) * sin(theta) * sin(theta),
                p - Wgs84.E2 * Wgs84.A * cos(theta) * cos(theta) * cos(theta)
            )
            if (abs(newLat - lat) < 1e-12) {
                lat = newLat
                return@repeat
            }
            lat = newLat
            if (h.isNaN()) return@repeat
        }
        val sinLat = sin(lat)
        val n2 = Wgs84.A / sqrt(1.0 - Wgs84.E2 * sinLat * sinLat)
        val h = p / cos(lat) - n2
        return GeoPoint(Math.toDegrees(lat), Math.toDegrees(lon), h)
    }

    /**
     * 计算 target 相对 ref 的 ENU 坐标（米）。这是把 GNSS 转到局地导航系的唯一入口。
     */
    fun enuDelta(ref: GeoPoint, target: GeoPoint): Vec3 {
        val e1 = geodeticToEcef(ref)
        val e2 = geodeticToEcef(target)
        val dx = e2.x - e1.x
        val dy = e2.y - e1.y
        val dz = e2.z - e1.z
        val lat = Math.toRadians(ref.latDeg)
        val lon = Math.toRadians(ref.lonDeg)
        val sinLat = sin(lat); val cosLat = cos(lat)
        val sinLon = sin(lon); val cosLon = cos(lon)
        val east = -sinLon * dx + cosLon * dy
        val north = -sinLat * cosLon * dx - sinLat * sinLon * dy + cosLat * dz
        val up = cosLat * cosLon * dx + cosLat * sinLon * dy + sinLat * dz
        return Vec3(east, north, up)
    }

    /** ENU 偏移反算经纬高。 */
    fun enuToGeodetic(ref: GeoPoint, enu: Vec3): GeoPoint {
        val refEcef = geodeticToEcef(ref)
        val lat = Math.toRadians(ref.latDeg)
        val lon = Math.toRadians(ref.lonDeg)
        val sinLat = sin(lat); val cosLat = cos(lat)
        val sinLon = sin(lon); val cosLon = cos(lon)
        val dx = -sinLon * enu.x - sinLat * cosLon * enu.y + cosLat * cosLon * enu.z
        val dy = cosLon * enu.x - sinLat * sinLon * enu.y + cosLat * sinLon * enu.z
        val dz = cosLat * enu.y + sinLat * enu.z
        return ecefToGeodetic(Vec3(refEcef.x + dx, refEcef.y + dy, refEcef.z + dz))
    }

    /** 两个经纬点之间的近似水平距离（米），用于站间距校验。 */
    fun horizontalDistance(a: GeoPoint, b: GeoPoint): Double {
        val d = enuDelta(a, b)
        return sqrt(d.x * d.x + d.y * d.y)
    }
}

/**
 * 磁场模型（IGRF-13 截断到 2 阶，即偶极子 + 四极子）。
 *
 * 说明：只用于磁力计门控与初始航向，不参与高精度解算。
 * 2 阶截断在中国区域的地磁偏角误差约 2~4 度，足够做「是否被干扰」的判断，
 * 但若把磁航向当强观测使用，系统性的偏角误差会直接引入航向偏差，
 * 因此默认不开启磁航向更新（见 NavParams.useMagYaw）。
 *
 * 输出单位：nT（Android 磁力计为 uT，使用前需 /1000）。
 */
data class MagField(
    val north: Double,
    val east: Double,
    val down: Double
) {
    val horizontal: Double get() = sqrt(north * north + east * east)
    val total: Double get() = sqrt(north * north + east * east + down * down)
    val declinationRad: Double get() = atan2(east, north)
    val inclinationRad: Double get() = atan2(down, horizontal)

    /** 转换为导航系（ENU）下的磁场向量，单位 nT。 */
    fun toEnu(): Vec3 = Vec3(east, north, -down)
}

object GeomagneticField {

    // IGRF-13 (2020.0) 高斯系数，单位 nT（n=1,2）
    private const val G10 = -29404.8
    private const val G11 = -1450.9
    private const val H11 = 4652.5
    private const val G20 = -2499.6
    private const val G21 = 2982.0
    private const val H21 = -2991.6
    private const val G22 = 1677.0
    private const val H22 = -734.6

    private const val REF_RADIUS_KM = 6371.2

    /**
     * 计算给定位置的地磁要素。
     * 坐标为大地经纬高；内部使用地心纬度近似（对 2 阶截断而言误差可忽略）。
     */
    fun field(p: GeoPoint): MagField {
        val latRad = Math.toRadians(p.latDeg)
        val lonRad = Math.toRadians(p.lonDeg)
        val rKm = (Wgs84.A * (1.0 - Wgs84.F * sin(latRad) * sin(latRad)) + p.altM) / 1000.0

        // 地心余纬（偶极轴为地球自转轴近似，2 阶模型直接使用地理余纬）
        val theta = PI / 2.0 - latRad
        val cosT = cos(theta)
        val sinT = sin(theta)
        val phi = lonRad

        val ar = REF_RADIUS_KM / rKm
        val ar2 = ar * ar          // (a/r)^2 -> n=1 需 (a/r)^3
        val a3 = ar2 * ar          // (a/r)^3
        val a4 = a3 * ar           // (a/r)^4

        val cosP = cos(phi); val sinP = sin(phi)
        val cos2P = cos(2.0 * phi); val sin2P = sin(2.0 * phi)

        // Schmidt 半归一化缔合勒让德函数及其对 theta 的导数
        val p10 = cosT
        val p11 = sinT
        val p20 = 0.5 * (3.0 * cosT * cosT - 1.0)
        val p21 = sqrt(3.0) * sinT * cosT
        val p22 = 0.5 * sqrt(3.0) * sinT * sinT

        val dp10 = -sinT
        val dp11 = cosT
        val dp20 = -3.0 * sinT * cosT
        val dp21 = sqrt(3.0) * (cosT * cosT - sinT * sinT)
        val dp22 = sqrt(3.0) * sinT * cosT

        // n = 1
        val b1 = G10 * p10 + (G11 * cosP + H11 * sinP) * p11
        val db1 = G10 * dp10 + (G11 * cosP + H11 * sinP) * dp11
        val m1 = 1.0 * (G11 * sinP - H11 * cosP) * p11

        // n = 2
        val b2 = G20 * p20 + (G21 * cosP + H21 * sinP) * p21 + (G22 * cos2P + H22 * sin2P) * p22
        val db2 = G20 * dp20 + (G21 * cosP + H21 * sinP) * dp21 + (G22 * cos2P + H22 * sin2P) * dp22
        val m2 = 1.0 * (G21 * sinP - H21 * cosP) * p21 + 2.0 * (G22 * sin2P - H22 * cos2P) * p22

        val xNorth = -(a3 * db1 + a4 * db2)
        val yEast = if (sinT < 1e-9) 0.0 else (a3 * m1 + a4 * m2) / sinT
        val zDown = -(2.0 * a3 * b1 + 3.0 * a4 * b2)

        return MagField(xNorth, yEast, zDown)
    }

    /** 磁偏角（弧度，东偏为正）。 */
    fun declination(p: GeoPoint): Double = field(p).declinationRad

    /** 磁倾角（弧度，向下为正）。 */
    fun inclination(p: GeoPoint): Double = field(p).inclinationRad
}
