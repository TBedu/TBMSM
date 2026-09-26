package com.tbmsm.subway.model

/**
 * 一帧 IMU 原始数据。
 *
 * 坐标系为 Android 手机坐标（x=屏幕右, y=屏幕上, z=屏幕外），
 * 单位分别为 m/s^2（含重力）与 rad/s。时间戳取 SensorEvent.timestamp（纳秒，单调时钟）。
 */
data class ImuSample(
    val tNs: Long,
    val ax: Double, val ay: Double, val az: Double,
    val gx: Double, val gy: Double, val gz: Double
)

/** 一帧磁力计，单位 uT（Android 原始单位）。 */
data class MagSample(
    val tNs: Long,
    val mx: Double, val my: Double, val mz: Double
)

/**
 * 一次 GNSS 定位。
 *
 * @param tNs elapsedRealtimeNanos，与传感器时间戳同一时基
 * @param accuracyM 水平精度估计（米）
 * @param speedMps 多普勒速度（m/s），无速度解时为 -1
 * @param bearingDeg 航迹角（度，正北顺时针），无方位解时为 -1
 */
data class GnssFix(
    val tNs: Long,
    val lat: Double,
    val lon: Double,
    val alt: Double,
    val accuracyM: Double,
    val speedMps: Double,
    val bearingDeg: Double,
    val provider: String
) {
    val hasSpeed: Boolean get() = speedMps >= 0.0
    val hasBearing: Boolean get() = bearingDeg >= 0.0
}

/**
 * 轨迹输出样本（导航系 ENU，原点为本次会话的参考点）。
 */
data class TrajSample(
    val tNs: Long,
    val tRelS: Double,
    val east: Double,
    val north: Double,
    val up: Double,
    val vEast: Double,
    val vNorth: Double,
    val vUp: Double,
    val speedMps: Double,
    /** 沿列车前进方向的加速度（已去重力、去零偏），m/s^2。 */
    val accelAlong: Double,
    /** 垂向加速度，m/s^2。 */
    val accelVertical: Double,
    /** 横向加速度，m/s^2。 */
    val accelLateral: Double,
    val headingDeg: Double,
    val pitchDeg: Double,
    val rollDeg: Double,
    val still: Boolean,
    val posSigmaM: Double,
    val smoothed: Boolean
) {
    fun csvRow(): String = String.format(
        java.util.Locale.US,
        "%d,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%d,%.4f,%d",
        tNs, tRelS, east, north, up, vEast, vNorth, vUp, speedMps,
        accelAlong, accelVertical, accelLateral, headingDeg, pitchDeg, rollDeg,
        if (still) 1 else 0, posSigmaM, if (smoothed) 1 else 0
    )

    companion object {
        const val CSV_HEADER =
            "t_ns,t_rel_s,east_m,north_m,up_m,ve_mps,vn_mps,vu_mps,speed_mps," +
                "accel_along,accel_vertical,accel_lateral,heading_deg,pitch_deg,roll_deg," +
                "still,pos_sigma_m,smoothed"
    }
}

/**
 * 一个站间运行段（从「起动」到「下一次停稳」）。
 */
data class RunSegment(
    val index: Int,
    val departTNs: Long,
    val arriveTNs: Long,
    val runTimeS: Double,
    val distanceM: Double,
    val meanSpeedMps: Double,
    val maxSpeedMps: Double,
    val maxAccelMps2: Double,
    val maxDecelMps2: Double,
    val stationDistancePriorM: Double,
    val stationDistanceErrorM: Double
)
