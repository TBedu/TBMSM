package com.tbmsm.subway.model

/** 引擎状态机。 */
enum class EngineState {
    /** 未开始。 */
    IDLE,
    /** 静止粗对准中（等待连续静止窗口）。 */
    ALIGNING,
    /** 正常运行解算。 */
    RUNNING,
    /** 已结束（正常）。 */
    FINISHED,
    /** 出现不可恢复错误。 */
    ERROR
}

/**
 * 供界面消费的实时状态快照。服务与界面之间通过 SessionBus 传递该对象。
 */
data class LiveState(
    val state: EngineState = EngineState.IDLE,
    val sessionId: String = "",
    val elapsedS: Double = 0.0,
    val speedMps: Double = 0.0,
    val distanceM: Double = 0.0,
    val headingDeg: Double = 0.0,
    val pitchDeg: Double = 0.0,
    val rollDeg: Double = 0.0,
    val accelAlong: Double = 0.0,
    val accelVertical: Double = 0.0,
    val accelLateral: Double = 0.0,
    val still: Boolean = false,
    val zuptStatistic: Double = -1.0,
    val posSigmaM: Double = 0.0,
    val headingSigmaDeg: Double = 0.0,
    val gyroBiasDegPerS: Double = 0.0,
    val alignProgress: Float = 0f,
    val gnssActive: Boolean = false,
    val gnssAccuracyM: Double = 0.0,
    val gnssCount: Int = 0,
    /** 因精度超限被整帧丢弃的 GNSS 帧数。 */
    val gnssRejected: Int = 0,
    val magOk: Boolean = false,
    val trackActive: Boolean = false,
    val segmentCount: Int = 0,
    val lastSegmentRunTimeS: Double = 0.0,
    val lastSegmentDistanceM: Double = 0.0,
    val imuCount: Long = 0,
    val imuRateHz: Double = 0.0,
    val warning: String = "",
    val fatalError: String = ""
) {
    val speedKmh: Double get() = speedMps * 3.6
}

/** 会话结束后落盘的总结。 */
data class SessionSummary(
    val sessionId: String,
    val startedAtMs: Long,
    val durationS: Double,
    val totalDistanceM: Double,
    val maxSpeedMps: Double,
    val meanSpeedMps: Double,
    val maxAccelMps2: Double,
    val maxDecelMps2: Double,
    val imuSamples: Long,
    val gnssFixes: Int,
    /** 因精度超限被丢弃的 GNSS 帧数。 */
    val gnssRejected: Int,
    val stillRatio: Double,
    val nisRejected: Int,
    val divergenceCount: Int,
    val smoothed: Boolean,
    val trackUsed: Boolean,
    val trackName: String,
    val warnings: List<String>,
    val segments: List<RunSegment>,
    val originLat: Double,
    val originLon: Double,
    val originAlt: Double
)
