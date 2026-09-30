package com.tbmsm.subway.algo

import kotlin.math.PI

/**
 * 全部噪声与门限参数。
 *
 * 默认值针对典型消费级手机 MEMS（BMI160 / ICM-42605 / LSM6DSO 一类）。
 * 若条件允许，应用一段 1~2 小时静置数据做 Allan 方差标定后替换：
 *  - sigmaGyroNoise  ：陀螺角度随机游走（ARW）
 *  - sigmaGyroBiasRw ：陀螺零偏不稳定性
 *  - sigmaAccNoise   ：加计速度随机游走（VRW）
 *  - sigmaAccBiasRw  ：加计零偏不稳定性
 */
data class NavParams(

    // ---------------- 过程噪声（连续时间谱密度） ----------------
    /** 加计噪声密度，单位 m/s^2/sqrt(Hz)。 */
    val sigmaAccNoise: Double = 1.5e-3,
    /** 陀螺噪声密度，单位 rad/s/sqrt(Hz)，对应 ARW≈0.3 deg/sqrt(h)。 */
    val sigmaGyroNoise: Double = 1.0e-4,
    /** 加计零偏随机游走，单位 m/s^2/sqrt(Hz)。 */
    val sigmaAccBiasRw: Double = 1.0e-4,
    /** 陀螺零偏随机游走，单位 rad/s/sqrt(Hz)。 */
    val sigmaGyroBiasRw: Double = 1.0e-6,

    // ---------------- 初始协方差 ----------------
    val initPosSigma: Double = 10.0,
    val initVelSigma: Double = 0.05,
    /** 初始姿态不确定度。安装偏航不可观测，必须给大。 */
    val initAttSigma: Double = 15.0 * PI / 180.0,
    val initGyroBiasSigma: Double = 0.5 * PI / 180.0,
    val initAccBiasSigma: Double = 0.10,

    // ---------------- 静止检测（GLRT + 迟滞） ----------------
    /** 滑动窗口长度（采样点数），100Hz 下 50 点 = 0.5s。 */
    val zuptWindow: Int = 50,
    /**
     * GLRT 中假定的加计样本噪声标准差，m/s^2。
     *
     * 地铁车厢停站时并非绝对静止：空调与空压机持续振动、乘客走动、
     * 车体受轨道与邻线列车扰动，实测残余加速度可达 0.05~0.15 m/s²。
     * 取 0.02 会让统计量长期高于门限，导致停站期间无法进入静止、
     * 丢失最宝贵的零速锚点。放宽到 0.10 以覆盖这些低频扰动。
     */
    val zuptSigmaAcc: Double = 0.10,
    /**
     * GLRT 中假定的陀螺样本噪声标准差，rad/s。
     *
     * 同理，停站时车体仍有微小角振动（约 0.005~0.02 rad/s）。
     * 取 0.002 过于严格，放宽到 0.01。
     */
    val zuptSigmaGyro: Double = 0.01,
    /**
     * GLRT 判决门限（6 自由度卡方，理论均值 6）。
     *
     * 放宽噪声假设后统计量整体变小，门限相应下调到 25，
     * 使「停站」与「运行」的判决边界仍落在合理位置。
     */
    val zuptThreshold: Double = 25.0,
    /** 进入静止需连续满足的采样点数（100Hz 下 100 点 = 1.0s）。 */
    val zuptEnterHold: Int = 100,
    /** 退出静止需连续失败的采样点数，取小值以免在停站期间丢失锚定。 */
    val zuptExitHold: Int = 5,
    /**
     * 安全阀：估计速度超过该值时拒绝 ZUPT。
     * 用于防御「匀速平顺运行被误判为静止」这一最危险的失效模式。
     *
     * 取 2.0 而非 5.0：地铁站间最高速度通常 60~80 km/h（17~22 m/s），
     * 但进站前会长时间低速滑行（1~3 m/s）。若门限设成 5.0，
     * 这段低速滑行会被误判为静止并强行把速度拉零，造成里程严重偏短。
     * 2.0 能在保留停站锚定的同时挡住低速滑行误判。
     */
    val zuptMaxSpeedGuard: Double = 2.0,
    /** ZUPT 速度观测噪声，m/s。放宽后允许停站时存在微小残余速度。 */
    val zuptVelSigma: Double = 0.03,
    /** 重力调平观测噪声（弧度），仅静止时使用。 */
    val levelSigma: Double = 0.03,

    // ---------------- GNSS ----------------
    /** GNSS 位置观测噪声（水平），m。城市峡谷建议 10~20。 */
    val gnssPosSigma: Double = 8.0,
    /** GNSS 速度观测噪声，m/s。多普勒速度精度较高。 */
    val gnssVelSigma: Double = 0.30,
    /** 航迹角观测噪声，弧度。 */
    val gnssCourseSigma: Double = 2.0 * PI / 180.0,
    /** 使用航迹角所需的最小速度，m/s。 */
    val gnssCourseMinSpeed: Double = 3.0,
    /**
     * GNSS 可用性门限：水平精度超过该值即整帧丢弃（位置、速度、航迹角都不用）。
     *
     * 取 10 m 的理由：城市峡谷与地铁出入口的多径误差常达 20~50 m 且带系统性偏置，
     * 这种量级的观测一旦注入，会把已经收敛的航向和位置重新拉偏，
     * 而它带来的信息量远小于轨道约束与 ZUPT。宁可不用。
     */
    val gnssMaxAccuracyM: Double = 10.0,
    /** 是否使用 GNSS 位置观测量（弱约束）。 */
    val useGnssPosition: Boolean = true,

    // ---------------- 磁力计 ----------------
    /**
     * 是否使用磁航向更新。
     * 默认关闭：2 阶磁场模型存在 2~4 度系统性偏角误差，直接当观测会引入航向偏差。
     * 仅在确认当地偏角准确、且环境干扰小时才建议打开。
     */
    val useMagYaw: Boolean = false,
    /** 是否允许用磁力计确定初始航向（比持续更新安全）。 */
    val useMagHeadingInit: Boolean = true,
    /** 磁力计总场模长门限（相对误差）。 */
    val magNormTolRel: Double = 0.25,
    /** 磁倾角门限（弧度）。 */
    val magInclTolRad: Double = 10.0 * PI / 180.0,
    /** 磁航向与滤波器航向一致性门限（弧度）。 */
    val magConsistTolRad: Double = 15.0 * PI / 180.0,
    /** 磁航向观测噪声（弧度），刻意给大。 */
    val magYawSigma: Double = 12.0 * PI / 180.0,

    // ---------------- 轨道约束 ----------------
    /** 轨道航向观测噪声（直线段），弧度。 */
    val trackYawSigmaStraight: Double = 2.0 * PI / 180.0,
    /** 轨道航向观测噪声（曲线段），弧度。 */
    val trackYawSigmaCurve: Double = 8.0 * PI / 180.0,
    /** 轨道横向位置观测噪声，m。 */
    val trackLateralSigma: Double = 2.0,
    /** 站间里程先验观测噪声，m。 */
    val stationDistanceSigma: Double = 5.0,
    /** 最近点投影的最大搜索半径，m。超出则认为轨道数据失效。 */
    val trackMaxLateralM: Double = 300.0,

    // ---------------- 初始化 ----------------
    /** 粗对准所需的静置时长（秒）。 */
    val alignDurationS: Double = 3.0,
    /** 粗对准期间允许的最大加计模长偏差，m/s^2。 */
    val alignAccTolMps2: Double = 0.30,
    /** 粗对准期间允许的最大陀螺模长，rad/s。 */
    val alignGyroTolRad: Double = 0.02,
    /** 粗对准期间允许的最大加计方差，m^2/s^4。 */
    val alignAccVarMax: Double = 0.01,
    /**
     * 粗对准期间允许的最大陀螺方差，rad^2/s^2。
     *
     * 原值 1.0e-4（σ≈0.01 rad/s）对地铁车厢偏严：停站时车体仍有
     * 空调与空压机引起的角振动，容易导致对准反复被拒、迟迟无法开始。
     * 放宽到 4.0e-4（σ≈0.02 rad/s），与 ZUPT 的噪声假设保持一致。
     */
    val alignGyroVarMax: Double = 4.0e-4,

    // ---------------- 输出 ----------------
    /** 输出低通滤波时间常数，s。 */
    val outputLpfTauS: Double = 0.5,
    /** 轨迹抽稀保存间隔，s。 */
    val outputStrideS: Double = 0.1
) {
    val g: Double get() = 9.80665
}
