package com.tbmsm.subway.sensors

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.Bundle
import android.os.Handler
import android.os.HandlerThread
import android.os.Looper
import com.tbmsm.subway.model.GnssFix
import com.tbmsm.subway.model.ImuSample
import com.tbmsm.subway.model.MagSample

/**
 * 传感器采集器。
 *
 * 关键实现细节：
 *  1. 使用 SensorEvent.timestamp（elapsedRealtimeNanos 时基）而不是回调到达时间。
 *     Android 的 sensor batching 会把多个样本一次性上报，用到达时间会造成
 *     时间戳挤在一起、dt 严重失真，直接毁掉积分。
 *  2. 传感器回调放在独立 HandlerThread 上，避免主线程抖动影响采样间隔。
 *  3. 加计与陀螺分别注册，各自带自己的时间戳；引擎按到达顺序处理，
 *     用相邻同类型样本的时间差作为 dt。
 *  4. GNSS 使用 LocationManager（系统定位），优先取多普勒速度与航迹角。
 */
class SensorCollector(
    private val context: Context,
    private val listener: Listener
) {

    interface Listener {
        fun onImu(sample: ImuSample)
        fun onMag(sample: MagSample)
        fun onGnss(fix: GnssFix)
        fun onSensorError(message: String)
    }

    private val sensorManager =
        context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val locationManager =
        context.getSystemService(Context.LOCATION_SERVICE) as LocationManager

    private var thread: HandlerThread? = null

    private var accel: Sensor? = null
    private var gyro: Sensor? = null
    private var mag: Sensor? = null

    private var lastAccelTNs = 0L

    /** 加计与陀螺各自缓存最近一帧，凑齐后合成一个 ImuSample。 */
    private var pendingAx = 0.0
    private var pendingAy = 0.0
    private var pendingAz = 0.0
    private var pendingAccelTNs = 0L
    private var hasAccel = false

    private var pendingGx = 0.0
    private var pendingGy = 0.0
    private var pendingGz = 0.0
    private var pendingGyroTNs = 0L
    private var hasGyro = false

    var imuRateHz: Double = 0.0
        private set

    private val imuListener = object : SensorEventListener {
        override fun onSensorChanged(event: SensorEvent) {
            when (event.sensor.type) {
                Sensor.TYPE_ACCELEROMETER -> {
                    pendingAx = event.values[0].toDouble()
                    pendingAy = event.values[1].toDouble()
                    pendingAz = event.values[2].toDouble()
                    pendingAccelTNs = event.timestamp
                    hasAccel = true
                    if (lastAccelTNs != 0L) {
                        val dt = (event.timestamp - lastAccelTNs) / 1e9
                        if (dt > 0) imuRateHz = 1.0 / dt
                    }
                    lastAccelTNs = event.timestamp
                }
                Sensor.TYPE_GYROSCOPE -> {
                    pendingGx = event.values[0].toDouble()
                    pendingGy = event.values[1].toDouble()
                    pendingGz = event.values[2].toDouble()
                    pendingGyroTNs = event.timestamp
                    hasGyro = true
                }
            }
            if (hasAccel && hasGyro) {
                // 以较新的时间戳作为该帧时间
                val t = maxOf(pendingAccelTNs, pendingGyroTNs)
                listener.onImu(
                    ImuSample(
                        tNs = t,
                        ax = pendingAx, ay = pendingAy, az = pendingAz,
                        gx = pendingGx, gy = pendingGy, gz = pendingGz
                    )
                )
                hasAccel = false
                hasGyro = false
            }
        }

        override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit
    }

    private val magListener = object : SensorEventListener {
        override fun onSensorChanged(event: SensorEvent) {
            listener.onMag(
                MagSample(
                    tNs = event.timestamp,
                    mx = event.values[0].toDouble(),
                    my = event.values[1].toDouble(),
                    mz = event.values[2].toDouble()
                )
            )
        }

        override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit
    }

    private val locationListener = object : LocationListener {
        override fun onLocationChanged(location: Location) {
            val tNs = if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.JELLY_BEAN_MR1) {
                location.elapsedRealtimeNanos
            } else {
                // 低版本没有 elapsedRealtimeNanos，用同一时基的 SystemClock 兜底，
                // 不能用 System.nanoTime()（时基不同，会让时长从几百秒开始）。
                android.os.SystemClock.elapsedRealtimeNanos()
            }
            val speed = if (location.hasSpeed()) location.speed.toDouble() else -1.0
            val bearing = if (location.hasBearing()) location.bearing.toDouble() else -1.0
            listener.onGnss(
                GnssFix(
                    tNs = tNs,
                    lat = location.latitude,
                    lon = location.longitude,
                    alt = location.altitude,
                    accuracyM = if (location.hasAccuracy()) location.accuracy.toDouble() else 99.0,
                    speedMps = speed,
                    bearingDeg = bearing,
                    provider = location.provider ?: "unknown"
                )
            )
        }

        @Deprecated("Deprecated in Java")
        override fun onStatusChanged(provider: String?, status: Int, extras: Bundle?) = Unit

        override fun onProviderEnabled(provider: String) = Unit

        override fun onProviderDisabled(provider: String) = Unit
    }

    /** 检查必需传感器是否存在。 */
    fun checkAvailability(): String? {
        accel = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        gyro = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        mag = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)
        if (accel == null) return "设备没有加速度计，无法使用"
        if (gyro == null) return "设备没有陀螺仪，无法使用"
        return null
    }

    val hasMagnetometer: Boolean get() = mag != null

    fun start() {
        val ht = HandlerThread("sensor-collector").also { it.start() }
        thread = ht
        val h = Handler(ht.looper)

        accel = accel ?: sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        gyro = gyro ?: sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        mag = mag ?: sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)

        // SENSOR_DELAY_GAME 通常为 50Hz，部分机型可达 100Hz 以上
        accel?.let {
            sensorManager.registerListener(imuListener, it, SensorManager.SENSOR_DELAY_GAME, h)
        }
        gyro?.let {
            sensorManager.registerListener(imuListener, it, SensorManager.SENSOR_DELAY_GAME, h)
        }
        mag?.let {
            sensorManager.registerListener(magListener, it, SensorManager.SENSOR_DELAY_UI, h)
        }

        startLocation()
    }

    private fun startLocation() {
        try {
            val providers = ArrayList<String>()
            if (locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER)) {
                providers.add(LocationManager.GPS_PROVIDER)
            }
            if (locationManager.isProviderEnabled(LocationManager.NETWORK_PROVIDER)) {
                providers.add(LocationManager.NETWORK_PROVIDER)
            }
            if (providers.isEmpty()) {
                listener.onSensorError("定位服务未开启，地上段将无法获得绝对位置与航向")
                return
            }
            for (p in providers) {
                locationManager.requestLocationUpdates(
                    p, 1000L, 0f, locationListener, Looper.getMainLooper()
                )
            }
        } catch (e: SecurityException) {
            listener.onSensorError("缺少定位权限，无法使用 GNSS")
        } catch (e: Exception) {
            listener.onSensorError("定位初始化失败: ${e.message}")
        }
    }

    fun stop() {
        try {
            sensorManager.unregisterListener(imuListener)
            sensorManager.unregisterListener(magListener)
            locationManager.removeUpdates(locationListener)
        } catch (e: Exception) {
            // 忽略注销异常
        }
        thread?.quitSafely()
        thread = null
    }
}
