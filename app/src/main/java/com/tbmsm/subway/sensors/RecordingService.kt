package com.tbmsm.subway.sensors

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.app.Service
import android.content.Context
import android.content.Intent
import android.os.Build
import android.os.IBinder
import android.os.PowerManager
import android.os.SystemClock
import androidx.core.app.NotificationCompat
import com.tbmsm.subway.MainActivity
import com.tbmsm.subway.R
import com.tbmsm.subway.algo.NavEngine
import com.tbmsm.subway.algo.NavParams
import com.tbmsm.subway.core.GeoPoint
import com.tbmsm.subway.model.EngineState
import com.tbmsm.subway.model.GnssFix
import com.tbmsm.subway.model.ImuSample
import com.tbmsm.subway.model.MagSample
import com.tbmsm.subway.model.SessionSummary
import com.tbmsm.subway.storage.RawDataLogger
import com.tbmsm.subway.storage.SessionBus
import com.tbmsm.subway.storage.SessionStore
import com.tbmsm.subway.track.TrackModel
import com.tbmsm.subway.track.TrackRepository
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancel
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import java.io.File

/**
 * 采集前台服务。
 *
 * 职责：
 *  - 持有 NavEngine 与 SensorCollector，保证界面退到后台后仍在解算；
 *  - 原始数据边采边落盘；
 *  - 结束后执行 RTS 平滑并写出结果文件；
 *  - 通过 SessionBus 向界面推送实时状态。
 *
 * 之所以用前台服务而不是 ViewModel 持有引擎：地铁测量一次可能持续十几分钟，
 * 期间用户很可能切到别的应用，只有前台服务能保证不被系统回收。
 */
class RecordingService : Service(), SensorCollector.Listener {

    companion object {
        const val ACTION_START = "com.tbmsm.subway.action.START"
        const val ACTION_STOP = "com.tbmsm.subway.action.STOP"
        const val ACTION_FORCE_START = "com.tbmsm.subway.action.FORCE_START"

        const val EXTRA_TRACK_NAME = "track_name"
        const val EXTRA_ORIGIN_LAT = "origin_lat"
        const val EXTRA_ORIGIN_LON = "origin_lon"
        const val EXTRA_ORIGIN_ALT = "origin_alt"
        const val EXTRA_USE_TRACK = "use_track"

        private const val CHANNEL_ID = "tbmsm_recording"
        private const val NOTIF_ID = 1001

        fun start(context: Context, trackName: String?, origin: GeoPoint?, useTrack: Boolean) {
            val intent = Intent(context, RecordingService::class.java).apply {
                action = ACTION_START
                putExtra(EXTRA_TRACK_NAME, trackName)
                putExtra(EXTRA_USE_TRACK, useTrack)
                if (origin != null) {
                    putExtra(EXTRA_ORIGIN_LAT, origin.latDeg)
                    putExtra(EXTRA_ORIGIN_LON, origin.lonDeg)
                    putExtra(EXTRA_ORIGIN_ALT, origin.altM)
                }
            }
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                context.startForegroundService(intent)
            } else {
                context.startService(intent)
            }
        }

        fun stop(context: Context) {
            val intent = Intent(context, RecordingService::class.java).apply { action = ACTION_STOP }
            context.startService(intent)
        }

        fun forceStart(context: Context) {
            val intent = Intent(context, RecordingService::class.java).apply {
                action = ACTION_FORCE_START
            }
            context.startService(intent)
        }
    }

    private var engine: NavEngine? = null
    private var collector: SensorCollector? = null
    private var logger: RawDataLogger? = null
    private var wakeLock: PowerManager.WakeLock? = null

    private var sessionId: String = ""
    private var startedAtMs: Long = 0L
    private var trackName: String = ""
    private var trackUsed = false

    private val scope = CoroutineScope(SupervisorJob() + Dispatchers.Default)
    private var publishJob: Job? = null

    override fun onBind(intent: Intent?): IBinder? = null

    override fun onCreate() {
        super.onCreate()
        createChannel()
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        when (intent?.action) {
            ACTION_START -> handleStart(intent)
            ACTION_STOP -> handleStop()
            ACTION_FORCE_START -> engine?.forceStart()
        }
        return START_NOT_STICKY
    }

    private fun handleStart(intent: Intent) {
        if (engine != null) return

        startForeground(NOTIF_ID, buildNotification("正在准备…"))

        sessionId = SessionStore.newSessionId()
        startedAtMs = System.currentTimeMillis()
        trackName = intent.getStringExtra(EXTRA_TRACK_NAME) ?: ""
        trackUsed = intent.getBooleanExtra(EXTRA_USE_TRACK, false)

        val origin = if (intent.hasExtra(EXTRA_ORIGIN_LAT)) {
            GeoPoint(
                intent.getDoubleExtra(EXTRA_ORIGIN_LAT, 0.0),
                intent.getDoubleExtra(EXTRA_ORIGIN_LON, 0.0),
                intent.getDoubleExtra(EXTRA_ORIGIN_ALT, 0.0)
            )
        } else null

        val track: TrackModel? = if (trackUsed && trackName.isNotEmpty()) {
            loadTrack(trackName)
        } else null

        val eng = NavEngine(NavParams(), track, origin)
        engine = eng
        // 必须用 elapsedRealtimeNanos 作为起点：SensorEvent.timestamp 与
        // Location.elapsedRealtimeNanos 都是这个时基，而 System.nanoTime() 不是。
        // 两者混用会让「时长」一开始就等于设备开机时长（几百秒）。
        eng.start(sessionId, SystemClock.elapsedRealtimeNanos())

        val rawDir = SessionStore.rawDir(filesDir, sessionId)
        logger = RawDataLogger(rawDir).also { it.open() }

        val col = SensorCollector(applicationContext, this)
        val err = col.checkAvailability()
        if (err != null) {
            eng.stop()
            SessionBus.publish(eng.live.copy(state = EngineState.ERROR, fatalError = err))
            stopSelf()
            return
        }
        collector = col
        col.start()

        acquireWakeLock()
        SessionBus.setServiceRunning(true)
        startPublishing()
    }

    private fun loadTrack(name: String): TrackModel? {
        val fromAssets = TrackRepository.loadFromAssets(assets)
        val fromFiles = TrackRepository.loadFromFiles(File(filesDir, "tracks"))
        return (fromAssets + fromFiles).firstOrNull { it.name == name }
    }

    private fun handleStop() {
        val eng = engine ?: return
        eng.stop()
        collector?.stop()
        collector = null
        logger?.close()
        logger = null
        releaseWakeLock()
        finalizeSession(eng)
    }

    /** 结束后处理：RTS 平滑 + 写文件 + 通知界面。 */
    private fun finalizeSession(eng: NavEngine) {
        val raw = eng.rawSamples().toList()
        val segments = eng.runSegments().toList()
        val warnings = eng.warningList().toList()
        val dir = SessionStore.sessionDir(filesDir, sessionId)

        scope.launch {
            val extraWarnings = ArrayList<String>()
            val smoothed = withContext(Dispatchers.Default) {
                try {
                    eng.smooth()
                } catch (e: Exception) {
                    extraWarnings.add("RTS 平滑失败: ${e.message}")
                    raw
                }
            }
            val allWarnings = warnings + extraWarnings
            withContext(Dispatchers.IO) {
                try {
                    SessionStore.writeTrack(File(dir, "track.csv"), raw)
                    SessionStore.writeTrack(File(dir, "track_smoothed.csv"), smoothed)
                    SessionStore.writeSegments(File(dir, "segments.csv"), segments)

                    val origin = eng.referenceOrigin()
                    val duration = if (raw.isEmpty()) 0.0 else raw.last().tRelS
                    val maxSpeed = raw.maxOfOrNull { it.speedMps } ?: 0.0
                    val meanSpeed = if (duration > 0) {
                        (raw.lastOrNull()?.let { Math.hypot(it.east, it.north) } ?: 0.0) / duration
                    } else 0.0
                    val maxAccel = raw.maxOfOrNull { it.accelAlong } ?: 0.0
                    val maxDecel = raw.minOfOrNull { it.accelAlong } ?: 0.0

                    val summary = SessionSummary(
                        sessionId = sessionId,
                        startedAtMs = startedAtMs,
                        durationS = duration,
                        totalDistanceM = raw.lastOrNull()?.let { Math.hypot(it.east, it.north) } ?: 0.0,
                        maxSpeedMps = maxSpeed,
                        meanSpeedMps = meanSpeed,
                        maxAccelMps2 = maxAccel,
                        maxDecelMps2 = maxDecel,
                        imuSamples = eng.live.imuCount,
                        gnssFixes = eng.live.gnssCount,
                        gnssRejected = eng.gnssRejectedCount(),
                        stillRatio = eng.stillRatio(),
                        nisRejected = eng.nisRejectedCount(),
                        divergenceCount = eng.divergenceCountValue(),
                        smoothed = smoothed !== raw,
                        trackUsed = trackUsed,
                        trackName = trackName,
                        warnings = allWarnings,
                        segments = segments,
                        originLat = origin?.latDeg ?: 0.0,
                        originLon = origin?.lonDeg ?: 0.0,
                        originAlt = origin?.altM ?: 0.0
                    )
                    SessionStore.writeSummary(File(dir, "summary.json"), summary)
                } catch (e: Exception) {
                    // 写盘失败不阻塞服务退出
                }
            }
            SessionBus.notifyFinished(sessionId)
            SessionBus.setServiceRunning(false)
            SessionBus.publish(
                SessionBus.live.value.copy(state = EngineState.FINISHED, sessionId = sessionId)
            )
            engine = null
            stopForegroundCompat()
            stopSelf()
        }
    }

    private fun startPublishing() {
        publishJob?.cancel()
        publishJob = scope.launch {
            while (true) {
                val eng = engine ?: break
                SessionBus.publish(eng.live)
                updateNotification(eng.live.speedKmh, eng.live.distanceM, eng.live.state)
                kotlinx.coroutines.delay(200)
            }
        }
    }

    // ---------------- SensorCollector.Listener ----------------

    override fun onImu(sample: ImuSample) {
        logger?.logImu(sample)
        engine?.onImu(sample)
    }

    override fun onMag(sample: MagSample) {
        logger?.logMag(sample)
        engine?.onMag(sample)
    }

    override fun onGnss(fix: GnssFix) {
        logger?.logGnss(fix)
        val eng = engine ?: return
        if (!eng.hasGnssOrigin()) eng.setGnssOrigin(fix)
        eng.onGnss(fix)
    }

    override fun onSensorError(message: String) {
        val eng = engine ?: return
        SessionBus.publish(eng.live.copy(warning = message))
    }

    // ---------------- 通知与电源 ----------------

    private fun createChannel() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            val nm = getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager
            if (nm.getNotificationChannel(CHANNEL_ID) == null) {
                val ch = NotificationChannel(
                    CHANNEL_ID,
                    getString(R.string.notif_channel_name),
                    NotificationManager.IMPORTANCE_LOW
                ).apply {
                    description = getString(R.string.notif_channel_desc)
                    setShowBadge(false)
                }
                nm.createNotificationChannel(ch)
            }
        }
    }

    private fun buildNotification(text: String): Notification {
        val openIntent = PendingIntent.getActivity(
            this, 0,
            Intent(this, MainActivity::class.java),
            PendingIntent.FLAG_UPDATE_CURRENT or PendingIntent.FLAG_IMMUTABLE
        )
        val stopIntent = PendingIntent.getService(
            this, 1,
            Intent(this, RecordingService::class.java).apply { action = ACTION_STOP },
            PendingIntent.FLAG_UPDATE_CURRENT or PendingIntent.FLAG_IMMUTABLE
        )
        return NotificationCompat.Builder(this, CHANNEL_ID)
            .setContentTitle(getString(R.string.notif_title))
            .setContentText(text)
            .setSmallIcon(R.drawable.ic_launcher_foreground)
            .setOngoing(true)
            .setOnlyAlertOnce(true)
            .setContentIntent(openIntent)
            .addAction(0, getString(R.string.notif_action_stop), stopIntent)
            .build()
    }

    private fun updateNotification(speedKmh: Double, distanceM: Double, state: EngineState) {
        val text = when (state) {
            EngineState.ALIGNING -> "静止对准中，请勿移动手机"
            EngineState.RUNNING -> String.format("速度 %.1f km/h   里程 %.0f m", speedKmh, distanceM)
            else -> "已结束"
        }
        val nm = getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager
        nm.notify(NOTIF_ID, buildNotification(text))
    }

    private fun stopForegroundCompat() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
            stopForeground(STOP_FOREGROUND_REMOVE)
        } else {
            @Suppress("DEPRECATION")
            stopForeground(true)
        }
    }

    private fun acquireWakeLock() {
        try {
            val pm = getSystemService(Context.POWER_SERVICE) as PowerManager
            wakeLock = pm.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "tbmsm:recording").apply {
                setReferenceCounted(false)
                acquire(60 * 60 * 1000L)
            }
        } catch (e: Exception) {
            // 没有 wakelock 也能运行，只是后台可能被限制
        }
    }

    private fun releaseWakeLock() {
        try {
            wakeLock?.let { if (it.isHeld) it.release() }
        } catch (e: Exception) {
            // 忽略
        }
        wakeLock = null
    }

    override fun onDestroy() {
        publishJob?.cancel()
        collector?.stop()
        logger?.close()
        releaseWakeLock()
        SessionBus.setServiceRunning(false)
        scope.cancel()
        super.onDestroy()
    }
}
