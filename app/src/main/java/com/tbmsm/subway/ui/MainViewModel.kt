package com.tbmsm.subway.ui

import android.app.Application
import android.content.Context
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import com.tbmsm.subway.core.GeoPoint
import com.tbmsm.subway.model.LiveState
import com.tbmsm.subway.model.SessionSummary
import com.tbmsm.subway.model.TrajSample
import com.tbmsm.subway.sensors.RecordingService
import com.tbmsm.subway.storage.SessionBus
import com.tbmsm.subway.storage.SessionStore
import com.tbmsm.subway.track.TrackBuilder
import com.tbmsm.subway.track.TrackModel
import com.tbmsm.subway.track.TrackRepository
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import java.io.File

/** 会话列表项。 */
data class SessionItem(
    val sessionId: String,
    val summary: SessionSummary?,
    val sizeBytes: Long
)

/** 结果页数据。 */
data class ResultData(
    val sessionId: String,
    val summary: SessionSummary?,
    val raw: List<TrajSample>,
    val smoothed: List<TrajSample>
)

/**
 * 轨道生成预览状态。
 *
 * @param previewPoints 生成的中心线（ENU），用于界面叠加显示
 * @param issues 合理性检查发现的问题
 */
data class TrackBuildState(
    val name: String,
    val sourceSessions: List<String>,
    val rawLengthM: Double,
    val pointCount: Int,
    val stationCount: Int,
    val stationPriors: List<Double>,
    val previewPoints: List<Pair<Double, Double>>,
    val warnings: List<String>,
    val issues: List<String>,
    val json: String
)

class MainViewModel(app: Application) : AndroidViewModel(app) {

    private val filesDir: File get() = getApplication<Application>().filesDir

    val live: StateFlow<LiveState> = SessionBus.live
    val serviceRunning: StateFlow<Boolean> = SessionBus.serviceRunning

    private val _sessions = MutableStateFlow<List<SessionItem>>(emptyList())
    val sessions: StateFlow<List<SessionItem>> = _sessions.asStateFlow()

    private val _tracks = MutableStateFlow<List<TrackModel>>(emptyList())
    val tracks: StateFlow<List<TrackModel>> = _tracks.asStateFlow()

    private val _selectedTrack = MutableStateFlow<String?>(null)
    val selectedTrack: StateFlow<String?> = _selectedTrack.asStateFlow()

    private val _useTrack = MutableStateFlow(true)
    val useTrack: StateFlow<Boolean> = _useTrack.asStateFlow()

    private val _result = MutableStateFlow<ResultData?>(null)
    val result: StateFlow<ResultData?> = _result.asStateFlow()

    private val _message = MutableStateFlow<String?>(null)
    val message: StateFlow<String?> = _message.asStateFlow()

    /** 轨道生成结果预览。 */
    private val _trackBuild = MutableStateFlow<TrackBuildState?>(null)
    val trackBuild: StateFlow<TrackBuildState?> = _trackBuild.asStateFlow()

    private val _building = MutableStateFlow(false)
    val building: StateFlow<Boolean> = _building.asStateFlow()

    init {
        refreshSessions()
        loadTracks()
        viewModelScope.launch {
            SessionBus.finishedSessionId.collect { id ->
                if (id != null) {
                    refreshSessions()
                    loadResult(id)
                    SessionBus.clearFinished()
                }
            }
        }
    }

    fun loadTracks() {
        viewModelScope.launch {
            val list = withContext(Dispatchers.IO) {
                val assets = TrackRepository.loadFromAssets(getApplication<Application>().assets)
                val files = TrackRepository.loadFromFiles(File(filesDir, "tracks"))
                assets + files
            }
            _tracks.value = list
            if (_selectedTrack.value == null) {
                _selectedTrack.value = list.firstOrNull()?.name
            }
        }
    }

    fun selectTrack(name: String?) {
        _selectedTrack.value = name
    }

    fun setUseTrack(value: Boolean) {
        _useTrack.value = value
    }

    fun refreshSessions() {
        viewModelScope.launch {
            val items = withContext(Dispatchers.IO) {
                SessionStore.listSessions(filesDir).map { dir ->
                    SessionItem(
                        sessionId = dir.name,
                        summary = SessionStore.readSummary(filesDir, dir.name),
                        sizeBytes = SessionStore.sessionSizeBytes(filesDir, dir.name)
                    )
                }
            }
            _sessions.value = items
        }
    }

    fun startRecording(context: Context, origin: GeoPoint?) {
        val trackName = if (_useTrack.value) _selectedTrack.value else null
        RecordingService.start(context, trackName, origin, trackName != null)
    }

    fun stopRecording(context: Context) {
        RecordingService.stop(context)
    }

    fun forceStart(context: Context) {
        RecordingService.forceStart(context)
    }

    fun loadResult(sessionId: String) {
        viewModelScope.launch {
            val data = withContext(Dispatchers.IO) {
                val dir = SessionStore.sessionDir(filesDir, sessionId)
                ResultData(
                    sessionId = sessionId,
                    summary = SessionStore.readSummary(filesDir, sessionId),
                    raw = SessionStore.readTrack(File(dir, "track.csv")),
                    smoothed = SessionStore.readTrack(File(dir, "track_smoothed.csv"))
                )
            }
            _result.value = data
        }
    }

    fun clearResult() {
        _result.value = null
    }

    fun deleteSession(sessionId: String) {
        viewModelScope.launch {
            withContext(Dispatchers.IO) { SessionStore.deleteSession(filesDir, sessionId) }
            refreshSessions()
            _message.value = "已删除会话 $sessionId"
        }
    }

    fun sessionDir(sessionId: String): File = SessionStore.sessionDir(filesDir, sessionId)

    // ==================================================================
    //  由传感器数据生成轨道
    // ==================================================================

    /**
     * 由若干次会话的解算轨迹生成轨道中心线。
     *
     * 优先使用平滑后的轨迹（track_smoothed.csv），它比正向解算更接近真实走向。
     * 多次会话会做起点对齐后融合，能显著削弱单次解算的随机漂移。
     */
    fun buildTrackFromSessions(sessionIds: List<String>, name: String) {
        if (sessionIds.isEmpty()) {
            _message.value = "请至少选择一次会话"
            return
        }
        if (name.isBlank()) {
            _message.value = "请填写轨道名称"
            return
        }
        _building.value = true
        viewModelScope.launch {
            val state = withContext(Dispatchers.Default) {
                try {
                    val inputs = ArrayList<Pair<List<TrajSample>, com.tbmsm.subway.core.GeoPoint>>()
                    for (id in sessionIds) {
                        val dir = SessionStore.sessionDir(filesDir, id)
                        val summary = SessionStore.readSummary(filesDir, id) ?: continue
                        if (summary.originLat == 0.0 && summary.originLon == 0.0) continue
                        var samples = SessionStore.readTrack(File(dir, "track_smoothed.csv"))
                        if (samples.size < 10) {
                            samples = SessionStore.readTrack(File(dir, "track.csv"))
                        }
                        if (samples.size < 10) continue
                        inputs.add(samples to SessionStore.originOf(summary))
                    }
                    if (inputs.isEmpty()) {
                        _message.value = "所选会话没有可用的轨迹数据"
                        return@withContext null
                    }
                    val result = TrackBuilder.buildFromSessions(inputs, name) ?: run {
                        _message.value = "轨迹太短或质量不足，无法生成轨道"
                        return@withContext null
                    }
                    val issues = TrackBuilder.sanityCheck(result)
                    TrackBuildState(
                        name = name,
                        sourceSessions = sessionIds,
                        rawLengthM = result.rawLengthM,
                        pointCount = result.track.points.size,
                        stationCount = result.stationS.size,
                        stationPriors = result.stationDistancePriors,
                        previewPoints = TrackBuilder.previewPoints(result),
                        warnings = result.warnings,
                        issues = issues,
                        json = TrackBuilder.toJson(result, name)
                    )
                } catch (e: Exception) {
                    _message.value = "生成失败: ${e.message}"
                    null
                }
            }
            _trackBuild.value = state
            _building.value = false
        }
    }

    /** 保存生成结果到应用私有目录，之后即可在测量页选用。 */
    fun saveBuiltTrack() {
        val state = _trackBuild.value ?: return
        viewModelScope.launch {
            val file = withContext(Dispatchers.IO) {
                TrackRepository.saveGenerated(filesDir, state.name, state.json)
            }
            if (file != null) {
                _message.value = "已保存轨道「${state.name}」"
                _trackBuild.value = null
                loadTracks()
                _selectedTrack.value = state.name
            } else {
                _message.value = "保存失败"
            }
        }
    }

    fun discardBuiltTrack() {
        _trackBuild.value = null
    }

    /** 删除程序生成的轨道。 */
    fun deleteTrack(name: String) {
        viewModelScope.launch {
            val ok = withContext(Dispatchers.IO) {
                TrackRepository.deleteGenerated(filesDir, name)
            }
            if (ok) {
                _message.value = "已删除轨道「$name」"
                if (_selectedTrack.value == name) _selectedTrack.value = null
                loadTracks()
            } else {
                _message.value = "内置轨道不可删除"
            }
        }
    }

    fun isGeneratedTrack(name: String): Boolean =
        TrackRepository.isGenerated(filesDir, name)

    fun showMessage(text: String) {
        _message.value = text
    }

    fun consumeMessage() {
        _message.value = null
    }
}
