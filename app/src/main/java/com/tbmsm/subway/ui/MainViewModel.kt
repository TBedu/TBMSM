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

    fun showMessage(text: String) {
        _message.value = text
    }

    fun consumeMessage() {
        _message.value = null
    }
}
