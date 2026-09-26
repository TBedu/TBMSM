package com.tbmsm.subway.storage

import com.tbmsm.subway.model.LiveState
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

/**
 * 采集服务与界面之间的单向数据通道。
 *
 * 用进程内单例而不是绑定 Service，是为了让界面在旋转/重建时无需重新绑定，
 * 同时避免把 NavEngine 暴露给 UI 层。
 */
object SessionBus {

    private val _live = MutableStateFlow(LiveState())
    val live: StateFlow<LiveState> = _live.asStateFlow()

    private val _finishedSessionId = MutableStateFlow<String?>(null)
    val finishedSessionId: StateFlow<String?> = _finishedSessionId.asStateFlow()

    /** 采集服务是否正在运行。 */
    private val _serviceRunning = MutableStateFlow(false)
    val serviceRunning: StateFlow<Boolean> = _serviceRunning.asStateFlow()

    fun publish(state: LiveState) {
        _live.value = state
    }

    fun setServiceRunning(running: Boolean) {
        _serviceRunning.value = running
    }

    fun notifyFinished(sessionId: String) {
        _finishedSessionId.value = sessionId
    }

    fun clearFinished() {
        _finishedSessionId.value = null
    }

    fun reset() {
        _live.value = LiveState()
    }
}
