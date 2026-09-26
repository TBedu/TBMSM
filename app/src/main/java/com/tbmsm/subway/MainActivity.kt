package com.tbmsm.subway

import android.Manifest
import android.content.Intent
import android.content.pm.PackageManager
import android.net.Uri
import android.os.Build
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.NavigationBar
import androidx.compose.material3.NavigationBarItem
import androidx.compose.material3.Scaffold
import androidx.compose.material3.SnackbarHost
import androidx.compose.material3.SnackbarHostState
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.core.content.ContextCompat
import androidx.core.content.FileProvider
import androidx.lifecycle.viewmodel.compose.viewModel
import com.tbmsm.subway.core.GeoPoint
import com.tbmsm.subway.model.EngineState
import com.tbmsm.subway.ui.LiveScreen
import com.tbmsm.subway.ui.MainViewModel
import com.tbmsm.subway.ui.ResultScreen
import com.tbmsm.subway.ui.SessionsScreen
import kotlinx.coroutines.launch
import java.io.File

class MainActivity : ComponentActivity() {

    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { result ->
        val fine = result[Manifest.permission.ACCESS_FINE_LOCATION] == true
        if (!fine) {
            pendingStart = false
        } else if (pendingStart) {
            pendingStart = false
            startRecordingInternal()
        }
    }

    private var pendingStart = false
    private var viewModelRef: MainViewModel? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContent {
            MaterialTheme {
                Surface(Modifier.fillMaxSize()) {
                    AppRoot(
                        onRequestStart = { vm ->
                            viewModelRef = vm
                            requestPermissionsAndStart()
                        }
                    )
                }
            }
        }
    }

    private fun requestPermissionsAndStart() {
        val needed = ArrayList<String>()
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION)
            != PackageManager.PERMISSION_GRANTED
        ) {
            needed.add(Manifest.permission.ACCESS_FINE_LOCATION)
            needed.add(Manifest.permission.ACCESS_COARSE_LOCATION)
        }
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU &&
            ContextCompat.checkSelfPermission(this, Manifest.permission.POST_NOTIFICATIONS)
            != PackageManager.PERMISSION_GRANTED
        ) {
            needed.add(Manifest.permission.POST_NOTIFICATIONS)
        }
        if (needed.isEmpty()) {
            startRecordingInternal()
        } else {
            pendingStart = true
            permissionLauncher.launch(needed.toTypedArray())
        }
    }

    private fun startRecordingInternal() {
        val vm = viewModelRef ?: return
        // 没有 GNSS 时以 0,0 为参考原点，此时位置只有相对意义；
        // 一旦收到第一帧定位，引擎会切换到真实原点。
        vm.startRecording(this, GeoPoint(0.0, 0.0, 0.0))
    }
}

@Composable
private fun AppRoot(onRequestStart: (MainViewModel) -> Unit) {
    val vm: MainViewModel = viewModel()
    val context = LocalContext.current
    val scope = rememberCoroutineScope()
    val snackbar = remember { SnackbarHostState() }

    val live by vm.live.collectAsState()
    val running by vm.serviceRunning.collectAsState()
    val sessions by vm.sessions.collectAsState()
    val tracks by vm.tracks.collectAsState()
    val selectedTrack by vm.selectedTrack.collectAsState()
    val useTrack by vm.useTrack.collectAsState()
    val result by vm.result.collectAsState()
    val message by vm.message.collectAsState()
    val trackBuild by vm.trackBuild.collectAsState()
    val building by vm.building.collectAsState()

    var tab by remember { mutableIntStateOf(0) }

    LaunchedEffect(message) {
        message?.let {
            snackbar.showSnackbar(it)
            vm.consumeMessage()
        }
    }

    // 采集结束后自动跳到结果页
    LaunchedEffect(live.state) {
        if (live.state == EngineState.FINISHED && live.sessionId.isNotEmpty()) {
            vm.loadResult(live.sessionId)
            tab = 1
        }
    }

    Scaffold(
        snackbarHost = { SnackbarHost(snackbar) },
        bottomBar = {
            if (result == null) {
                NavigationBar {
                    NavigationBarItem(
                        selected = tab == 0,
                        onClick = { tab = 0 },
                        icon = {},
                        label = { Text("测量") }
                    )
                    NavigationBarItem(
                        selected = tab == 1,
                        onClick = { tab = 1; vm.refreshSessions() },
                        icon = {},
                        label = { Text("记录") }
                    )
                }
            }
        }
    ) { padding ->
        Column(Modifier.padding(padding)) {
            val res = result
            if (res != null) {
                val trackPoints = remember(res.sessionId, tracks, selectedTrack) {
                    val t = tracks.firstOrNull { it.name == selectedTrack }
                    t?.points?.map { it.x to it.y } ?: emptyList()
                }
                ResultScreen(
                    data = res,
                    trackPoints = trackPoints,
                    onBack = { vm.clearResult() },
                    onExport = {
                        scope.launch {
                            val ok = exportSession(context, vm, res.sessionId)
                            vm.showMessage(if (ok) "已导出到所选目录" else "导出失败或已取消")
                        }
                    }
                )
            } else if (tab == 0) {
                LiveScreen(
                    live = live,
                    serviceRunning = running,
                    trackNames = tracks.map { it.name },
                    selectedTrack = selectedTrack,
                    useTrack = useTrack,
                    onSelectTrack = vm::selectTrack,
                    onToggleUseTrack = vm::setUseTrack,
                    onStart = { onRequestStart(vm) },
                    onStop = { vm.stopRecording(context) },
                    onForceStart = { vm.forceStart(context) }
                )
            } else {
                SessionsScreen(
                    sessions = sessions,
                    tracks = tracks,
                    trackBuild = trackBuild,
                    building = building,
                    onOpen = { vm.loadResult(it) },
                    onDelete = { vm.deleteSession(it) },
                    onRefresh = { vm.refreshSessions() },
                    onBuildTrack = { ids, name -> vm.buildTrackFromSessions(ids, name) },
                    onSaveBuiltTrack = { vm.saveBuiltTrack() },
                    onDiscardBuiltTrack = { vm.discardBuiltTrack() },
                    onDeleteTrack = { vm.deleteTrack(it) },
                    isGeneratedTrack = { vm.isGeneratedTrack(it) }
                )
            }
        }
    }
}

/**
 * 导出会话：把整个会话目录打包成 zip 后通过系统分享/保存。
 * 这里用 ACTION_CREATE_DOCUMENT 让用户选择保存位置，避免申请存储权限。
 */
private fun exportSession(
    context: android.content.Context,
    vm: MainViewModel,
    sessionId: String
): Boolean {
    return try {
        val dir = vm.sessionDir(sessionId)
        if (!dir.exists()) return false
        val zipFile = File(context.cacheDir, "$sessionId.zip")
        zipDirectory(dir, zipFile)
        val uri: Uri = FileProvider.getUriForFile(
            context, "${context.packageName}.fileprovider", zipFile
        )
        val intent = Intent(Intent.ACTION_SEND).apply {
            type = "application/zip"
            putExtra(Intent.EXTRA_STREAM, uri)
            putExtra(Intent.EXTRA_SUBJECT, "地铁运行参数测量 $sessionId")
            addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
        }
        context.startActivity(Intent.createChooser(intent, "导出会话数据"))
        true
    } catch (e: Exception) {
        false
    }
}

private fun zipDirectory(sourceDir: File, zipFile: File) {
    java.util.zip.ZipOutputStream(zipFile.outputStream().buffered()).use { zos ->
        sourceDir.walkTopDown().filter { it.isFile }.forEach { file ->
            val entryName = file.relativeTo(sourceDir).path.replace('\\', '/')
            zos.putNextEntry(java.util.zip.ZipEntry(entryName))
            file.inputStream().use { it.copyTo(zos) }
            zos.closeEntry()
        }
    }
}
