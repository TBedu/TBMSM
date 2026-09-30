package com.tbmsm.subway.ui

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.Checkbox
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import com.tbmsm.subway.model.RunSegment
import com.tbmsm.subway.model.SessionSummary
import com.tbmsm.subway.model.TrajSample
import com.tbmsm.subway.track.TrackModel
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/** 会话列表页。 */
@Composable
fun SessionsScreen(
    sessions: List<SessionItem>,
    tracks: List<TrackModel>,
    trackBuild: TrackBuildState?,
    building: Boolean,
    onOpen: (String) -> Unit,
    onDelete: (String) -> Unit,
    onRefresh: () -> Unit,
    onBuildTrack: (List<String>, String) -> Unit,
    onSaveBuiltTrack: () -> Unit,
    onDiscardBuiltTrack: () -> Unit,
    onDeleteTrack: (String) -> Unit,
    isGeneratedTrack: (String) -> Boolean,
    onImport: () -> Unit,
    modifier: Modifier = Modifier
) {
    var showBuilder by remember { mutableStateOf(false) }
    var selected by remember { mutableStateOf(setOf<String>()) }
    var trackName by remember { mutableStateOf("") }

    Column(modifier.fillMaxSize().padding(16.dp)) {
        Row(verticalAlignment = Alignment.CenterVertically) {
            Text("历史会话", style = MaterialTheme.typography.titleMedium, modifier = Modifier.weight(1f))
            TextButton(onClick = onImport) { Text("导入") }
            TextButton(onClick = onRefresh) { Text("刷新") }
            TextButton(onClick = { showBuilder = !showBuilder }) {
                Text(if (showBuilder) "收起" else "生成轨道")
            }
        }

        if (showBuilder) {
            TrackBuilderCard(
                sessions = sessions,
                selected = selected,
                onToggle = { id ->
                    selected = if (selected.contains(id)) selected - id else selected + id
                },
                trackName = trackName,
                onNameChange = { trackName = it },
                building = building,
                onBuild = { onBuildTrack(selected.toList(), trackName.trim()) }
            )
        }

        if (trackBuild != null) {
            TrackPreviewCard(
                state = trackBuild,
                onSave = onSaveBuiltTrack,
                onDiscard = onDiscardBuiltTrack
            )
        }

        if (tracks.isNotEmpty()) {
            TrackListCard(tracks, onDeleteTrack, isGeneratedTrack)
        }

        if (sessions.isEmpty()) {
            Text(
                "还没有测量记录。",
                style = MaterialTheme.typography.bodyMedium,
                color = Color(0xFF666666),
                modifier = Modifier.padding(top = 24.dp)
            )
            return
        }
        LazyColumn(verticalArrangement = Arrangement.spacedBy(8.dp)) {
            items(sessions, key = { it.sessionId }) { item ->
                SessionCard(
                    item = item,
                    selectable = showBuilder,
                    checked = selected.contains(item.sessionId),
                    onToggleCheck = {
                        selected = if (selected.contains(item.sessionId)) {
                            selected - item.sessionId
                        } else {
                            selected + item.sessionId
                        }
                    },
                    onOpen = onOpen,
                    onDelete = onDelete
                )
            }
        }
    }
}

/** 轨道生成面板：选择会话 + 命名 + 生成。 */
@Composable
private fun TrackBuilderCard(
    sessions: List<SessionItem>,
    selected: Set<String>,
    onToggle: (String) -> Unit,
    trackName: String,
    onNameChange: (String) -> Unit,
    building: Boolean,
    onBuild: () -> Unit
) {
    Card(
        colors = CardDefaults.cardColors(containerColor = Color(0xFFE8F0FE)),
        modifier = Modifier.fillMaxWidth().padding(bottom = 8.dp)
    ) {
        Column(Modifier.padding(14.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text("由传感器数据生成轨道", style = MaterialTheme.typography.titleSmall)
            Text(
                "勾选同一线路的多次会话（建议 2 次以上，起点相同），" +
                    "程序会做起点对齐后融合，削弱单次解算的随机漂移。",
                style = MaterialTheme.typography.labelSmall,
                color = Color(0xFF555555)
            )
            OutlinedTextField(
                value = trackName,
                onValueChange = onNameChange,
                label = { Text("轨道名称") },
                singleLine = true,
                modifier = Modifier.fillMaxWidth()
            )
            Row(verticalAlignment = Alignment.CenterVertically) {
                Text(
                    "已选 ${selected.size} / ${sessions.size} 次会话",
                    style = MaterialTheme.typography.labelSmall,
                    color = Color(0xFF555555),
                    modifier = Modifier.weight(1f)
                )
                TextButton(
                    onClick = {
                        if (selected.size == sessions.size) {
                            sessions.forEach { if (selected.contains(it.sessionId)) onToggle(it.sessionId) }
                        } else {
                            sessions.forEach { if (!selected.contains(it.sessionId)) onToggle(it.sessionId) }
                        }
                    }
                ) { Text(if (selected.size == sessions.size) "全不选" else "全选") }
                OutlinedButton(
                    onClick = onBuild,
                    enabled = !building && selected.isNotEmpty() && trackName.isNotBlank()
                ) { Text(if (building) "生成中…" else "生成") }
            }
        }
    }
}

/** 生成结果预览：中心线缩略图 + 统计 + 问题清单 + 保存/丢弃。 */
@Composable
private fun TrackPreviewCard(
    state: TrackBuildState,
    onSave: () -> Unit,
    onDiscard: () -> Unit
) {
    Card(
        colors = CardDefaults.cardColors(containerColor = Color(0xFFF1F8E9)),
        modifier = Modifier.fillMaxWidth().padding(bottom = 8.dp)
    ) {
        Column(Modifier.padding(14.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text("生成预览：${state.name}", style = MaterialTheme.typography.titleSmall)
            TrackMap(
                samples = emptyList(),
                trackPoints = state.previewPoints,
                modifier = Modifier.fillMaxWidth().height(200.dp)
            )
            Text(
                "原始里程 ${fmt(state.rawLengthM, 0)} m   中心线 ${state.pointCount} 点   " +
                    "识别停站 ${state.stationCount} 处",
                style = MaterialTheme.typography.bodySmall
            )
            if (state.stationPriors.isNotEmpty()) {
                Text(
                    "站间距先验(m): " + state.stationPriors.joinToString(" ") { fmt(it, 0) },
                    style = MaterialTheme.typography.labelSmall,
                    color = Color(0xFF555555)
                )
            }
            if (state.warnings.isNotEmpty()) {
                Text("生成提示", style = MaterialTheme.typography.labelSmall, color = Color(0xFF8D6E00))
                state.warnings.forEach {
                    Text("· $it", style = MaterialTheme.typography.labelSmall, color = Color(0xFF8D6E00))
                }
            }
            if (state.issues.isNotEmpty()) {
                Text("合理性检查", style = MaterialTheme.typography.labelSmall, color = Color(0xFFC62828))
                state.issues.forEach {
                    Text("· $it", style = MaterialTheme.typography.labelSmall, color = Color(0xFFC62828))
                }
            }
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                OutlinedButton(onClick = onSave) { Text("保存并选用") }
                TextButton(onClick = onDiscard) { Text("丢弃") }
            }
        }
    }
}

/** 已有轨道列表，程序生成的可以删除。 */
@Composable
private fun TrackListCard(
    tracks: List<TrackModel>,
    onDeleteTrack: (String) -> Unit,
    isGeneratedTrack: (String) -> Boolean
) {
    var expanded by remember { mutableStateOf(false) }
    Card(modifier = Modifier.fillMaxWidth().padding(bottom = 8.dp)) {
        Column(Modifier.padding(14.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Text(
                    "可用轨道 ${tracks.size} 条",
                    style = MaterialTheme.typography.titleSmall,
                    modifier = Modifier.weight(1f)
                )
                TextButton(onClick = { expanded = !expanded }) {
                    Text(if (expanded) "收起" else "展开")
                }
            }
            if (expanded) {
                tracks.forEach { t ->
                    val generated = isGeneratedTrack(t.name)
                    Row(verticalAlignment = Alignment.CenterVertically) {
                        Text(
                            "${t.name}（${t.points.size} 点）" + if (generated) "  程序生成" else "  内置",
                            style = MaterialTheme.typography.bodySmall,
                            modifier = Modifier.weight(1f)
                        )
                        if (generated) {
                            TextButton(onClick = { onDeleteTrack(t.name) }) {
                                Text("删除", color = Color(0xFFC62828))
                            }
                        }
                    }
                }
            }
        }
    }
}

@Composable
private fun SessionCard(
    item: SessionItem,
    selectable: Boolean,
    checked: Boolean,
    onToggleCheck: () -> Unit,
    onOpen: (String) -> Unit,
    onDelete: (String) -> Unit
) {
    var confirmDelete by remember { mutableStateOf(false) }
    val s = item.summary
    Card(modifier = Modifier.fillMaxWidth()) {
        Column(Modifier.padding(14.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                if (selectable) {
                    Checkbox(checked = checked, onCheckedChange = { onToggleCheck() })
                }
                Text(
                    item.sessionId,
                    style = MaterialTheme.typography.titleSmall,
                    fontWeight = FontWeight.Bold,
                    modifier = Modifier.weight(1f)
                )
                Text(
                    formatSize(item.sizeBytes),
                    style = MaterialTheme.typography.labelSmall,
                    color = Color(0xFF888888)
                )
            }
            if (s != null) {
                Text(
                    "时长 ${fmt(s.durationS, 0)} s   里程 ${fmt(s.totalDistanceM, 0)} m   " +
                        "最高 ${fmt(s.maxSpeedMps * 3.6, 1)} km/h",
                    style = MaterialTheme.typography.bodySmall
                )
                Text(
                    "站间段 ${s.segments.size} 段   静止占比 ${fmt(s.stillRatio * 100, 0)}%   " +
                        "GNSS ${s.gnssFixes} 帧" +
                        if (s.gnssRejected > 0) "（丢弃 ${s.gnssRejected}）" else "" +
                        if (s.trackUsed) "   轨道:${s.trackName}" else "",
                    style = MaterialTheme.typography.bodySmall,
                    color = Color(0xFF666666)
                )
                if (s.warnings.isNotEmpty()) {
                    Text(
                        "告警 ${s.warnings.size} 条",
                        style = MaterialTheme.typography.labelSmall,
                        color = Color(0xFF8D6E00)
                    )
                }
            } else {
                Text("缺少 summary.json", style = MaterialTheme.typography.bodySmall, color = Color(0xFFC62828))
            }
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                OutlinedButton(onClick = { onOpen(item.sessionId) }) { Text("查看结果") }
                if (confirmDelete) {
                    OutlinedButton(
                        onClick = { onDelete(item.sessionId); confirmDelete = false },
                        colors = androidx.compose.material3.ButtonDefaults.outlinedButtonColors(
                            contentColor = Color(0xFFC62828)
                        )
                    ) { Text("确认删除") }
                } else {
                    TextButton(onClick = { confirmDelete = true }) { Text("删除") }
                }
            }
        }
    }
}

/** 结果页。 */
@Composable
fun ResultScreen(
    data: ResultData,
    trackPoints: List<Pair<Double, Double>>,
    onBack: () -> Unit,
    onExport: () -> Unit,
    modifier: Modifier = Modifier
) {
    var showSmoothed by remember { mutableStateOf(true) }
    val samples: List<TrajSample> =
        if (showSmoothed && data.smoothed.isNotEmpty()) data.smoothed else data.raw

    Column(
        modifier
            .fillMaxSize()
            .verticalScroll(rememberScrollState())
            .padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp)
    ) {
        Row(verticalAlignment = Alignment.CenterVertically) {
            TextButton(onClick = onBack) { Text("返回") }
            Text(
                data.sessionId,
                style = MaterialTheme.typography.titleMedium,
                modifier = Modifier.weight(1f)
            )
            TextButton(onClick = onExport) { Text("导出") }
        }

        data.summary?.let { SummaryCard(it) }

        Card(modifier = Modifier.fillMaxWidth()) {
            Column(Modifier.padding(12.dp)) {
                Row(verticalAlignment = Alignment.CenterVertically) {
                    Text("轨迹俯视图", style = MaterialTheme.typography.titleSmall, modifier = Modifier.weight(1f))
                    TextButton(onClick = { showSmoothed = !showSmoothed }) {
                        Text(if (showSmoothed) "显示平滑" else "显示原始")
                    }
                }
                TrackMap(
                    samples = samples,
                    trackPoints = trackPoints,
                    modifier = Modifier.fillMaxWidth().height(260.dp)
                )
                Text(
                    "绿点=起点  红点=终点  灰线=轨道中心线",
                    style = MaterialTheme.typography.labelSmall,
                    color = Color(0xFF888888)
                )
            }
        }

        Card(modifier = Modifier.fillMaxWidth()) {
            Column(Modifier.padding(12.dp)) {
                Text("速度曲线", style = MaterialTheme.typography.titleSmall)
                LineChart(
                    series = listOf(
                        Triple("速度", Color(0xFF0B4F8A), samples.map { it.tRelS to it.speedMps * 3.6 })
                    ),
                    modifier = Modifier.fillMaxWidth().height(180.dp),
                    yLabel = "km/h"
                )
            }
        }

        Card(modifier = Modifier.fillMaxWidth()) {
            Column(Modifier.padding(12.dp)) {
                Text("纵向加速度曲线", style = MaterialTheme.typography.titleSmall)
                LineChart(
                    series = listOf(
                        Triple("纵向", Color(0xFFC62828), samples.map { it.tRelS to it.accelAlong }),
                        Triple("垂向", Color(0xFF2E7D32), samples.map { it.tRelS to it.accelVertical })
                    ),
                    modifier = Modifier.fillMaxWidth().height(180.dp),
                    yLabel = "m/s²",
                    zeroLine = true
                )
            }
        }

        Card(modifier = Modifier.fillMaxWidth()) {
            Column(Modifier.padding(12.dp)) {
                Text("航向与俯仰", style = MaterialTheme.typography.titleSmall)
                LineChart(
                    series = listOf(
                        Triple("航向", Color(0xFF6A1B9A), samples.map { it.tRelS to it.headingDeg }),
                        Triple("俯仰", Color(0xFFEF6C00), samples.map { it.tRelS to it.pitchDeg })
                    ),
                    modifier = Modifier.fillMaxWidth().height(180.dp),
                    yLabel = "°"
                )
            }
        }

        data.summary?.let { s ->
            if (s.segments.isNotEmpty()) {
                Card(modifier = Modifier.fillMaxWidth()) {
                    Column(Modifier.padding(12.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
                        Text("站间运行段", style = MaterialTheme.typography.titleSmall)
                        SegmentHeader()
                        s.segments.forEach { seg -> SegmentRow(seg) }
                    }
                }
            }
            if (s.warnings.isNotEmpty()) {
                Card(
                    colors = CardDefaults.cardColors(containerColor = Color(0xFFFFF8E1)),
                    modifier = Modifier.fillMaxWidth()
                ) {
                    Column(Modifier.padding(12.dp), verticalArrangement = Arrangement.spacedBy(4.dp)) {
                        Text("告警与提示", style = MaterialTheme.typography.titleSmall)
                        s.warnings.forEach {
                            Text("· $it", style = MaterialTheme.typography.bodySmall)
                        }
                    }
                }
            }
        }

        Spacer(Modifier.height(24.dp))
    }
}

@Composable
private fun SummaryCard(s: SessionSummary) {
    Card(modifier = Modifier.fillMaxWidth()) {
        Column(Modifier.padding(14.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
            Text("测量总结", style = MaterialTheme.typography.titleSmall)
            val date = SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US).format(Date(s.startedAtMs))
            Text("开始时间 $date", style = MaterialTheme.typography.bodySmall)
            Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceEvenly) {
                Metric("总里程", fmt(s.totalDistanceM, 0), "m")
                Metric("最高速度", fmt(s.maxSpeedMps * 3.6, 1), "km/h")
                Metric("平均速度", fmt(s.meanSpeedMps * 3.6, 1), "km/h")
            }
            Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceEvenly) {
                Metric("最大牵引", fmt(s.maxAccelMps2, 3), "m/s²")
                Metric("最大制动", fmt(s.maxDecelMps2, 3), "m/s²")
                Metric("静止占比", fmt(s.stillRatio * 100, 0), "%")
            }
            Text(
                "IMU ${s.imuSamples} 帧   GNSS ${s.gnssFixes} 帧（丢弃 ${s.gnssRejected}）   " +
                    "NIS 拒绝 ${s.nisRejected}   发散 ${s.divergenceCount}   " +
                    if (s.smoothed) "已平滑" else "未平滑",
                style = MaterialTheme.typography.labelSmall,
                color = Color(0xFF666666)
            )
        }
    }
}

@Composable
private fun SegmentHeader() {
    Row(Modifier.fillMaxWidth()) {
        Text("段", style = MaterialTheme.typography.labelSmall, modifier = Modifier.weight(0.5f))
        Text("时分(s)", style = MaterialTheme.typography.labelSmall, modifier = Modifier.weight(1f))
        Text("里程(m)", style = MaterialTheme.typography.labelSmall, modifier = Modifier.weight(1f))
        Text("最高(km/h)", style = MaterialTheme.typography.labelSmall, modifier = Modifier.weight(1.2f))
        Text("先验误差(m)", style = MaterialTheme.typography.labelSmall, modifier = Modifier.weight(1.3f))
    }
}

@Composable
private fun SegmentRow(seg: RunSegment) {
    Row(Modifier.fillMaxWidth()) {
        Text("${seg.index + 1}", style = MaterialTheme.typography.bodySmall, modifier = Modifier.weight(0.5f))
        Text(fmt(seg.runTimeS, 1), style = MaterialTheme.typography.bodySmall, modifier = Modifier.weight(1f))
        Text(fmt(seg.distanceM, 0), style = MaterialTheme.typography.bodySmall, modifier = Modifier.weight(1f))
        Text(fmt(seg.maxSpeedMps * 3.6, 1), style = MaterialTheme.typography.bodySmall, modifier = Modifier.weight(1.2f))
        Text(
            if (seg.stationDistancePriorM > 0) fmt(seg.stationDistanceErrorM, 1) else "--",
            style = MaterialTheme.typography.bodySmall,
            modifier = Modifier.weight(1.3f)
        )
    }
}

private fun formatSize(bytes: Long): String = when {
    bytes > 1024 * 1024 -> String.format("%.1f MB", bytes / 1024.0 / 1024.0)
    bytes > 1024 -> String.format("%.0f KB", bytes / 1024.0)
    else -> "$bytes B"
}
