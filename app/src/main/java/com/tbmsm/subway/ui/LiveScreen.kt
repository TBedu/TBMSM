package com.tbmsm.subway.ui

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.LinearProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Switch
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import com.tbmsm.subway.model.EngineState
import com.tbmsm.subway.model.LiveState

/**
 * 实时采集页。
 */
@Composable
fun LiveScreen(
    live: LiveState,
    serviceRunning: Boolean,
    trackNames: List<String>,
    selectedTrack: String?,
    useTrack: Boolean,
    onSelectTrack: (String?) -> Unit,
    onToggleUseTrack: (Boolean) -> Unit,
    onStart: () -> Unit,
    onStop: () -> Unit,
    onForceStart: () -> Unit,
    modifier: Modifier = Modifier
) {
    Column(
        modifier
            .fillMaxSize()
            .verticalScroll(rememberScrollState())
            .padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp)
    ) {
        StatusCard(live, serviceRunning)

        if (live.state == EngineState.ALIGNING) {
            AlignCard(live, onForceStart)
        }

        if (live.state == EngineState.RUNNING || live.state == EngineState.FINISHED) {
            SpeedCard(live)
            MotionCard(live)
            QualityCard(live)
        }

        if (live.state == EngineState.IDLE || live.state == EngineState.FINISHED) {
            SetupCard(
                trackNames = trackNames,
                selectedTrack = selectedTrack,
                useTrack = useTrack,
                onSelectTrack = onSelectTrack,
                onToggleUseTrack = onToggleUseTrack
            )
        }

        if (live.warning.isNotEmpty()) {
            Card(
                colors = CardDefaults.cardColors(containerColor = Color(0xFFFFF3E0)),
                modifier = Modifier.fillMaxWidth()
            ) {
                Text(
                    live.warning,
                    modifier = Modifier.padding(12.dp),
                    style = MaterialTheme.typography.bodySmall,
                    color = Color(0xFF8D6E00)
                )
            }
        }

        if (live.fatalError.isNotEmpty()) {
            Card(
                colors = CardDefaults.cardColors(containerColor = Color(0xFFFFEBEE)),
                modifier = Modifier.fillMaxWidth()
            ) {
                Text(
                    live.fatalError,
                    modifier = Modifier.padding(12.dp),
                    color = Color(0xFFB71C1C)
                )
            }
        }

        Spacer(Modifier.height(4.dp))

        when {
            !serviceRunning -> Button(
                onClick = onStart,
                modifier = Modifier.fillMaxWidth().height(56.dp),
                enabled = live.fatalError.isEmpty()
            ) { Text("开始测量", style = MaterialTheme.typography.titleMedium) }

            else -> Button(
                onClick = onStop,
                modifier = Modifier.fillMaxWidth().height(56.dp),
                colors = ButtonDefaults.buttonColors(containerColor = Color(0xFFC62828))
            ) { Text("停止并保存", style = MaterialTheme.typography.titleMedium) }
        }

        Text(
            "使用提示：手机平放、屏幕朝上、顶部朝列车前进方向，放在地板或座椅上并保持不动。" +
                "开始后请勿触碰手机，直到本次测量结束。",
            style = MaterialTheme.typography.bodySmall,
            color = Color(0xFF666666)
        )
    }
}

@Composable
private fun StatusCard(live: LiveState, serviceRunning: Boolean) {
    val (text, color) = when (live.state) {
        EngineState.IDLE -> "未开始" to Color(0xFF607D8B)
        EngineState.ALIGNING -> "静止对准中" to Color(0xFFF9A825)
        EngineState.RUNNING -> "测量中" to Color(0xFF2E7D32)
        EngineState.FINISHED -> "已结束" to Color(0xFF1565C0)
        EngineState.ERROR -> "错误" to Color(0xFFC62828)
    }
    Card(modifier = Modifier.fillMaxWidth()) {
        Row(
            Modifier.fillMaxWidth().padding(16.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            Box(
                Modifier
                    .width(12.dp)
                    .height(12.dp)
                    .background(color, RoundedCornerShape(6.dp))
            )
            Spacer(Modifier.width(10.dp))
            Column(Modifier.weight(1f)) {
                Text(text, style = MaterialTheme.typography.titleMedium, fontWeight = FontWeight.Bold)
                Text(
                    "会话 ${live.sessionId.ifEmpty { "--" }}   时长 ${fmt(live.elapsedS, 1)} s",
                    style = MaterialTheme.typography.bodySmall
                )
            }
            if (serviceRunning) {
                Text(
                    "IMU ${fmt(live.imuRateHz, 0)} Hz",
                    style = MaterialTheme.typography.bodySmall,
                    color = Color(0xFF666666)
                )
            }
        }
    }
}

@Composable
private fun AlignCard(live: LiveState, onForceStart: () -> Unit) {
    Card(modifier = Modifier.fillMaxWidth()) {
        Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text("正在做静止对准", style = MaterialTheme.typography.titleSmall)
            LinearProgressIndicator(
                progress = { live.alignProgress },
                modifier = Modifier.fillMaxWidth()
            )
            Text(
                "请保持手机完全静止。对准用于估计安装角与陀螺零偏，" +
                    "若期间车辆移动会自动重新开始。",
                style = MaterialTheme.typography.bodySmall
            )
            OutlinedButton(onClick = onForceStart, modifier = Modifier.fillMaxWidth()) {
                Text("跳过对准，直接开始（精度下降）")
            }
        }
    }
}

@Composable
private fun SpeedCard(live: LiveState) {
    Card(modifier = Modifier.fillMaxWidth()) {
        Column(Modifier.padding(16.dp)) {
            SpeedGauge(
                speedKmh = live.speedKmh,
                maxKmh = 100.0,
                modifier = Modifier.fillMaxWidth().height(150.dp)
            )
            Row(
                Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceEvenly
            ) {
                Metric("里程", fmt(live.distanceM, 0), "m")
                Metric("航向", fmt(live.headingDeg, 1), "°")
                Metric("俯仰", fmt(live.pitchDeg, 2), "°")
            }
        }
    }
}

@Composable
private fun MotionCard(live: LiveState) {
    Card(modifier = Modifier.fillMaxWidth()) {
        Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text("运动参数", style = MaterialTheme.typography.titleSmall)
            Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceEvenly) {
                Metric("纵向加速度", fmt(live.accelAlong, 3), "m/s²")
                Metric("垂向加速度", fmt(live.accelVertical, 3), "m/s²")
                Metric("横向加速度", fmt(live.accelLateral, 3), "m/s²")
            }
            Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceEvenly) {
                Metric("站间段数", live.segmentCount.toString(), "段")
                Metric("上段运行时分", fmt(live.lastSegmentRunTimeS, 1), "s")
                Metric("上段里程", fmt(live.lastSegmentDistanceM, 0), "m")
            }
        }
    }
}

@Composable
private fun QualityCard(live: LiveState) {
    Card(modifier = Modifier.fillMaxWidth()) {
        Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
            Text("解算质量", style = MaterialTheme.typography.titleSmall)
            QualityRow("零速检测", if (live.still) "静止" else "运动", live.still)
            QualityRow("ZUPT 统计量", fmt(live.zuptStatistic, 1), live.zuptStatistic in 0.0..25.0)
            QualityRow("位置不确定度", "${fmt(live.posSigmaM, 1)} m", live.posSigmaM < 50)
            QualityRow("航向不确定度", "${fmt(live.headingSigmaDeg, 2)} °", live.headingSigmaDeg < 5)
            QualityRow("陀螺零偏", "${fmt(live.gyroBiasDegPerS, 4)} °/s", live.gyroBiasDegPerS < 0.5)
            QualityRow(
                "GNSS",
                if (live.gnssActive) {
                    "${live.gnssCount} 帧 / ±${fmt(live.gnssAccuracyM, 0)} m"
                } else if (live.gnssRejected > 0) {
                    "已停用（丢弃 ${live.gnssRejected} 帧）"
                } else {
                    "无信号"
                },
                live.gnssActive
            )
            QualityRow("磁力计", if (live.magOk) "可用" else "受干扰/未启用", live.magOk)
            QualityRow("轨道约束", if (live.trackActive) "生效" else "未生效", live.trackActive)
        }
    }
}

@Composable
private fun QualityRow(label: String, value: String, ok: Boolean) {
    Row(Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically) {
        Text(label, style = MaterialTheme.typography.bodySmall, modifier = Modifier.weight(1f))
        Text(
            value,
            style = MaterialTheme.typography.bodySmall,
            color = if (ok) Color(0xFF2E7D32) else Color(0xFFC62828),
            fontWeight = FontWeight.Medium
        )
    }
}

@Composable
private fun SetupCard(
    trackNames: List<String>,
    selectedTrack: String?,
    useTrack: Boolean,
    onSelectTrack: (String?) -> Unit,
    onToggleUseTrack: (Boolean) -> Unit
) {
    Card(modifier = Modifier.fillMaxWidth()) {
        Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text("轨道数据", style = MaterialTheme.typography.titleSmall)
            Row(verticalAlignment = Alignment.CenterVertically) {
                Column(Modifier.weight(1f)) {
                    Text("启用轨道约束", style = MaterialTheme.typography.bodyMedium)
                    Text(
                        if (useTrack) "沿轨道推算，地下段航向稳定" else "纯惯性推算，不依赖任何线路数据",
                        style = MaterialTheme.typography.labelSmall,
                        color = Color(0xFF666666)
                    )
                }
                Switch(checked = useTrack, onCheckedChange = onToggleUseTrack)
            }

            if (!useTrack) {
                Text(
                    "已关闭轨道约束：完全由 IMU + 零速修正推算速度与里程，" +
                        "不需要线路数据。地下段航向会缓慢漂移，里程精度低于启用轨道时。",
                    style = MaterialTheme.typography.bodySmall,
                    color = Color(0xFF8D6E00)
                )
            } else if (trackNames.isEmpty()) {
                Text(
                    "未找到轨道数据。可在 assets/tracks/ 或应用私有目录 tracks/ 下放置 JSON 文件，" +
                        "也可在「记录」页由历史会话生成。若暂时没有，请关闭上方开关。",
                    style = MaterialTheme.typography.bodySmall,
                    color = Color(0xFF8D6E00)
                )
            } else {
                Text("选择线路", style = MaterialTheme.typography.bodySmall)
                Column(verticalArrangement = Arrangement.spacedBy(4.dp)) {
                    trackNames.forEach { name ->
                        val selected = name == selectedTrack
                        OutlinedButton(
                            onClick = { onSelectTrack(name) },
                            modifier = Modifier.fillMaxWidth(),
                            colors = if (selected) {
                                ButtonDefaults.outlinedButtonColors(
                                    containerColor = Color(0xFFE3F2FD)
                                )
                            } else {
                                ButtonDefaults.outlinedButtonColors()
                            }
                        ) { Text(name) }
                    }
                }
            }
        }
    }
}

@Composable
internal fun Metric(label: String, value: String, unit: String) {
    Column(horizontalAlignment = Alignment.CenterHorizontally) {
        Text(label, style = MaterialTheme.typography.labelSmall, color = Color(0xFF666666))
        Row(verticalAlignment = Alignment.Bottom) {
            Text(value, style = MaterialTheme.typography.titleMedium, fontWeight = FontWeight.Bold)
            Spacer(Modifier.width(2.dp))
            Text(unit, style = MaterialTheme.typography.labelSmall, color = Color(0xFF666666))
        }
    }
}
