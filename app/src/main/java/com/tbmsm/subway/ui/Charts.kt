package com.tbmsm.subway.ui

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.unit.dp
import com.tbmsm.subway.model.TrajSample
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

/**
 * 通用折线图。用于速度曲线与加速度曲线。
 *
 * @param series 每条曲线：(名称, 颜色, 数据点)
 */
@Composable
fun LineChart(
    series: List<Triple<String, Color, List<Pair<Double, Double>>>>,
    modifier: Modifier = Modifier,
    yLabel: String = "",
    zeroLine: Boolean = false
) {
    val allPoints = series.flatMap { it.third }
    if (allPoints.isEmpty()) {
        Box(modifier, contentAlignment = Alignment.Center) {
            Text("暂无数据", style = MaterialTheme.typography.bodySmall)
        }
        return
    }
    val minX = allPoints.minOf { it.first }
    val maxX = allPoints.maxOf { it.first }
    var minY = allPoints.minOf { it.second }
    var maxY = allPoints.maxOf { it.second }
    if (maxY - minY < 1e-6) {
        minY -= 1.0
        maxY += 1.0
    }
    val padY = (maxY - minY) * 0.1
    minY -= padY
    maxY += padY

    Box(modifier) {
        Canvas(Modifier.fillMaxSize().padding(4.dp)) {
            val w = size.width
            val h = size.height
            val spanX = max(1e-9, maxX - minX)
            val spanY = max(1e-9, maxY - minY)

            fun px(x: Double) = ((x - minX) / spanX * w).toFloat()
            fun py(y: Double) = (h - (y - minY) / spanY * h).toFloat()

            // 网格
            val gridColor = Color(0x22000000)
            for (i in 0..4) {
                val y = h * i / 4f
                drawLine(gridColor, Offset(0f, y), Offset(w, y), strokeWidth = 1f)
            }
            if (zeroLine && minY < 0 && maxY > 0) {
                val y0 = py(0.0)
                drawLine(Color(0x66000000), Offset(0f, y0), Offset(w, y0), strokeWidth = 2f)
            }

            for ((_, color, pts) in series) {
                if (pts.size < 2) continue
                val path = Path()
                pts.forEachIndexed { i, p ->
                    val x = px(p.first)
                    val y = py(p.second)
                    if (i == 0) path.moveTo(x, y) else path.lineTo(x, y)
                }
                drawPath(path, color, style = Stroke(width = 3f))
            }
        }
        if (yLabel.isNotEmpty()) {
            Text(
                yLabel,
                style = MaterialTheme.typography.labelSmall,
                modifier = Modifier.align(Alignment.TopStart).padding(6.dp)
            )
        }
    }
}

/**
 * 轨迹俯视图（ENU 平面）。可选叠加轨道中心线。
 */
@Composable
fun TrackMap(
    samples: List<TrajSample>,
    trackPoints: List<Pair<Double, Double>>,
    modifier: Modifier = Modifier
) {
    if (samples.isEmpty() && trackPoints.isEmpty()) {
        Box(modifier, contentAlignment = Alignment.Center) {
            Text("暂无轨迹", style = MaterialTheme.typography.bodySmall)
        }
        return
    }
    val xs = samples.map { it.east } + trackPoints.map { it.first }
    val ys = samples.map { it.north } + trackPoints.map { it.second }
    val minX = xs.min()
    val maxX = xs.max()
    val minY = ys.min()
    val maxY = ys.max()
    val spanX = max(1e-6, maxX - minX)
    val spanY = max(1e-6, maxY - minY)
    val span = max(spanX, spanY)

    Box(modifier) {
        Canvas(Modifier.fillMaxSize().padding(8.dp)) {
            val w = size.width
            val h = size.height
            val scale = min(w, h) / span.toFloat()
            val offX = (w - spanX.toFloat() * scale) / 2f
            val offY = (h - spanY.toFloat() * scale) / 2f

            fun px(x: Double) = (offX + (x - minX).toFloat() * scale)
            fun py(y: Double) = (h - offY - (y - minY).toFloat() * scale)

            if (trackPoints.size >= 2) {
                val path = Path()
                trackPoints.forEachIndexed { i, p ->
                    val x = px(p.first)
                    val y = py(p.second)
                    if (i == 0) path.moveTo(x, y) else path.lineTo(x, y)
                }
                drawPath(path, Color(0x5590A4B7), style = Stroke(width = 10f))
            }

            if (samples.size >= 2) {
                val path = Path()
                samples.forEachIndexed { i, s ->
                    val x = px(s.east)
                    val y = py(s.north)
                    if (i == 0) path.moveTo(x, y) else path.lineTo(x, y)
                }
                drawPath(path, Color(0xFF0B4F8A), style = Stroke(width = 4f))

                // 起点与终点
                val first = samples.first()
                val last = samples.last()
                drawCircle(Color(0xFF2E7D32), 10f, Offset(px(first.east), py(first.north)))
                drawCircle(Color(0xFFC62828), 10f, Offset(px(last.east), py(last.north)))
            }
        }
    }
}

/** 速度表盘（半圆）。 */
@Composable
fun SpeedGauge(
    speedKmh: Double,
    maxKmh: Double,
    modifier: Modifier = Modifier
) {
    Box(modifier, contentAlignment = Alignment.Center) {
        Canvas(Modifier.fillMaxSize()) {
            val w = size.width
            val h = size.height
            val cx = w / 2f
            val cy = h * 0.78f
            val radius = min(w, h * 1.35f) / 2f * 0.86f
            val stroke = radius * 0.16f

            // 背景弧（从 180 度到 360 度）
            drawArc(
                color = Color(0x22000000),
                startAngle = 180f,
                sweepAngle = 180f,
                useCenter = false,
                topLeft = Offset(cx - radius, cy - radius),
                size = Size(radius * 2, radius * 2),
                style = Stroke(width = stroke)
            )
            val frac = (speedKmh / maxKmh).coerceIn(0.0, 1.0)
            drawArc(
                color = Color(0xFF0B4F8A),
                startAngle = 180f,
                sweepAngle = (180f * frac).toFloat(),
                useCenter = false,
                topLeft = Offset(cx - radius, cy - radius),
                size = Size(radius * 2, radius * 2),
                style = Stroke(width = stroke)
            )
            // 指针
            val angle = Math.toRadians(180.0 + 180.0 * frac)
            val nx = cx + (radius * 0.92 * Math.cos(angle)).toFloat()
            val ny = cy + (radius * 0.92 * Math.sin(angle)).toFloat()
            drawLine(Color(0xFFC62828), Offset(cx, cy), Offset(nx, ny), strokeWidth = stroke * 0.35f)
        }
        Text(
            String.format("%.1f", speedKmh),
            style = MaterialTheme.typography.headlineMedium,
            modifier = Modifier.align(Alignment.Center)
        )
    }
}

/** 数值格式化辅助。 */
fun fmt(v: Double, digits: Int = 2): String =
    if (abs(v) < 1e-9) "0" else String.format("%.${digits}f", v)
