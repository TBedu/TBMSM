package com.tbmsm.subway.storage

import com.tbmsm.subway.core.GeoPoint
import com.tbmsm.subway.model.RunSegment
import com.tbmsm.subway.model.SessionSummary
import com.tbmsm.subway.model.TrajSample
import org.json.JSONArray
import org.json.JSONObject
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/**
 * 会话目录结构：
 *
 *   filesDir/sessions/<sessionId>/
 *       raw/imu.csv, raw/mag.csv, raw/gnss.csv
 *       track.csv        正向解算轨迹
 *       track_smoothed.csv  RTS 平滑轨迹
 *       segments.csv     站间运行段
 *       summary.json     会话总结
 */
object SessionStore {

    private const val DIR = "sessions"

    fun sessionsRoot(filesDir: File): File = File(filesDir, DIR).also { it.mkdirs() }

    fun sessionDir(filesDir: File, sessionId: String): File =
        File(sessionsRoot(filesDir), sessionId).also { it.mkdirs() }

    fun rawDir(filesDir: File, sessionId: String): File =
        File(sessionDir(filesDir, sessionId), "raw").also { it.mkdirs() }

    fun newSessionId(): String =
        SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())

    fun listSessions(filesDir: File): List<File> {
        val root = sessionsRoot(filesDir)
        val dirs = root.listFiles { f -> f.isDirectory } ?: return emptyList()
        return dirs.sortedByDescending { it.name }
    }

    fun deleteSession(filesDir: File, sessionId: String): Boolean {
        val dir = File(sessionsRoot(filesDir), sessionId)
        return dir.exists() && dir.deleteRecursively()
    }

    /** 写出轨迹 CSV。 */
    fun writeTrack(file: File, samples: List<TrajSample>) {
        file.bufferedWriter().use { w ->
            w.write(TrajSample.CSV_HEADER)
            w.write("\n")
            for (s in samples) {
                w.write(s.csvRow())
                w.write("\n")
            }
        }
    }

    /** 写出站间运行段 CSV。 */
    fun writeSegments(file: File, segments: List<RunSegment>) {
        file.bufferedWriter().use { w ->
            w.write(
                "index,depart_t_ns,arrive_t_ns,run_time_s,distance_m,mean_speed_mps," +
                    "max_speed_mps,max_accel_mps2,max_decel_mps2,station_prior_m,station_error_m\n"
            )
            for (s in segments) {
                w.write(
                    String.format(
                        Locale.US,
                        "%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.1f,%.3f\n",
                        s.index, s.departTNs, s.arriveTNs, s.runTimeS, s.distanceM,
                        s.meanSpeedMps, s.maxSpeedMps, s.maxAccelMps2, s.maxDecelMps2,
                        s.stationDistancePriorM, s.stationDistanceErrorM
                    )
                )
            }
        }
    }

    /** 写出会话总结 JSON。 */
    fun writeSummary(file: File, summary: SessionSummary) {
        val root = JSONObject()
        root.put("sessionId", summary.sessionId)
        root.put("startedAtMs", summary.startedAtMs)
        root.put("durationS", summary.durationS)
        root.put("totalDistanceM", summary.totalDistanceM)
        root.put("maxSpeedMps", summary.maxSpeedMps)
        root.put("meanSpeedMps", summary.meanSpeedMps)
        root.put("maxAccelMps2", summary.maxAccelMps2)
        root.put("maxDecelMps2", summary.maxDecelMps2)
        root.put("imuSamples", summary.imuSamples)
        root.put("gnssFixes", summary.gnssFixes)
        root.put("stillRatio", summary.stillRatio)
        root.put("nisRejected", summary.nisRejected)
        root.put("divergenceCount", summary.divergenceCount)
        root.put("smoothed", summary.smoothed)
        root.put("trackUsed", summary.trackUsed)
        root.put("trackName", summary.trackName)
        root.put("originLat", summary.originLat)
        root.put("originLon", summary.originLon)
        root.put("originAlt", summary.originAlt)
        root.put("warnings", JSONArray(summary.warnings))
        val segArr = JSONArray()
        for (s in summary.segments) {
            segArr.put(JSONObject().apply {
                put("index", s.index)
                put("runTimeS", s.runTimeS)
                put("distanceM", s.distanceM)
                put("meanSpeedMps", s.meanSpeedMps)
                put("maxSpeedMps", s.maxSpeedMps)
                put("maxAccelMps2", s.maxAccelMps2)
                put("maxDecelMps2", s.maxDecelMps2)
                put("stationDistancePriorM", s.stationDistancePriorM)
                put("stationDistanceErrorM", s.stationDistanceErrorM)
            })
        }
        root.put("segments", segArr)
        file.writeText(root.toString(2))
    }

    /** 读取会话总结，用于列表展示。 */
    fun readSummary(filesDir: File, sessionId: String): SessionSummary? {
        val f = File(sessionDir(filesDir, sessionId), "summary.json")
        if (!f.exists()) return null
        return try {
            val root = JSONObject(f.readText())
            val segArr = root.optJSONArray("segments") ?: JSONArray()
            val segs = ArrayList<RunSegment>()
            for (i in 0 until segArr.length()) {
                val o = segArr.optJSONObject(i) ?: continue
                segs.add(
                    RunSegment(
                        index = o.optInt("index", i),
                        departTNs = 0L,
                        arriveTNs = 0L,
                        runTimeS = o.optDouble("runTimeS", 0.0),
                        distanceM = o.optDouble("distanceM", 0.0),
                        meanSpeedMps = o.optDouble("meanSpeedMps", 0.0),
                        maxSpeedMps = o.optDouble("maxSpeedMps", 0.0),
                        maxAccelMps2 = o.optDouble("maxAccelMps2", 0.0),
                        maxDecelMps2 = o.optDouble("maxDecelMps2", 0.0),
                        stationDistancePriorM = o.optDouble("stationDistancePriorM", 0.0),
                        stationDistanceErrorM = o.optDouble("stationDistanceErrorM", 0.0)
                    )
                )
            }
            val warnArr = root.optJSONArray("warnings") ?: JSONArray()
            val warns = ArrayList<String>()
            for (i in 0 until warnArr.length()) warns.add(warnArr.optString(i))

            SessionSummary(
                sessionId = root.optString("sessionId", sessionId),
                startedAtMs = root.optLong("startedAtMs", 0L),
                durationS = root.optDouble("durationS", 0.0),
                totalDistanceM = root.optDouble("totalDistanceM", 0.0),
                maxSpeedMps = root.optDouble("maxSpeedMps", 0.0),
                meanSpeedMps = root.optDouble("meanSpeedMps", 0.0),
                maxAccelMps2 = root.optDouble("maxAccelMps2", 0.0),
                maxDecelMps2 = root.optDouble("maxDecelMps2", 0.0),
                imuSamples = root.optLong("imuSamples", 0L),
                gnssFixes = root.optInt("gnssFixes", 0),
                stillRatio = root.optDouble("stillRatio", 0.0),
                nisRejected = root.optInt("nisRejected", 0),
                divergenceCount = root.optInt("divergenceCount", 0),
                smoothed = root.optBoolean("smoothed", false),
                trackUsed = root.optBoolean("trackUsed", false),
                trackName = root.optString("trackName", ""),
                warnings = warns,
                segments = segs,
                originLat = root.optDouble("originLat", 0.0),
                originLon = root.optDouble("originLon", 0.0),
                originAlt = root.optDouble("originAlt", 0.0)
            )
        } catch (e: Exception) {
            null
        }
    }

    /** 读取轨迹 CSV（用于结果页绘图）。 */
    fun readTrack(file: File): List<TrajSample> {
        if (!file.exists()) return emptyList()
        val out = ArrayList<TrajSample>(4096)
        try {
            file.bufferedReader().useLines { lines ->
                lines.drop(1).forEach { line ->
                    if (line.isBlank()) return@forEach
                    val p = line.split(',')
                    if (p.size < 18) return@forEach
                    out.add(
                        TrajSample(
                            tNs = p[0].toLong(),
                            tRelS = p[1].toDouble(),
                            east = p[2].toDouble(),
                            north = p[3].toDouble(),
                            up = p[4].toDouble(),
                            vEast = p[5].toDouble(),
                            vNorth = p[6].toDouble(),
                            vUp = p[7].toDouble(),
                            speedMps = p[8].toDouble(),
                            accelAlong = p[9].toDouble(),
                            accelVertical = p[10].toDouble(),
                            accelLateral = p[11].toDouble(),
                            headingDeg = p[12].toDouble(),
                            pitchDeg = p[13].toDouble(),
                            rollDeg = p[14].toDouble(),
                            still = p[15] == "1",
                            posSigmaM = p[16].toDouble(),
                            smoothed = p[17] == "1"
                        )
                    )
                }
            }
        } catch (e: Exception) {
            // 返回已解析部分
        }
        return out
    }

    /** 会话目录总大小（字节）。 */
    fun sessionSizeBytes(filesDir: File, sessionId: String): Long {
        val dir = File(sessionsRoot(filesDir), sessionId)
        if (!dir.exists()) return 0L
        var total = 0L
        dir.walkTopDown().forEach { if (it.isFile) total += it.length() }
        return total
    }

    fun originOf(summary: SessionSummary): GeoPoint =
        GeoPoint(summary.originLat, summary.originLon, summary.originAlt)
}
