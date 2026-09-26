package com.tbmsm.subway.track

import com.tbmsm.subway.core.GeoPoint
import org.json.JSONArray
import org.json.JSONObject
import java.io.File

/**
 * 轨道数据的 JSON 读写。
 *
 * 文件格式（assets/tracks/ 目录或 filesDir/tracks/ 目录下的 .json 文件）：
 * {
 *   "name": "示例线路",
 *   "origin": { "lat": 39.9, "lon": 116.4, "alt": 0 },
 *   "points": [ {"lat":..,"lon":..}, ... ],
 *   "stations": [ {"name":"A站","s":0}, {"name":"B站","s":1200} ],
 *   "stationDistancePriors": [1200, 980]
 * }
 *
 * 说明：points 为轨道中心线折线（经纬度），stations 的 s 为沿轨里程（米），
 * 若没有实测数据，可只填 points，站间距先验留空即可（此时不启用站间距约束）。
 */
object TrackRepository {

    private const val ASSET_DIR = "tracks"

    /** 读取 assets 中所有轨道文件。 */
    fun loadFromAssets(assetManager: android.content.res.AssetManager): List<TrackModel> {
        val result = ArrayList<TrackModel>()
        val names = try {
            assetManager.list(ASSET_DIR) ?: emptyArray()
        } catch (e: Exception) {
            emptyArray()
        }
        for (n in names) {
            if (!n.endsWith(".json", ignoreCase = true)) continue
            try {
                val text = assetManager.open("$ASSET_DIR/$n").bufferedReader().use { it.readText() }
                parse(text)?.let { result.add(it) }
            } catch (e: Exception) {
                // 单个文件损坏不影响其它轨道
            }
        }
        return result
    }

    /** 读取应用私有目录中用户导入的轨道。 */
    fun loadFromFiles(dir: File): List<TrackModel> {
        val result = ArrayList<TrackModel>()
        val files = dir.listFiles { f -> f.isFile && f.name.endsWith(".json", true) } ?: return result
        for (f in files) {
            try {
                parse(f.readText())?.let { result.add(it) }
            } catch (e: Exception) {
                // 忽略损坏文件
            }
        }
        return result
    }

    fun parse(text: String): TrackModel? {
        val root = JSONObject(text)
        val name = root.optString("name", "未命名轨道")
        val originObj = root.optJSONObject("origin") ?: return null
        val origin = GeoPoint(
            originObj.optDouble("lat", 0.0),
            originObj.optDouble("lon", 0.0),
            originObj.optDouble("alt", 0.0)
        )
        val ptsArr: JSONArray = root.optJSONArray("points") ?: return null
        if (ptsArr.length() < 2) return null
        val geoPoints = ArrayList<GeoPoint>(ptsArr.length())
        for (i in 0 until ptsArr.length()) {
            val o = ptsArr.optJSONObject(i) ?: continue
            geoPoints.add(GeoPoint(o.optDouble("lat"), o.optDouble("lon"), o.optDouble("alt", 0.0)))
        }
        if (geoPoints.size < 2) return null

        val stationS = ArrayList<Double>()
        val stationNames = ArrayList<String>()
        root.optJSONArray("stations")?.let { arr ->
            for (i in 0 until arr.length()) {
                val o = arr.optJSONObject(i) ?: continue
                stationS.add(o.optDouble("s", 0.0))
                stationNames.add(o.optString("name", "站${i + 1}"))
            }
        }

        val priors = ArrayList<Double>()
        root.optJSONArray("stationDistancePriors")?.let { arr ->
            for (i in 0 until arr.length()) priors.add(arr.optDouble(i, 0.0))
        }

        val model = TrackModel.fromGeoPolyline(name, geoPoints, stationS, priors) ?: return null
        return model
    }

    /** 生成一份示例轨道文件内容（供用户参考格式）。 */
    fun sampleJson(): String {
        val root = JSONObject()
        root.put("name", "示例线路（请替换为实测数据）")
        root.put("origin", JSONObject().apply {
            put("lat", 39.9000)
            put("lon", 116.4000)
            put("alt", 0.0)
        })
        val pts = JSONArray()
        // 一段直线 + 一段右转 + 一段直线，仅用于演示格式
        val legs = listOf(
            0.0 to 800.0,
            Math.toRadians(20.0) to 600.0,
            Math.toRadians(20.0) to 700.0
        )
        var lat = 39.9000
        var lon = 116.4000
        pts.put(JSONObject().apply { put("lat", lat); put("lon", lon) })
        for ((bearing, len) in legs) {
            val dNorth = len * Math.cos(bearing)
            val dEast = len * Math.sin(bearing)
            lat += dNorth / 111320.0
            lon += dEast / (111320.0 * Math.cos(Math.toRadians(lat)))
            pts.put(JSONObject().apply { put("lat", lat); put("lon", lon) })
        }
        root.put("points", pts)
        root.put("stations", JSONArray().apply {
            put(JSONObject().apply { put("name", "A站"); put("s", 0.0) })
            put(JSONObject().apply { put("name", "B站"); put("s", 800.0) })
            put(JSONObject().apply { put("name", "C站"); put("s", 2100.0) })
        })
        root.put("stationDistancePriors", JSONArray().apply {
            put(800.0)
            put(1300.0)
        })
        return root.toString(2)
    }

    // ==================================================================
    //  由传感器数据生成的轨道
    // ==================================================================

    /** 生成轨道的存放目录（应用私有，可写）。 */
    fun generatedDir(filesDir: File): File =
        File(filesDir, "tracks").also { it.mkdirs() }

    /**
     * 保存由 TrackBuilder 生成的轨道。
     *
     * @param name 轨道名称，会作为文件名（做安全化处理）
     * @return 保存后的文件，失败返回 null
     */
    fun saveGenerated(filesDir: File, name: String, json: String): File? {
        return try {
            val safe = name.replace(Regex("[\\\\/:*?\"<>|]"), "_").trim()
            if (safe.isEmpty()) return null
            val f = File(generatedDir(filesDir), "$safe.json")
            f.writeText(json)
            f
        } catch (e: Exception) {
            null
        }
    }

    /** 删除生成的轨道。 */
    fun deleteGenerated(filesDir: File, name: String): Boolean {
        val safe = name.replace(Regex("[\\\\/:*?\"<>|]"), "_").trim()
        val f = File(generatedDir(filesDir), "$safe.json")
        return f.exists() && f.delete()
    }

    /** 判断某个轨道是否为程序生成（而非 assets 内置）。 */
    fun isGenerated(filesDir: File, name: String): Boolean {
        val safe = name.replace(Regex("[\\\\/:*?\"<>|]"), "_").trim()
        return File(generatedDir(filesDir), "$safe.json").exists()
    }
}
