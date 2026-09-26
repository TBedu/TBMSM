package com.tbmsm.subway.storage

import com.tbmsm.subway.model.GnssFix
import com.tbmsm.subway.model.ImuSample
import com.tbmsm.subway.model.MagSample
import java.io.BufferedWriter
import java.io.File
import java.io.FileWriter
import java.util.Locale

/**
 * 原始数据落盘。
 *
 * 采用「边采边写」的追加式 CSV，避免会话结束时一次性写大文件导致内存峰值。
 * 三个文件：
 *   imu.csv   —— 加计 + 陀螺（机体系原始，手机坐标系）
 *   mag.csv   —— 磁力计
 *   gnss.csv  —— 定位
 *
 * 写入在采集线程上同步进行（BufferedWriter + 定期 flush），
 * 100Hz 下每帧约 100 字节，对存储压力很小。
 */
class RawDataLogger(private val dir: File) {

    private var imuWriter: BufferedWriter? = null
    private var magWriter: BufferedWriter? = null
    private var gnssWriter: BufferedWriter? = null

    private var imuFlushCounter = 0
    private var magFlushCounter = 0

    val imuFile: File get() = File(dir, "imu.csv")
    val magFile: File get() = File(dir, "mag.csv")
    val gnssFile: File get() = File(dir, "gnss.csv")

    fun open() {
        if (!dir.exists()) dir.mkdirs()
        imuWriter = BufferedWriter(FileWriter(imuFile, false), 1 shl 16)
        magWriter = BufferedWriter(FileWriter(magFile, false), 1 shl 14)
        gnssWriter = BufferedWriter(FileWriter(gnssFile, false), 1 shl 12)
        imuWriter?.write("t_ns,ax,ay,az,gx,gy,gz\n")
        magWriter?.write("t_ns,mx,my,mz\n")
        gnssWriter?.write("t_ns,lat,lon,alt,accuracy,speed,bearing,provider\n")
    }

    fun logImu(s: ImuSample) {
        val w = imuWriter ?: return
        w.write(
            String.format(
                Locale.US, "%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                s.tNs, s.ax, s.ay, s.az, s.gx, s.gy, s.gz
            )
        )
        if (++imuFlushCounter >= 200) {
            imuFlushCounter = 0
            w.flush()
        }
    }

    fun logMag(s: MagSample) {
        val w = magWriter ?: return
        w.write(String.format(Locale.US, "%d,%.4f,%.4f,%.4f\n", s.tNs, s.mx, s.my, s.mz))
        if (++magFlushCounter >= 50) {
            magFlushCounter = 0
            w.flush()
        }
    }

    fun logGnss(f: GnssFix) {
        val w = gnssWriter ?: return
        w.write(
            String.format(
                Locale.US, "%d,%.8f,%.8f,%.3f,%.2f,%.3f,%.3f,%s\n",
                f.tNs, f.lat, f.lon, f.alt, f.accuracyM, f.speedMps, f.bearingDeg, f.provider
            )
        )
        w.flush()
    }

    fun close() {
        try {
            imuWriter?.flush(); imuWriter?.close()
            magWriter?.flush(); magWriter?.close()
            gnssWriter?.flush(); gnssWriter?.close()
        } catch (e: Exception) {
            // 忽略关闭异常
        }
        imuWriter = null
        magWriter = null
        gnssWriter = null
    }
}
