package com.tbmsm.subway.core

import kotlin.math.sqrt

/**
 * 通用稠密矩阵（行主序）。本项目仅用于 15 维误差状态协方差等小矩阵，
 * 不做任何稀疏优化，但所有运算都避免装箱与中间对象爆炸。
 */
class MatN(val rows: Int, val cols: Int) {

    val d = DoubleArray(rows * cols)

    operator fun get(r: Int, c: Int): Double = d[r * cols + c]

    operator fun set(r: Int, c: Int, value: Double) {
        d[r * cols + c] = value
    }

    fun copy(): MatN {
        val m = MatN(rows, cols)
        System.arraycopy(d, 0, m.d, 0, d.size)
        return m
    }

    /** 返回 this * o。 */
    fun multiply(o: MatN): MatN {
        require(cols == o.rows) { "维度不匹配: ${rows}x$cols * ${o.rows}x${o.cols}" }
        val r = MatN(rows, o.cols)
        for (i in 0 until rows) {
            val ri = i * cols
            val oi = i * r.cols
            for (k in 0 until cols) {
                val a = d[ri + k]
                if (a == 0.0) continue
                val ok = k * o.cols
                for (j in 0 until o.cols) {
                    r.d[oi + j] += a * o.d[ok + j]
                }
            }
        }
        return r
    }

    fun multiply(v: DoubleArray): DoubleArray {
        require(cols == v.size)
        val out = DoubleArray(rows)
        for (i in 0 until rows) {
            var s = 0.0
            val ri = i * cols
            for (k in 0 until cols) s += d[ri + k] * v[k]
            out[i] = s
        }
        return out
    }

    fun transpose(): MatN {
        val t = MatN(cols, rows)
        for (i in 0 until rows) for (j in 0 until cols) t[j, i] = this[i, j]
        return t
    }

    operator fun plus(o: MatN): MatN {
        require(rows == o.rows && cols == o.cols)
        val r = MatN(rows, cols)
        for (i in d.indices) r.d[i] = d[i] + o.d[i]
        return r
    }

    operator fun minus(o: MatN): MatN {
        require(rows == o.rows && cols == o.cols)
        val r = MatN(rows, cols)
        for (i in d.indices) r.d[i] = d[i] - o.d[i]
        return r
    }

    fun scaled(s: Double): MatN {
        val r = MatN(rows, cols)
        for (i in d.indices) r.d[i] = d[i] * s
        return r
    }

    /** 强制对称化，抑制浮点积累导致的非对称协方差。 */
    fun symmetrize() {
        require(rows == cols)
        for (i in 0 until rows) {
            for (j in 0 until i) {
                val avg = 0.5 * (this[i, j] + this[j, i])
                this[i, j] = avg
                this[j, i] = avg
            }
        }
    }

    fun isFinite(): Boolean {
        for (v in d) if (v.isNaN() || v.isInfinite()) return false
        return true
    }

    companion object {
        fun zeros(r: Int, c: Int): MatN = MatN(r, c)

        fun identity(n: Int): MatN {
            val m = MatN(n, n)
            for (i in 0 until n) m[i, i] = 1.0
            return m
        }

        fun diag(values: DoubleArray): MatN {
            val m = MatN(values.size, values.size)
            for (i in values.indices) m[i, i] = values[i]
            return m
        }
    }
}

/**
 * Cholesky 分解：A = L * L^T。A 必须对称正定。
 * 返回下三角 L 的紧凑存储（n*n 行主序），失败（非正定）返回 null。
 */
fun choleskyFactor(a: MatN): DoubleArray? {
    val n = a.rows
    val l = DoubleArray(n * n)
    for (i in 0 until n) {
        for (j in 0..i) {
            var sum = a[i, j]
            for (k in 0 until j) sum -= l[i * n + k] * l[j * n + k]
            if (i == j) {
                if (sum <= 1e-15) return null
                l[i * n + i] = sqrt(sum)
            } else {
                l[i * n + j] = sum / l[j * n + j]
            }
        }
    }
    return l
}

/** 用已分解的 L 求解 A x = b。 */
fun choleskySolve(l: DoubleArray, n: Int, b: DoubleArray): DoubleArray {
    val y = DoubleArray(n)
    for (i in 0 until n) {
        var sum = b[i]
        for (k in 0 until i) sum -= l[i * n + k] * y[k]
        y[i] = sum / l[i * n + i]
    }
    val x = DoubleArray(n)
    for (i in n - 1 downTo 0) {
        var sum = y[i]
        for (k in i + 1 until n) sum -= l[k * n + i] * x[k]
        x[i] = sum / l[i * n + i]
    }
    return x
}

/**
 * 用已分解的 L 求解 A X = B（多列），B 为 n x m 行主序。
 */
fun choleskySolveMatrix(l: DoubleArray, n: Int, b: MatN): MatN {
    val m = b.cols
    val out = MatN(n, m)
    val col = DoubleArray(n)
    for (j in 0 until m) {
        for (i in 0 until n) col[i] = b[i, j]
        val x = choleskySolve(l, n, col)
        for (i in 0 until n) out[i, j] = x[i]
    }
    return out
}

/**
 * 卡方分布上分位点近似（用于 NIS 门限）。
 * 采用 Wilson–Hilferty 变换，自由度 dof、置信度 0.99 下的近似分位点。
 */
private val CHI2_99 = doubleArrayOf(
    /* dof=0 */ 0.0,
    /* 1 */ 6.635, /* 2 */ 9.210, /* 3 */ 11.345, /* 4 */ 13.277,
    /* 5 */ 15.086, /* 6 */ 16.812, /* 7 */ 18.475, /* 8 */ 20.090
)

fun chi2Quantile99(dof: Int): Double =
    if (dof in 1..8) CHI2_99[dof] else CHI2_99[8] * dof / 8.0
