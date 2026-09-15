namespace MotionCore.Calibration.LinearAlgebra;

/// <summary>
/// 超定线性方程组的最小二乘解。
/// <para>
/// <b>为什么不用正规方程 (AᵀA)⁻¹Aᵀb：</b> 正规方程会把条件数平方，
/// 标定数据里像素量级 ~1e2、毫米量级 ~1e1，量级差异本身就大，
/// 平方之后 double 的有效位会掉得非常快。这里用 <b>Householder QR</b> 直接分解 A，
/// 数值稳定性好得多，代价只是多一倍的乘法。
/// </para>
/// </summary>
public static class LeastSquaresSolver
{
    /// <summary>
    /// 求解 min ‖A·x − b‖₂。A 为 m×n（m ≥ n），返回长度 n 的解向量。
    /// </summary>
    /// <exception cref="ArgumentException">方程数少于未知数。</exception>
    /// <exception cref="InvalidOperationException">矩阵秩亏（标定点退化，例如 9 点共线）。</exception>
    public static double[] Solve(double[][] a, double[] b)
    {
        ArgumentNullException.ThrowIfNull(a);
        ArgumentNullException.ThrowIfNull(b);

        int rows = a.Length;
        if (rows == 0)
        {
            throw new ArgumentException("方程组为空", nameof(a));
        }

        int columns = a[0].Length;
        if (rows < columns)
        {
            throw new ArgumentException($"方程数 {rows} 少于未知数 {columns}，无法求解", nameof(a));
        }

        if (b.Length != rows)
        {
            throw new ArgumentException($"右端项长度 {b.Length} 与方程数 {rows} 不一致", nameof(b));
        }

        // 复制输入，避免副作用
        double[][] r = new double[rows][];
        for (int i = 0; i < rows; i++)
        {
            if (a[i].Length != columns)
            {
                throw new ArgumentException($"第 {i} 行长度 {a[i].Length} 与首行 {columns} 不一致", nameof(a));
            }

            r[i] = (double[])a[i].Clone();
        }

        double[] rhs = (double[])b.Clone();
        double[] houseHolder = new double[rows];

        // Householder QR：逐列把 A 的下三角消成 0，同时把同样的反射作用到 b 上
        for (int k = 0; k < columns; k++)
        {
            double normSquared = 0d;
            for (int i = k; i < rows; i++)
            {
                normSquared += r[i][k] * r[i][k];
            }

            double norm = Math.Sqrt(normSquared);
            if (norm < 1e-300)
            {
                // 该列全 0：秩亏。继续做后面的列，最后由回代阶段的奇异判定报错。
                continue;
            }

            double alpha = r[k][k] > 0 ? -norm : norm;
            double vNormSquared = 0d;
            for (int i = k; i < rows; i++)
            {
                double value = i == k ? r[i][k] - alpha : r[i][k];
                houseHolder[i] = value;
                vNormSquared += value * value;
            }

            if (vNormSquared < 1e-300)
            {
                continue;
            }

            for (int j = k; j < columns; j++)
            {
                double dot = 0d;
                for (int i = k; i < rows; i++)
                {
                    dot += houseHolder[i] * r[i][j];
                }

                double factor = 2d * dot / vNormSquared;
                for (int i = k; i < rows; i++)
                {
                    r[i][j] -= factor * houseHolder[i];
                }
            }

            double dotRhs = 0d;
            for (int i = k; i < rows; i++)
            {
                dotRhs += houseHolder[i] * rhs[i];
            }

            double factorRhs = 2d * dotRhs / vNormSquared;
            for (int i = k; i < rows; i++)
            {
                rhs[i] -= factorRhs * houseHolder[i];
            }
        }

        // 回代
        double[] solution = new double[columns];
        for (int i = columns - 1; i >= 0; i--)
        {
            double sum = rhs[i];
            for (int j = i + 1; j < columns; j++)
            {
                sum -= r[i][j] * solution[j];
            }

            double pivot = r[i][i];
            if (Math.Abs(pivot) < 1e-12)
            {
                throw new InvalidOperationException(
                    $"设计矩阵秩亏（第 {i} 个主元 = {pivot:E3}）。标定点退化，请检查 9 个点是否共线、是否有重复点。");
            }

            solution[i] = sum / pivot;
        }

        return solution;
    }

    /// <summary>计算残差向量 b − A·x，用于精度评估。</summary>
    public static double[] ComputeResidual(double[][] a, double[] b, double[] x)
    {
        double[] residual = new double[a.Length];
        for (int i = 0; i < a.Length; i++)
        {
            double predicted = 0d;
            for (int j = 0; j < x.Length; j++)
            {
                predicted += a[i][j] * x[j];
            }

            residual[i] = b[i] - predicted;
        }

        return residual;
    }
}
