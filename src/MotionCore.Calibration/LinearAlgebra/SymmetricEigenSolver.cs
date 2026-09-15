namespace MotionCore.Calibration.LinearAlgebra;

/// <summary>特征分解结果。</summary>
public sealed record EigenDecomposition(double[] Values, double[][] Vectors)
{
    /// <summary>最小特征值对应的特征向量（按列取）。</summary>
    public double[] SmallestEigenVector()
    {
        int index = 0;
        for (int i = 1; i < Values.Length; i++)
        {
            if (Values[i] < Values[index])
            {
                index = i;
            }
        }

        return Vectors.Select(row => row[index]).ToArray();
    }

    public double LargestValue => Values.Max();

    public double SmallestValue => Values.Min();
}

/// <summary>
/// 对称矩阵特征分解（循环 Jacobi 旋转法）。
/// <para>
/// <b>为什么自己写而不是引 MathNet：</b> 单应矩阵的 DLT 解本质是求
/// AᵀA 最小特征值对应的特征向量，只要有一个稳定的对称特征分解就够了。
/// 循环 Jacobi 对 3×3 / 9×9 这种规模收敛极快（通常 &lt; 10 个 sweep），
/// 代码量不到 100 行，而且完全没有外部依赖。
/// </para>
/// <para>
/// 顺带还能拿到条件数：cond(A) = σmax / σmin = √(λmax / λmin)，
/// 标定数据是否退化（共线、点太密）就直接体现在这个数上。
/// </para>
/// </summary>
public static class SymmetricEigenSolver
{
    /// <summary>
    /// 分解实对称矩阵。输入 <paramref name="matrix"/> 必须对称（只读其上三角部分计算）。
    /// </summary>
    public static EigenDecomposition Decompose(double[][] matrix, int maxSweeps = 100, double tolerance = 1e-14)
    {
        ArgumentNullException.ThrowIfNull(matrix);

        int n = matrix.Length;
        for (int i = 0; i < n; i++)
        {
            if (matrix[i].Length != n)
            {
                throw new ArgumentException("必须传入方阵", nameof(matrix));
            }
        }

        double[][] a = new double[n][];
        for (int i = 0; i < n; i++)
        {
            a[i] = (double[])matrix[i].Clone();
        }

        double[][] v = new double[n][];
        for (int i = 0; i < n; i++)
        {
            v[i] = new double[n];
            v[i][i] = 1d;
        }

        for (int sweep = 0; sweep < maxSweeps; sweep++)
        {
            double offDiagonal = 0d;
            for (int i = 0; i < n; i++)
            {
                for (int j = i + 1; j < n; j++)
                {
                    offDiagonal += a[i][j] * a[i][j];
                }
            }

            if (Math.Sqrt(offDiagonal) < tolerance)
            {
                break;
            }

            for (int p = 0; p < n - 1; p++)
            {
                for (int q = p + 1; q < n; q++)
                {
                    if (Math.Abs(a[p][q]) < 1e-300)
                    {
                        continue;
                    }

                    double theta = (a[q][q] - a[p][p]) / (2d * a[p][q]);
                    double t = Math.Sign(theta) / (Math.Abs(theta) + Math.Sqrt((theta * theta) + 1d));
                    if (theta == 0)
                    {
                        t = 1d;
                    }

                    double c = 1d / Math.Sqrt((t * t) + 1d);
                    double s = t * c;

                    for (int k = 0; k < n; k++)
                    {
                        double akp = a[k][p];
                        double akq = a[k][q];
                        a[k][p] = (c * akp) - (s * akq);
                        a[k][q] = (s * akp) + (c * akq);
                    }

                    for (int k = 0; k < n; k++)
                    {
                        double apk = a[p][k];
                        double aqk = a[q][k];
                        a[p][k] = (c * apk) - (s * aqk);
                        a[q][k] = (s * apk) + (c * aqk);
                    }

                    for (int k = 0; k < n; k++)
                    {
                        double vkp = v[k][p];
                        double vkq = v[k][q];
                        v[k][p] = (c * vkp) - (s * vkq);
                        v[k][q] = (s * vkp) + (c * vkq);
                    }
                }
            }
        }

        double[] values = new double[n];
        for (int i = 0; i < n; i++)
        {
            values[i] = a[i][i];
        }

        return new EigenDecomposition(values, v);
    }

    /// <summary>
    /// 计算矩阵的条件数（基于 AᵀA 的特征值）。
    /// 标定场景下 &gt; 1e6 基本可以判定 9 点布局退化，需要重新示教。
    /// </summary>
    public static double ConditionNumber(double[][] a)
    {
        int columns = a[0].Length;
        double[][] ata = new double[columns][];
        for (int i = 0; i < columns; i++)
        {
            ata[i] = new double[columns];
        }

        for (int i = 0; i < columns; i++)
        {
            for (int j = i; j < columns; j++)
            {
                double sum = 0d;
                for (int k = 0; k < a.Length; k++)
                {
                    sum += a[k][i] * a[k][j];
                }

                ata[i][j] = sum;
                ata[j][i] = sum;
            }
        }

        EigenDecomposition eigen = Decompose(ata);
        double max = eigen.Values.Max();

        if (max <= 0)
        {
            return double.PositiveInfinity;
        }

        // 用小相对阈值而不是绝对阈值判断"数值零"：
        // 存在全零列（例如标定点全部落在一条直线上，v 分量恒为 0）时，
        // 最小特征值就是 0，此时条件数应为无穷大 —— 而不是随便返回一个有限值。
        double floor = max * 1e-14;
        double min = eigen.Values.Min();

        if (min <= floor)
        {
            return double.PositiveInfinity;
        }

        return Math.Sqrt(max / min);
    }
}
