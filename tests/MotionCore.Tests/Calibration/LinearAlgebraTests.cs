using MotionCore.Calibration.LinearAlgebra;
using Xunit;

namespace MotionCore.Tests.Calibration;

/// <summary>
/// 自研线性代数工具的正确性测试。
/// <para>
/// 这套最小二乘 / 特征分解是标定精度的地基，一旦它有偏差，
/// 上层所有"RMSE 很小"的结论都是假的 —— 所以它必须有独立的、可对照的测试。
/// </para>
/// </summary>
public class LinearAlgebraTests
{
    private const double Tolerance = 1e-10;

    [Fact]
    public void 最小二乘应精确求解恰定方程组()
    {
        // 2x +  y = 5
        //  x + 3y = 10   → x = 1, y = 3
        double[][] a = [[2d, 1d], [1d, 3d]];
        double[] b = [5d, 10d];

        double[] x = LeastSquaresSolver.Solve(a, b);

        Assert.Equal(1d, x[0], 10);
        Assert.Equal(3d, x[1], 10);
    }

    [Fact]
    public void 最小二乘应对超定方程组给出最小残差解()
    {
        // 拟合 y = c0 + c1·t，数据取真实的 y = 1 + 2t 并加一点扰动
        double[] t = [0d, 1d, 2d, 3d, 4d];
        double[] y = [1.01d, 3.02d, 4.97d, 7.03d, 8.98d];

        double[][] a = t.Select(value => new[] { 1d, value }).ToArray();
        double[] x = LeastSquaresSolver.Solve(a, y);

        Assert.Equal(1d, x[0], 1);
        Assert.Equal(2d, x[1], 1);
    }

    [Fact]
    public void 秩亏方程组应抛出可读异常()
    {
        // 第二列与第一列线性相关 → 无法唯一确定参数
        double[][] a = [[1d, 2d], [2d, 4d], [3d, 6d]];
        double[] b = [1d, 2d, 3d];

        InvalidOperationException exception = Assert.Throws<InvalidOperationException>(
            () => LeastSquaresSolver.Solve(a, b));

        Assert.Contains("秩亏", exception.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void 方程数少于未知数应被拒绝()
    {
        double[][] a = [[1d, 2d, 3d]];
        double[] b = [1d];

        Assert.Throws<ArgumentException>(() => LeastSquaresSolver.Solve(a, b));
    }

    [Fact]
    public void Householder_QR_的精度应优于正规方程()
    {
        // 构造一个条件数很大的方程组（Hilbert 型）
        const int n = 5;
        double[][] a = new double[n][];
        for (int i = 0; i < n; i++)
        {
            a[i] = new double[n];
            for (int j = 0; j < n; j++)
            {
                a[i][j] = 1d / (i + j + 1);
            }
        }

        double[] expected = [1d, -2d, 3d, -4d, 5d];
        double[] b = new double[n];
        for (int i = 0; i < n; i++)
        {
            b[i] = a[i].Select((value, j) => value * expected[j]).Sum();
        }

        double[] solved = LeastSquaresSolver.Solve(a, b);

        for (int i = 0; i < n; i++)
        {
            Assert.Equal(expected[i], solved[i], 6);
        }
    }

    [Fact]
    public void 对称矩阵特征分解应还原已知特征值()
    {
        // [[2,1],[1,2]] 的特征值显然是 1 与 3
        EigenDecomposition eigen = SymmetricEigenSolver.Decompose([[2d, 1d], [1d, 2d]]);

        double[] sorted = [.. eigen.Values.OrderBy(value => value)];
        Assert.Equal(1d, sorted[0], 9);
        Assert.Equal(3d, sorted[1], 9);
    }

    [Fact]
    public void 对角矩阵的特征值应等于对角元()
    {
        double[][] matrix =
        [
            [10.42d, 0d, 0d],
            [0d, 10.42d, 0d],
            [0d, 0d, 9d],
        ];

        EigenDecomposition eigen = SymmetricEigenSolver.Decompose(matrix);

        double[] sorted = [.. eigen.Values.OrderBy(value => value)];
        Assert.Equal(9d, sorted[0], 8);
        Assert.Equal(10.42d, sorted[1], 8);
        Assert.Equal(10.42d, sorted[2], 8);
    }

    [Fact]
    public void 特征向量应满足_Av_等于_λv()
    {
        double[][] matrix =
        [
            [4d, 1d, -2d],
            [1d, 2d, 0d],
            [-2d, 0d, 3d],
        ];

        EigenDecomposition eigen = SymmetricEigenSolver.Decompose(matrix);

        for (int index = 0; index < eigen.Values.Length; index++)
        {
            double lambda = eigen.Values[index];
            double[] v = [.. eigen.Vectors.Select(row => row[index])];

            for (int row = 0; row < matrix.Length; row++)
            {
                double left = matrix[row].Select((value, column) => value * v[column]).Sum();
                double right = lambda * v[row];
                Assert.True(
                    Math.Abs(left - right) < 1e-8,
                    $"第 {index} 个特征向量不满足 Av = λv（第 {row} 行：{left} vs {right}）");
            }
        }
    }

    [Fact]
    public void 归一化后的九点标定设计矩阵条件数应接近一()
    {
        // 仿射模型的设计矩阵（Hartley 归一化之后）本质是分块对角，
        // 每块为 9×3 的 [u, v, 1]。理论条件数应接近 1，绝不是 1e6 量级。
        double[][] design = new double[18][];
        int index = 0;

        // 3×3 网格，均值距离归一化到 √2 之后的坐标
        double scale = Math.Sqrt(2d) / 3.219d;
        double[] offsets = [-3d * scale, 0d, 3d * scale];

        foreach (double u in offsets)
        {
            foreach (double v in offsets)
            {
                design[index * 2] = [u, v, 1d, 0d, 0d, 0d];
                design[(index * 2) + 1] = [0d, 0d, 0d, u, v, 1d];
                index++;
            }
        }

        double conditionNumber = SymmetricEigenSolver.ConditionNumber(design);

        Assert.True(
            conditionNumber < 2d,
            $"归一化后的 9 点设计矩阵条件数应接近 1，实际为 {conditionNumber:E3}");
    }

    [Fact]
    public void 共线点构成的设计矩阵条件数应极大()
    {
        // 9 个点全部落在一条直线上 → 设计矩阵接近奇异
        double[][] design = new double[18][];
        for (int i = 0; i < 9; i++)
        {
            double u = i * 0.1d;
            design[i * 2] = [u, 0d, 1d, 0d, 0d, 0d];
            design[(i * 2) + 1] = [0d, 0d, 0d, u, 0d, 1d];
        }

        double conditionNumber = SymmetricEigenSolver.ConditionNumber(design);

        Assert.True(
            conditionNumber > 1e8,
            $"共线点应当被判为病态，实际条件数仅 {conditionNumber:E3}");
    }

    [Fact]
    public void 三乘三矩阵求逆应正确()
    {
        Matrix3x3 matrix = new(2, 0, 0, 0, 4, 0, 0, 0, 8);
        Matrix3x3 inverse = matrix.Inverse();

        Assert.Equal(0.5d, inverse[0, 0], Tolerance);
        Assert.Equal(0.25d, inverse[1, 1], Tolerance);
        Assert.Equal(0.125d, inverse[2, 2], Tolerance);

        Matrix3x3 product = matrix * inverse;
        Assert.Equal(1d, product[0, 0], Tolerance);
        Assert.Equal(1d, product[1, 1], Tolerance);
        Assert.Equal(1d, product[2, 2], Tolerance);
        Assert.Equal(0d, product[0, 1], Tolerance);
    }

    [Fact]
    public void 奇异矩阵求逆应抛出异常()
    {
        Matrix3x3 singular = new(1, 2, 3, 2, 4, 6, 1, 1, 1);

        Assert.False(singular.IsInvertible);
        Assert.Throws<InvalidOperationException>(() => singular.Inverse());
    }
}
