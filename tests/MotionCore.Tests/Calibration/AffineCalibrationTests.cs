using MotionCore.Abstractions.Geometry;
using MotionCore.Calibration;
using MotionCore.Calibration.Models;
using Xunit;

namespace MotionCore.Tests.Calibration;

/// <summary>
/// 九点标定解算的核心正确性测试。
/// <para>
/// 思路：<b>用已知的仿射矩阵造数据，再要求解算器把它原样还原</b>。
/// 这是标定算法唯一有说服力的验证方式 —— 只看"标定完成了"说明不了任何问题。
/// </para>
/// </summary>
public class AffineCalibrationTests
{
    /// <summary>标定中心（机械坐标）。</summary>
    internal static readonly Point2D MachineCenter = new(12d, -4d);

    /// <summary>标定中心对应的像素位置（取图像中心）。</summary>
    internal static readonly Point2D ImageCenter = new(320d, 240d);

    /// <summary>
    /// 合成一组有量级差异的真实参数：24 px/mm、安装角 1.7°、带 Y 翻转。
    /// <para>
    /// 数学关系：<c>仿射矩阵 = −(相机安装模型的逆)</c>，
    /// 负号来自"平台移动方向与物体在图像中的移动方向相反"。
    /// </para>
    /// </summary>
    internal static AffineTransform2D CreateGroundTruth()
    {
        const double pixelsPerMm = 24d;
        const double rotationDeg = 1.7d;
        double cos = Math.Cos(rotationDeg * Math.PI / 180d);
        double sin = Math.Sin(rotationDeg * Math.PI / 180d);

        // machine → pixel 的线性部分（含 v 轴翻转）
        double m00 = pixelsPerMm * cos;
        double m01 = pixelsPerMm * sin;
        double m10 = pixelsPerMm * sin;
        double m11 = -pixelsPerMm * cos;

        // 求 M 的逆，再取负 → 像素 → 机械 的线性部分
        double determinant = (m00 * m11) - (m01 * m10);
        double a = -m11 / determinant;
        double b = m01 / determinant;
        double d = m10 / determinant;
        double e = -m00 / determinant;

        // 平移项由"标定中心 ↔ 图像中心"反算，保证像素落在图像内
        double c = MachineCenter.X - (a * ImageCenter.X) - (b * ImageCenter.Y);
        double f = MachineCenter.Y - (d * ImageCenter.X) - (e * ImageCenter.Y);

        return new AffineTransform2D(a, b, c, d, e, f);
    }

    private static CalibrationPointSet CreatePointSet(AffineTransform2D truth, double noisePixels = 0d, int seed = 12345)
    {
        Random random = new(seed);
        CalibrationPointSet set = new();
        IReadOnlyList<Point2D> grid = TestHarness.BuildGrid(MachineCenter, span: 6d);

        for (int i = 0; i < grid.Count; i++)
        {
            Point2D machine = grid[i];
            Point2D pixel = truth.MachineToImage(machine);

            if (noisePixels > 0)
            {
                pixel = new Point2D(
                    pixel.X + (NextGaussian(random) * noisePixels),
                    pixel.Y + (NextGaussian(random) * noisePixels));
            }

            // 注意参数顺序：Image = 像素，Machine = 机械
            set.Add($"P{i + 1}", pixel, machine);
        }

        return set;
    }

    private static double NextGaussian(Random random)
    {
        double u1 = 1.0 - random.NextDouble();
        double u2 = random.NextDouble();
        return Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Sin(2.0 * Math.PI * u2);
    }

    [Fact]
    public void 无噪声数据_应精确还原仿射矩阵全部六个参数()
    {
        AffineTransform2D truth = CreateGroundTruth();
        CalibrationPointSet set = CreatePointSet(truth);

        CalibrationResult result = NinePointCalibrator.Calibrate(set, new CalibrationOptions
        {
            Model = CalibrationModelKind.Affine,
            RunLeaveOneOutValidation = false,
        });

        AffineTransform2D fitted = result.AsAffine()!;

        Assert.Equal(truth.A, fitted.A, 8);
        Assert.Equal(truth.B, fitted.B, 8);
        Assert.Equal(truth.C, fitted.C, 8);
        Assert.Equal(truth.D, fitted.D, 8);
        Assert.Equal(truth.E, fitted.E, 8);
        Assert.Equal(truth.F, fitted.F, 8);

        Assert.True(result.RmseMm < 1e-9, $"无噪声数据的残差应接近 0，实际 {result.RmseMm:E3} mm");
        Assert.Equal(CalibrationQuality.Excellent, result.Quality);
    }

    [Fact]
    public void 无噪声数据_应还原出正确的像素当量与相机安装角()
    {
        AffineTransform2D truth = CreateGroundTruth();
        CalibrationResult result = NinePointCalibrator.Calibrate(
            CreatePointSet(truth),
            new CalibrationOptions { Model = CalibrationModelKind.Affine, RunLeaveOneOutValidation = false });

        AffineTransform2D fitted = result.AsAffine()!;

        Assert.Equal(24d, fitted.PixelsPerMmU, 6);
        Assert.Equal(24d, fitted.PixelsPerMmV, 6);
        Assert.Equal(1.7d, fitted.RotationDegrees, 6);
        Assert.Equal(1d / 24d, fitted.ScaleMmPerPixelU, 9);
        Assert.True(Math.Abs(fitted.OrthogonalityError) < 1e-9, "正交安装下正交性偏差应为 0");
        Assert.True(fitted.IsReflected, "相机 v 轴向下时映射必然是镜像的");
    }

    [Theory]
    [InlineData(0.1d)]
    [InlineData(0.3d)]
    [InlineData(0.5d)]
    public void 像素噪声下的标定精度应满足工程要求(double noisePixels)
    {
        AffineTransform2D truth = CreateGroundTruth();
        CalibrationResult result = NinePointCalibrator.Calibrate(
            CreatePointSet(truth, noisePixels),
            new CalibrationOptions { Model = CalibrationModelKind.Affine, RunLeaveOneOutValidation = false });

        // 每个点在 u、v 两个方向上各有 σ 的噪声，因此单点的二维位置噪声是 √2·σ 像素
        double singlePointNoiseMm = noisePixels * Math.Sqrt(2d) / 24d;

        Assert.True(
            result.RmseMm < singlePointNoiseMm,
            $"RMSE {result.RmseMm:F6} mm 应小于单点噪声 {singlePointNoiseMm:F6} mm（最小二乘起到了平均作用）");

        // 6 个参数、18 个方程时，残差应被压到噪声的约 0.82 倍
        Assert.True(
            result.RmseMm > singlePointNoiseMm * 0.5d,
            $"RMSE {result.RmseMm:F6} mm 显著低于理论下限，说明精度评估可能失真");

        // 参数还原误差也要小
        AffineTransform2D fitted = result.AsAffine()!;
        Assert.True(
            Math.Abs(fitted.PixelsPerMmU - 24d) < 0.5d,
            $"像素当量还原误差过大：{fitted.PixelsPerMmU:F4} px/mm");
    }

    [Fact]
    public void 标定结果应满足闭环一致性()
    {
        AffineTransform2D truth = CreateGroundTruth();
        CalibrationResult result = NinePointCalibrator.Calibrate(
            CreatePointSet(truth),
            new CalibrationOptions { Model = CalibrationModelKind.Affine, RunLeaveOneOutValidation = false });

        // 像素 → 机械 → 像素，应回到原点
        Point2D probe = new(317.75d, 202.5d);
        double closedLoopError = result.ClosedLoopErrorPixels(probe);
        Assert.True(closedLoopError < 1e-8, $"闭环误差应接近 0，实际 {closedLoopError:E3} px");
    }

    [Fact]
    public void 九点共线属于退化数据_应抛出明确异常()
    {
        CalibrationPointSet set = new();
        for (int i = 0; i < 9; i++)
        {
            // 全部落在同一条直线上：无论如何都解不出二维映射
            set.Add($"P{i + 1}", new Point2D(320d + (i * 10d), 240d), new Point2D(i * 1d, 0d));
        }

        InvalidOperationException exception = Assert.Throws<InvalidOperationException>(
            () => NinePointCalibrator.Calibrate(set, new CalibrationOptions { Model = CalibrationModelKind.Affine }));

        Assert.Contains("秩亏", exception.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void 标定点过少应被拒绝()
    {
        CalibrationPointSet set = new();
        set.Add("P1", new Point2D(300d, 200d), new Point2D(0d, 0d));
        set.Add("P2", new Point2D(400d, 220d), new Point2D(5d, 0d));

        ArgumentException exception = Assert.Throws<ArgumentException>(
            () => NinePointCalibrator.Calibrate(set, new CalibrationOptions { Model = CalibrationModelKind.Affine }));

        Assert.Contains("标定点数量不足", exception.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void 标定点在图像上过于集中应被拒绝()
    {
        CalibrationPointSet set = new();
        for (int i = 0; i < 9; i++)
        {
            set.Add(
                $"P{i + 1}",
                new Point2D(320d + (i % 3), 240d + (i / 3)),
                new Point2D(i * 0.01d, i * 0.01d));
        }

        ArgumentException exception = Assert.Throws<ArgumentException>(
            () => NinePointCalibrator.Calibrate(set, new CalibrationOptions { Model = CalibrationModelKind.Affine }));

        Assert.Contains("过于集中", exception.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void 标定报告应包含关键精度指标()
    {
        CalibrationResult result = NinePointCalibrator.Calibrate(CreatePointSet(CreateGroundTruth()));

        string report = result.BuildReport();
        Assert.Contains("RMSE", report, StringComparison.Ordinal);
        Assert.Contains("条件数", report, StringComparison.Ordinal);
        Assert.Contains("像素当量", report, StringComparison.Ordinal);
        Assert.Contains("相机安装角", report, StringComparison.Ordinal);
        Assert.Contains("逐点残差", report, StringComparison.Ordinal);
    }
}
