using MotionCore.Abstractions.Geometry;
using MotionCore.Calibration;
using MotionCore.Calibration.LinearAlgebra;
using MotionCore.Calibration.Models;
using MotionCore.Calibration.Validation;
using Xunit;

namespace MotionCore.Tests.Calibration;

/// <summary>
/// 三种标定模型的正确性，以及"什么时候该用哪种模型"的判据测试。
/// </summary>
public class ModelComparisonTests
{
    // ───────────────────────── 单应模型 ─────────────────────────

    /// <summary>构造一个带真实透视分量的投影变换（像素 → 机械）。</summary>
    private static Matrix3x3 CreateProjectiveGroundTruth()
    {
        const double pixelsPerMm = 24d;
        const double rotationDeg = 1.7d;
        double cos = Math.Cos(rotationDeg * Math.PI / 180d);
        double sin = Math.Sin(rotationDeg * Math.PI / 180d);

        double m00 = pixelsPerMm * cos;
        double m01 = pixelsPerMm * sin;
        double m10 = pixelsPerMm * sin;
        double m11 = -pixelsPerMm * cos;

        double determinant = (m00 * m11) - (m01 * m10);
        double a = -m11 / determinant;
        double b = m01 / determinant;
        double d = m10 / determinant;
        double e = -m00 / determinant;

        Point2D machineCenter = AffineCalibrationTests.MachineCenter;
        Point2D imageCenter = AffineCalibrationTests.ImageCenter;
        double c = machineCenter.X - (a * imageCenter.X) - (b * imageCenter.Y);
        double f = machineCenter.Y - (d * imageCenter.X) - (e * imageCenter.Y);

        // g / h 是透视分量：在 u = 320 处 w ≈ 1.0088，即约 0.9% 的投影畸变，
        // 足以被算法识别，也必须被单应模型吸收
        return new Matrix3x3(a, b, c, d, e, f, 5e-5, -3e-5, 1d);
    }

    [Fact]
    public void 单应模型应精确还原带透视的映射()
    {
        Matrix3x3 truth = CreateProjectiveGroundTruth();
        CalibrationPointSet set = new();

        IReadOnlyList<Point2D> machineGrid = TestHarness.BuildGrid(AffineCalibrationTests.MachineCenter, span: 6d);
        foreach (Point2D machine in machineGrid)
        {
            // 由真值反解像素：需要真值的逆
            Point2D pixel = truth.Inverse().Transform(machine);
            set.Add($"P{set.Count + 1}", pixel, machine);
        }

        CalibrationResult result = NinePointCalibrator.Calibrate(set, new CalibrationOptions
        {
            Model = CalibrationModelKind.Homography,
            RunLeaveOneOutValidation = false,
        });

        Assert.True(result.RmseMm < 1e-8, $"单应模型残差应接近 0，实际 {result.RmseMm:E3} mm");
        Assert.True(result.Model is Homography2D);

        // 不但训练点要准，任意探测点也要准 —— 这才是"学到了真映射"
        Point2D probePixel = new(360.5d, 210.25d);
        Point2D expected = truth.Transform(probePixel);
        Point2D actual = result.Model.ImageToMachine(probePixel);

        Assert.True(
            expected.DistanceTo(actual) < 1e-8,
            $"探测点映射误差过大：期望 {expected}，实际 {actual}");
    }

    [Fact]
    public void 数据实为仿射时_单应模型的透视分量应接近零()
    {
        AffineTransform2D truth = AffineCalibrationTests.CreateGroundTruth();
        CalibrationPointSet set = new();

        foreach (Point2D machine in TestHarness.BuildGrid(AffineCalibrationTests.MachineCenter, span: 6d))
        {
            set.Add($"P{set.Count + 1}", truth.MachineToImage(machine), machine);
        }

        CalibrationResult result = NinePointCalibrator.Calibrate(set, new CalibrationOptions
        {
            Model = CalibrationModelKind.Homography,
            RunLeaveOneOutValidation = false,
        });

        Homography2D homography = Assert.IsType<Homography2D>(result.Model);

        // 归一化后 h8 = 1，因此 g / h 就是透视强度
        Assert.True(
            homography.PerspectiveMagnitude < 1e-6,
            $"仿射数据的透视分量应接近 0，实际 {homography.PerspectiveMagnitude:E3}");
    }

    // ───────────────────────── 二次多项式模型 ─────────────────────────

    [Fact]
    public void 二次多项式模型应精确还原含畸变的映射_且逆映射可收敛()
    {
        // 真值系数：[常数, u, v, uv, u², v²]
        // 主项与前面仿射测试保持一致，二次项用来模拟镜头畸变
        QuadraticPolynomialTransform2D truth = new(
            [25.6d, -0.0416d, -0.0012d, 1.0e-7d, 3.0e-8d, -2.0e-8d],
            [-13.6d, -0.0012d, 0.0416d, -8.0e-8d, 1.5e-8d, -3.2e-8d]);

        CalibrationPointSet set = new();

        // 用 16 个点铺满视野（二次模型有 12 个参数，点多一些更稳）
        for (int row = 0; row < 4; row++)
        {
            for (int column = 0; column < 4; column++)
            {
                Point2D pixel = new(200d + (column * 80d), 120d + (row * 80d));
                Point2D machine = truth.ImageToMachine(pixel);
                set.Add($"P{row}{column}", pixel, machine);
            }
        }

        CalibrationResult result = NinePointCalibrator.Calibrate(set, new CalibrationOptions
        {
            Model = CalibrationModelKind.QuadraticPolynomial,
            RunLeaveOneOutValidation = false,
        });

        Assert.True(result.RmseMm < 1e-7, $"二次模型残差应接近 0，实际 {result.RmseMm:E3} mm");

        // 逆映射（牛顿迭代）必须能收敛回原像素
        foreach (CalibrationPoint point in set.Points)
        {
            Point2D machine = result.Model.ImageToMachine(point.Image);
            Point2D back = result.Model.MachineToImage(machine);

            Assert.True(
                point.Image.DistanceTo(back) < 1e-9,
                $"逆映射不收敛：正向 {point.Image}，反向 {back}");
        }
    }

    // ───────────────────────── 模型选型判据 ─────────────────────────

    [Fact]
    public void 数据实为仿射且含噪声时_仿射模型的留一泛化误差应优于二次模型()
    {
        AffineTransform2D truth = AffineCalibrationTests.CreateGroundTruth();
        Random random = new(4242);
        CalibrationPointSet set = new();

        foreach (Point2D machine in TestHarness.BuildGrid(AffineCalibrationTests.MachineCenter, span: 6d))
        {
            Point2D pixel = truth.MachineToImage(machine);
            pixel = new Point2D(
                pixel.X + (NextGaussian(random) * 0.4d),
                pixel.Y + (NextGaussian(random) * 0.4d));

            set.Add($"P{set.Count + 1}", pixel, machine);
        }

        CalibrationOptions options = new() { RunLeaveOneOutValidation = true };

        CalibrationResult affine = NinePointCalibrator.Calibrate(set, options.WithModel(CalibrationModelKind.Affine));
        CalibrationResult quadratic = NinePointCalibrator.Calibrate(set, options.WithModel(CalibrationModelKind.QuadraticPolynomial));

        Assert.NotNull(affine.LeaveOneOut);
        Assert.NotNull(quadratic.LeaveOneOut);

        // 训练残差：参数多的二次模型一定不差
        Assert.True(
            quadratic.RmseMm <= affine.RmseMm + 1e-9,
            "参数更多的模型在训练集上不应更差");

        // 但留一泛化误差：二次模型把噪声也拟合进去了，应该更差
        Assert.True(
            quadratic.LeaveOneOut!.MeanErrorMm >= affine.LeaveOneOut!.MeanErrorMm,
            $"过拟合判据失效：仿射留一 {affine.LeaveOneOut.MeanErrorMm:F6} mm，"
            + $"二次留一 {quadratic.LeaveOneOut.MeanErrorMm:F6} mm");
    }

    [Fact]
    public void 留一验证应对每个标定点都给出独立结果()
    {
        AffineTransform2D truth = AffineCalibrationTests.CreateGroundTruth();
        CalibrationPointSet set = new();

        foreach (Point2D machine in TestHarness.BuildGrid(AffineCalibrationTests.MachineCenter, span: 6d))
        {
            set.Add($"P{set.Count + 1}", truth.MachineToImage(machine), machine);
        }

        CalibrationLeaveOneOutResult loocv = LeaveOneOutValidator.Validate(set, new CalibrationOptions());

        Assert.Equal(set.Count, loocv.SampleCount);
        Assert.All(loocv.Residuals, residual => Assert.Contains("留一", residual.Label, StringComparison.Ordinal));
        Assert.True(loocv.MaxErrorMm < 1e-8, "无噪声数据下留一误差应接近 0");
    }

    [Fact]
    public void 模型对比接口应返回所有数据支持的模型()
    {
        AffineTransform2D truth = AffineCalibrationTests.CreateGroundTruth();
        CalibrationPointSet set = new();

        foreach (Point2D machine in TestHarness.BuildGrid(AffineCalibrationTests.MachineCenter, span: 6d))
        {
            set.Add($"P{set.Count + 1}", truth.MachineToImage(machine), machine);
        }

        IReadOnlyDictionary<CalibrationModelKind, CalibrationResult> results =
            NinePointCalibrator.CompareAllModels(set, new CalibrationOptions { RunLeaveOneOutValidation = false });

        Assert.Equal(3, results.Count);
        Assert.All(results.Values, result => Assert.True(result.RmseMm < 1e-6));
    }

    private static double NextGaussian(Random random)
    {
        double u1 = 1.0 - random.NextDouble();
        double u2 = random.NextDouble();
        return Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Sin(2.0 * Math.PI * u2);
    }
}
