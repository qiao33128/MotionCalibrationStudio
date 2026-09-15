using MotionCore.Abstractions.Geometry;
using MotionCore.Abstractions.Motion;
using MotionCore.Abstractions.Vision;
using MotionCore.Calibration;
using MotionCore.Calibration.Alignment;
using MotionCore.Calibration.Models;
using MotionCore.Calibration.Teaching;
using MotionCore.Simulation.Vision;
using Xunit;
using Xunit.Abstractions;

namespace MotionCore.Tests.EndToEnd;

/// <summary>
/// <b>端到端全链路测试：九点示教 → 标定解算 → 视觉对位。</b>
/// <para>
/// 整条链路跑的是"真的"东西：仿真卡按梯形曲线驱动平台，仿真相机真的渲染出一张位图，
/// 视觉定位器用 Otsu 阈值 + 灰度加权质心从图里算出 Mark 的亚像素坐标，
/// 标定器再做最小二乘解算。**全程没有一行硬编码的中间结果。**
/// </para>
/// <para>
/// 这个用例本身就是这套架构最大的价值证明：一台没有接任何运动卡、没有接任何相机的机器，
/// 也能把设备软件开发中最难验证的那一段（运动 + 视觉 + 标定的耦合）完整跑通并断言精度。
/// </para>
/// </summary>
public class VisionAlignmentEndToEndTests
{
    private readonly ITestOutputHelper _output;

    public VisionAlignmentEndToEndTests(ITestOutputHelper output) => _output = output;

    /// <summary>执行一次完整的示教 + 标定。</summary>
    private static async Task<(CalibrationResult Calibration, double SpanMm)> TeachAndCalibrateAsync(
        TestHarness.System system,
        double spanMm = 3d,
        CalibrationModelKind model = CalibrationModelKind.Affine)
    {
        NinePointTeachingService teaching = new(
            system.Controller,
            system.Camera,
            system.Locator,
            new NinePointTeachingOptions
            {
                SpanMm = spanMm,
                SettleDelayMs = 0,
                Center = TestHarness.DefaultMarkPosition,
                DemoProfile = TestHarness.FastProfile,
            });

        CalibrationPointSet points = await teaching.TeachAsync();

        Assert.Equal(9, points.Count);
        Assert.Equal(9, points.Points.Select(point => point.Label).Distinct().Count());

        CalibrationResult calibration = NinePointCalibrator.Calibrate(points, new CalibrationOptions
        {
            Model = model,
            RunLeaveOneOutValidation = true,
        });

        return (calibration, spanMm);
    }

    [Fact]
    public async Task 全链路_示教标定对位应端到端收敛()
    {
        await using TestHarness.System system = await TestHarness.CreateSystemAsync();

        // ── 1. 九点示教 + 标定 ─────────────────────────────
        (CalibrationResult calibration, _) = await TeachAndCalibrateAsync(system);

        _output.WriteLine(calibration.BuildReport());

        Assert.True(
            calibration.RmseMm < 0.02d,
            $"标定 RMSE {calibration.RmseMm:F6} mm 超出预期。\n{calibration.BuildReport()}");

        Assert.True(calibration.IsAcceptable, calibration.BuildReport());
        Assert.NotNull(calibration.LeaveOneOut);

        // Hartley 归一化之后，九点仿射设计矩阵的条件数应当是个很小的数 ——
        // 这条断言同时守护了"归一化真的生效了"这件事
        Assert.True(
            calibration.ConditionNumber < 10d,
            $"标定设计矩阵条件数 {calibration.ConditionNumber:E6} 异常偏大，说明坐标归一化没有生效");

        // ── 2. 标定参数应与仿真相机的真实安装参数一致 ────────
        AffineTransform2D affine = calibration.AsAffine()!;

        Assert.True(
            Math.Abs(affine.PixelsPerMmU - system.Mount.PixelsPerMmU) < 0.3d,
            $"像素当量还原失败：真值 {system.Mount.PixelsPerMmU:F4}，标定值 {affine.PixelsPerMmU:F4}");
        Assert.True(
            Math.Abs(affine.PixelsPerMmV - system.Mount.PixelsPerMmV) < 0.3d,
            $"像素当量还原失败：真值 {system.Mount.PixelsPerMmV:F4}，标定值 {affine.PixelsPerMmV:F4}");
        Assert.True(
            Math.Abs(affine.RotationDegrees - system.Mount.RotationDeg) < 0.3d,
            $"安装角还原失败：真值 {system.Mount.RotationDeg:F4}°，标定值 {affine.RotationDegrees:F4}°");

        // ── 3. 故意把平台挪开，再让对位把它拉回来 ────────────
        Point2D disturbance = new(1.35d, -0.85d);
        await system.Controller.MoveAbsoluteAsync(
            TestHarness.DefaultMarkPosition.Offset(disturbance.X, disturbance.Y),
            TestHarness.FastProfile);

        AlignmentService alignment = new(
            system.Controller,
            system.Camera,
            system.Locator,
            calibration,
            new AlignmentOptions { ToleranceMm = 0.01d, MaxIterations = 4, SettleDelayMs = 0 },
            MarkTemplate.Default,
            TestHarness.FastProfile);

        MarkMeasurement? disturbed = await alignment.MeasureAsync();
        Assert.NotNull(disturbed);

        // 对位到图像中心：标定中心就是 Mark 的真实位置，所以 Mark 居中时平台应正好停在 Mark 处
        Point2D referencePixel = new(system.Camera.Width / 2d, system.Camera.Height / 2d);
        AlignmentResult result = await alignment.AlignToPixelAsync(referencePixel);

        Assert.True(result.Success, result.BuildTrace());
        Assert.True(result.CompensationCount >= 1, "应至少执行过一次补偿移动");
        Assert.True(
            result.FinalErrorMm <= 0.01d,
            $"对位残差 {result.FinalErrorMm:F6} mm 未达阈值。\n{result.BuildTrace()}");

        // ── 4. 平台最终位置应回到 Mark 的真实位置 ─────────────
        MotionPose finalPose = await system.Controller.GetPositionAsync();
        double positionError = finalPose.Planar.DistanceTo(TestHarness.DefaultMarkPosition);

        Assert.True(
            positionError < 0.02d,
            $"对位后平台与 Mark 实际位置的偏差 {positionError:F6} mm 过大");

        // ── 5. 对位后 Mark 应落在图像中心附近 ────────────────
        MarkMeasurement? after = await alignment.MeasureAsync();
        Assert.NotNull(after);
        Assert.True(
            after!.Mark.Pixel.DistanceTo(referencePixel) < 1d,
            $"对位后 Mark 像素 {after.Mark.Pixel} 偏离图像中心 {referencePixel}");
    }

    [Fact]
    public async Task 极端安装角下_标定仍应正确还原()
    {
        // 相机装了约 7.5°：不校正的话对位会引入明显的 X/Y 交叉误差
        CameraMount tilted = new()
        {
            PixelsPerMmU = 18d,
            PixelsPerMmV = 18d,
            RotationDeg = -7.5d,
            FlipVertical = true,
        };

        await using TestHarness.System system = await TestHarness.CreateSystemAsync(tilted);

        (CalibrationResult calibration, _) = await TeachAndCalibrateAsync(system, spanMm: 4d);

        AffineTransform2D affine = calibration.AsAffine()!;
        Assert.True(
            Math.Abs(affine.RotationDegrees - tilted.RotationDeg) < 0.3d,
            $"安装角还原失败：真值 {tilted.RotationDeg}°，标定值 {affine.RotationDegrees:F4}°");
        Assert.True(Math.Abs(affine.PixelsPerMmU - tilted.PixelsPerMmU) < 0.3d);

        AlignmentService alignment = new(
            system.Controller, system.Camera, system.Locator, calibration,
            new AlignmentOptions { ToleranceMm = 0.015d },
            MarkTemplate.Default,
            TestHarness.FastProfile);

        await system.Controller.MoveAbsoluteAsync(
            TestHarness.DefaultMarkPosition.Offset(1.1d, 1.4d),
            TestHarness.FastProfile);

        AlignmentResult result = await alignment.AlignToPixelAsync(
            new Point2D(system.Camera.Width / 2d, system.Camera.Height / 2d));

        Assert.True(result.Success, result.BuildTrace());
    }

    [Fact]
    public async Task 初始偏差超过安全上限时应拒绝补偿移动()
    {
        await using TestHarness.System system = await TestHarness.CreateSystemAsync();

        (CalibrationResult calibration, _) = await TeachAndCalibrateAsync(system, spanMm: 2d);

        AlignmentService alignment = new(
            system.Controller, system.Camera, system.Locator, calibration,
            new AlignmentOptions
            {
                ToleranceMm = 0.01d,
                MaxInitialErrorMm = 0.3d, // 故意设得很严，用来验证护栏
                SettleDelayMs = 0,
            },
            MarkTemplate.Default,
            TestHarness.FastProfile);

        // 先人为偏出 0.8 mm（超过 0.3 mm 的安全上限），再尝试对位
        await system.Controller.MoveAbsoluteAsync(
            TestHarness.DefaultMarkPosition.Offset(0.55d, -0.6d),
            TestHarness.FastProfile);

        MotionPose before = await system.Controller.GetPositionAsync();

        AlignmentResult result = await alignment.AlignToPixelAsync(
            new Point2D(system.Camera.Width / 2d, system.Camera.Height / 2d));

        Assert.False(result.Success);
        Assert.Contains("拒绝补偿移动", result.Message, StringComparison.Ordinal);

        // 关键：被拦截时平台必须纹丝不动，绝不能"先动再说"
        MotionPose after = await system.Controller.GetPositionAsync();
        Assert.Equal(before.X, after.X, 9);
        Assert.Equal(before.Y, after.Y, 9);
    }

    [Fact]
    public async Task Mark丢失时_示教与对位都应给出明确失败原因()
    {
        await using TestHarness.System system = await TestHarness.CreateSystemAsync();

        NinePointTeachingService teaching = new(
            system.Controller,
            system.Camera,
            system.Locator,
            new NinePointTeachingOptions
            {
                SpanMm = 2d,
                SettleDelayMs = 0,
                Center = TestHarness.DefaultMarkPosition,
                DemoProfile = TestHarness.FastProfile,
            });

        system.Camera.HideMark = true;

        MotionException exception = await Assert.ThrowsAsync<MotionException>(() => teaching.TeachAsync());

        Assert.Contains("未定位到 Mark", exception.Message, StringComparison.Ordinal);
        Assert.Equal(AlarmCategory.Process, exception.Category);

        // 恢复 Mark 后应能正常完成示教
        system.Camera.HideMark = false;
        CalibrationPointSet points = await teaching.TeachAsync();
        Assert.Equal(9, points.Count);
    }

    [Fact]
    public async Task 视觉置信度过低时_应判定图像不可用()
    {
        await using TestHarness.System system = await TestHarness.CreateSystemAsync(imageNoise: 3d);

        NinePointTeachingService teaching = new(
            system.Controller,
            system.Camera,
            system.Locator,
            new NinePointTeachingOptions
            {
                SpanMm = 2d,
                SettleDelayMs = 0,
                Center = TestHarness.DefaultMarkPosition,
                DemoProfile = TestHarness.FastProfile,
                MinimumMarkScore = 1.01d, // 不可能达到的阈值，用于触发质量门禁
            });

        MotionException exception = await Assert.ThrowsAsync<MotionException>(() => teaching.TeachAsync());

        Assert.Contains("置信度", exception.Message, StringComparison.Ordinal);
        Assert.Equal("CAL-TEACH-002", exception.AlarmCode);
    }

    [Fact]
    public async Task 加入干扰斑块时_质心定位器应仍能锁定目标Mark()
    {
        await using TestHarness.System system = await TestHarness.CreateSystemAsync(addDecoyBlob: true);

        (CalibrationResult calibration, _) = await TeachAndCalibrateAsync(system, spanMm: 3d);

        // 干扰斑块不在 9 点网格的 Mark 移动范围内，因此标定精度不应明显退化
        Assert.True(
            calibration.RmseMm < 0.05d,
            $"存在干扰斑块时标定精度退化过多：{calibration.RmseMm:F6} mm");
    }

    [Fact]
    public async Task 标定结果存盘再加载_对位精度应保持一致()
    {
        await using TestHarness.System system = await TestHarness.CreateSystemAsync();

        (CalibrationResult original, _) = await TeachAndCalibrateAsync(system, spanMm: 3d);

        string path = Path.Combine(Path.GetTempPath(), $"e2e-calibration-{Guid.NewGuid():N}.csv");
        try
        {
            MotionCore.Calibration.Persistence.CalibrationFile.Save(original, path);
            CalibrationResult reloaded = MotionCore.Calibration.Persistence.CalibrationFile.Load(path);

            Assert.Equal(original.RmseMm, reloaded.RmseMm, 6);

            AlignmentService alignment = new(
                system.Controller, system.Camera, system.Locator, reloaded,
                new AlignmentOptions { ToleranceMm = 0.01d, SettleDelayMs = 0 },
                MarkTemplate.Default,
                TestHarness.FastProfile);

            await system.Controller.MoveAbsoluteAsync(
                TestHarness.DefaultMarkPosition.Offset(-1.1d, 0.9d),
                TestHarness.FastProfile);

            AlignmentResult result = await alignment.AlignToPixelAsync(
                new Point2D(system.Camera.Width / 2d, system.Camera.Height / 2d));

            Assert.True(result.Success, result.BuildTrace());
        }
        finally
        {
            if (File.Exists(path))
            {
                File.Delete(path);
            }
        }
    }
}
