using MotionCore.Abstractions.Geometry;
using MotionCore.Abstractions.Motion;
using MotionCore.Abstractions.Vision;

namespace MotionCore.Calibration.Teaching;

/// <summary>九点示教参数。</summary>
public sealed record NinePointTeachingOptions
{
    /// <summary>标定半幅（mm）。最终 9 点铺满 ±SpanMm 的正方形区域。</summary>
    public double SpanMm { get; init; } = 6d;

    /// <summary>网格边长（标定用 3 → 3×3 = 9 点）。</summary>
    public int GridSize { get; init; } = 3;

    /// <summary>蛇形走位：奇数行反向，减少空行程约 1/3。</summary>
    public bool SnakeOrder { get; init; } = true;

    /// <summary>到位后的稳定等待（ms）。真实设备上这一步不能省：机械残余振动 + 相机曝光都需要时间。</summary>
    public int SettleDelayMs { get; init; } = 80;

    /// <summary>示教结束后是否回到中心位置。</summary>
    public bool ReturnToCenter { get; init; } = true;

    /// <summary>示教时使用精定位 profile（慢速），避免过冲把 Mark 甩出视野。</summary>
    public bool UseFineProfile { get; init; } = true;

    /// <summary>视觉定位的最低置信度，低于该值直接判定示教失败。</summary>
    public double MinimumMarkScore { get; init; } = 0.55d;

    /// <summary>标定中心。为空时取当前位置。</summary>
    public Point2D? Center { get; init; }

    /// <summary>示教使用的运动 profile。为空时取 <see cref="MoveProfile.Fine"/>。</summary>
    public MoveProfile? DemoProfile { get; init; }

    /// <summary>生成 9 个机械坐标（按示教顺序）。</summary>
    public IReadOnlyList<(string Label, Point2D Machine)> BuildGrid(Point2D center)
    {
        if (GridSize < 2)
        {
            throw new ArgumentOutOfRangeException(nameof(GridSize), GridSize, "网格边长至少为 2");
        }

        double spacing = (2d * SpanMm) / (GridSize - 1);
        List<(string, Point2D)> grid = new(GridSize * GridSize);

        for (int row = 0; row < GridSize; row++)
        {
            double y = center.Y - SpanMm + (row * spacing);

            for (int column = 0; column < GridSize; column++)
            {
                int actualColumn = SnakeOrder && (row % 2 == 1) ? (GridSize - 1 - column) : column;
                double x = center.X - SpanMm + (actualColumn * spacing);
                grid.Add(($"P{row + 1}{actualColumn + 1}", new Point2D(x, y)));
            }
        }

        return grid;
    }
}

/// <summary>示教过程进度。</summary>
public sealed record TeachingProgress(
    int Index,
    int Total,
    string Label,
    Point2D Machine,
    VisionMark? Mark,
    string Message)
{
    public double Percent => Total == 0 ? 0d : (Index * 100d) / Total;
}

/// <summary>
/// <b>九点示教流程</b>
/// <para>
/// 把"移动平台 → 等待稳定 → 拍照 → 找 Mark → 记录一对 (机械坐标, 像素坐标)"这五步
/// 循环 9 次。注意这里只负责"采集数据"，不做任何数学 —— 解算全部交给
/// <see cref="NinePointCalibrator"/>，这样示教流程和解算算法可以各自独立地替换与测试。
/// </para>
/// <para>
/// <b>为什么标定中心要取"Mark 在视野中心时平台的位置"：</b>
/// 九点标定的本质是让相机随平台移动、观察同一个固定 Mark 在图像中的位移。
/// 把中心取在 Mark 成像中心处，可以让 9 个点对称铺满视野，最大化利用像素分辨率。
/// </para>
/// </summary>
public sealed class NinePointTeachingService
{
    private readonly IMotionController _controller;
    private readonly ICamera _camera;
    private readonly IVisionLocator _locator;

    public NinePointTeachingService(
        IMotionController controller,
        ICamera camera,
        IVisionLocator locator,
        NinePointTeachingOptions? options = null)
    {
        _controller = controller ?? throw new ArgumentNullException(nameof(controller));
        _camera = camera ?? throw new ArgumentNullException(nameof(camera));
        _locator = locator ?? throw new ArgumentNullException(nameof(locator));
        Options = options ?? new NinePointTeachingOptions();
    }

    public NinePointTeachingOptions Options { get; }

    /// <summary>执行示教，返回标定点集合。</summary>
    public async Task<CalibrationPointSet> TeachAsync(
        IProgress<TeachingProgress>? progress = null,
        CancellationToken cancellationToken = default)
    {
        if (_controller.State != CardState.Ready)
        {
            throw new MotionStateException($"控制器状态为 {_controller.State}，无法开始标定示教");
        }

        if (!_camera.IsOpen)
        {
            await _camera.OpenAsync(cancellationToken).ConfigureAwait(false);
        }

        Point2D center = Options.Center ?? (await _controller.GetPositionAsync(cancellationToken).ConfigureAwait(false)).Planar;
        IReadOnlyList<(string Label, Point2D Machine)> grid = Options.BuildGrid(center);

        CalibrationPointSet pointSet = new();
        int index = 0;

        try
        {
            for (int i = 0; i < grid.Count; i++)
            {
                cancellationToken.ThrowIfCancellationRequested();

                (string label, Point2D machine) = grid[i];
                MoveProfile? profile = Options.UseFineProfile
                    ? ResolveFineProfile()
                    : null;

                await _controller.MoveAbsoluteAsync(machine, profile, cancellationToken).ConfigureAwait(false);

                if (Options.SettleDelayMs > 0)
                {
                    await Task.Delay(Options.SettleDelayMs, cancellationToken).ConfigureAwait(false);
                }

                VisionMark? mark = await CaptureMarkAsync(cancellationToken).ConfigureAwait(false);

                if (mark is null)
                {
                    progress?.Report(new TeachingProgress(
                        ++index,
                        grid.Count,
                        label,
                        machine,
                        null,
                        $"点位 {label} 未找到 Mark，示教中止"));

                    throw new MotionException(
                        $"标定点 {label} 处未定位到 Mark（平台 {machine}）。请确认 Mark 在视野内、光照正常、模板参数匹配")
                    {
                        Category = AlarmCategory.Process,
                        AlarmCode = "CAL-TEACH-001",
                    };
                }

                if (mark.Score < Options.MinimumMarkScore)
                {
                    throw new MotionException(
                        $"标定点 {label} 的 Mark 置信度仅 {mark.Score:P1}，低于阈值 {Options.MinimumMarkScore:P1}。图像质量不可用于标定")
                    {
                        Category = AlarmCategory.Process,
                        AlarmCode = "CAL-TEACH-002",
                    };
                }

                pointSet.Add(label, mark.Pixel, machine);

                progress?.Report(new TeachingProgress(
                    ++index,
                    grid.Count,
                    label,
                    machine,
                    mark,
                    $"采集 {label}：平台 {machine} → 像素 {mark.Pixel}（置信度 {mark.Score:P1}）"));
            }
        }
        finally
        {
            if (Options.ReturnToCenter)
            {
                try
                {
                    await _controller.MoveAbsoluteAsync(center, ResolveFineProfile(), CancellationToken.None).ConfigureAwait(false);
                }
                catch (Exception)
                {
                    // 回中心失败不应掩盖原始异常
                }
            }
        }

        return pointSet;
    }

    /// <summary>拍照并定位 Mark。</summary>
    public async Task<VisionMark?> CaptureMarkAsync(CancellationToken cancellationToken = default)
    {
        CameraFrame frame = await _camera.CaptureAsync(cancellationToken).ConfigureAwait(false);
        frame.Validate();
        return await _locator.FindMarkAsync(frame, MarkTemplate.Default, cancellationToken).ConfigureAwait(false);
    }

    private MoveProfile ResolveFineProfile() => Options.DemoProfile ?? MoveProfile.Fine;
}
