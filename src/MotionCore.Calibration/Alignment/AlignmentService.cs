using MotionCore.Abstractions.Geometry;
using MotionCore.Abstractions.Motion;
using MotionCore.Abstractions.Vision;

namespace MotionCore.Calibration.Alignment;

/// <summary>
/// <b>视觉对位（闭环补偿）</b>
/// <para>流程：</para>
/// <list type="number">
///   <item>拍照 → 找出 Mark 当前像素坐标；</item>
///   <item>用标定模型算出"要让 Mark 落在参考像素上，平台必须待在哪里"；</item>
///   <item>与平台当前位置求差，得到补偿量；</item>
///   <item>移动补偿 → 复检；未达阈值则继续迭代。</item>
/// </list>
/// <para>
/// <b>为什么标定对了还要迭代：</b> 理论上一份精确的仿射模型一次补偿就能到位，
/// 但现场存在反向间隙、伺服整定误差、镜头畸变残差、Mark 识别噪声，
/// 所以必须做成"测量—补偿—复检"的闭环，并且带上最大迭代次数与初始偏差护栏。
/// </para>
/// <para>
/// <b>安全护栏（比算法本身更重要）：</b>
/// 初始偏差超过 <see cref="AlignmentOptions.MaxInitialErrorMm"/> 时直接拒绝移动。
/// 标定参数被改坏或视觉误识别时算出的补偿量可能高达几十毫米，直接执行就是撞机事故。
/// </para>
/// </summary>
public sealed class AlignmentService
{
    private readonly IMotionController _controller;
    private readonly ICamera _camera;
    private readonly IVisionLocator _locator;
    private readonly CalibrationResult _calibration;
    private readonly MarkTemplate _template;
    private readonly MoveProfile _profile;

    public AlignmentService(
        IMotionController controller,
        ICamera camera,
        IVisionLocator locator,
        CalibrationResult calibration,
        AlignmentOptions? options = null,
        MarkTemplate? template = null,
        MoveProfile? profile = null)
    {
        _controller = controller ?? throw new ArgumentNullException(nameof(controller));
        _camera = camera ?? throw new ArgumentNullException(nameof(camera));
        _locator = locator ?? throw new ArgumentNullException(nameof(locator));
        _calibration = calibration ?? throw new ArgumentNullException(nameof(calibration));
        _template = template ?? MarkTemplate.Default;
        _profile = profile ?? MoveProfile.Fine;
        Options = options ?? new AlignmentOptions();
        Options.Validate();
    }

    public AlignmentOptions Options { get; }

    /// <summary>拍照并返回一次完整测量。</summary>
    public async Task<MarkMeasurement?> MeasureAsync(CancellationToken cancellationToken = default)
    {
        if (!_camera.IsOpen)
        {
            await _camera.OpenAsync(cancellationToken).ConfigureAwait(false);
        }

        CameraFrame frame = await _camera.CaptureAsync(cancellationToken).ConfigureAwait(false);
        frame.Validate();

        VisionMark? mark = await _locator.FindMarkAsync(frame, _template, cancellationToken).ConfigureAwait(false);
        if (mark is null)
        {
            return null;
        }

        MotionPose pose = await _controller.GetPositionAsync(cancellationToken).ConfigureAwait(false);

        // 标定模型 ImageToMachine(pixel) 的物理含义：
        // "要让 Mark 成像在该像素上，平台必须位于哪个机械坐标"。
        Point2D centered = _calibration.ImageToMachine(mark.Pixel);

        return new MarkMeasurement(mark, pose.Planar, centered, frame.Index);
    }

    /// <summary>以当前画面为参考，把 Mark 拉回当前位置（常用于示教对位基准）。</summary>
    public async Task<AlignmentResult> AlignToCurrentPixelAsync(
        IProgress<string>? log = null,
        CancellationToken cancellationToken = default)
    {
        MarkMeasurement? measurement = await MeasureAsync(cancellationToken).ConfigureAwait(false);
        if (measurement is null)
        {
            return AlignmentResult.Failed([], 0d, "首次拍照未定位到 Mark，无法建立对位基准");
        }

        log?.Report($"参考像素已记录：{measurement.Mark.Pixel}（置信度 {measurement.Mark.Score:P0}）");
        return await AlignToPixelAsync(measurement.Mark.Pixel, log, cancellationToken).ConfigureAwait(false);
    }

    /// <summary>把 Mark 对位到指定参考像素。</summary>
    public async Task<AlignmentResult> AlignToPixelAsync(
        Point2D referencePixel,
        IProgress<string>? log = null,
        CancellationToken cancellationToken = default)
    {
        Options.Validate();

        if (_controller.State != CardState.Ready)
        {
            return AlignmentResult.Failed([], 0d, $"控制器状态为 {_controller.State}，无法执行对位");
        }

        Point2D targetPlatformPosition = _calibration.ImageToMachine(referencePixel);
        List<AlignmentStep> steps = new();
        double initialError = 0d;

        for (int iteration = 1; iteration <= Options.MaxIterations; iteration++)
        {
            cancellationToken.ThrowIfCancellationRequested();

            MarkMeasurement? measurement = await MeasureAsync(cancellationToken).ConfigureAwait(false);
            if (measurement is null)
            {
                return AlignmentResult.Failed(steps, initialError, $"第 {iteration} 次测量未定位到 Mark（Mark 可能已移出视野）");
            }

            if (measurement.Mark.Score < Options.MinimumMarkScore)
            {
                return AlignmentResult.Failed(
                    steps,
                    initialError,
                    $"第 {iteration} 次测量的置信度仅 {measurement.Mark.Score:P1}，低于阈值 {Options.MinimumMarkScore:P1}，判定识别不可信");
            }

            Point2D offset = targetPlatformPosition - measurement.PlatformPosition;
            double error = offset.Length;

            if (iteration == 1)
            {
                initialError = error;
            }

            AlignmentStep step = new(
                iteration,
                measurement.Mark.Pixel,
                measurement.PlatformPosition,
                targetPlatformPosition,
                offset,
                error,
                measurement.Mark.Score);

            steps.Add(step);

            log?.Report(
                $"第 {iteration} 次：Mark 像素 {measurement.Mark.Pixel}（置信度 {measurement.Mark.Score:P0}）"
                + $" → 残差 {error:F6} mm");

            if (error <= Options.ToleranceMm)
            {
                return new AlignmentResult(
                    true,
                    steps,
                    initialError,
                    error,
                    $"已收敛：残差 {error:F6} mm ≤ 阈值 {Options.ToleranceMm:F6} mm");
            }

            if (iteration == 1 && error > Options.MaxInitialErrorMm)
            {
                return AlignmentResult.Failed(
                    steps,
                    initialError,
                    $"初始偏差 {error:F4} mm 超过安全上限 {Options.MaxInitialErrorMm:F4} mm，已拒绝补偿移动。"
                    + "请检查标定参数是否有效、视觉是否误识别到其它特征");
            }

            // 补偿移动（带阻尼系数，抑制振荡）
            Point2D compensation = step.Offset * Options.Damping;
            await _controller
                .MoveRelativeAsync(compensation.X, compensation.Y, _profile, cancellationToken)
                .ConfigureAwait(false);

            if (Options.SettleDelayMs > 0)
            {
                await Task.Delay(Options.SettleDelayMs, cancellationToken).ConfigureAwait(false);
            }
        }

        // 所有迭代用完仍未达标，做最后一次复检测量，给出真实残差
        MarkMeasurement? final = await MeasureAsync(cancellationToken).ConfigureAwait(false);
        if (final is null)
        {
            return AlignmentResult.Failed(steps, initialError, "最后一次复检未定位到 Mark");
        }

        Point2D finalOffset = targetPlatformPosition - final.PlatformPosition;
        double finalError = finalOffset.Length;

        steps.Add(new AlignmentStep(
            steps.Count + 1,
            final.Mark.Pixel,
            final.PlatformPosition,
            targetPlatformPosition,
            finalOffset,
            finalError,
            final.Mark.Score));

        bool converged = finalError <= Options.ToleranceMm;

        return new AlignmentResult(
            converged,
            steps,
            initialError,
            finalError,
            converged
                ? $"已收敛：残差 {finalError:F6} mm"
                : $"未收敛：{Options.MaxIterations} 次补偿后残差仍为 {finalError:F6} mm（阈值 {Options.ToleranceMm:F6} mm）。"
                  + "建议检查标定 RMSE、机构反向间隙与相机安装刚性");
    }
}
