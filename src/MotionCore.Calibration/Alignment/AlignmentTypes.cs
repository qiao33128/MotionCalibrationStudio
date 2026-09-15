using System.Globalization;
using System.Text;
using MotionCore.Abstractions.Geometry;
using MotionCore.Abstractions.Vision;

namespace MotionCore.Calibration.Alignment;

/// <summary>视觉对位参数。</summary>
public sealed record AlignmentOptions
{
    /// <summary>收敛判据（mm）。3C 光学对位一般取 0.005 ~ 0.02 mm。</summary>
    public double ToleranceMm { get; init; } = 0.01d;

    /// <summary>最大迭代次数。理想仿射模型 1 次就够，多余次数用于吸收噪声与反向间隙。</summary>
    public int MaxIterations { get; init; } = 4;

    /// <summary>
    /// 允许的最大初始偏差（mm）。超过该值直接拒绝移动。
    /// <para>
    /// 这是设备安全里最容易被忽略的一条：如果标定参数被改坏、或者视觉误识别到别的特征，
    /// 算出来的 offset 可能是几十毫米，直接执行就是撞机。宁可报错让人来看。
    /// </para>
    /// </summary>
    public double MaxInitialErrorMm { get; init; } = 5d;

    /// <summary>补偿系数（0 &lt; k ≤ 1）。取 &lt; 1 可抑制振荡，代价是收敛变慢。</summary>
    public double Damping { get; init; } = 1d;

    /// <summary>运动后的稳定等待（ms）。</summary>
    public int SettleDelayMs { get; init; } = 60;

    /// <summary>视觉置信度下限，低于该值判定测量不可信。</summary>
    public double MinimumMarkScore { get; init; } = 0.55d;

    public void Validate()
    {
        if (ToleranceMm <= 0)
        {
            throw new ArgumentException("收敛阈值必须为正数", nameof(ToleranceMm));
        }

        if (MaxIterations < 1)
        {
            throw new ArgumentException("最大迭代次数至少为 1", nameof(MaxIterations));
        }

        if (Damping is <= 0 or > 1)
        {
            throw new ArgumentException("补偿系数必须在 (0, 1] 区间内", nameof(Damping));
        }
    }
}

/// <summary>一次视觉测量结果。</summary>
public sealed record MarkMeasurement(
    VisionMark Mark,
    Point2D PlatformPosition,
    Point2D CenteredPlatformPosition,
    int FrameIndex)
{
    public override string ToString() =>
        $"像素 {Mark.Pixel}｜平台 {PlatformPosition}｜居中位置 {CenteredPlatformPosition}｜置信度 {Mark.Score:P0}";
}

/// <summary>对位过程中的一次迭代记录。</summary>
public sealed record AlignmentStep(
    int Iteration,
    Point2D MarkPixel,
    Point2D PlatformPosition,
    Point2D TargetPlatformPosition,
    Point2D Offset,
    double ErrorMm,
    double MarkScore);

/// <summary>对位结果。</summary>
public sealed record AlignmentResult(
    bool Success,
    IReadOnlyList<AlignmentStep> Steps,
    double InitialErrorMm,
    double FinalErrorMm,
    string Message)
{
    /// <summary>实际迭代次数（含最后一次复检测量）。</summary>
    public int Iterations => Steps.Count;

    /// <summary>补偿移动次数 = 迭代次数 − 1（最后一次只测量不移动）。</summary>
    public int CompensationCount => Math.Max(0, Steps.Count - 1);

    /// <summary>收敛比：初始偏差被压缩了多少倍。</summary>
    public double ConvergenceRatio =>
        FinalErrorMm > 1e-12 && InitialErrorMm > 1e-12 ? InitialErrorMm / FinalErrorMm : double.PositiveInfinity;

    public static AlignmentResult Failed(IReadOnlyList<AlignmentStep> steps, double initialError, string message) =>
        new(false, steps, initialError, steps.Count > 0 ? steps[^1].ErrorMm : initialError, message);

    /// <summary>生成对位过程追溯文本（直接进设备日志）。</summary>
    public string BuildTrace()
    {
        CultureInfo culture = CultureInfo.InvariantCulture;
        StringBuilder builder = new();

        builder.AppendLine("============== 视觉对位追溯 ==============");
        builder.AppendLine(string.Format(culture, "结果      : {0}", Success ? "成功（已收敛）" : "失败"));
        builder.AppendLine(string.Format(culture, "说明      : {0}", Message));
        builder.AppendLine(string.Format(culture, "初始偏差  : {0:F6} mm", InitialErrorMm));
        builder.AppendLine(string.Format(culture, "最终偏差  : {0:F6} mm", FinalErrorMm));
        builder.AppendLine(string.Format(
            culture,
            "迭代/补偿 : {0} / {1} 次，收敛比 {2}",
            Iterations,
            CompensationCount,
            double.IsPositiveInfinity(ConvergenceRatio) ? "∞" : ConvergenceRatio.ToString("F1", culture)));

        foreach (AlignmentStep step in Steps)
        {
            builder.AppendLine(string.Format(
                culture,
                "  #{0}  像素 {1,-22} 平台 {2,-22} 补偿 {3,-22} 残差 {4:F6} mm  置信度 {5:P0}",
                step.Iteration,
                step.MarkPixel.ToString("F3"),
                step.PlatformPosition.ToString("F4"),
                step.Offset.ToString("F4"),
                step.ErrorMm,
                step.MarkScore));
        }

        builder.AppendLine("=========================================");
        return builder.ToString();
    }
}
