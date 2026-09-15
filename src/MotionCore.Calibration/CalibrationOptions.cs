using MotionCore.Calibration.Models;

namespace MotionCore.Calibration;

/// <summary>标定质量分级。现场只需要看这一栏就能判断能不能投产。</summary>
public enum CalibrationQuality
{
    /// <summary>数据不合法或无法解算。</summary>
    Invalid = 0,

    /// <summary>优秀：RMSE 在优秀阈值以内。</summary>
    Excellent = 1,

    /// <summary>良好：可投产。</summary>
    Good = 2,

    /// <summary>勉强：可试产，但建议复查标定点。</summary>
    Fair = 3,

    /// <summary>不合格：必须重新示教。</summary>
    Poor = 4,
}

/// <summary>标定参数。</summary>
public sealed class CalibrationOptions
{
    /// <summary>使用的模型。</summary>
    public CalibrationModelKind Model { get; init; } = CalibrationModelKind.Affine;

    /// <summary>
    /// 是否做 Hartley 坐标归一化。
    /// <para>
    /// 像素量级 ~1e2、毫米量级 ~1e1，两组坐标的量级差异会直接体现在设计矩阵的条件数上；
    /// 归一化把两组点各自平移到质心、缩放成平均距离 √2，能把条件数压低好几个数量级。
    /// 除了大视野下的二次多项式（归一化收益有限）以外都建议开启。
    /// </para>
    /// </summary>
    public bool NormalizeCoordinates { get; init; } = true;

    /// <summary>RMSE 达到该值即评为"优秀"（mm）。</summary>
    public double ExcellentRmseMm { get; init; } = 0.01;

    /// <summary>RMSE 达到该值仍可投产（mm）。</summary>
    public double AcceptableRmseMm { get; init; } = 0.03;

    /// <summary>
    /// 调用方要求的最少标定点数（默认 4）。
    /// <para>实际生效值取它与"模型理论最少点数"的较大者，见 <see cref="EffectiveMinimumPointCount"/>。</para>
    /// </summary>
    public int MinimumPointCount { get; init; } = 4;

    /// <summary>
    /// 实际生效的最少标定点数。
    /// <para>
    /// 刻意做成"自动取较大者"而不是"低于理论值就报错"：
    /// 换模型（例如从仿射切到二次多项式）时不应该因为默认配置没跟着改就直接抛异常，
    /// 那种失败方式对现场工程师非常不友好。
    /// </para>
    /// </summary>
    public int EffectiveMinimumPointCount =>
        Math.Max(MinimumPointCount, GetMinimumPointCount(Model));

    /// <summary>条件数超过该值时给出"标定点布局退化"的警告。</summary>
    public double ConditionNumberWarning { get; init; } = 1e6;

    /// <summary>是否在解算后自动执行留一交叉验证，用于判断是否过拟合。</summary>
    public bool RunLeaveOneOutValidation { get; init; } = true;

    public CalibrationOptions WithModel(CalibrationModelKind model) => new()
    {
        Model = model,
        NormalizeCoordinates = NormalizeCoordinates,
        ExcellentRmseMm = ExcellentRmseMm,
        AcceptableRmseMm = AcceptableRmseMm,
        MinimumPointCount = MinimumPointCount,
        ConditionNumberWarning = ConditionNumberWarning,
        RunLeaveOneOutValidation = RunLeaveOneOutValidation,
    };

    /// <summary>留一交叉验证内部会反复调用标定，必须显式关掉它自己，否则会无限递归。</summary>
    public CalibrationOptions WithLeaveOneOutValidation(bool enabled) => new()
    {
        Model = Model,
        NormalizeCoordinates = NormalizeCoordinates,
        ExcellentRmseMm = ExcellentRmseMm,
        AcceptableRmseMm = AcceptableRmseMm,
        MinimumPointCount = MinimumPointCount,
        ConditionNumberWarning = ConditionNumberWarning,
        RunLeaveOneOutValidation = enabled,
    };

    /// <summary>模型需要的理论最少点数。</summary>
    public static int GetMinimumPointCount(CalibrationModelKind model) => model switch
    {
        CalibrationModelKind.Affine => 3,
        CalibrationModelKind.Homography => 4,
        CalibrationModelKind.QuadraticPolynomial => 6,
        _ => 3,
    };

    public void Validate()
    {
        if (ExcellentRmseMm <= 0 || AcceptableRmseMm <= 0)
        {
            throw new ArgumentException("精度阈值必须为正数");
        }

        if (ExcellentRmseMm >= AcceptableRmseMm)
        {
            throw new ArgumentException("优秀阈值必须小于合格阈值");
        }

        if (MinimumPointCount < 3)
        {
            throw new ArgumentException($"最少标定点数至少为 3，当前设置为 {MinimumPointCount}");
        }
    }
}
