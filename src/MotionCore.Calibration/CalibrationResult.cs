using System.Globalization;
using System.Text;
using MotionCore.Abstractions.Geometry;
using MotionCore.Calibration.Models;

namespace MotionCore.Calibration;

/// <summary>单个标定点的残差。</summary>
public sealed record CalibrationResidual(
    string Label,
    Point2D Image,
    Point2D MachineMeasured,
    Point2D MachinePredicted)
{
    /// <summary>预测误差向量（mm）：预测 − 实测。</summary>
    public Point2D Error => MachinePredicted - MachineMeasured;

    /// <summary>误差模长（mm）。</summary>
    public double ErrorMagnitude => Error.Length;

    public override string ToString() =>
        $"{Label}: 实测 {MachineMeasured} → 预测 {MachinePredicted}，误差 {ErrorMagnitude:F6} mm";
}

/// <summary>
/// 标定结果。除了模型本身，还带上整套精度证据（逐点残差、RMSE、最大误差、条件数、LOOCV），
/// 因为"标定完了"和"标定对了"是两件事。
/// </summary>
public sealed class CalibrationResult
{
    public required ICalibrationModel Model { get; init; }

    public required CalibrationModelKind Kind { get; init; }

    public required CalibrationOptions Options { get; init; }

    public required IReadOnlyList<CalibrationResidual> Residuals { get; init; }

    /// <summary>均方根误差（mm）。</summary>
    public required double RmseMm { get; init; }

    public required double MeanErrorMm { get; init; }

    public required double MaxErrorMm { get; init; }

    /// <summary>设计矩阵条件数，用于判断标定点布局是否退化。</summary>
    public required double ConditionNumber { get; init; }

    public required Point2D ImageCentroid { get; init; }

    public required Point2D MachineCentroid { get; init; }

    /// <summary>机械侧标定行程（平均到质心距离，mm）。</summary>
    public required double MachineSpreadMm { get; init; }

    /// <summary>留一交叉验证结果（未开启时为 null）。</summary>
    public CalibrationLeaveOneOutResult? LeaveOneOut { get; init; }

    public DateTimeOffset CreatedAt { get; init; } = DateTimeOffset.Now;

    /// <summary>质量分级。</summary>
    public CalibrationQuality Quality =>
        !double.IsFinite(RmseMm) ? CalibrationQuality.Invalid
        : RmseMm <= Options.ExcellentRmseMm ? CalibrationQuality.Excellent
        : RmseMm <= Options.AcceptableRmseMm ? CalibrationQuality.Good
        : RmseMm <= Options.AcceptableRmseMm * 2 ? CalibrationQuality.Fair
        : CalibrationQuality.Poor;

    public bool IsAcceptable => Quality is CalibrationQuality.Excellent or CalibrationQuality.Good;

    /// <summary>条件数是否超限（标定点退化）。</summary>
    public bool IsIllConditioned => ConditionNumber > Options.ConditionNumberWarning;

    /// <summary>
    /// 是否存在过拟合：训练 RMSE 明显小于留一误差。
    /// 出现这个信号说明模型参数过多 / 标定点太少，应该降级成仿射。
    /// </summary>
    public bool IsOverFitting =>
        LeaveOneOut is not null
        && LeaveOneOut.MeanErrorMm > RmseMm * 3
        && LeaveOneOut.MeanErrorMm > Options.ExcellentRmseMm;

    /// <summary>把像素映射到机械坐标（业务代码最常调用的入口）。</summary>
    public Point2D ImageToMachine(Point2D image) => Model.ImageToMachine(image);

    /// <summary>把机械坐标映射到像素。</summary>
    public Point2D MachineToImage(Point2D machine) => Model.MachineToImage(machine);

    /// <summary>闭环校验：像素 → 机械 → 像素，返回闭环误差（像素）。</summary>
    public double ClosedLoopErrorPixels(Point2D image)
    {
        Point2D machine = Model.ImageToMachine(image);
        Point2D back = Model.MachineToImage(machine);
        return back.DistanceTo(image);
    }

    public AffineTransform2D? AsAffine() => Model as AffineTransform2D;

    /// <summary>生成可直接贴进设备日志 / 交接文档的标定报告。</summary>
    public string BuildReport()
    {
        StringBuilder builder = new();
        CultureInfo culture = CultureInfo.InvariantCulture;

        builder.AppendLine("================ 九点标定报告 ================");
        builder.AppendLine(string.Format(culture, "标定时间     : {0:yyyy-MM-dd HH:mm:ss}", CreatedAt));
        builder.AppendLine(string.Format(culture, "标定模型     : {0}", Model.Name));
        builder.AppendLine(string.Format(culture, "标定点数     : {0}", Residuals.Count));
        builder.AppendLine(string.Format(culture, "标定行程     : ±{0:F3} mm（机械侧平均半径）", MachineSpreadMm));
        builder.AppendLine(string.Format(culture, "坐标归一化   : {0}", Options.NormalizeCoordinates ? "已启用（Hartley）" : "未启用"));
        builder.AppendLine("---------------------------------------------");
        builder.AppendLine(string.Format(culture, "RMSE         : {0:F6} mm", RmseMm));
        builder.AppendLine(string.Format(culture, "平均误差     : {0:F6} mm", MeanErrorMm));
        builder.AppendLine(string.Format(culture, "最大误差     : {0:F6} mm", MaxErrorMm));
        builder.AppendLine(string.Format(culture, "条件数       : {0:E3}{1}", ConditionNumber, IsIllConditioned ? "  ⚠ 布局退化" : string.Empty));
        builder.AppendLine(string.Format(culture, "质量评级     : {0}{1}", Quality, IsAcceptable ? "  ✔ 可投产" : "  ✘ 需重新示教"));

        if (LeaveOneOut is not null)
        {
            builder.AppendLine(string.Format(
                culture,
                "留一验证     : 平均 {0:F6} mm / 最大 {1:F6} mm{2}",
                LeaveOneOut.MeanErrorMm,
                LeaveOneOut.MaxErrorMm,
                IsOverFitting ? "  ⚠ 疑似过拟合，建议改用仿射模型" : string.Empty));
        }

        builder.AppendLine("---------------------------------------------");
        builder.AppendLine("模型参数     : " + Model.DescribeParameters());

        if (Model is AffineTransform2D affine)
        {
            builder.AppendLine(string.Format(
                culture,
                "  像素当量   : {0:F4} / {1:F4} px/mm（即 {2:F6} / {3:F6} mm/px）",
                affine.PixelsPerMmU,
                affine.PixelsPerMmV,
                affine.ScaleMmPerPixelU,
                affine.ScaleMmPerPixelV));
            builder.AppendLine(string.Format(
                culture,
                "  相机安装角 : {0:F4}°{1}",
                affine.RotationDegrees,
                Math.Abs(affine.OrthogonalityError) > 0.02 ? "  ⚠ 正交性偏差偏大，检查相机是否倾斜" : string.Empty));
        }

        builder.AppendLine("---------------------------------------------");
        builder.AppendLine("逐点残差（mm）：");
        foreach (CalibrationResidual residual in Residuals)
        {
            builder.AppendLine(string.Format(
                culture,
                "  {0,-6} 像素 {1,-22} 实测 {2,-22} 预测 {3,-22} 误差 {4:F6}",
                residual.Label,
                residual.Image.ToString("F3"),
                residual.MachineMeasured.ToString("F4"),
                residual.MachinePredicted.ToString("F4"),
                residual.ErrorMagnitude));
        }

        builder.AppendLine("=============================================");
        return builder.ToString();
    }
}

/// <summary>留一交叉验证结果。</summary>
public sealed record CalibrationLeaveOneOutResult(
    IReadOnlyList<CalibrationResidual> Residuals,
    double MeanErrorMm,
    double MaxErrorMm)
{
    /// <summary>样本数（= 标定点数）。</summary>
    public int SampleCount => Residuals.Count;
}
