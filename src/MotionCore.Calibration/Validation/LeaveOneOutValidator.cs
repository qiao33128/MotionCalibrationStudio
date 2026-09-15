using MotionCore.Abstractions.Geometry;

namespace MotionCore.Calibration.Validation;

/// <summary>
/// 留一交叉验证（LOOCV）。
/// <para>
/// <b>为什么标定一定要做这个：</b> RMSE 只反映"模型对训练点的拟合程度"，
/// 参数越多的模型在训练点上一定拟合得越好 —— 单看 RMSE，
/// 二次多项式永远赢过仿射，但那很可能只是把视觉噪声也拟合进去了。
/// </para>
/// <para>
/// LOOCV 每次留出一个点不参与解算，用它检验模型对<b>没见过的点</b>的预测能力。
/// 只有 LOOCV 也好的模型才是真的好。工程上的判据很直接：
/// 如果 LOOCV 误差达到训练 RMSE 的 3 倍以上，就是过拟合，应该降级用仿射。
/// </para>
/// </summary>
public static class LeaveOneOutValidator
{
    public static CalibrationLeaveOneOutResult Validate(CalibrationPointSet pointSet, CalibrationOptions options)
    {
        ArgumentNullException.ThrowIfNull(pointSet);
        ArgumentNullException.ThrowIfNull(options);

        int minimum = options.EffectiveMinimumPointCount;
        CalibrationOptions innerOptions = options.WithLeaveOneOutValidation(false);

        List<CalibrationResidual> residuals = new();
        double sum = 0d;
        double max = 0d;

        for (int index = 0; index < pointSet.Count; index++)
        {
            CalibrationPointSet subset = pointSet.Without(index);
            if (subset.Count < minimum)
            {
                continue;
            }

            CalibrationResult partial;
            try
            {
                partial = NinePointCalibrator.Calibrate(subset, innerOptions);
            }
            catch (Exception ex) when (ex is InvalidOperationException or ArgumentException)
            {
                // 留一后剩余的 8 个点不足以解算（例如全部共线），跳过该样本
                continue;
            }

            CalibrationPoint heldOut = pointSet.Points[index];
            Point2D predicted = partial.Model.ImageToMachine(heldOut.Image);

            CalibrationResidual residual = new(
                $"{heldOut.Label}·留一",
                heldOut.Image,
                heldOut.Machine,
                predicted);

            residuals.Add(residual);
            sum += residual.ErrorMagnitude;
            max = Math.Max(max, residual.ErrorMagnitude);
        }

        double mean = residuals.Count > 0 ? sum / residuals.Count : 0d;

        return new CalibrationLeaveOneOutResult(residuals, mean, max);
    }
}
