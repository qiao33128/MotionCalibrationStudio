using System.Globalization;
using MotionCore.App.Mvvm;
using MotionCore.Calibration;

namespace MotionCore.App.ViewModels;

/// <summary>标定点表格的一行（含残差与预测值，便于现场一眼看出哪个点采坏了）。</summary>
public sealed class CalibrationPointRowViewModel : ObservableObject
{
    private double _residualX;
    private double _residualY;
    private double _residualMagnitude;

    public CalibrationPointRowViewModel(CalibrationPoint point)
    {
        Label = point.Label;
        Image = point.Image;
        Machine = point.Machine;
        Predicted = point.Machine;
    }

    public string Label { get; }

    public MotionCore.Abstractions.Geometry.Point2D Image { get; }

    public MotionCore.Abstractions.Geometry.Point2D Machine { get; }

    public MotionCore.Abstractions.Geometry.Point2D Predicted { get; private set; }

    public double ImageU => Image.X;

    public double ImageV => Image.Y;

    public double MachineX => Machine.X;

    public double MachineY => Machine.Y;

    public double PredictedX => Predicted.X;

    public double PredictedY => Predicted.Y;

    public double ResidualX
    {
        get => _residualX;
        private set => SetProperty(ref _residualX, value);
    }

    public double ResidualY
    {
        get => _residualY;
        private set => SetProperty(ref _residualY, value);
    }

    public double ResidualMagnitude
    {
        get => _residualMagnitude;
        private set => SetProperty(ref _residualMagnitude, value);
    }

    /// <summary>
    /// 残差是否超限。
    /// <para>现场判据：单点残差超过 3 倍 RMSE，就应该怀疑这个点是"采坏了"（Mark 识别跳变、机械振动）。</para>
    /// </summary>
    public bool IsResidualOutlier { get; private set; }

    public void Apply(CalibrationResidual residual, double outlierThresholdMm)
    {
        Predicted = residual.MachinePredicted;
        ResidualX = residual.Error.X;
        ResidualY = residual.Error.Y;
        ResidualMagnitude = residual.ErrorMagnitude;
        IsResidualOutlier = ResidualMagnitude > outlierThresholdMm;

        OnPropertyChanged(nameof(PredictedX));
        OnPropertyChanged(nameof(PredictedY));
        OnPropertyChanged(nameof(IsResidualOutlier));
    }

    public string DescribeResidual() => string.Format(
        CultureInfo.InvariantCulture,
        "Δ = ({0:F4}, {1:F4}) mm，|Δ| = {2:F6} mm",
        ResidualX,
        ResidualY,
        ResidualMagnitude);
}
