using System.Globalization;
using System.Windows;
using System.Windows.Media;
using MotionCore.Abstractions.Geometry;
using MotionCore.Calibration;

namespace MotionCore.App.Controls;

/// <summary>
/// 标定结果可视化。
/// <para>
/// 9 点标定做完如果只显示一个 RMSE 数字，现场是没法判断问题出在哪的。
/// 这块画布把 <b>实测点、模型预测点、残差向量</b> 同时画出来：
/// </para>
/// <list type="bullet">
///   <item>残差向量整体朝同一个方向 → 系统偏差（标定参数有偏、机构反向间隙）；</item>
///   <item>某个点单独偏得特别远 → 该点采集异常（Mark 识别跳变、机械振动、光照突变）；</item>
///   <item>残差呈环形分布 → 镜头畸变没被模型吸收，应改用电/二次多项式模型。</item>
/// </list>
/// <para>
/// 残差很小（亚微米级）时按原比例根本看不见，因此默认放大 200 倍显示，并在图例中明确标注放大倍数 ——
/// <b>可视化可以放大，但标注必须诚实</b>。
/// </para>
/// </summary>
public sealed class CalibrationCanvas : FrameworkElement
{
    public static readonly DependencyProperty ResultProperty = DependencyProperty.Register(
        nameof(Result),
        typeof(CalibrationResult),
        typeof(CalibrationCanvas),
        new FrameworkPropertyMetadata(null, FrameworkPropertyMetadataOptions.AffectsRender));

    /// <summary>残差放大倍数。</summary>
    public static readonly DependencyProperty ErrorAmplificationProperty = DependencyProperty.Register(
        nameof(ErrorAmplification),
        typeof(double),
        typeof(CalibrationCanvas),
        new FrameworkPropertyMetadata(200d, FrameworkPropertyMetadataOptions.AffectsRender));

    public CalibrationResult? Result
    {
        get => (CalibrationResult?)GetValue(ResultProperty);
        set => SetValue(ResultProperty, value);
    }

    public double ErrorAmplification
    {
        get => (double)GetValue(ErrorAmplificationProperty);
        set => SetValue(ErrorAmplificationProperty, value);
    }

    private static readonly Brush Background = new SolidColorBrush(Color.FromRgb(0x0D, 0x15, 0x1E));
    private static readonly Brush GridBrush = new SolidColorBrush(Color.FromRgb(0x1E, 0x2C, 0x3A));
    private static readonly Brush AxisBrush = new SolidColorBrush(Color.FromRgb(0x35, 0x4A, 0x5E));
    private static readonly Brush MeasuredBrush = new SolidColorBrush(Color.FromRgb(0x4C, 0xC9, 0x7A));
    private static readonly Brush PredictedBrush = new SolidColorBrush(Color.FromRgb(0xE8, 0xC3, 0x4A));
    private static readonly Brush ErrorBrush = new SolidColorBrush(Color.FromRgb(0xFF, 0x5A, 0x5A));
    private static readonly Brush TextBrush = new SolidColorBrush(Color.FromRgb(0x9F, 0xB4, 0xC8));
    private static readonly Brush TitleBrush = new SolidColorBrush(Color.FromRgb(0xE4, 0xEE, 0xF8));

    private static readonly Typeface Face = new("Consolas");

    static CalibrationCanvas()
    {
        Background.Freeze();
        GridBrush.Freeze();
        AxisBrush.Freeze();
        MeasuredBrush.Freeze();
        PredictedBrush.Freeze();
        ErrorBrush.Freeze();
        TextBrush.Freeze();
        TitleBrush.Freeze();
    }

    protected override void OnRender(DrawingContext dc)
    {
        double width = ActualWidth;
        double height = ActualHeight;

        if (width < 20 || height < 20)
        {
            return;
        }

        dc.DrawRectangle(Background, null, new Rect(0, 0, width, height));

        CalibrationResult? result = Result;
        if (result is null || result.Residuals.Count == 0)
        {
            DrawText(dc, "尚未标定 —— 完成九点示教后这里会显示实测点、预测点与残差向量", 16, 18, TextBrush, 13);
            return;
        }

        const double margin = 46d;
        double plotWidth = width - (2 * margin);
        double plotHeight = height - (2 * margin);

        if (plotWidth < 40 || plotHeight < 40)
        {
            return;
        }

        double minX = result.Residuals.Min(item => item.MachineMeasured.X);
        double maxX = result.Residuals.Max(item => item.MachineMeasured.X);
        double minY = result.Residuals.Min(item => item.MachineMeasured.Y);
        double maxY = result.Residuals.Max(item => item.MachineMeasured.Y);

        double spanX = Math.Max(maxX - minX, 1e-6);
        double spanY = Math.Max(maxY - minY, 1e-6);
        double paddingX = spanX * 0.18d;
        double paddingY = spanY * 0.18d;

        minX -= paddingX;
        maxX += paddingX;
        minY -= paddingY;
        maxY += paddingY;
        spanX = maxX - minX;
        spanY = maxY - minY;

        Point ToScreen(Point2D machine) => new(
            margin + ((machine.X - minX) / spanX * plotWidth),
            height - margin - ((machine.Y - minY) / spanY * plotHeight));

        // 参考网格
        for (int i = 0; i <= 4; i++)
        {
            double ratio = i / 4d;
            double x = margin + (ratio * plotWidth);
            double y = margin + (ratio * plotHeight);
            dc.DrawLine(new Pen(GridBrush, 1d), new Point(x, margin), new Point(x, height - margin));
            dc.DrawLine(new Pen(GridBrush, 1d), new Point(margin, y), new Point(width - margin, y));

            double machineX = minX + (ratio * spanX);
            double machineY = maxY - (ratio * spanY);
            DrawText(
                dc,
                machineX.ToString("F2", CultureInfo.InvariantCulture),
                x - 14,
                (height - margin) + 6,
                TextBrush,
                10);
            DrawText(
                dc,
                machineY.ToString("F2", CultureInfo.InvariantCulture),
                margin - 40,
                y - 7,
                TextBrush,
                10);
        }

        dc.DrawRectangle(null, new Pen(AxisBrush, 1.2d), new Rect(margin, margin, plotWidth, plotHeight));

        // 残差向量（放大显示）
        Pen errorPen = new(ErrorBrush, 1.6d);
        foreach (CalibrationResidual residual in result.Residuals)
        {
            Point from = ToScreen(residual.MachineMeasured);
            Point2D amplified = residual.MachineMeasured + (residual.Error * ErrorAmplification);
            Point to = ToScreen(amplified);

            dc.DrawLine(errorPen, from, to);
            dc.DrawEllipse(ErrorBrush, null, to, 2.4d, 2.4d);
        }

        // 预测点（空心）与实测点（实心）
        foreach (CalibrationResidual residual in result.Residuals)
        {
            Point predicted = ToScreen(residual.MachinePredicted);
            Point measured = ToScreen(residual.MachineMeasured);

            dc.DrawEllipse(null, new Pen(PredictedBrush, 1.4d), predicted, 6d, 6d);
            dc.DrawEllipse(MeasuredBrush, null, measured, 4d, 4d);

            DrawText(dc, residual.Label, measured.X + 7, measured.Y - 18, TitleBrush, 11);
        }

        // 图例与指标
        double legendY = 14d;
        DrawText(dc, "● 实测位置", 14, legendY, MeasuredBrush, 12);
        DrawText(dc, "○ 模型预测", 96, legendY, PredictedBrush, 12);
        DrawText(dc, $"―― 残差（放大 {ErrorAmplification:F0} 倍）", 182, legendY, ErrorBrush, 12);

        DrawText(
            dc,
            $"模型 {result.Kind}｜RMSE {result.RmseMm:F6} mm｜最大 {result.MaxErrorMm:F6} mm｜"
            + $"条件数 {result.ConditionNumber:E2}｜质量 {result.Quality}",
            14,
            height - 22,
            TitleBrush,
            12);
    }

    private void DrawText(DrawingContext dc, string text, double x, double y, Brush brush, double size)
    {
        FormattedText formatted = new(
            text,
            CultureInfo.CurrentCulture,
            FlowDirection.LeftToRight,
            Face,
            size,
            brush,
            VisualTreeHelper.GetDpi(this).PixelsPerDip);

        dc.DrawText(formatted, new Point(x, y));
    }
}
