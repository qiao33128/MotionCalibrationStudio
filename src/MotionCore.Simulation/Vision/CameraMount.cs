using MotionCore.Abstractions.Geometry;

namespace MotionCore.Simulation.Vision;

/// <summary>
/// <b>相机安装模型（相机随平台移动、Mark 固定不动）</b>
/// <para>
/// 这是九点标定要反解的目标：
/// <code>
/// 像素偏移 = M · (Mark 位置 − 平台位置)
/// </code>
/// 其中 M 由三个物理量决定：像素当量（px/mm）、相机安装角、图像 v 轴方向（是否翻转）。
/// </para>
/// <para>
/// 注意 <b>翻转</b>这一项：图像的 v 轴向下、机械的 Y 轴向上，
/// 所以两者天然差一个反射。如果实现时忘了这个符号，标定出来的安装角会差一个"镜像"，
/// 现象就是"X 方向对了、Y 方向反了"—— 这是现场最常见的低级但是致命的问题。
/// </para>
/// </summary>
public sealed record CameraMount
{
    /// <summary>图像 u 方向的像素当量（px/mm）。</summary>
    public double PixelsPerMmU { get; init; } = 24d;

    /// <summary>图像 v 方向的像素当量（px/mm）。</summary>
    public double PixelsPerMmV { get; init; } = 24d;

    /// <summary>相机安装角（度）。</summary>
    public double RotationDeg { get; init; }

    /// <summary>图像 v 轴是否向下（真实相机几乎总是 true）。</summary>
    public bool FlipVertical { get; init; } = true;

    public double RotationRadians => RotationDeg * Math.PI / 180d;

    /// <summary>平均像素当量（mm/px），用于估算视野。</summary>
    public double MmPerPixel => 2d / (PixelsPerMmU + PixelsPerMmV);

    /// <summary>返回 2×2 线性映射 M。</summary>
    public (double M00, double M01, double M10, double M11) LinearMap()
    {
        double cosine = Math.Cos(RotationRadians);
        double sine = Math.Sin(RotationRadians);
        double sign = FlipVertical ? 1d : -1d;

        return (
            PixelsPerMmU * cosine,
            PixelsPerMmU * sine,
            sign * PixelsPerMmV * sine,
            -sign * PixelsPerMmV * cosine);
    }

    /// <summary>机械相对位移 → 像素偏移。</summary>
    public Point2D MachineToPixelOffset(Point2D relativeMachine)
    {
        (double m00, double m01, double m10, double m11) = LinearMap();
        return new Point2D(
            (m00 * relativeMachine.X) + (m01 * relativeMachine.Y),
            (m10 * relativeMachine.X) + (m11 * relativeMachine.Y));
    }

    /// <summary>像素偏移 → 机械相对位移。</summary>
    public Point2D PixelToMachineOffset(Point2D pixelOffset)
    {
        (double m00, double m01, double m10, double m11) = LinearMap();
        double determinant = (m00 * m11) - (m01 * m10);
        if (Math.Abs(determinant) < 1e-300)
        {
            throw new InvalidOperationException("相机安装模型退化，无法反解");
        }

        return new Point2D(
            ((m11 * pixelOffset.X) - (m01 * pixelOffset.Y)) / determinant,
            ((m00 * pixelOffset.Y) - (m10 * pixelOffset.X)) / determinant);
    }

    public override string ToString() =>
        $"像素当量 {PixelsPerMmU:F2}/{PixelsPerMmV:F2} px/mm，安装角 {RotationDeg:F3}°，v 轴{(FlipVertical ? "向下" : "向上")}";
}
