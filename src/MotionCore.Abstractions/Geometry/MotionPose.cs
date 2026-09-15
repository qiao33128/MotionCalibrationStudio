using System.Globalization;
using MotionCore.Abstractions.Motion;

namespace MotionCore.Abstractions.Geometry;

/// <summary>
/// 四轴位姿。左上角是设备软件里最容易被写坏的数据结构之一：
/// 一旦到处用 double[] 或 4 个独立字段，语义就会在层与层之间丢失。
/// </summary>
public readonly record struct MotionPose(double X, double Y, double Z = 0, double R = 0)
{
    public static readonly MotionPose Zero = new(0, 0, 0, 0);

    public Point2D Planar => new(X, Y);

    public double this[AxisId axis] => axis switch
    {
        AxisId.X => X,
        AxisId.Y => Y,
        AxisId.Z => Z,
        AxisId.R => R,
        _ => throw new ArgumentOutOfRangeException(nameof(axis), axis, "未定义的轴号"),
    };

    public MotionPose With(AxisId axis, double value) => axis switch
    {
        AxisId.X => this with { X = value },
        AxisId.Y => this with { Y = value },
        AxisId.Z => this with { Z = value },
        AxisId.R => this with { R = value },
        _ => throw new ArgumentOutOfRangeException(nameof(axis), axis, "未定义的轴号"),
    };

    public bool HasAxis(AxisId axis) => axis is AxisId.X or AxisId.Y or AxisId.Z or AxisId.R;

    public static MotionPose FromPlanar(Point2D point, double z = 0, double r = 0) => new(point.X, point.Y, z, r);

    public MotionPose Offset(double dx, double dy) => this with { X = X + dx, Y = Y + dy };

    public override string ToString() =>
        string.Format(
            CultureInfo.InvariantCulture,
            "X={0:F4} Y={1:F4} Z={2:F4} R={3:F4}",
            X, Y, Z, R);
}
