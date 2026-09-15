namespace MotionCore.Abstractions.Geometry;

/// <summary>
/// 二维点/向量。图像侧单位为像素（u 向右、v 向下），机械侧单位为毫米（X 向右、Y 向上）。
/// </summary>
public readonly record struct Point2D(double X, double Y)
{
    public static readonly Point2D Zero = new(0, 0);

    public double Length => Math.Sqrt(X * X + Y * Y);

    public double LengthSquared => (X * X) + (Y * Y);

    public bool IsFinite => double.IsFinite(X) && double.IsFinite(Y);

    public static Point2D operator +(Point2D a, Point2D b) => new(a.X + b.X, a.Y + b.Y);

    public static Point2D operator -(Point2D a, Point2D b) => new(a.X - b.X, a.Y - b.Y);

    public static Point2D operator -(Point2D a) => new(-a.X, -a.Y);

    public static Point2D operator *(Point2D a, double k) => new(a.X * k, a.Y * k);

    public static Point2D operator *(double k, Point2D a) => a * k;

    public static Point2D operator /(Point2D a, double k) => new(a.X / k, a.Y / k);

    public double DistanceTo(Point2D other) => (this - other).Length;

    /// <summary>绕原点逆时针旋转（弧度）。</summary>
    public Point2D Rotate(double radians)
    {
        double c = Math.Cos(radians);
        double s = Math.Sin(radians);
        return new Point2D((X * c) - (Y * s), (X * s) + (Y * c));
    }

    public Point2D WithX(double x) => new(x, Y);

    public Point2D WithY(double y) => new(X, y);

    /// <summary>平移。</summary>
    public Point2D Offset(double dx, double dy) => new(X + dx, Y + dy);

    public override string ToString() => $"({X:F4}, {Y:F4})";

    public string ToString(string format) =>
        $"({X.ToString(format, System.Globalization.CultureInfo.InvariantCulture)}, " +
        $"{Y.ToString(format, System.Globalization.CultureInfo.InvariantCulture)})";
}
