using System.Globalization;
using MotionCore.Abstractions.Geometry;

namespace MotionCore.Calibration.LinearAlgebra;

/// <summary>
/// 3×3 齐次变换矩阵。仿射与单应共用它做"归一化坐标 → 原始坐标"的还原，
/// 也是机器人/视觉里最常用的最小线性代数单元。
/// </summary>
public readonly struct Matrix3x3
{
    private readonly double _m00, _m01, _m02;
    private readonly double _m10, _m11, _m12;
    private readonly double _m20, _m21, _m22;

    public Matrix3x3(
        double m00, double m01, double m02,
        double m10, double m11, double m12,
        double m20, double m21, double m22)
    {
        _m00 = m00;
        _m01 = m01;
        _m02 = m02;
        _m10 = m10;
        _m11 = m11;
        _m12 = m12;
        _m20 = m20;
        _m21 = m21;
        _m22 = m22;
    }

    public static Matrix3x3 Identity => new(1, 0, 0, 0, 1, 0, 0, 0, 1);

    /// <summary>构造仿射矩阵：第三行固定为 (0, 0, 1)。</summary>
    public static Matrix3x3 Affine(double m00, double m01, double m02, double m10, double m11, double m12) =>
        new(m00, m01, m02, m10, m11, m12, 0, 0, 1);

    /// <summary>构造归一化矩阵 T = [[s, 0, -s·cx], [0, s, -s·cy], [0, 0, 1]]（Hartley 归一化）。</summary>
    public static Matrix3x3 Normalization(double scale, Point2D centroid) =>
        new(scale, 0, -scale * centroid.X, 0, scale, -scale * centroid.Y, 0, 0, 1);

    public double this[int row, int column] => (row, column) switch
    {
        (0, 0) => _m00, (0, 1) => _m01, (0, 2) => _m02,
        (1, 0) => _m10, (1, 1) => _m11, (1, 2) => _m12,
        (2, 0) => _m20, (2, 1) => _m21, (2, 2) => _m22,
        _ => throw new ArgumentOutOfRangeException(nameof(row), $"索引越界 ({row},{column})"),
    };

    public double Determinant =>
        (_m00 * ((_m11 * _m22) - (_m12 * _m21)))
        - (_m01 * ((_m10 * _m22) - (_m12 * _m20)))
        + (_m02 * ((_m10 * _m21) - (_m11 * _m20)));

    public bool IsInvertible => Math.Abs(Determinant) > 1e-14;

    public static Matrix3x3 operator *(Matrix3x3 a, Matrix3x3 b) => new(
        (a._m00 * b._m00) + (a._m01 * b._m10) + (a._m02 * b._m20),
        (a._m00 * b._m01) + (a._m01 * b._m11) + (a._m02 * b._m21),
        (a._m00 * b._m02) + (a._m01 * b._m12) + (a._m02 * b._m22),
        (a._m10 * b._m00) + (a._m11 * b._m10) + (a._m12 * b._m20),
        (a._m10 * b._m01) + (a._m11 * b._m11) + (a._m12 * b._m21),
        (a._m10 * b._m02) + (a._m11 * b._m12) + (a._m12 * b._m22),
        (a._m20 * b._m00) + (a._m21 * b._m10) + (a._m22 * b._m20),
        (a._m20 * b._m01) + (a._m21 * b._m11) + (a._m22 * b._m21),
        (a._m20 * b._m02) + (a._m21 * b._m12) + (a._m22 * b._m22));

    /// <summary>伴随矩阵法求逆（3×3 有闭式解，比通用高斯消元更快也更稳）。</summary>
    public Matrix3x3 Inverse()
    {
        double det = Determinant;
        if (Math.Abs(det) < 1e-14)
        {
            throw new InvalidOperationException("矩阵不可逆（行列式接近 0，说明标定点退化或模型参数异常）");
        }

        double inv = 1d / det;
        return new Matrix3x3(
            ((_m11 * _m22) - (_m12 * _m21)) * inv,
            ((_m02 * _m21) - (_m01 * _m22)) * inv,
            ((_m01 * _m12) - (_m02 * _m11)) * inv,
            ((_m12 * _m20) - (_m10 * _m22)) * inv,
            ((_m00 * _m22) - (_m02 * _m20)) * inv,
            ((_m02 * _m10) - (_m00 * _m12)) * inv,
            ((_m10 * _m21) - (_m11 * _m20)) * inv,
            ((_m01 * _m20) - (_m00 * _m21)) * inv,
            ((_m00 * _m11) - (_m01 * _m10)) * inv);
    }

    /// <summary>齐次变换：(w·x, w·y, w) → (x, y)。</summary>
    public Point2D Transform(Point2D point)
    {
        double w = (_m20 * point.X) + (_m21 * point.Y) + _m22;
        if (Math.Abs(w) < 1e-14)
        {
            throw new InvalidOperationException("齐次变换出现 w = 0，点位于无穷远（单应矩阵参数异常）");
        }

        return new Point2D(
            ((_m00 * point.X) + (_m01 * point.Y) + _m02) / w,
            ((_m10 * point.X) + (_m11 * point.Y) + _m12) / w);
    }

    public double[] ToRowMajor() =>
    [
        _m00, _m01, _m02,
        _m10, _m11, _m12,
        _m20, _m21, _m22,
    ];

    public override string ToString() =>
        string.Format(
            CultureInfo.InvariantCulture,
            "[{0:F6} {1:F6} {2:F6}; {3:F6} {4:F6} {5:F6}; {6:F6} {7:F6} {8:F6}]",
            _m00, _m01, _m02, _m10, _m11, _m12, _m20, _m21, _m22);
}
