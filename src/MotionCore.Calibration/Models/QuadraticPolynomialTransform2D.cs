using System.Globalization;
using MotionCore.Abstractions.Geometry;

namespace MotionCore.Calibration.Models;

/// <summary>
/// 二次多项式模型（12 参数）。
/// <code>
/// X = a0 + a1·u + a2·v + a3·u·v + a4·u² + a5·v²
/// Y = b0 + b1·u + b2·v + b3·u·v + b4·u² + b5·v²
/// </code>
/// <para>
/// 二次项用来吸收镜头的径向/切向畸变。视野大（&gt; 30mm）或镜头畸变明显时，
/// 它比仿射准；反过来在视野小、畸变可忽略的场合，多出来的 6 个参数只是
/// 在拟合噪声 —— <b>所以它必须和仿射一起用留一交叉验证比较后再选</b>，
/// 而不是"参数多就更准"。
/// </para>
/// </summary>
public sealed class QuadraticPolynomialTransform2D : ICalibrationModel
{
    private const int MaxNewtonIterations = 40;
    private const double NewtonTolerance = 1e-12;

    private readonly double[] _x; // a0..a5
    private readonly double[] _y; // b0..b5

    public QuadraticPolynomialTransform2D(double[] xCoefficients, double[] yCoefficients)
    {
        if (xCoefficients.Length != 6 || yCoefficients.Length != 6)
        {
            throw new ArgumentException("二次多项式模型需要 X/Y 各 6 个系数", nameof(xCoefficients));
        }

        _x = (double[])xCoefficients.Clone();
        _y = (double[])yCoefficients.Clone();
    }

    public string Name => "二次多项式（12 参数，含畸变补偿）";

    public int ParameterCount => 12;

    /// <summary>多项式映射不是解析可逆的，逆映射由牛顿迭代求解，因此恒为可用。</summary>
    public bool IsInvertible => true;

    /// <summary>二次项能量，用于判断畸变补偿是否"真的在干活"。</summary>
    public double DistortionEnergy =>
        Math.Sqrt((_x[3] * _x[3]) + (_x[4] * _x[4]) + (_x[5] * _x[5])
                  + (_y[3] * _y[3]) + (_y[4] * _y[4]) + (_y[5] * _y[5]));

    public Point2D ImageToMachine(Point2D image)
    {
        double u = image.X;
        double v = image.Y;
        double uv = u * v;
        double uu = u * u;
        double vv = v * v;

        return new Point2D(
            _x[0] + (_x[1] * u) + (_x[2] * v) + (_x[3] * uv) + (_x[4] * uu) + (_x[5] * vv),
            _y[0] + (_y[1] * u) + (_y[2] * v) + (_y[3] * uv) + (_y[4] * uu) + (_y[5] * vv));
    }

    /// <summary>
    /// 机械 → 像素：牛顿-拉夫逊迭代。
    /// 初值取"线性部分"的解 —— 二次项在大视野下相对线性项是小量，
    /// 因此这个初值几乎总在收敛域内，通常 3~5 次迭代就到达 1e-12。
    /// </summary>
    public Point2D MachineToImage(Point2D machine)
    {
        double linearDet = (_x[1] * _y[2]) - (_x[2] * _y[1]);
        if (Math.Abs(linearDet) < 1e-300)
        {
            throw new InvalidOperationException("二次多项式模型的线性部分退化，无法反算像素坐标");
        }

        double bx = machine.X - _x[0];
        double by = machine.Y - _y[0];

        double u = ((_y[2] * bx) - (_x[2] * by)) / linearDet;
        double v = ((_x[1] * by) - (_y[1] * bx)) / linearDet;

        for (int iteration = 0; iteration < MaxNewtonIterations; iteration++)
        {
            double fx = _x[0] + (_x[1] * u) + (_x[2] * v) + (_x[3] * u * v) + (_x[4] * u * u) + (_x[5] * v * v) - machine.X;
            double fy = _y[0] + (_y[1] * u) + (_y[2] * v) + (_y[3] * u * v) + (_y[4] * u * u) + (_y[5] * v * v) - machine.Y;

            if (Math.Abs(fx) < NewtonTolerance && Math.Abs(fy) < NewtonTolerance)
            {
                return new Point2D(u, v);
            }

            // 雅可比矩阵
            double j00 = _x[1] + (_x[3] * v) + (2d * _x[4] * u);
            double j01 = _x[2] + (_x[3] * u) + (2d * _x[5] * v);
            double j10 = _y[1] + (_y[3] * v) + (2d * _y[4] * u);
            double j11 = _y[2] + (_y[3] * u) + (2d * _y[5] * v);

            double det = (j00 * j11) - (j01 * j10);
            if (Math.Abs(det) < 1e-300)
            {
                throw new InvalidOperationException("牛顿迭代中雅可比矩阵奇异，无法反算像素坐标");
            }

            double du = ((j11 * fx) - (j01 * fy)) / det;
            double dv = ((j00 * fy) - (j10 * fx)) / det;

            u -= du;
            v -= dv;
        }

        throw new InvalidOperationException(
            $"二次多项式反算在 {MaxNewtonIterations} 次迭代内未收敛，标定参数可能严重失真");
    }

    public double[] ToCoefficients() => [.. _x, .. _y];

    public static QuadraticPolynomialTransform2D FromCoefficients(double[] coefficients)
    {
        if (coefficients.Length != 12)
        {
            throw new ArgumentException($"二次多项式模型需要 12 个参数，实际传入 {coefficients.Length} 个", nameof(coefficients));
        }

        return new QuadraticPolynomialTransform2D(coefficients[..6], coefficients[6..12]);
    }

    public string DescribeParameters() =>
        string.Format(
            CultureInfo.InvariantCulture,
            "线性项 ≈ ({0:F6}, {1:F6}) / ({2:F6}, {3:F6})；二次项能量 = {4:E3}",
            _x[1], _x[2], _y[1], _y[2], DistortionEnergy);

    public override string ToString() => $"{Name}: {DescribeParameters()}";
}
