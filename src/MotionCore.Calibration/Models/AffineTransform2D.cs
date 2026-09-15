using System.Globalization;
using MotionCore.Abstractions.Geometry;
using MotionCore.Calibration.LinearAlgebra;

namespace MotionCore.Calibration.Models;

/// <summary>
/// 仿射变换模型（6 参数）。
/// <code>
/// X = a·u + b·v + c
/// Y = d·u + e·v + f
/// </code>
/// <para>
/// 九点标定最常用的模型。它是"平行线映射后仍平行"的变换，
/// 能表达平移、旋转、各向异性缩放与剪切，覆盖了相机正常安装（无倾角、镜头畸变小）的全部情况。
/// 参数少 → 需要的标定点少 → 抗噪声能力最强，所以是首选而不是"最简单的那个"。
/// </para>
/// </summary>
public sealed class AffineTransform2D : ICalibrationModel
{
    public AffineTransform2D(double a, double b, double c, double d, double e, double f)
    {
        A = a;
        B = b;
        C = c;
        D = d;
        E = e;
        F = f;
    }

    public double A { get; }

    public double B { get; }

    public double C { get; }

    public double D { get; }

    public double E { get; }

    public double F { get; }

    public string Name => "仿射变换（6 参数）";

    public int ParameterCount => 6;

    public bool IsInvertible => Math.Abs(Determinant) > 1e-14;

    public double Determinant => (A * E) - (B * D);

    /// <summary>
    /// 从"像素→机械"反推"机械→像素"的线性部分。
    /// <para>
    /// <b>为什么要绕这一圈：</b> 相机随平台移动时，物体在图像中的移动方向与平台相反，
    /// 所以"机械→像素"的线性映射 M 与仿射矩阵的逆 P 之间差一个负号（M = −P）。
    /// 如果不做这个符号修正，直接对仿射矩阵取 atan2 算安装角，会得到 ≈180° 的结果 ——
    /// 这个数看着像"装反了"，实际只是坐标约定差了一个反射。
    /// </para>
    /// </summary>
    private (double M00, double M01, double M10, double M11) MachineToPixelLinear()
    {
        if (!IsInvertible)
        {
            return (double.NaN, double.NaN, double.NaN, double.NaN);
        }

        double det = Determinant;

        // P = 仿射矩阵的逆（像素/机械量的映射），再取负号得到真正的"机械→像素"
        double p00 = E / det;
        double p01 = -B / det;
        double p10 = -D / det;
        double p11 = A / det;

        return (-p00, -p01, -p10, -p11);
    }

    /// <summary>图像 u 方向的像素当量（px / mm）。</summary>
    public double PixelsPerMmU
    {
        get
        {
            (double m00, _, double m10, _) = MachineToPixelLinear();
            return Math.Sqrt((m00 * m00) + (m10 * m10));
        }
    }

    /// <summary>图像 v 方向的像素当量（px / mm）。</summary>
    public double PixelsPerMmV
    {
        get
        {
            (_, double m01, _, double m11) = MachineToPixelLinear();
            return Math.Sqrt((m01 * m01) + (m11 * m11));
        }
    }

    /// <summary>机械 X 方向的分辨率（mm / px）。</summary>
    public double ScaleMmPerPixelU => PixelsPerMmU > 0 ? 1d / PixelsPerMmU : double.NaN;

    /// <summary>机械 Y 方向的分辨率（mm / px）。</summary>
    public double ScaleMmPerPixelV => PixelsPerMmV > 0 ? 1d / PixelsPerMmV : double.NaN;

    /// <summary>相机安装角（弧度）：机械 X 轴在图像中相对 u 轴的夹角。</summary>
    public double RotationRadians
    {
        get
        {
            (double m00, _, double m10, _) = MachineToPixelLinear();
            return Math.Atan2(m10, m00);
        }
    }

    /// <summary>相机安装角（度）。</summary>
    public double RotationDegrees => RotationRadians * 180d / Math.PI;

    /// <summary>
    /// 正交性偏差：理想安装下两个基向量互相垂直，该值为 0。
    /// 明显偏离 0 说明相机相对运动平面有倾角（应改用单应模型）或镜头畸变较大。
    /// </summary>
    public double OrthogonalityError
    {
        get
        {
            (double m00, double m01, double m10, double m11) = MachineToPixelLinear();
            double denominator = PixelsPerMmU * PixelsPerMmV;
            return denominator > 0 ? (((m00 * m01) + (m10 * m11)) / denominator) : double.NaN;
        }
    }

    /// <summary>是否为"镜像"映射（由图像 v 轴向下导致，正常情况恒为 true）。</summary>
    public bool IsReflected => Determinant < 0;

    public Point2D ImageToMachine(Point2D image) =>
        new((A * image.X) + (B * image.Y) + C, (D * image.X) + (E * image.Y) + F);

    public Point2D MachineToImage(Point2D machine)
    {
        if (!IsInvertible)
        {
            throw new InvalidOperationException("仿射矩阵不可逆，无法反算");
        }

        double det = Determinant;
        double x = machine.X - C;
        double y = machine.Y - F;

        return new Point2D(
            ((E * x) - (B * y)) / det,
            ((A * y) - (D * x)) / det);
    }

    public Matrix3x3 ToMatrix() => Matrix3x3.Affine(A, B, C, D, E, F);

    public static AffineTransform2D FromMatrix(Matrix3x3 matrix) =>
        new(matrix[0, 0], matrix[0, 1], matrix[0, 2], matrix[1, 0], matrix[1, 1], matrix[1, 2]);

    public double[] ToCoefficients() => [A, B, C, D, E, F];

    public static AffineTransform2D FromCoefficients(double[] coefficients)
    {
        if (coefficients.Length != 6)
        {
            throw new ArgumentException($"仿射模型需要 6 个参数，实际传入 {coefficients.Length} 个", nameof(coefficients));
        }

        return new AffineTransform2D(
            coefficients[0], coefficients[1], coefficients[2],
            coefficients[3], coefficients[4], coefficients[5]);
    }

    public string DescribeParameters() =>
        string.Format(
            CultureInfo.InvariantCulture,
            "像素当量 = {0:F4} / {1:F4} px/mm（即 {2:F6} / {3:F6} mm/px）；安装角 = {4:F4}°；平移 = ({5:F4}, {6:F4}) mm；正交性偏差 = {7:E3}",
            PixelsPerMmU,
            PixelsPerMmV,
            ScaleMmPerPixelU,
            ScaleMmPerPixelV,
            RotationDegrees,
            C,
            F,
            OrthogonalityError);

    public override string ToString() => $"{Name}: {DescribeParameters()}";
}
