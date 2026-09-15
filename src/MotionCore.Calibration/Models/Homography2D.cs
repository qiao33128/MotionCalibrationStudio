using System.Globalization;
using MotionCore.Abstractions.Geometry;
using MotionCore.Calibration.LinearAlgebra;

namespace MotionCore.Calibration.Models;

/// <summary>
/// 单应变换模型（8 参数，含透视分量）。
/// <code>
/// [X']   [h0 h1 h2] [u]
/// [Y'] = [h3 h4 h5] [v]
/// [W ]   [h6 h7 h8] [1]
/// X = X'/W,  Y = Y'/W
/// </code>
/// <para>
/// 相比仿射多出 h6/h7 两个透视分量，能表达相机有安装倾角、
/// 或标定平面与运动平面不平行的情况。代价是参数更多、对噪声更敏感
/// （至少需要 4 个点，实际建议 ≥ 9 点），因此 <b>能用仿射就不要用单应</b>。
/// </para>
/// </summary>
public sealed class Homography2D : ICalibrationModel
{
    private readonly Matrix3x3 _matrix;

    public Homography2D(Matrix3x3 matrix)
    {
        if (!matrix.IsInvertible)
        {
            throw new ArgumentException("单应矩阵不可逆", nameof(matrix));
        }

        _matrix = matrix;
    }

    public string Name => "单应变换（8 参数，含透视）";

    public int ParameterCount => 8;

    public bool IsInvertible => true;

    public double Determinant => _matrix.Determinant;

    /// <summary>透视强度指标：h6/h7 越接近 0，模型越接近仿射。</summary>
    public double PerspectiveMagnitude => Math.Sqrt((_matrix[2, 0] * _matrix[2, 0]) + (_matrix[2, 1] * _matrix[2, 1]));

    public Point2D ImageToMachine(Point2D image) => _matrix.Transform(image);

    public Point2D MachineToImage(Point2D machine) => _matrix.Inverse().Transform(machine);

    public Matrix3x3 ToMatrix() => _matrix;

    public double[] ToCoefficients() => _matrix.ToRowMajor();

    public static Homography2D FromCoefficients(double[] coefficients)
    {
        if (coefficients.Length != 9)
        {
            throw new ArgumentException($"单应模型需要 9 个参数，实际传入 {coefficients.Length} 个", nameof(coefficients));
        }

        return new Homography2D(new Matrix3x3(
            coefficients[0], coefficients[1], coefficients[2],
            coefficients[3], coefficients[4], coefficients[5],
            coefficients[6], coefficients[7], coefficients[8]));
    }

    public string DescribeParameters() =>
        string.Format(
            CultureInfo.InvariantCulture,
            "行列式 = {0:E4}；透视强度 = {1:E4}（该值接近 0 说明实际就是仿射，建议改用仿射模型）",
            Determinant,
            PerspectiveMagnitude);

    public override string ToString() => $"{Name}: {DescribeParameters()}";
}
