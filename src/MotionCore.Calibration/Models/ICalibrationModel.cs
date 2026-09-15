using MotionCore.Abstractions.Geometry;

namespace MotionCore.Calibration.Models;

/// <summary>标定模型种类。</summary>
public enum CalibrationModelKind
{
    /// <summary>仿射（6 参数）：平移 + 旋转 + 各向异性缩放 + 剪切。3C 固定相机对位首选。</summary>
    Affine = 0,

    /// <summary>单应（8 参数）：额外容纳透视，相机有倾角时使用。</summary>
    Homography = 1,

    /// <summary>二次多项式（12 参数）：容纳镜头径向畸变，视野大 / 畸变明显时使用。</summary>
    QuadraticPolynomial = 2,
}

/// <summary>
/// 像素坐标 ↔ 机械坐标 的映射模型。
/// <para>
/// 约定：<c>ImageToMachine</c> 输入像素 (u,v)，输出机械坐标 (X,Y)；
/// <c>MachineToImage</c> 为其逆映射。
/// </para>
/// </summary>
public interface ICalibrationModel
{
    string Name { get; }

    /// <summary>模型参数个数（用于判断需要的最少标定点数）。</summary>
    int ParameterCount { get; }

    bool IsInvertible { get; }

    /// <summary>像素 → 机械。</summary>
    Point2D ImageToMachine(Point2D image);

    /// <summary>机械 → 像素。</summary>
    Point2D MachineToImage(Point2D machine);

    /// <summary>参数展平（用于持久化与比对）。</summary>
    double[] ToCoefficients();

    /// <summary>人类可读的参数说明，直接进标定报告。</summary>
    string DescribeParameters();
}
