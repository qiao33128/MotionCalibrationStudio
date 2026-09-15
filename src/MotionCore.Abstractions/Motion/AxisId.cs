namespace MotionCore.Abstractions.Motion;

/// <summary>
/// 标准轴号。真实设备上不同厂商的轴号编号不同，统一在这里做一层映射，
/// 上层业务只认 AxisId，不认厂商的 0/1/2/3 或 "0,1" 字符串写法。
/// </summary>
public enum AxisId
{
    /// <summary>X 轴（水平方向）。</summary>
    X = 0,

    /// <summary>Y 轴（竖直方向）。</summary>
    Y = 1,

    /// <summary>Z 轴（升降方向）。</summary>
    Z = 2,

    /// <summary>R 轴（旋转 / 角度）。</summary>
    R = 3,
}

/// <summary>轴号扩展。</summary>
public static class AxisIdExtensions
{
    public static string ToShortName(this AxisId axis) => axis switch
    {
        AxisId.X => "X",
        AxisId.Y => "Y",
        AxisId.Z => "Z",
        AxisId.R => "R",
        _ => axis.ToString(),
    };

    /// <summary>是否为直线轴（用于判断是否参与平面插补）。</summary>
    public static bool IsLinear(this AxisId axis) => axis is AxisId.X or AxisId.Y or AxisId.Z;

    /// <summary>该轴的单位。</summary>
    public static string Unit(this AxisId axis) => axis == AxisId.R ? "°" : "mm";
}
