namespace MotionCore.Abstractions.Motion;

/// <summary>
/// 轴的完整状态快照（只读值对象）。每帧从卡里读回来的原始数据都投影成它，
/// 上层永远不会直接接触厂商结构体。
/// </summary>
public sealed record AxisStatus
{
    public required AxisId Axis { get; init; }

    /// <summary>指令位置（卡内部规划的计数器值）。</summary>
    public required double CommandPosition { get; init; }

    /// <summary>实际位置（编码器反馈）。指令与实际的差值即跟随误差。</summary>
    public required double ActualPosition { get; init; }

    /// <summary>当前速度（单位/秒）。</summary>
    public required double Velocity { get; init; }

    public required bool ServoEnabled { get; init; }

    public required bool InPosition { get; init; }

    /// <summary>是否已回零（建立用户坐标系）。</summary>
    public bool Homed { get; init; }

    public bool PositiveLimit { get; init; }

    public bool NegativeLimit { get; init; }

    public bool HomeSwitch { get; init; }

    public bool Alarm { get; init; }

    public int AlarmCode { get; init; }

    public string? AlarmMessage { get; init; }

    /// <summary>跟随误差 = 指令位置 − 实际位置。</summary>
    public double FollowingError => CommandPosition - ActualPosition;

    /// <summary>是否可以安全地下发运动指令。</summary>
    public bool CanMove => ServoEnabled && !Alarm && !PositiveLimit && !NegativeLimit;

    public static AxisStatus Unknown(AxisId axis) => new()
    {
        Axis = axis,
        CommandPosition = double.NaN,
        ActualPosition = double.NaN,
        Velocity = 0,
        ServoEnabled = false,
        InPosition = false,
        Homed = false,
        Alarm = true,
        AlarmCode = -1,
        AlarmMessage = "状态不可用",
    };
}
