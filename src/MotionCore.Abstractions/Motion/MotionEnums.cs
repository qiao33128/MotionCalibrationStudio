namespace MotionCore.Abstractions.Motion;

/// <summary>
/// 控制器状态机。上位机所有动作都必须先看状态，这是设备软件最常见的踩坑点
/// （在没使能 / 报警未复位时下发运动指令 → 卡死或飞车）。
/// </summary>
public enum CardState
{
    /// <summary>未连接。</summary>
    Disconnected = 0,

    /// <summary>正在初始化（连接、配置参数、使能）。</summary>
    Initializing = 1,

    /// <summary>就绪，可下发运动指令。</summary>
    Ready = 2,

    /// <summary>运动进行中。</summary>
    Moving = 3,

    /// <summary>暂停保持（暂停后位置保持，可继续）。</summary>
    Hold = 4,

    /// <summary>报警，需复位。</summary>
    Alarm = 5,

    /// <summary>急停。必须重新初始化才能恢复。</summary>
    EmergencyStop = 6,

    /// <summary>已释放。</summary>
    Disposed = 7,
}

/// <summary>运动方向。</summary>
public enum MotionDirection
{
    Negative = -1,

    Positive = 1,
}

/// <summary>回零方式（对应各家卡的 homing mode）。</summary>
public enum HomeMode
{
    /// <summary>负限位 / 原点开关回零（3C 设备最常见）。</summary>
    NegativeLimit = 0,

    /// <summary>正限位回零。</summary>
    PositiveLimit = 1,

    /// <summary>原点开关 + 索引信号（Z 相）回零，精度最高。</summary>
    HomeSwitchWithIndex = 2,

    /// <summary>当前位置直接置零（无原点开关时使用，慎用）。</summary>
    CurrentPosition = 3,
}

/// <summary>轴的运动能力配置。</summary>
public sealed record MoveProfile(
    double Velocity,
    double Acceleration,
    double Deceleration,
    double? SmoothTime = null)
{
    /// <summary>常用默认：20 mm/s，500 mm/s²，急停减速 1000 mm/s²。</summary>
    public static readonly MoveProfile Default = new(20d, 500d, 1000d);

    /// <summary>视觉对位用的慢速精定位 profile，速度低、加速度柔和，避免过冲。</summary>
    public static readonly MoveProfile Fine = new(5d, 200d, 200d, 0.02);

    /// <summary>回零 profile。</summary>
    public static readonly MoveProfile Homing = new(10d, 400d, 800d);

    /// <summary>按比例缩放速度与加速度（多轴联动时用于矢量合成同步到位）。</summary>
    public MoveProfile Scale(double factor)
    {
        if (factor <= 0 || !double.IsFinite(factor))
        {
            throw new ArgumentOutOfRangeException(nameof(factor), factor, "缩放系数必须为正的有限值");
        }

        return this with
        {
            Velocity = Velocity * factor,
            Acceleration = Acceleration * factor,
            Deceleration = Deceleration * factor,
        };
    }

    public void Validate()
    {
        if (Velocity <= 0 || Acceleration <= 0 || Deceleration <= 0)
        {
            throw new ArgumentException("速度与加减速度必须为正数", nameof(Velocity));
        }
    }
}
