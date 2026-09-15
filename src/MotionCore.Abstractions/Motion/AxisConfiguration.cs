namespace MotionCore.Abstractions.Motion;

/// <summary>
/// 单轴配置。参数集中在配置对象里，不散落在业务代码中 ——
/// 换机型时只改配置，不改逻辑。
/// </summary>
public sealed record AxisConfiguration
{
    public required AxisId Axis { get; init; }

    /// <summary>常规定位 profile。</summary>
    public MoveProfile Profile { get; init; } = MoveProfile.Default;

    /// <summary>视觉精定位 profile（慢速、柔和加减速，避免过冲与振动）。</summary>
    public MoveProfile FineProfile { get; init; } = MoveProfile.Fine;

    /// <summary>软限位下限（mm 或 °）。</summary>
    public double SoftLimitMin { get; init; } = -200d;

    /// <summary>软限位上限。</summary>
    public double SoftLimitMax { get; init; } = 200d;

    public bool SoftLimitEnabled { get; init; } = true;

    /// <summary>初始化时是否自动使能伺服。</summary>
    public bool EnableOnInitialize { get; init; } = true;

    public HomeMode HomeMode { get; init; } = HomeMode.NegativeLimit;

    /// <summary>回零搜索速度。</summary>
    public double HomeSpeed { get; init; } = 10d;

    /// <summary>回零偏移（原点在开关之外时使用）。</summary>
    public double HomeOffset { get; init; }

    /// <summary>跟随误差报警阈值。</summary>
    public double FollowingErrorLimit { get; init; } = 0.5d;

    public static AxisConfiguration Linear(AxisId axis, double min = -200d, double max = 200d) => new()
    {
        Axis = axis,
        SoftLimitMin = min,
        SoftLimitMax = max,
    };
}

/// <summary>控制器级配置。</summary>
public sealed class MotionControllerOptions
{
    /// <summary>控制器地址（网口卡为 IP，串口/PCI 卡可忽略）。</summary>
    public string Address { get; init; } = "192.168.0.11";

    /// <summary>端口（网口卡默认 0 表示厂商默认端口）。</summary>
    public int Port { get; init; }

    /// <summary>状态轮询周期。真实卡建议 10~20ms，或用卡的中断回调替代轮询。</summary>
    public int StatusPollIntervalMs { get; init; } = 20;

    /// <summary>等待到位默认超时。</summary>
    public TimeSpan MotionTimeout { get; init; } = TimeSpan.FromSeconds(15);

    /// <summary>回零超时。</summary>
    public TimeSpan HomeTimeout { get; init; } = TimeSpan.FromSeconds(30);

    /// <summary>是否在上位机侧做软限位前置拦截（强烈建议开启，比让卡去撞限位安全得多）。</summary>
    public bool GuardSoftLimit { get; init; } = true;

    public IReadOnlyList<AxisConfiguration> Axes { get; init; } = DefaultAxes();

    /// <summary>典型四轴直角坐标 + 旋转平台机型的默认配置。</summary>
    public static IReadOnlyList<AxisConfiguration> DefaultAxes() => new[]
    {
        AxisConfiguration.Linear(AxisId.X),
        AxisConfiguration.Linear(AxisId.Y),
        AxisConfiguration.Linear(AxisId.Z, -100d, 50d),
        new AxisConfiguration
        {
            Axis = AxisId.R,
            SoftLimitMin = -180d,
            SoftLimitMax = 180d,
            Profile = new MoveProfile(90d, 720d, 720d),
            FineProfile = new MoveProfile(15d, 180d, 180d),
        },
    };

    public AxisConfiguration GetAxis(AxisId axis) =>
        Axes.FirstOrDefault(configuration => configuration.Axis == axis)
        ?? new AxisConfiguration { Axis = axis };
}
