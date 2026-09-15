using System.Diagnostics;
using MotionCore.Abstractions.Motion;
using MotionCore.Abstractions.Vendors;

namespace MotionCore.Simulation;

/// <summary>仿真卡配置。</summary>
public sealed record SimulatedCardOptions
{
    public IReadOnlyList<AxisId> Axes { get; init; } = [AxisId.X, AxisId.Y, AxisId.Z, AxisId.R];

    public int InputCount { get; init; } = 16;

    public int OutputCount { get; init; } = 16;

    public double InitialPosition { get; init; }

    /// <summary>重复定位误差（标准正态标准差，mm）。</summary>
    public double PositioningNoise { get; init; } = 0.004d;

    /// <summary>跟随滞后时间常数（秒）。</summary>
    public double FollowingLagSeconds { get; init; } = 0.003d;

    /// <summary>连接握手耗时（ms）。真实网口卡首次连接通常需要上百毫秒。</summary>
    public int ConnectDelayMs { get; init; } = 120;

    public int Seed { get; init; } = 20260915;

    public double NegativeHardLimit { get; init; } = -215d;

    public double PositiveHardLimit { get; init; } = 215d;

    /// <summary>原点开关位置。</summary>
    public double HomeSwitchPosition { get; init; } = -150d;

    /// <summary>模拟连接失败（用于验证异常处理分支）。</summary>
    public bool FailOnConnect { get; init; }
}

/// <summary>
/// <b>仿真运动控制卡</b>
/// <para>
/// 它实现的是 <see cref="IVendorCardApi"/> —— 也就是说，在架构上它<b>和雷赛/固高/正运动是同一个层级的东西</b>。
/// 这一点是整个项目最重要的设计决策：
/// </para>
/// <list type="bullet">
///   <item>上层业务代码（标定示教、视觉对位、流程编排）完全不知道底下是仿真卡还是真卡；</item>
///   <item>整条链路可以在没有任何硬件的机器上跑通、被单元测试覆盖；</item>
///   <item>现场调试时可以先挂仿真卡把流程逻辑跑顺，再换真卡，把"逻辑问题"和"硬件问题"彻底分开。</item>
/// </list>
/// </summary>
public sealed class SimulatedCardApi : IVendorCardApi
{
    private readonly object _gate = new();
    private readonly Dictionary<AxisId, SimulatedAxis> _axes = new();
    private readonly bool[] _inputs;
    private readonly bool[] _outputs;
    private readonly SimulatedCardOptions _options;
    private readonly Stopwatch _clock = Stopwatch.StartNew();
    private readonly Random _random;

    private bool _connected;

    public SimulatedCardApi(SimulatedCardOptions? options = null)
    {
        _options = options ?? new SimulatedCardOptions();
        _random = new Random(_options.Seed);
        _inputs = new bool[_options.InputCount];
        _outputs = new bool[_options.OutputCount];

        foreach (AxisId axis in _options.Axes)
        {
            _axes[axis] = new SimulatedAxis(
                axis,
                _options.InitialPosition,
                _options.PositioningNoise,
                _options.FollowingLagSeconds,
                _random)
            {
                NegativeHardLimit = _options.NegativeHardLimit,
                PositiveHardLimit = _options.PositiveHardLimit,
                HomeSwitchPosition = _options.HomeSwitchPosition,
            };
        }

        // 用输入点模拟几个现场常驻的信号：气压正常 / 安全门关闭
        if (_inputs.Length > 0)
        {
            _inputs[0] = true;
        }

        if (_inputs.Length > 1)
        {
            _inputs[1] = true;
        }
    }

    public string VendorName => "仿真卡（SimulatedCard，无需硬件）";

    public IReadOnlyList<AxisId> Axes => _options.Axes;

    public int InputCount => _inputs.Length;

    public int OutputCount => _outputs.Length;

    /// <summary>仿真时钟（秒），可用于把运动流程暂停/加速。</summary>
    public double ElapsedSeconds => _clock.Elapsed.TotalSeconds;

    /// <summary>是否已连接。</summary>
    public bool IsConnected
    {
        get
        {
            lock (_gate)
            {
                return _connected;
            }
        }
    }

    private double Now => _clock.Elapsed.TotalSeconds;

    public int Connect(string address, int port)
    {
        // 真实网口卡握手是阻塞的，这里把耗时也算进去，方便观察 UI 是否会卡
        if (_options.ConnectDelayMs > 0)
        {
            Thread.Sleep(_options.ConnectDelayMs);
        }

        lock (_gate)
        {
            if (_options.FailOnConnect)
            {
                return SimulatedErrorCodes.CommunicationTimeout;
            }

            _connected = true;
            return 0;
        }
    }

    public void Disconnect()
    {
        lock (_gate)
        {
            _connected = false;
        }
    }

    public string GetErrorMessage(int errorCode) => errorCode switch
    {
        SimulatedErrorCodes.Ok => "正常",
        SimulatedErrorCodes.NotConnected => "控制器未连接",
        SimulatedErrorCodes.InvalidAxis => "轴号非法",
        SimulatedErrorCodes.InvalidParameter => "参数非法",
        SimulatedErrorCodes.ServoDisabled => "伺服未使能",
        SimulatedErrorCodes.SoftLimitExceeded => "目标位置超出软限位",
        SimulatedErrorCodes.AxisAlarm => "轴处于报警状态，需先复位",
        SimulatedErrorCodes.HardLimit => "触发硬限位开关",
        SimulatedErrorCodes.EmergencyStop => "急停已触发",
        SimulatedErrorCodes.CommunicationTimeout => "通讯超时 / 连接失败",
        _ => $"未知错误码 {errorCode}",
    };

    // ───────────────────────── 原语实现 ─────────────────────────

    public int SetServoEnabled(AxisId axis, bool enabled)
    {
        lock (_gate)
        {
            if (!_connected)
            {
                return SimulatedErrorCodes.NotConnected;
            }

            if (!_axes.TryGetValue(axis, out SimulatedAxis? simulated))
            {
                return SimulatedErrorCodes.InvalidAxis;
            }

            return simulated.SetServoEnabled(enabled, Now);
        }
    }

    public int ConfigureProfile(AxisId axis, MoveProfile profile)
    {
        lock (_gate)
        {
            if (!_axes.TryGetValue(axis, out SimulatedAxis? simulated))
            {
                return SimulatedErrorCodes.InvalidAxis;
            }

            return simulated.ConfigureProfile(profile, Now);
        }
    }

    public int ConfigureSoftLimit(AxisId axis, double negativeLimit, double positiveLimit, bool enabled)
    {
        lock (_gate)
        {
            if (!_axes.TryGetValue(axis, out SimulatedAxis? simulated))
            {
                return SimulatedErrorCodes.InvalidAxis;
            }

            return simulated.ConfigureSoftLimit(negativeLimit, positiveLimit, enabled, Now);
        }
    }

    public int SetPosition(AxisId axis, double position)
    {
        lock (_gate)
        {
            if (!_axes.TryGetValue(axis, out SimulatedAxis? simulated))
            {
                return SimulatedErrorCodes.InvalidAxis;
            }

            return simulated.SetPosition(position, Now);
        }
    }

    public int MoveAbsolute(AxisId axis, double position)
    {
        lock (_gate)
        {
            if (!_connected)
            {
                return SimulatedErrorCodes.NotConnected;
            }

            if (!_axes.TryGetValue(axis, out SimulatedAxis? simulated))
            {
                return SimulatedErrorCodes.InvalidAxis;
            }

            return simulated.MoveAbsolute(position, Now);
        }
    }

    public int MoveRelative(AxisId axis, double distance)
    {
        lock (_gate)
        {
            if (!_axes.TryGetValue(axis, out SimulatedAxis? simulated))
            {
                return SimulatedErrorCodes.InvalidAxis;
            }

            return simulated.MoveRelative(distance, Now);
        }
    }

    public int StartJog(AxisId axis, MotionDirection direction, double velocity)
    {
        lock (_gate)
        {
            if (!_axes.TryGetValue(axis, out SimulatedAxis? simulated))
            {
                return SimulatedErrorCodes.InvalidAxis;
            }

            return simulated.StartJog(direction, velocity, Now);
        }
    }

    public int StopJog(AxisId axis)
    {
        lock (_gate)
        {
            if (!_axes.TryGetValue(axis, out SimulatedAxis? simulated))
            {
                return SimulatedErrorCodes.InvalidAxis;
            }

            return simulated.StopJog(Now);
        }
    }

    public int Home(AxisId axis, HomeMode mode, double speed, double offset)
    {
        lock (_gate)
        {
            if (!_axes.TryGetValue(axis, out SimulatedAxis? simulated))
            {
                return SimulatedErrorCodes.InvalidAxis;
            }

            return simulated.Home(mode, speed, offset, Now);
        }
    }

    public int Stop(AxisId axis, bool emergency)
    {
        lock (_gate)
        {
            if (!_axes.TryGetValue(axis, out SimulatedAxis? simulated))
            {
                return SimulatedErrorCodes.InvalidAxis;
            }

            return simulated.Stop(emergency, Now);
        }
    }

    public int ResetAlarm(AxisId axis)
    {
        lock (_gate)
        {
            if (!_axes.TryGetValue(axis, out SimulatedAxis? simulated))
            {
                return SimulatedErrorCodes.InvalidAxis;
            }

            return simulated.ResetAlarm(Now);
        }
    }

    public VendorAxisSnapshot ReadAxis(AxisId axis)
    {
        lock (_gate)
        {
            if (!_axes.TryGetValue(axis, out SimulatedAxis? simulated))
            {
                return default;
            }

            simulated.Update(Now);

            return new VendorAxisSnapshot(
                CommandPosition: simulated.CommandPosition,
                ActualPosition: simulated.Position,
                Velocity: simulated.Velocity,
                ServoEnabled: simulated.ServoEnabled,
                InPosition: simulated.InPosition,
                Homed: simulated.Homed,
                PositiveLimit: simulated.AtPositiveLimit,
                NegativeLimit: simulated.AtNegativeLimit,
                HomeSwitch: simulated.Position <= simulated.HomeSwitchPosition,
                Alarm: simulated.Alarm,
                AlarmCode: simulated.AlarmCode);
        }
    }

    public bool ReadInput(int index) => ReadDigital(_inputs, index);

    public bool ReadOutput(int index) => ReadDigital(_outputs, index);

    public bool WriteOutput(int index, bool value) => WriteDigital(_outputs, index, value);

    private bool ReadDigital(bool[] buffer, int index)
    {
        if (index < 0 || index >= buffer.Length)
        {
            return false;
        }

        lock (_gate)
        {
            return buffer[index];
        }
    }

    private bool WriteDigital(bool[] buffer, int index, bool value)
    {
        if (index < 0 || index >= buffer.Length)
        {
            return false;
        }

        lock (_gate)
        {
            buffer[index] = value;
            return true;
        }
    }

    // ───────────────────────── 仿真专用扩展（测试 / UI 用）─────────────────────────

    /// <summary>直接操控某根轴的仿真状态（注入报警、越限等，用于验证异常分支）。</summary>
    public SimulatedAxis Axis(AxisId axis) =>
        _axes.TryGetValue(axis, out SimulatedAxis? simulated)
            ? simulated
            : throw new ArgumentOutOfRangeException(nameof(axis), axis, "仿真卡中不存在该轴");

    /// <summary>强制触发某根轴的报警。</summary>
    public void InjectAlarm(AxisId axis, int errorCode)
    {
        lock (_gate)
        {
            if (!_axes.TryGetValue(axis, out SimulatedAxis? simulated))
            {
                return;
            }

            simulated.InjectAlarmForTest(errorCode);
        }
    }

    /// <summary>设置输入点（模拟气缸到位、安全门等信号）。</summary>
    public void SetInput(int index, bool value) => WriteDigital(_inputs, index, value);

    /// <summary>读回全部输出点，用于校验 IO 时序。</summary>
    public IReadOnlyList<bool> SnapshotOutputs()
    {
        lock (_gate)
        {
            return (bool[])_outputs.Clone();
        }
    }

    public void Dispose()
    {
        lock (_gate)
        {
            _connected = false;
        }
    }
}

/// <summary>
/// 仿真控制器。
/// <para>
/// 整个类只有几行 —— 因为状态机、等待到位、多轴同步、软限位拦截、报警归一化
/// 全部在 <see cref="MotionControllerBase"/> 里，这里只负责把仿真卡接上去并暴露出来给 UI / 测试使用。
/// 这也正是"厂商差异收敛到原语接口"这个设计带来的直接收益。
/// </para>
/// </summary>
public sealed class SimulatedMotionController : MotionControllerBase
{
    public SimulatedMotionController(SimulatedCardOptions? cardOptions = null, MotionControllerOptions? options = null)
        : this(new SimulatedCardApi(cardOptions), options)
    {
    }

    public SimulatedMotionController(SimulatedCardApi card, MotionControllerOptions? options = null)
        : base(card, options) => Card = card;

    /// <summary>底层仿真卡，用于注入故障、操控 IO 等测试与演示操作。</summary>
    public SimulatedCardApi Card { get; }
}
