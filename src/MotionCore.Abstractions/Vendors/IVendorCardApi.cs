using MotionCore.Abstractions.Motion;

namespace MotionCore.Abstractions.Vendors;

/// <summary>
/// 厂商轴状态原始快照。刻意做成"哑数据"，不包含任何业务含义。
/// </summary>
public readonly record struct VendorAxisSnapshot(
    double CommandPosition,
    double ActualPosition,
    double Velocity,
    bool ServoEnabled,
    bool InPosition,
    bool Homed,
    bool PositiveLimit,
    bool NegativeLimit,
    bool HomeSwitch,
    bool Alarm,
    int AlarmCode);

/// <summary>
/// <b>厂商原语契约（整个架构的关键）</b>
/// <para>
/// 雷赛 / 固高 / 凌华 / 正运动的 SDK 在函数名、参数语义、错误码上都不一样，
/// 但真正有差异的其实只有下面这十几个"原语"。把所有厂商差异收敛到这一层之后，
/// 状态机、等待到位、多轴同步、软限位拦截、报警归一化这些"上层编排"只需要写一份
/// （见 <c>MotionControllerBase</c>）。
/// </para>
/// <para>
/// 反过来，只要实现了这个接口，<b>仿真卡就是一张卡</b> —— 所以整套上位机逻辑
/// 不需要任何硬件就能端到端跑通、并且可以被单元测试覆盖。
/// </para>
/// </summary>
public interface IVendorCardApi : IDisposable
{
    /// <summary>厂商名（用于日志与诊断）。</summary>
    string VendorName { get; }

    /// <summary>最近一次操作返回的原始错误码文本。</summary>
    string GetErrorMessage(int errorCode);

    /// <summary>可用轴列表。</summary>
    IReadOnlyList<AxisId> Axes { get; }

    int InputCount { get; }

    int OutputCount { get; }

    /// <summary>连接控制器/打开卡。返回 0 表示成功。</summary>
    int Connect(string address, int port);

    /// <summary>断开连接。</summary>
    void Disconnect();

    /// <summary>使能 / 关闭伺服。</summary>
    int SetServoEnabled(AxisId axis, bool enabled);

    /// <summary>下发速度、加减速度参数。</summary>
    int ConfigureProfile(AxisId axis, MoveProfile profile);

    /// <summary>配置软限位。</summary>
    int ConfigureSoftLimit(AxisId axis, double negativeLimit, double positiveLimit, bool enabled);

    /// <summary>重设位置计数器（建立用户坐标系零点）。</summary>
    int SetPosition(AxisId axis, double position);

    /// <summary>绝对定位（非阻塞，卡内部自行规划速度曲线）。</summary>
    int MoveAbsolute(AxisId axis, double position);

    /// <summary>相对定位。</summary>
    int MoveRelative(AxisId axis, double distance);

    /// <summary>开始点动。</summary>
    int StartJog(AxisId axis, MotionDirection direction, double velocity);

    /// <summary>停止点动。</summary>
    int StopJog(AxisId axis);

    /// <summary>回零。</summary>
    int Home(AxisId axis, HomeMode mode, double speed, double offset);

    /// <summary>停止轴。</summary>
    int Stop(AxisId axis, bool emergency);

    /// <summary>复位报警。</summary>
    int ResetAlarm(AxisId axis);

    /// <summary>读取轴状态快照。</summary>
    VendorAxisSnapshot ReadAxis(AxisId axis);

    bool ReadInput(int index);

    bool ReadOutput(int index);

    bool WriteOutput(int index, bool value);
}
