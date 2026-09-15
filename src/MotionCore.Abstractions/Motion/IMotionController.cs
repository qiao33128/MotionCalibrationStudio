using MotionCore.Abstractions.Geometry;

namespace MotionCore.Abstractions.Motion;

/// <summary>
/// 与厂商无关的运动控制器。上位机的所有业务代码（标定、对位、流程编排）
/// 只依赖这个接口 —— 换卡时业务层一行不改。
/// </summary>
public interface IMotionController : IAsyncDisposable
{
    /// <summary>控制器名称（一般取厂商名 + 型号）。</summary>
    string Name { get; }

    /// <summary>当前状态机状态。</summary>
    CardState State { get; }

    /// <summary>是否处于可下发运动指令的就绪态。</summary>
    bool IsReady { get; }

    /// <summary>本控制器实际存在的轴（不同机型轴数不同，业务层不能假定一定是 4 轴）。</summary>
    IReadOnlyList<AxisId> Axes { get; }

    /// <summary>数字量 IO。</summary>
    IDigitalIo Io { get; }

    /// <summary>状态机变化。</summary>
    event EventHandler<CardStateChangedEventArgs>? StateChanged;

    /// <summary>报警（含通讯、伺服、限位、超时、工艺类）。</summary>
    event EventHandler<MotionAlarm>? AlarmRaised;

    /// <summary>轴状态周期刷新（<b>注意：在后台线程触发</b>，UI 需要自行切回 UI 线程）。</summary>
    event EventHandler<AxisStatus>? StatusUpdated;

    /// <summary>运动完成。</summary>
    event EventHandler<MoveCompletedEventArgs>? MoveCompleted;

    /// <summary>初始化：连接、下发参数、使能、启动状态轮询。</summary>
    Task InitializeAsync(CancellationToken cancellationToken = default);

    /// <summary>关闭连接并释放资源。</summary>
    Task ShutdownAsync(CancellationToken cancellationToken = default);

    Task EnableAsync(AxisId axis, CancellationToken cancellationToken = default);

    Task DisableAsync(AxisId axis, CancellationToken cancellationToken = default);

    /// <summary>单轴回零（建立用户坐标系）。</summary>
    Task HomeAsync(AxisId axis, CancellationToken cancellationToken = default);

    /// <summary>全部轴依次回零。</summary>
    Task HomeAllAsync(CancellationToken cancellationToken = default);

    /// <summary>单轴绝对定位。</summary>
    Task MoveAbsoluteAsync(AxisId axis, double position, MoveProfile? profile = null, CancellationToken cancellationToken = default);

    /// <summary>多轴联动绝对定位（矢量合成，所有轴同时到达）。</summary>
    Task MoveAbsoluteAsync(MotionPose target, MoveProfile? profile = null, CancellationToken cancellationToken = default);

    /// <summary>单轴相对运动。</summary>
    Task MoveRelativeAsync(AxisId axis, double delta, MoveProfile? profile = null, CancellationToken cancellationToken = default);

    /// <summary>XY 平面相对运动（矢量合成）。</summary>
    Task MoveRelativeAsync(double dx, double dy, MoveProfile? profile = null, CancellationToken cancellationToken = default);

    /// <summary>开始点动（按住走、松开停）。</summary>
    Task StartJogAsync(AxisId axis, MotionDirection direction, MoveProfile? profile = null, CancellationToken cancellationToken = default);

    /// <summary>停止点动。</summary>
    Task StopJogAsync(AxisId axis, CancellationToken cancellationToken = default);

    /// <summary>停止。<paramref name="emergency"/> = true 时进入急停态，必须重新初始化。</summary>
    Task StopAsync(AxisId? axis = null, bool emergency = false, CancellationToken cancellationToken = default);

    /// <summary>报警复位（复位后会自动重新使能）。</summary>
    Task ResetAlarmAsync(CancellationToken cancellationToken = default);

    /// <summary>重设当前位置（建立/修正用户坐标系零点）。</summary>
    Task SetPositionAsync(AxisId axis, double position, CancellationToken cancellationToken = default);

    Task<AxisStatus> GetStatusAsync(AxisId axis, CancellationToken cancellationToken = default);

    /// <summary>一次性读取全部轴的位置，取同一时刻的快照。</summary>
    Task<MotionPose> GetPositionAsync(CancellationToken cancellationToken = default);

    /// <summary>等待指定轴全部到位，超时抛 <see cref="MotionTimeoutException"/>。</summary>
    Task WaitInPositionAsync(IEnumerable<AxisId> axes, TimeSpan? timeout = null, CancellationToken cancellationToken = default);
}

/// <summary>状态机变化事件参数。</summary>
public sealed class CardStateChangedEventArgs : EventArgs
{
    public required CardState OldState { get; init; }

    public required CardState NewState { get; init; }

    public string? Reason { get; init; }
}

/// <summary>运动完成事件参数。</summary>
public sealed class MoveCompletedEventArgs : EventArgs
{
    public required IReadOnlyList<AxisId> Axes { get; init; }

    public required TimeSpan Elapsed { get; init; }

    public MotionPose Target { get; init; }
}
