using System.Diagnostics;
using MotionCore.Abstractions.Geometry;
using MotionCore.Abstractions.Vendors;

namespace MotionCore.Abstractions.Motion;

/// <summary>
/// <b>通用控制器编排（全项目只写一份）</b>
/// <para>
/// 这里承载的是"所有厂商都绕不开、又都不该由厂商实现"的逻辑：
/// 状态机、软限位前置拦截、等待到位、多轴矢量同步、回零超时、报警归一化、
/// 状态周期广播、线程模型的收口。
/// </para>
/// <para>
/// 具体厂商只需要实现 <see cref="IVendorCardApi"/> 的十几个原语。
/// </para>
/// </summary>
public abstract class MotionControllerBase : IMotionController
{
    private readonly IVendorCardApi _api;
    private readonly object _gate = new();
    private readonly Dictionary<AxisId, AxisConfiguration> _configurations = new();

    private CancellationTokenSource? _pollCts;
    private Task? _pollTask;
    private CardState _state = CardState.Disconnected;
    private bool _disposed;

    protected MotionControllerBase(IVendorCardApi api, MotionControllerOptions? options = null)
    {
        _api = api ?? throw new ArgumentNullException(nameof(api));
        Options = options ?? new MotionControllerOptions();

        foreach (AxisConfiguration configuration in Options.Axes)
        {
            _configurations[configuration.Axis] = configuration;
        }

        Io = new VendorDigitalIo(_api);
    }

    public MotionControllerOptions Options { get; }

    public string Name => _api.VendorName;

    public CardState State
    {
        get
        {
            lock (_gate)
            {
                return _state;
            }
        }
    }

    public bool IsReady => State == CardState.Ready;

    public IReadOnlyList<AxisId> Axes => _api.Axes;

    public IDigitalIo Io { get; }

    public event EventHandler<CardStateChangedEventArgs>? StateChanged;

    public event EventHandler<MotionAlarm>? AlarmRaised;

    public event EventHandler<AxisStatus>? StatusUpdated;

    public event EventHandler<MoveCompletedEventArgs>? MoveCompleted;

    // ───────────────────────────── 生命周期 ─────────────────────────────

    /// <summary>
    /// 初始化。
    /// <para>
    /// 厂商 SDK 全部是同步阻塞的 P/Invoke，直接放在 UI 线程调用会把界面卡死，
    /// 所以这里统一丢到线程池；而 <b>等待到位</b> 用异步轮询而不是 Thread.Sleep，
    /// 保证不占用线程池线程。
    /// </para>
    /// </summary>
    public Task InitializeAsync(CancellationToken cancellationToken = default) =>
        Task.Run(() => InitializeCore(cancellationToken), cancellationToken);

    private void InitializeCore(CancellationToken cancellationToken)
    {
        ThrowIfDisposed();
        SetState(CardState.Initializing, $"连接 {Options.Address}");

        try
        {
            EnsureSuccess(_api.Connect(Options.Address, Options.Port), "连接控制器");

            foreach (AxisId axis in _api.Axes)
            {
                cancellationToken.ThrowIfCancellationRequested();

                AxisConfiguration configuration = GetConfiguration(axis);
                configuration.Profile.Validate();

                EnsureSuccess(
                    _api.ConfigureProfile(axis, configuration.Profile),
                    $"配置轴 {axis.ToShortName()} 运动参数");

                EnsureSuccess(
                    _api.ConfigureSoftLimit(axis, configuration.SoftLimitMin, configuration.SoftLimitMax, configuration.SoftLimitEnabled),
                    $"配置轴 {axis.ToShortName()} 软限位");

                if (configuration.EnableOnInitialize)
                {
                    EnsureSuccess(_api.SetServoEnabled(axis, true), $"使能轴 {axis.ToShortName()}");
                }
            }
        }
        catch (Exception ex)
        {
            SetState(CardState.Alarm, $"初始化失败：{ex.Message}");
            RaiseAlarm(new MotionAlarm
            {
                Category = AlarmCategory.Communication,
                Severity = AlarmSeverity.Fatal,
                Code = "MC-INIT-001",
                Message = $"控制器初始化失败：{ex.Message}",
                Advice = "检查网线/供电、确认 IP 与卡型号配置正确、确认厂商动态库已随程序发布",
            });
            throw;
        }

        SetState(CardState.Ready, "初始化完成");
        StartStatusPolling();
    }

    // ───────────────────────────── 使能 / 回零 ─────────────────────────────

    public Task EnableAsync(AxisId axis, CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();
        EnsureSuccess(_api.SetServoEnabled(axis, true), $"使能轴 {axis.ToShortName()}");
        return Task.CompletedTask;
    }

    public Task DisableAsync(AxisId axis, CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();
        EnsureSuccess(_api.SetServoEnabled(axis, false), $"关闭轴 {axis.ToShortName()} 使能");
        return Task.CompletedTask;
    }

    public async Task HomeAsync(AxisId axis, CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();
        ThrowIfNotReadyForMotion();

        AxisConfiguration configuration = GetConfiguration(axis);
        EnsureSuccess(_api.ConfigureProfile(axis, MoveProfile.Homing), $"配置轴 {axis.ToShortName()} 回零参数");
        EnsureSuccess(
            _api.Home(axis, configuration.HomeMode, configuration.HomeSpeed, configuration.HomeOffset),
            $"轴 {axis.ToShortName()} 回零");

        SetState(CardState.Moving, $"轴 {axis.ToShortName()} 回零中");

        Stopwatch watch = Stopwatch.StartNew();
        while (true)
        {
            cancellationToken.ThrowIfCancellationRequested();

            VendorAxisSnapshot snapshot = _api.ReadAxis(axis);
            if (snapshot.Alarm)
            {
                RaiseAlarm(axis, snapshot);
                throw new MotionException($"轴 {axis.ToShortName()} 回零过程中报警：{_api.GetErrorMessage(snapshot.AlarmCode)}", snapshot.AlarmCode)
                {
                    Category = AlarmCategory.Homing,
                };
            }

            if (snapshot.Homed && snapshot.InPosition)
            {
                break;
            }

            if (watch.Elapsed > Options.HomeTimeout)
            {
                EnsureSuccess(_api.Stop(axis, false), $"轴 {axis.ToShortName()} 停止");
                RaiseAlarm(new MotionAlarm
                {
                    Category = AlarmCategory.Homing,
                    Severity = AlarmSeverity.Error,
                    Code = "MC-HOME-002",
                    Axis = axis,
                    Message = $"轴 {axis.ToShortName()} 回零超时（{Options.HomeTimeout.TotalSeconds:F0}s）",
                    Advice = "检查原点开关信号是否有效、回零速度是否过低、机构是否卡滞",
                });
                throw new MotionTimeoutException($"轴 {axis.ToShortName()} 回零超时");
            }

            await Task.Delay(Options.StatusPollIntervalMs, cancellationToken).ConfigureAwait(false);
        }

        RestoreReadyAfterMotion();
    }

    public async Task HomeAllAsync(CancellationToken cancellationToken = default)
    {
        // 多轴回零必须依次进行：同时回零会同时冲向原点，机械干涉风险极高。
        foreach (AxisId axis in _api.Axes)
        {
            await HomeAsync(axis, cancellationToken).ConfigureAwait(false);
        }
    }

    // ───────────────────────────── 定位 ─────────────────────────────

    public async Task MoveAbsoluteAsync(
        AxisId axis,
        double position,
        MoveProfile? profile = null,
        CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();
        ThrowIfNotReadyForMotion();
        GuardSoftLimit(axis, position);

        AxisConfiguration configuration = GetConfiguration(axis);
        MoveProfile effective = profile ?? configuration.Profile;
        effective.Validate();

        EnsureSuccess(_api.ConfigureProfile(axis, effective), $"下发轴 {axis.ToShortName()} 运动参数");
        EnsureSuccess(_api.MoveAbsolute(axis, position), $"轴 {axis.ToShortName()} 绝对定位");

        SetState(CardState.Moving, $"轴 {axis.ToShortName()} → {position:F4}");
        Stopwatch watch = Stopwatch.StartNew();

        await WaitInPositionAsync(new[] { axis }, Options.MotionTimeout, cancellationToken).ConfigureAwait(false);

        RaiseMoveCompleted(new[] { axis }, MotionPose.Zero.With(axis, position), watch.Elapsed);
        RestoreReadyAfterMotion();
    }

    /// <summary>
    /// 多轴联动绝对定位。
    /// <para>
    /// <b>矢量合成速度同步</b>：以行程最大的轴为基准轴，其余轴的
    /// 速度与加速度按 <c>该轴行程 / 最大行程</c> 等比缩放。
    /// 由于梯形曲线的总时间 ∝ 行程 / 速度，等比缩放后所有轴的运行时间完全相同，
    /// 于是直线插补不需要卡支持插补功能也能保证同时到达（且轨迹是直线）。
    /// </para>
    /// </summary>
    public async Task MoveAbsoluteAsync(
        MotionPose target,
        MoveProfile? profile = null,
        CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();
        ThrowIfNotReadyForMotion();

        List<(AxisId Axis, double Target, double Distance)> moving = new();

        foreach (AxisId axis in _api.Axes)
        {
            double current = _api.ReadAxis(axis).ActualPosition;
            double destination = target[axis];
            GuardSoftLimit(axis, destination);

            double distance = Math.Abs(destination - current);
            if (distance > 1e-6)
            {
                moving.Add((axis, destination, distance));
            }
        }

        if (moving.Count == 0)
        {
            return;
        }

        MoveProfile baseProfile = profile ?? MoveProfile.Default;
        double maxDistance = moving.Max(item => item.Distance);
        Stopwatch watch = Stopwatch.StartNew();

        foreach ((AxisId axis, double destination, double distance) in moving)
        {
            MoveProfile synchronized = baseProfile.Scale(distance / maxDistance);
            EnsureSuccess(_api.ConfigureProfile(axis, synchronized), $"下发轴 {axis.ToShortName()} 联动参数");
            EnsureSuccess(_api.MoveAbsolute(axis, destination), $"轴 {axis.ToShortName()} 联动定位");
        }

        SetState(CardState.Moving, $"联动定位 → {target}");

        await WaitInPositionAsync(moving.Select(item => item.Axis), Options.MotionTimeout, cancellationToken)
            .ConfigureAwait(false);

        RaiseMoveCompleted(moving.Select(item => item.Axis).ToArray(), target, watch.Elapsed);
        RestoreReadyAfterMotion();
    }

    public Task MoveRelativeAsync(
        AxisId axis,
        double delta,
        MoveProfile? profile = null,
        CancellationToken cancellationToken = default)
    {
        double current = _api.ReadAxis(axis).ActualPosition;
        return MoveAbsoluteAsync(axis, current + delta, profile, cancellationToken);
    }

    public Task MoveRelativeAsync(
        double dx,
        double dy,
        MoveProfile? profile = null,
        CancellationToken cancellationToken = default)
    {
        MotionPose current = GetPositionCore();
        MotionPose target = current.Offset(dx, dy);
        return MoveAbsoluteAsync(target, profile, cancellationToken);
    }

    // ───────────────────────────── 点动 / 停止 ─────────────────────────────

    public Task StartJogAsync(
        AxisId axis,
        MotionDirection direction,
        MoveProfile? profile = null,
        CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();
        ThrowIfNotReadyForMotion();

        AxisConfiguration configuration = GetConfiguration(axis);
        double velocity = (profile ?? configuration.Profile).Velocity;

        // 点动前先判断方向上的限位，避免点动撞限位
        VendorAxisSnapshot snapshot = _api.ReadAxis(axis);
        if ((direction == MotionDirection.Positive && snapshot.PositiveLimit)
            || (direction == MotionDirection.Negative && snapshot.NegativeLimit))
        {
            throw new MotionSoftLimitException($"轴 {axis.ToShortName()} 该方向已触发限位，禁止点动");
        }

        EnsureSuccess(_api.StartJog(axis, direction, velocity), $"轴 {axis.ToShortName()} 点动");
        return Task.CompletedTask;
    }

    public Task StopJogAsync(AxisId axis, CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();
        EnsureSuccess(_api.StopJog(axis), $"轴 {axis.ToShortName()} 停止点动");
        return Task.CompletedTask;
    }

    public Task StopAsync(AxisId? axis = null, bool emergency = false, CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();

        IReadOnlyList<AxisId> targets = axis is null ? _api.Axes : new[] { axis.Value };
        foreach (AxisId target in targets)
        {
            EnsureSuccess(_api.Stop(target, emergency), $"停止轴 {target.ToShortName()}");
        }

        SetState(
            emergency ? CardState.EmergencyStop : CardState.Hold,
            emergency ? "急停已触发，必须重新初始化" : "轴停止");

        if (emergency)
        {
            // 注意这里用 ReportAlarm(transitionState: false)：
            // 状态机已经切到 EmergencyStop 了，如果再走一次报警状态迁移会被覆盖成 Alarm，
            // 现场就会分不清"急停"和"普通报警"——这两者的恢复流程完全不同。
            ReportAlarm(
                new MotionAlarm
                {
                    Category = AlarmCategory.Safety,
                    Severity = AlarmSeverity.Fatal,
                    Code = "MC-SAFETY-001",
                    Message = "急停已触发",
                    Advice = "排除安全风险后重新执行初始化（InitializeAsync）",
                },
                transitionState: false);
        }

        return Task.CompletedTask;
    }

    public Task ResetAlarmAsync(CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();

        foreach (AxisId axis in _api.Axes)
        {
            _api.ResetAlarm(axis);
        }

        // 报警复位后伺服通常会掉使能，必须重新使能，否则后续指令全部失败
        foreach (AxisId axis in _api.Axes)
        {
            if (GetConfiguration(axis).EnableOnInitialize)
            {
                EnsureSuccess(_api.SetServoEnabled(axis, true), $"重新使能轴 {axis.ToShortName()}");
            }
        }

        SetState(CardState.Ready, "报警已复位");
        return Task.CompletedTask;
    }

    // ───────────────────────────── 位置 / 状态 ─────────────────────────────

    public Task SetPositionAsync(AxisId axis, double position, CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();
        EnsureSuccess(_api.SetPosition(axis, position), $"重设轴 {axis.ToShortName()} 当前位置");
        return Task.CompletedTask;
    }

    public Task<AxisStatus> GetStatusAsync(AxisId axis, CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();
        return Task.FromResult(ToAxisStatus(axis, _api.ReadAxis(axis)));
    }

    public Task<MotionPose> GetPositionAsync(CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();
        return Task.FromResult(GetPositionCore());
    }

    private MotionPose GetPositionCore()
    {
        MotionPose pose = MotionPose.Zero;
        foreach (AxisId axis in _api.Axes)
        {
            // GetPositionAsync 用于取"同一时刻"的快照，因此这里不做任何 UI 通知
            pose = pose.With(axis, _api.ReadAxis(axis).ActualPosition);
        }

        return pose;
    }

    public async Task WaitInPositionAsync(
        IEnumerable<AxisId> axes,
        TimeSpan? timeout = null,
        CancellationToken cancellationToken = default)
    {
        AxisId[] list = axes.Distinct().ToArray();
        if (list.Length == 0)
        {
            return;
        }

        TimeSpan limit = timeout ?? Options.MotionTimeout;
        Stopwatch watch = Stopwatch.StartNew();

        while (true)
        {
            cancellationToken.ThrowIfCancellationRequested();

            bool allInPosition = true;
            foreach (AxisId axis in list)
            {
                VendorAxisSnapshot snapshot = _api.ReadAxis(axis);

                if (snapshot.Alarm)
                {
                    RaiseAlarm(axis, snapshot);
                    throw new MotionException(
                        $"轴 {axis.ToShortName()} 运动中报警：{_api.GetErrorMessage(snapshot.AlarmCode)}（错误码 {snapshot.AlarmCode}）",
                        snapshot.AlarmCode);
                }

                if (!snapshot.InPosition)
                {
                    allInPosition = false;
                }
            }

            if (allInPosition)
            {
                return;
            }

            if (watch.Elapsed > limit)
            {
                throw new MotionTimeoutException(
                    $"等待轴 [{string.Join(", ", list.Select(a => a.ToShortName()))}] 到位超时（{limit.TotalSeconds:F1}s）");
            }

            await Task.Delay(Options.StatusPollIntervalMs, cancellationToken).ConfigureAwait(false);
        }
    }

    // ───────────────────────────── 内部实现 ─────────────────────────────

    private void StartStatusPolling()
    {
        _pollCts = new CancellationTokenSource();
        CancellationToken token = _pollCts.Token;

        _pollTask = Task.Run(
            async () =>
            {
                while (!token.IsCancellationRequested)
                {
                    try
                    {
                        foreach (AxisId axis in _api.Axes)
                        {
                            VendorAxisSnapshot snapshot = _api.ReadAxis(axis);
                            StatusUpdated?.Invoke(this, ToAxisStatus(axis, snapshot));

                            CardState current = State;
                            if (snapshot.Alarm && current is not (CardState.Alarm or CardState.EmergencyStop))
                            {
                                RaiseAlarm(axis, snapshot);
                            }
                        }
                    }
                    catch (Exception ex) when (ex is not OperationCanceledException)
                    {
                        RaiseAlarm(new MotionAlarm
                        {
                            Category = AlarmCategory.Communication,
                            Severity = AlarmSeverity.Error,
                            Code = "MC-COMM-001",
                            Message = $"状态轮询异常：{ex.Message}",
                            Advice = "检查通讯链路；若为偶发丢包可放宽轮询周期",
                        });
                    }

                    try
                    {
                        await Task.Delay(Options.StatusPollIntervalMs, token).ConfigureAwait(false);
                    }
                    catch (OperationCanceledException)
                    {
                        break;
                    }
                }
            },
            token);
    }

    private static AxisStatus ToAxisStatus(AxisId axis, VendorAxisSnapshot snapshot) => new()
    {
        Axis = axis,
        CommandPosition = snapshot.CommandPosition,
        ActualPosition = snapshot.ActualPosition,
        Velocity = snapshot.Velocity,
        ServoEnabled = snapshot.ServoEnabled,
        InPosition = snapshot.InPosition,
        Homed = snapshot.Homed,
        PositiveLimit = snapshot.PositiveLimit,
        NegativeLimit = snapshot.NegativeLimit,
        HomeSwitch = snapshot.HomeSwitch,
        Alarm = snapshot.Alarm,
        AlarmCode = snapshot.AlarmCode,
        AlarmMessage = snapshot.Alarm ? $"厂商错误码 {snapshot.AlarmCode}" : null,
    };

    private AxisConfiguration GetConfiguration(AxisId axis) =>
        _configurations.TryGetValue(axis, out AxisConfiguration? configuration)
            ? configuration
            : new AxisConfiguration { Axis = axis };

    private void GuardSoftLimit(AxisId axis, double target)
    {
        if (!Options.GuardSoftLimit)
        {
            return;
        }

        AxisConfiguration configuration = GetConfiguration(axis);
        if (!configuration.SoftLimitEnabled)
        {
            return;
        }

        if (target < configuration.SoftLimitMin - 1e-9 || target > configuration.SoftLimitMax + 1e-9)
        {
            string message =
                $"轴 {axis.ToShortName()} 目标位置 {target:F4} 超出软限位 " +
                $"[{configuration.SoftLimitMin:F4}, {configuration.SoftLimitMax:F4}]，已在上位机侧拦截";

            // 注意：软限位拦截属于"指令被拒绝"，不是控制器故障，
            // 因此只上报报警、不把状态机切到 Alarm，否则一次误操作会让整台设备停机。
            ReportAlarm(
                new MotionAlarm
                {
                    Category = AlarmCategory.Limit,
                    Severity = AlarmSeverity.Warning,
                    Code = "MC-LIMIT-001",
                    Axis = axis,
                    Message = message,
                    Advice = "确认目标位置是否算错（标定参数 / 对位偏移量异常时最容易出现）",
                },
                transitionState: false);

            throw new MotionSoftLimitException(message);
        }
    }

    protected void EnsureReadyForMotion()
    {
        ThrowIfDisposed();
        ThrowIfNotReadyForMotion();
    }

    private void ThrowIfNotReadyForMotion()
    {
        CardState state = State;
        if (state != CardState.Ready)
        {
            throw new MotionStateException($"控制器当前状态为 {state}，不允许下发运动指令（需要 Ready）");
        }
    }

    private void EnsureSuccess(int errorCode, string operation)
    {
        if (errorCode == 0)
        {
            return;
        }

        string description = _api.GetErrorMessage(errorCode);
        throw new MotionException($"{operation}失败：{description}（厂商错误码 {errorCode}）", errorCode);
    }

    private void SetState(CardState newState, string? reason = null)
    {
        CardState oldState;
        lock (_gate)
        {
            if (_state == newState)
            {
                return;
            }

            oldState = _state;
            _state = newState;
        }

        StateChanged?.Invoke(this, new CardStateChangedEventArgs
        {
            OldState = oldState,
            NewState = newState,
            Reason = reason,
        });
    }

    private void RestoreReadyAfterMotion()
    {
        if (State == CardState.Moving)
        {
            SetState(CardState.Ready, "运动完成");
        }
    }

    private void RaiseMoveCompleted(IReadOnlyList<AxisId> axes, MotionPose target, TimeSpan elapsed) =>
        MoveCompleted?.Invoke(this, new MoveCompletedEventArgs
        {
            Axes = axes,
            Target = target,
            Elapsed = elapsed,
        });

    private void RaiseAlarm(AxisId axis, VendorAxisSnapshot snapshot) =>
        RaiseAlarm(new MotionAlarm
        {
            Category = AlarmCategory.Servo,
            Severity = AlarmSeverity.Error,
            VendorCode = snapshot.AlarmCode,
            Code = $"MC-SERVO-{snapshot.AlarmCode:D4}",
            Axis = axis,
            Message = $"轴 {axis.ToShortName()} 报警：{_api.GetErrorMessage(snapshot.AlarmCode)}",
            Advice = "排除机械卡滞/过载后调用 ResetAlarmAsync",
        });

    private void RaiseAlarm(MotionAlarm alarm) => ReportAlarm(alarm, transitionState: true);

    private void ReportAlarm(MotionAlarm alarm, bool transitionState)
    {
        if (transitionState)
        {
            SetState(CardState.Alarm, alarm.Message);
        }

        AlarmRaised?.Invoke(this, alarm);
    }

    protected void ThrowIfDisposed() => ObjectDisposedException.ThrowIf(_disposed, this);

    public async Task ShutdownAsync(CancellationToken cancellationToken = default)
    {
        if (_disposed)
        {
            return;
        }

        _disposed = true;

        try
        {
            _pollCts?.Cancel();
            if (_pollTask is not null)
            {
                await Task.WhenAny(_pollTask, Task.Delay(500, CancellationToken.None)).ConfigureAwait(false);
            }
        }
        catch (Exception)
        {
            // 关闭阶段的异常不应向上传播
        }

        try
        {
            foreach (AxisId axis in _api.Axes)
            {
                _api.Stop(axis, false);
            }

            _api.Disconnect();
        }
        catch (Exception)
        {
            // 同上
        }

        SetState(CardState.Disconnected, "已断开");
    }

    public async ValueTask DisposeAsync()
    {
        await ShutdownAsync().ConfigureAwait(false);
        _api.Dispose();
        _pollCts?.Dispose();
        SetState(CardState.Disposed, "已释放");
        GC.SuppressFinalize(this);
    }

    /// <summary>IO 转发：把原语的 0/1 索引直接映射成 IO 端口。</summary>
    private sealed class VendorDigitalIo : IDigitalIo
    {
        private readonly IVendorCardApi _api;

        public VendorDigitalIo(IVendorCardApi api) => _api = api;

        public int InputCount => _api.InputCount;

        public int OutputCount => _api.OutputCount;

        public bool ReadInput(int index) => _api.ReadInput(index);

        public bool ReadOutput(int index) => _api.ReadOutput(index);

        public bool WriteOutput(int index, bool value) => _api.WriteOutput(index, value);

        public IReadOnlyList<bool> ReadInputs(int start, int count)
        {
            bool[] buffer = new bool[count];
            for (int i = 0; i < count; i++)
            {
                buffer[i] = _api.ReadInput(start + i);
            }

            return buffer;
        }

        public void WriteOutputs(int start, IReadOnlyList<bool> values)
        {
            for (int i = 0; i < values.Count; i++)
            {
                _api.WriteOutput(start + i, values[i]);
            }
        }
    }
}
