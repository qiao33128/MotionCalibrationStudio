using MotionCore.Abstractions.Motion;

namespace MotionCore.Simulation;

/// <summary>
/// <b>仿真单轴</b>
/// <para>
/// 用梯形速度曲线 + 可注入的定位误差，模拟一张真实运动控制卡的轴行为：
/// 使能、点动、绝对定位、回零、软/硬限位、跟随误差、报警与复位。
/// 位置由 S 形或 T 形曲线的解析式直接算出（不依赖固定步长积分），
/// 因此仿真结果是<b>确定性</b>的，可以放心用于单元测试断言。
/// </para>
/// </summary>
public sealed class SimulatedAxis
{
    private enum AxisMode
    {
        Idle,
        Positioning,
        Jogging,
        Homing,
    }

    private readonly Random _random;
    private AxisMode _mode = AxisMode.Idle;
    private double _lastUpdateSeconds;
    private double _moveStartSeconds;
    private double _startPosition;
    private double _targetPosition;
    private double _finalOffset;
    private double _jogVelocity;
    private double _homeSpeed;
    private TrapezoidalProfile _profile;

    public SimulatedAxis(AxisId axis, double initialPosition, double positioningNoise, double followingLagSeconds, Random random)
    {
        Axis = axis;
        Position = initialPosition;
        CommandPosition = initialPosition;
        PositioningNoise = positioningNoise;
        FollowingLagSeconds = followingLagSeconds;
        _random = random;
    }

    public AxisId Axis { get; }

    /// <summary>伺服使能状态。</summary>
    public bool ServoEnabled { get; private set; }

    /// <summary>是否已完成回零（建立用户坐标系）。</summary>
    public bool Homed { get; private set; }

    /// <summary>位置（mm 或 °）。</summary>
    public double Position { get; private set; }

    /// <summary>指令位置（卡内部规划值）。</summary>
    public double CommandPosition { get; private set; }

    /// <summary>当前速度。</summary>
    public double Velocity { get; private set; }

    public bool Alarm { get; private set; }

    public int AlarmCode { get; private set; }

    public bool PositiveLimitTriggered { get; private set; }

    public bool NegativeLimitTriggered { get; private set; }

    public bool SoftLimitEnabled { get; set; } = true;

    public double NegativeSoftLimit { get; set; } = -200d;

    public double PositiveSoftLimit { get; set; } = 200d;

    public double NegativeHardLimit { get; set; } = -215d;

    public double PositiveHardLimit { get; set; } = 215d;

    /// <summary>原点开关的机械位置（负方向回零时使用）。</summary>
    public double HomeSwitchPosition { get; set; } = -150d;

    public double HomeOffset { get; set; }

    public MoveProfile Profile { get; set; } = MoveProfile.Default;

    /// <summary>重复定位误差（高斯标准差，mm）。真实丝杆平台一般在 0.002 ~ 0.01 mm。</summary>
    public double PositioningNoise { get; set; }

    /// <summary>跟随滞后时间常数（秒），用于模拟指令位置与编码器反馈之间的差值。</summary>
    public double FollowingLagSeconds { get; set; }

    public bool IsMoving => _mode != AxisMode.Idle;

    public bool InPosition => _mode == AxisMode.Idle && !Alarm;

    /// <summary>是否在正方向上触发限位。</summary>
    public bool AtPositiveLimit => PositiveLimitTriggered || Position >= PositiveHardLimit;

    public bool AtNegativeLimit => NegativeLimitTriggered || Position <= NegativeHardLimit;

    // ───────────────────────── 指令下发 ─────────────────────────

    public int SetServoEnabled(bool enabled, double now)
    {
        Update(now);

        if (enabled && Alarm)
        {
            return SimulatedErrorCodes.AxisAlarm;
        }

        ServoEnabled = enabled;
        if (!enabled)
        {
            _mode = AxisMode.Idle;
            Velocity = 0d;
        }

        return 0;
    }

    public int SetPosition(double position, double now)
    {
        Update(now);
        Position = position;
        CommandPosition = position;
        return 0;
    }

    public int ConfigureProfile(MoveProfile profile, double now)
    {
        Update(now);

        if (profile.Velocity <= 0 || profile.Acceleration <= 0 || profile.Deceleration <= 0)
        {
            return SimulatedErrorCodes.InvalidParameter;
        }

        Profile = profile;
        return 0;
    }

    public int ConfigureSoftLimit(double negative, double positive, bool enabled, double now)
    {
        Update(now);

        if (negative >= positive)
        {
            return SimulatedErrorCodes.InvalidParameter;
        }

        NegativeSoftLimit = negative;
        PositiveSoftLimit = positive;
        SoftLimitEnabled = enabled;
        return 0;
    }

    public int MoveAbsolute(double target, double now)
    {
        Update(now);

        if (!ServoEnabled)
        {
            return SimulatedErrorCodes.ServoDisabled;
        }

        if (Alarm)
        {
            return SimulatedErrorCodes.AxisAlarm;
        }

        if (SoftLimitEnabled && (target < NegativeSoftLimit || target > PositiveSoftLimit))
        {
            return SimulatedErrorCodes.SoftLimitExceeded;
        }

        double distance = target - Position;
        if (Math.Abs(distance) < 1e-9)
        {
            return 0;
        }

        // 重复定位误差：每次定位落点都略有不同，量级与真实平台一致。
        // 标定算法必须在这种噪声下仍然稳定 —— 这正是仿真卡存在的意义。
        _finalOffset = PositioningNoise > 0
            ? NextGaussian() * PositioningNoise
            : 0d;

        _targetPosition = target + _finalOffset;
        _startPosition = Position;
        _profile = TrapezoidalProfile.Create(
            _targetPosition - _startPosition,
            Profile.Velocity,
            Profile.Acceleration,
            Profile.Deceleration);

        _moveStartSeconds = now;
        _mode = AxisMode.Positioning;
        return 0;
    }

    public int MoveRelative(double distance, double now) => MoveAbsolute(Position + distance, now);

    public int StartJog(MotionDirection direction, double velocity, double now)
    {
        Update(now);

        if (!ServoEnabled)
        {
            return SimulatedErrorCodes.ServoDisabled;
        }

        if (Alarm)
        {
            return SimulatedErrorCodes.AxisAlarm;
        }

        if ((direction == MotionDirection.Positive && AtPositiveLimit)
            || (direction == MotionDirection.Negative && AtNegativeLimit))
        {
            return SimulatedErrorCodes.HardLimit;
        }

        _jogVelocity = (int)direction * Math.Abs(velocity);
        _mode = AxisMode.Jogging;
        return 0;
    }

    public int StopJog(double now)
    {
        Update(now);
        if (_mode == AxisMode.Jogging)
        {
            _mode = AxisMode.Idle;
            Velocity = 0d;
        }

        return 0;
    }

    public int Home(HomeMode mode, double speed, double offset, double now)
    {
        Update(now);

        if (!ServoEnabled)
        {
            return SimulatedErrorCodes.ServoDisabled;
        }

        if (Alarm)
        {
            return SimulatedErrorCodes.AxisAlarm;
        }

        HomeOffset = offset;

        switch (mode)
        {
            case HomeMode.CurrentPosition:
                Position = offset;
                CommandPosition = offset;
                Homed = true;
                return 0;

            case HomeMode.NegativeLimit:
            case HomeMode.HomeSwitchWithIndex:
                // 负方向回零：速度取负，Update 里用 Math.Sign 得到方向
                _homeSpeed = -(Math.Abs(speed) <= 0 ? 10d : Math.Abs(speed));
                _startPosition = Position;
                _moveStartSeconds = now;
                _mode = AxisMode.Homing;
                return 0;

            case HomeMode.PositiveLimit:
                // 正方向回零：复用同一套逻辑，只是方向相反
                _homeSpeed = Math.Abs(speed) <= 0 ? 10d : Math.Abs(speed);
                _startPosition = Position;
                _moveStartSeconds = now;
                _mode = AxisMode.Homing;
                return 0;

            default:
                return SimulatedErrorCodes.InvalidParameter;
        }
    }

    public int Stop(bool emergency, double now)
    {
        Update(now);
        _mode = AxisMode.Idle;
        Velocity = 0d;

        if (emergency)
        {
            ServoEnabled = false;
            RaiseAlarm(SimulatedErrorCodes.EmergencyStop);
        }

        return 0;
    }

    public int ResetAlarm(double now)
    {
        Update(now);
        Alarm = false;
        AlarmCode = 0;
        PositiveLimitTriggered = false;
        NegativeLimitTriggered = false;
        return 0;
    }

    /// <summary>测试 / 演示专用：强制注入一次报警。</summary>
    public void InjectAlarmForTest(int errorCode)
    {
        _mode = AxisMode.Idle;
        Velocity = 0d;
        RaiseAlarm(errorCode);
    }

    // ───────────────────────── 状态推进 ─────────────────────────

    /// <summary>
    /// 推进仿真时间。所有位置都是按时间<b>解析</b>计算的，不做步长积分，
    /// 因此同一时刻读到的值永远一致，测试不会 flaky。
    /// </summary>
    public void Update(double now)
    {
        double deltaTime = now - _lastUpdateSeconds;
        _lastUpdateSeconds = now;
        if (deltaTime < 0)
        {
            deltaTime = 0;
        }

        switch (_mode)
        {
            case AxisMode.Idle:
                Velocity = 0d;
                UpdateFollowingError();
                CheckHardLimit();
                return;

            case AxisMode.Positioning:
            {
                double elapsed = now - _moveStartSeconds;
                double travel = _profile.TravelAt(elapsed);
                Position = _startPosition + (_profile.Sign * travel);
                Velocity = _profile.VelocityAt(elapsed) * _profile.Sign;

                if (_profile.IsFinished(elapsed))
                {
                    Position = _targetPosition;
                    Velocity = 0d;
                    _mode = AxisMode.Idle;
                }

                break;
            }

            case AxisMode.Homing:
            {
                double elapsed = now - _moveStartSeconds;
                double direction = Math.Sign(_homeSpeed);
                Position = _startPosition + (direction * Math.Abs(_homeSpeed) * elapsed);
                Velocity = _homeSpeed;

                bool reached = direction < 0
                    ? Position <= HomeSwitchPosition
                    : Position >= Math.Abs(HomeSwitchPosition);

                if (reached)
                {
                    Position = HomeOffset;
                    Velocity = 0d;
                    Homed = true;
                    _mode = AxisMode.Idle;
                }

                break;
            }

            case AxisMode.Jogging:
            {
                Position += _jogVelocity * deltaTime;
                Velocity = _jogVelocity;

                if (_jogVelocity > 0 && Position >= PositiveSoftLimit)
                {
                    Position = PositiveSoftLimit;
                    _mode = AxisMode.Idle;
                    Velocity = 0d;
                    NegativeLimitTriggered = false;
                    PositiveLimitTriggered = true;
                }
                else if (_jogVelocity < 0 && Position <= NegativeSoftLimit)
                {
                    Position = NegativeSoftLimit;
                    _mode = AxisMode.Idle;
                    Velocity = 0d;
                    NegativeLimitTriggered = true;
                }

                break;
            }

            default:
                return;
        }

        UpdateFollowingError();
        CheckHardLimit();
    }

    private void UpdateFollowingError()
    {
        // 指令位置超前实际位置，超前量 ≈ 速度 × 滞后时间常数。
        // 真实卡里这个差值就是"跟随误差"，超过阈值会报位置偏差过大。
        CommandPosition = Position + (Velocity * FollowingLagSeconds);
    }

    private void CheckHardLimit()
    {
        if (Alarm)
        {
            return;
        }

        if (Position <= NegativeHardLimit)
        {
            NegativeLimitTriggered = true;
            _mode = AxisMode.Idle;
            Velocity = 0d;
            RaiseAlarm(SimulatedErrorCodes.HardLimit);
        }
        else if (Position >= PositiveHardLimit)
        {
            PositiveLimitTriggered = true;
            _mode = AxisMode.Idle;
            Velocity = 0d;
            RaiseAlarm(SimulatedErrorCodes.HardLimit);
        }
    }

    private void RaiseAlarm(int code)
    {
        Alarm = true;
        AlarmCode = code;
    }

    /// <summary>Box–Muller 变换生成标准正态随机数（模拟定位重复性误差）。</summary>
    private double NextGaussian()
    {
        double u1 = 1.0 - _random.NextDouble();
        double u2 = _random.NextDouble();
        return Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Sin(2.0 * Math.PI * u2);
    }
}

/// <summary>仿真卡的错误码表（数值刻意与真实 SDK 的常见分段保持一致）。</summary>
public static class SimulatedErrorCodes
{
    public const int Ok = 0;
    public const int NotConnected = 1;
    public const int InvalidAxis = 2;
    public const int InvalidParameter = 3;
    public const int ServoDisabled = 4;
    public const int SoftLimitExceeded = 5;
    public const int AxisAlarm = 6;
    public const int HardLimit = 7;
    public const int EmergencyStop = 8;
    public const int CommunicationTimeout = 20;
}
