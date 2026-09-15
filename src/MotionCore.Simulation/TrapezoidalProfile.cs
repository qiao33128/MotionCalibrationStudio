namespace MotionCore.Simulation;

/// <summary>
/// <b>梯形速度曲线（T 型曲线）</b>
/// <para>
/// 真实运动控制卡收到"绝对定位"指令后，内部做的就是这件事：按给定的最大速度、
/// 加速度、减速度规划出一条速度曲线，再逐周期输出给伺服。
/// 理解它是理解"为什么等待到位不能用固定延时"的前提。
/// </para>
/// <para>
/// 曲线分三段：匀加速 → 匀速 → 匀减速。当行程太短、来不及加速到设定速度时，
/// 退化成三角形曲线（没有匀速段），峰值速度由行程与加减速度共同决定。
/// </para>
/// <code>
/// 速度
///   ^        ┌──────────┐
///   │       /            \
///   │      /              \
///   └─────┴──────┴───────┴────→ 时间
///        tAcc    tCruise   tDec
/// </code>
/// </summary>
public readonly struct TrapezoidalProfile
{
    private readonly double _accelerationTime;
    private readonly double _cruiseTime;
    private readonly double _decelerationTime;
    private readonly double _accelerationDistance;
    private readonly double _cruiseDistance;
    private readonly double _decelerationDistance;

    private TrapezoidalProfile(
        double distance,
        double sign,
        double peakVelocity,
        double acceleration,
        double deceleration,
        double accelerationTime,
        double cruiseTime,
        double decelerationTime,
        double accelerationDistance,
        double cruiseDistance,
        double decelerationDistance,
        bool isTriangular)
    {
        Distance = distance;
        Sign = sign;
        PeakVelocity = peakVelocity;
        Acceleration = acceleration;
        Deceleration = deceleration;
        _accelerationTime = accelerationTime;
        _cruiseTime = cruiseTime;
        _decelerationTime = decelerationTime;
        _accelerationDistance = accelerationDistance;
        _cruiseDistance = cruiseDistance;
        _decelerationDistance = decelerationDistance;
        IsTriangular = isTriangular;
    }

    /// <summary>行程绝对值。</summary>
    public double Distance { get; }

    /// <summary>运动方向（+1 / −1）。</summary>
    public double Sign { get; }

    /// <summary>实际达到的峰值速度（三角形曲线下会低于设定速度）。</summary>
    public double PeakVelocity { get; }

    public double Acceleration { get; }

    public double Deceleration { get; }

    /// <summary>是否退化成三角形曲线（行程太短）。</summary>
    public bool IsTriangular { get; }

    /// <summary>总运行时间（秒）。</summary>
    public double TotalTime => _accelerationTime + _cruiseTime + _decelerationTime;

    public static TrapezoidalProfile Create(double distance, double velocity, double acceleration, double deceleration)
    {
        if (velocity <= 0 || acceleration <= 0 || deceleration <= 0)
        {
            throw new ArgumentException("速度与加减速度必须为正数");
        }

        double sign = distance >= 0 ? 1d : -1d;
        double d = Math.Abs(distance);

        if (d <= 0)
        {
            return new TrapezoidalProfile(0, sign, 0, acceleration, deceleration, 0, 0, 0, 0, 0, 0, false);
        }

        double accelerationDistanceAtMaxSpeed = (velocity * velocity) / (2d * acceleration);
        double decelerationDistanceAtMaxSpeed = (velocity * velocity) / (2d * deceleration);

        if (accelerationDistanceAtMaxSpeed + decelerationDistanceAtMaxSpeed <= d)
        {
            // 梯形：能加速到设定速度并匀速一段
            double accelerationTime = velocity / acceleration;
            double decelerationTime = velocity / deceleration;
            double cruiseDistance = d - accelerationDistanceAtMaxSpeed - decelerationDistanceAtMaxSpeed;
            double cruiseTime = cruiseDistance / velocity;

            return new TrapezoidalProfile(
                d, sign, velocity, acceleration, deceleration,
                accelerationTime, cruiseTime, decelerationTime,
                accelerationDistanceAtMaxSpeed, cruiseDistance, decelerationDistanceAtMaxSpeed,
                false);
        }

        // 三角形：加速到峰值后立刻开始减速
        // 由 ½·a·t_a² + ½·dec·t_d² = d 且 a·t_a = dec·t_d 解得：
        double peak = Math.Sqrt((2d * d * acceleration * deceleration) / (acceleration + deceleration));
        double accelerationTimeTri = peak / acceleration;
        double decelerationTimeTri = peak / deceleration;
        double accelerationDistance = (peak * peak) / (2d * acceleration);
        double decelerationDistance = (peak * peak) / (2d * deceleration);

        return new TrapezoidalProfile(
            d, sign, peak, acceleration, deceleration,
            accelerationTimeTri, 0d, decelerationTimeTri,
            accelerationDistance, 0d, decelerationDistance,
            true);
    }

    /// <summary>t 时刻已经走过的位移绝对值（0 ≤ t ≤ TotalTime）。</summary>
    public double TravelAt(double t)
    {
        if (t <= 0)
        {
            return 0d;
        }

        if (t >= TotalTime)
        {
            return Distance;
        }

        if (t <= _accelerationTime)
        {
            return 0.5d * Acceleration * t * t;
        }

        double afterAcceleration = t - _accelerationTime;
        if (afterAcceleration <= _cruiseTime)
        {
            return _accelerationDistance + (PeakVelocity * afterAcceleration);
        }

        double decelerationElapsed = afterAcceleration - _cruiseTime;
        return _accelerationDistance
               + _cruiseDistance
               + (PeakVelocity * decelerationElapsed)
               - (0.5d * Deceleration * decelerationElapsed * decelerationElapsed);
    }

    /// <summary>t 时刻的速度大小（用于 UI 显示与跟随误差计算）。</summary>
    public double VelocityAt(double t)
    {
        if (t <= 0 || t >= TotalTime)
        {
            return 0d;
        }

        if (t <= _accelerationTime)
        {
            return Acceleration * t;
        }

        double afterAcceleration = t - _accelerationTime;
        if (afterAcceleration <= _cruiseTime)
        {
            return PeakVelocity;
        }

        double decelerationElapsed = afterAcceleration - _cruiseTime;
        return Math.Max(0d, PeakVelocity - (Deceleration * decelerationElapsed));
    }

    public bool IsFinished(double t) => t >= TotalTime;

    /// <summary>按时间等间隔采样整条曲线，用于画曲线图 / 单元测试。</summary>
    public IReadOnlyList<(double Time, double Velocity)> Sample(int count = 50)
    {
        List<(double, double)> samples = new(count);
        for (int i = 0; i <= count; i++)
        {
            double t = TotalTime * i / count;
            samples.Add((t, VelocityAt(t)));
        }

        return samples;
    }
}
