using MotionCore.Simulation;
using Xunit;

namespace MotionCore.Tests.Motion;

/// <summary>梯形速度曲线的数学正确性。</summary>
public class TrapezoidalProfileTests
{
    [Fact]
    public void 行程足够长时应形成梯形曲线()
    {
        // 10 mm，20 mm/s，500 mm/s²：加速段 0.4 mm、减速段 0.4 mm，够加速到设定速度
        TrapezoidalProfile profile = TrapezoidalProfile.Create(10d, 20d, 500d, 500d);

        Assert.False(profile.IsTriangular);
        Assert.Equal(20d, profile.PeakVelocity, 9);

        // 加速段位移 = ½·a·t² = ½ × 500 × 0.04² = 0.4 mm
        Assert.Equal(0.4d, profile.TravelAt(0.04d), 9);
        Assert.Equal(10d, profile.TravelAt(profile.TotalTime), 9);

        // 总时间 = 加速 0.04 + 匀速 0.46 + 减速 0.04
        Assert.Equal(0.54d, profile.TotalTime, 9);
        Assert.Equal(20d, profile.VelocityAt(0.25d), 9);
    }

    [Fact]
    public void 行程太短时应退化为三角形曲线()
    {
        // 0.1 mm 太短：加速到 20 mm/s 需要 0.4 mm 的加速距离
        TrapezoidalProfile profile = TrapezoidalProfile.Create(0.1d, 20d, 500d, 500d);

        Assert.True(profile.IsTriangular);

        // vPeak = √(2·d·a·dec/(a+dec)) = √(2×0.1×500×500/1000) = √50 ≈ 7.0711
        Assert.Equal(Math.Sqrt(50d), profile.PeakVelocity, 9);
        Assert.True(profile.PeakVelocity < 20d);

        // 加减速距离之和必须正好等于行程
        Assert.Equal(0.1d, profile.TravelAt(profile.TotalTime), 9);
    }

    [Fact]
    public void 速度曲线应连续且首尾为零()
    {
        TrapezoidalProfile profile = TrapezoidalProfile.Create(25d, 30d, 800d, 1200d);

        const int sampleCount = 200;
        IReadOnlyList<(double Time, double Velocity)> samples = profile.Sample(sampleCount);

        Assert.Equal(0d, samples[0].Velocity, 9);
        Assert.Equal(0d, samples[^1].Velocity, 9);
        Assert.All(samples, sample => Assert.True(sample.Velocity >= 0d));
        Assert.All(samples, sample => Assert.True(sample.Velocity <= profile.PeakVelocity + 1e-9));

        // 连续性判据：每个采样间隔内的速度增量不可能超过「最大加减速度 × 采样间隔」。
        // 直接跟峰值速度比较是没有意义的 —— 那只是在检查采样的疏密程度。
        double sampleInterval = profile.TotalTime / sampleCount;
        double maximumDelta = Math.Max(profile.Acceleration, profile.Deceleration) * sampleInterval * 1.01d;

        for (int i = 1; i < samples.Count; i++)
        {
            double delta = Math.Abs(samples[i].Velocity - samples[i - 1].Velocity);
            Assert.True(
                delta <= maximumDelta,
                $"速度在 t={samples[i].Time:F4} 处出现跳变：Δv={delta:F6}，上限 {maximumDelta:F6}");
        }
    }

    [Fact]
    public void 位移应随时间单调不减()
    {
        TrapezoidalProfile profile = TrapezoidalProfile.Create(15d, 25d, 600d, 600d);
        double previous = -1d;

        for (int i = 0; i <= 100; i++)
        {
            double travel = profile.TravelAt(profile.TotalTime * i / 100d);
            Assert.True(travel >= previous - 1e-12, "位移必须单调不减");
            previous = travel;
        }

        Assert.Equal(15d, previous, 9);
    }

    [Fact]
    public void 负方向运动应正确记录方向()
    {
        TrapezoidalProfile profile = TrapezoidalProfile.Create(-8d, 20d, 500d, 500d);

        Assert.Equal(-1d, profile.Sign);
        Assert.Equal(8d, profile.Distance, 9);
        Assert.Equal(8d, profile.TravelAt(profile.TotalTime), 9);
    }

    [Fact]
    public void 零行程应返回零时间曲线()
    {
        TrapezoidalProfile profile = TrapezoidalProfile.Create(0d, 20d, 500d, 500d);

        Assert.Equal(0d, profile.TotalTime, 12);
        Assert.Equal(0d, profile.PeakVelocity, 12);
    }

    [Fact]
    public void 非法参数应被拒绝()
    {
        Assert.Throws<ArgumentException>(() => TrapezoidalProfile.Create(10d, 0d, 500d, 500d));
        Assert.Throws<ArgumentException>(() => TrapezoidalProfile.Create(10d, 20d, -1d, 500d));
    }
}
