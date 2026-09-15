using MotionCore.Abstractions.Geometry;
using MotionCore.Abstractions.Motion;
using MotionCore.Simulation;
using MotionCore.Simulation.Vision;

namespace MotionCore.Tests;

/// <summary>
/// 测试夹具：一行代码搭出"仿真卡 + 仿真相机 + 视觉定位器"的完整系统。
/// <para>
/// 这个夹具本身就是整个架构最直接的证明 ——
/// 一条真实的"运动 → 拍照 → 视觉 → 标定 → 对位"链路，在没有任何硬件的 CI 机器上就能完整跑通。
/// </para>
/// </summary>
internal static class TestHarness
{
    /// <summary>默认 Mark 的真实机械坐标。</summary>
    public static readonly Point2D DefaultMarkPosition = new(12.5d, -7.25d);

    internal sealed record System(
        SimulatedMotionController Controller,
        SimulatedCamera Camera,
        BlobCentroidLocator Locator,
        CameraMount Mount) : IAsyncDisposable
    {
        public async ValueTask DisposeAsync()
        {
            Camera.Dispose();
            await Controller.DisposeAsync().ConfigureAwait(false);
        }
    }

    internal static async Task<System> CreateSystemAsync(
        CameraMount? mount = null,
        Point2D? markPosition = null,
        double positioningNoise = 0.0015d,
        double imageNoise = 3d,
        bool addDecoyBlob = false)
    {
        CameraMount effectiveMount = mount ?? new CameraMount
        {
            PixelsPerMmU = 24d,
            PixelsPerMmV = 24d,
            RotationDeg = 1.7d,
            FlipVertical = true,
        };

        SimulatedCardOptions cardOptions = new()
        {
            PositioningNoise = positioningNoise,
            FollowingLagSeconds = 0.002d,
            ConnectDelayMs = 0, // 测试里不需要模拟握手耗时
            Seed = 20260915,
        };

        SimulatedMotionController controller = new(cardOptions, ControllerOptions());
        await controller.InitializeAsync().ConfigureAwait(false);

        SimulatedCamera camera = new(
            controller,
            new SimulatedCameraOptions
            {
                Mount = effectiveMount,
                NoiseAmplitude = imageNoise,
                ExposureDelayMs = 0,
                AddDecoyBlob = addDecoyBlob,
                Seed = 20260915,
            },
            markPosition ?? DefaultMarkPosition);

        await camera.OpenAsync().ConfigureAwait(false);

        return new System(controller, camera, new BlobCentroidLocator(), effectiveMount);
    }

    /// <summary>快速定位用的 profile（测试中不希望等太久）。</summary>
    internal static MoveProfile FastProfile => new(60d, 3000d, 3000d);

    /// <summary>极速 profile，用于大量重复定位的统计类测试。</summary>
    internal static MoveProfile UltraFastProfile => new(400d, 40000d, 40000d);

    /// <summary>测试用控制器配置：把回零速度调快，避免用例在 150 mm 的原点搜索上耗时。</summary>
    internal static MotionControllerOptions ControllerOptions() => new()
    {
        StatusPollIntervalMs = 2,
        MotionTimeout = TimeSpan.FromSeconds(10),
        HomeTimeout = TimeSpan.FromSeconds(10),
        Axes =
        [
            new AxisConfiguration { Axis = AxisId.X, HomeSpeed = 400d, Profile = FastProfile },
            new AxisConfiguration { Axis = AxisId.Y, HomeSpeed = 400d, Profile = FastProfile },
            new AxisConfiguration
            {
                Axis = AxisId.Z,
                SoftLimitMin = -100d,
                SoftLimitMax = 50d,
                HomeSpeed = 400d,
                Profile = FastProfile,
            },
            new AxisConfiguration
            {
                Axis = AxisId.R,
                SoftLimitMin = -180d,
                SoftLimitMax = 180d,
                HomeSpeed = 400d,
                Profile = new MoveProfile(90d, 720d, 720d),
            },
        ],
    };

    /// <summary>生成 3×3 的机械坐标网格。</summary>
    internal static IReadOnlyList<Point2D> BuildGrid(Point2D center, double span, int gridSize = 3)
    {
        double spacing = (2d * span) / (gridSize - 1);
        List<Point2D> points = new(gridSize * gridSize);

        for (int row = 0; row < gridSize; row++)
        {
            for (int column = 0; column < gridSize; column++)
            {
                points.Add(new Point2D(
                    center.X - span + (column * spacing),
                    center.Y - span + (row * spacing)));
            }
        }

        return points;
    }

    /// <summary>可复现的高斯噪声源（固定种子，保证测试不 flaky）。</summary>
    internal static Random SeededRandom(int seed = 12345) => new(seed);
}
