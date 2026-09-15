using MotionCore.Abstractions.Geometry;
using MotionCore.Abstractions.Motion;
using MotionCore.Abstractions.Vision;

namespace MotionCore.Simulation.Vision;

/// <summary>仿真相机参数。</summary>
public sealed record SimulatedCameraOptions
{
    public int Width { get; init; } = 640;

    public int Height { get; init; } = 480;

    public CameraMount Mount { get; init; } = new();

    /// <summary>Mark 半径（像素）。</summary>
    public double MarkRadiusPx { get; init; } = 14d;

    /// <summary>边缘模糊宽度（像素），模拟光学离焦与镜头点扩散函数。</summary>
    public double BlurPx { get; init; } = 1.6d;

    public byte BackgroundLevel { get; init; } = 205;

    public byte MarkLevel { get; init; } = 28;

    /// <summary>图像噪声（灰度级，均匀分布幅度）。</summary>
    public double NoiseAmplitude { get; init; } = 4d;

    /// <summary>模拟曝光耗时（ms）。</summary>
    public int ExposureDelayMs { get; init; } = 12;

    public double PixelSizeUm { get; init; } = 10d;

    public int Seed { get; init; } = 20260915;

    /// <summary>是否在画面里撒一个干扰亮点（用于验证算法的鲁棒性）。</summary>
    public bool AddDecoyBlob { get; init; }
}

/// <summary>
/// <b>仿真相机</b>
/// <para>
/// 它真的在生成一张位图：按 <see cref="CameraMount"/> 把固定 Mark 的机械坐标投影成像素坐标，
/// 画出带平滑边缘的圆斑、叠加噪声，再交给视觉定位器去"找"。
/// </para>
/// <para>
/// <b>为什么不直接在标定点里塞像素值：</b>
/// 那样整条链路就是假的，标定算法永远不会暴露问题。
/// 现在这条链路是真的 图像生成 → 阈值分割 → 质心提取，
/// 因此视觉噪声、离焦、干扰项对精度的影响全都能被真实地观察到。
/// </para>
/// </summary>
public sealed class SimulatedCamera : ICamera
{
    private readonly IMotionController _controller;
    private readonly Random _random;
    private byte[] _buffer = [];

    public SimulatedCamera(
        IMotionController controller,
        SimulatedCameraOptions? options = null,
        Point2D? markMachinePosition = null)
    {
        _controller = controller ?? throw new ArgumentNullException(nameof(controller));
        Options = options ?? new SimulatedCameraOptions();
        _random = new Random(Options.Seed);
        MarkMachinePosition = markMachinePosition ?? new Point2D(0d, 0d);
    }

    public SimulatedCameraOptions Options { get; }

    public string Name => "仿真相机（SimulatedCamera，无需硬件）";

    public int Width => Options.Width;

    public int Height => Options.Height;

    public bool IsOpen { get; private set; }

    /// <summary>固定 Mark 的真实机械坐标（"实物"在哪里）。</summary>
    public Point2D MarkMachinePosition { get; set; }

    /// <summary>模拟 Mark 丢失（被遮挡 / 超出视野）。</summary>
    public bool HideMark { get; set; }

    /// <summary>已输出帧数。</summary>
    public int FrameIndex { get; private set; }

    /// <summary>当前帧里 Mark 的仿真真值像素位置（仅用于测试对照，业务代码不应使用）。</summary>
    public Point2D? GroundTruthMarkPixel { get; private set; }

    public Task OpenAsync(CancellationToken cancellationToken = default)
    {
        IsOpen = true;
        _buffer = new byte[Width * Height];
        return Task.CompletedTask;
    }

    public async Task<CameraFrame> CaptureAsync(CancellationToken cancellationToken = default)
    {
        if (!IsOpen)
        {
            throw new InvalidOperationException("相机尚未打开，请先调用 OpenAsync");
        }

        if (Options.ExposureDelayMs > 0)
        {
            await Task.Delay(Options.ExposureDelayMs, cancellationToken).ConfigureAwait(false);
        }

        MotionPose pose = await _controller.GetPositionAsync(cancellationToken).ConfigureAwait(false);

        Point2D? markPixel = HideMark
            ? null
            : Render(MarkMachinePosition - pose.Planar);

        GroundTruthMarkPixel = markPixel;

        return new CameraFrame
        {
            Width = Width,
            Height = Height,
            Pixels = (byte[])_buffer.Clone(),
            PixelSizeUm = Options.PixelSizeUm,
            Index = ++FrameIndex,
        };
    }

    public Task CloseAsync()
    {
        IsOpen = false;
        return Task.CompletedTask;
    }

    /// <summary>按"Mark 相对相机的机械位移"渲染一帧。</summary>
    private Point2D? Render(Point2D relativeMachine)
    {
        Point2D pixelOffset = Options.Mount.MachineToPixelOffset(relativeMachine);
        Point2D markCenter = new((Width / 2d) + pixelOffset.X, (Height / 2d) + pixelOffset.Y);

        bool visible = markCenter.X > -Options.MarkRadiusPx * 3
                       && markCenter.X < Width + (Options.MarkRadiusPx * 3)
                       && markCenter.Y > -Options.MarkRadiusPx * 3
                       && markCenter.Y < Height + (Options.MarkRadiusPx * 3);

        double background = Options.BackgroundLevel;
        double mark = Options.MarkLevel;
        double radius = Options.MarkRadiusPx;
        double blur = Math.Max(0.5d, Options.BlurPx);
        double noise = Options.NoiseAmplitude;

        Point2D? decoyCenter = Options.AddDecoyBlob
            ? new Point2D(Width * 0.18d, Height * 0.78d)
            : null;

        for (int y = 0; y < Height; y++)
        {
            int rowOffset = y * Width;
            double dy = y + 0.5d;

            for (int x = 0; x < Width; x++)
            {
                double dx = x + 0.5d;
                double value = background;

                if (visible)
                {
                    value = Blend(value, mark, Coverage(dx, dy, markCenter, radius, blur));
                }

                if (decoyCenter is not null)
                {
                    value = Blend(value, mark + 90d, Coverage(dx, dy, decoyCenter.Value, radius * 0.7d, blur));
                }

                // 均匀噪声：比高斯噪声便宜一个数量级，视觉上效果一致
                value += ((_random.NextDouble() * 2d) - 1d) * noise;

                _buffer[rowOffset + x] = (byte)Math.Clamp(value, 0d, 255d);
            }
        }

        return visible ? markCenter : null;
    }

    /// <summary>圆斑覆盖率：0 = 完全在圆内，1 = 完全在圆外，边缘用 smoothstep 过渡。</summary>
    private static double Coverage(double x, double y, Point2D center, double radius, double blur)
    {
        double distance = Math.Sqrt(((x - center.X) * (x - center.X)) + ((y - center.Y) * (y - center.Y)));
        double t = ((distance - radius) / blur) + 0.5d;
        t = Math.Clamp(t, 0d, 1d);

        // smoothstep：让边缘过渡更接近真实光学成像
        return t * t * (3d - (2d * t));
    }

    private static double Blend(double background, double foreground, double coverage) =>
        (foreground * (1d - coverage)) + (background * coverage);

    public void Dispose()
    {
        IsOpen = false;
        _buffer = [];
    }
}
