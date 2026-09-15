using MotionCore.Abstractions.Geometry;

namespace MotionCore.Abstractions.Vision;

/// <summary>
/// 一帧灰度图。刻意保持成最朴素的 byte[] + 宽高，
/// 这样换任意相机 SDK / Halcon / OpenCV 都只是换一个 <see cref="ICamera"/> 实现。
/// </summary>
public sealed record CameraFrame
{
    public required int Width { get; init; }

    public required int Height { get; init; }

    /// <summary>8 位灰度像素，行优先，长度 = Width × Height。</summary>
    public required byte[] Pixels { get; init; }

    /// <summary>单像素物理尺寸（μm/px），用于估算视野。</summary>
    public double PixelSizeUm { get; init; } = 10d;

    /// <summary>帧序号，用于排查丢帧 / 用旧图对位。</summary>
    public int Index { get; init; }

    public DateTimeOffset Timestamp { get; init; } = DateTimeOffset.Now;

    public byte this[int x, int y] => Pixels[(y * Width) + x];

    /// <summary>图像中心（标定的参考点，一般让 Mark 落在这里）。</summary>
    public Point2D Center => new(Width / 2d, Height / 2d);

    /// <summary>视野尺寸（mm），= 宽 × PixelSizeUm / 1000。</summary>
    public Point2D FieldOfViewMm => new(Width * PixelSizeUm / 1000d, Height * PixelSizeUm / 1000d);

    public void Validate()
    {
        if (Width <= 0 || Height <= 0)
        {
            throw new ArgumentException("图像宽高必须为正");
        }

        if (Pixels.Length != Width * Height)
        {
            throw new ArgumentException($"像素缓冲长度 {Pixels.Length} 与 {Width}×{Height} 不匹配");
        }
    }
}

/// <summary>相机抽象。</summary>
public interface ICamera : IDisposable
{
    string Name { get; }

    int Width { get; }

    int Height { get; }

    bool IsOpen { get; }

    Task OpenAsync(CancellationToken cancellationToken = default);

    Task<CameraFrame> CaptureAsync(CancellationToken cancellationToken = default);

    Task CloseAsync();
}

/// <summary>Mark 模板参数。</summary>
public sealed record MarkTemplate(string Name, double ExpectedRadiusPx, int MinContrast = 40)
{
    /// <summary>默认模板：半径 12px 的圆形暗 Mark。</summary>
    public static readonly MarkTemplate Default = new("圆形 Mark", 12d, 40);
}

/// <summary>视觉定位结果。</summary>
public sealed record VisionMark(Point2D Pixel, double Score, double RadiusPx, int PixelCount)
{
    public override string ToString() =>
        $"{Pixel} 置信度 {Score:P1} 半径 {RadiusPx:F2}px 面积 {PixelCount}px";
}

/// <summary>视觉定位器：从一帧图里找出 Mark 的像素坐标。</summary>
public interface IVisionLocator
{
    string Name { get; }

    Task<VisionMark?> FindMarkAsync(CameraFrame frame, MarkTemplate? template = null, CancellationToken cancellationToken = default);
}
