using MotionCore.Abstractions.Geometry;
using MotionCore.Abstractions.Vision;

namespace MotionCore.Simulation.Vision;

/// <summary>
/// <b>基于 Otsu 阈值 + 灰度加权质心的 Mark 定位器</b>
/// <para>
/// 真实的工业视觉里，圆形 Mark 最常用的定位手段就是"阈值分割 + 质心"：
/// 它比模板匹配快一个数量级，对光照渐变也不敏感，亚像素精度可以做到 0.05 px 量级
/// —— 这对于标定 0.01 mm 级别的精度完全够用。
/// </para>
/// <list type="number">
///   <item><b>Otsu 自动阈值</b>：遍历 0~255 求类间方差最大处，不需要人工调阈值，
///         换料、换光照也不用重新配参数。</item>
///   <item><b>灰度加权质心</b>：不只用二值掩码，而是用 (阈值 − 灰度) 作为权重，
///         相当于把边缘的过渡像素也按比例算进去，因此天然具备亚像素精度。</item>
///   <item><b>置信度</b>：由"对比度"与"半径是否接近模板"两项合成，
///         用来挡住"Mark 丢失但阈值仍然切出一块噪声"这种假识别 ——
///         在设备上，一次假识别就是一次撞机。</item>
/// </list>
/// </summary>
public sealed class BlobCentroidLocator : IVisionLocator
{
    public string Name => "Otsu 阈值 + 灰度加权质心";

    public Task<VisionMark?> FindMarkAsync(
        CameraFrame frame,
        MarkTemplate? template = null,
        CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(frame);
        frame.Validate();

        MarkTemplate effective = template ?? MarkTemplate.Default;
        int threshold = ComputeOtsuThreshold(frame.Pixels);

        double weightedX = 0d;
        double weightedY = 0d;
        double weightSum = 0d;
        double darkSum = 0d;
        long darkCount = 0;

        for (int y = 0; y < frame.Height; y++)
        {
            int rowOffset = y * frame.Width;
            for (int x = 0; x < frame.Width; x++)
            {
                double value = frame.Pixels[rowOffset + x];
                if (value > threshold)
                {
                    continue;
                }

                double weight = threshold - value;
                weightedX += weight * (x + 0.5d);
                weightedY += weight * (y + 0.5d);
                weightSum += weight;
                darkSum += value;
                darkCount++;
            }
        }

        if (darkCount == 0 || weightSum <= 0)
        {
            return Task.FromResult<VisionMark?>(null);
        }

        Point2D centroid = new(weightedX / weightSum, weightedY / weightSum);

        double blobMean = darkSum / darkCount;
        double backgroundSum = 0d;
        long backgroundCount = 0;
        for (int i = 0; i < frame.Pixels.Length; i++)
        {
            if (frame.Pixels[i] > threshold)
            {
                backgroundSum += frame.Pixels[i];
                backgroundCount++;
            }
        }

        double backgroundMean = backgroundCount > 0 ? backgroundSum / backgroundCount : 0d;
        double contrast = backgroundMean - blobMean;

        if (contrast < effective.MinContrast)
        {
            return Task.FromResult<VisionMark?>(null);
        }

        double radius = Math.Sqrt(darkCount / Math.PI);

        // 置信度：对比度（归一化到 128 灰度级）+ 尺寸是否符合模板
        double contrastScore = Math.Clamp(contrast / 128d, 0d, 1d);
        double sizeScore = effective.ExpectedRadiusPx > 0
            ? Math.Clamp(1d - (Math.Abs(radius - effective.ExpectedRadiusPx) / effective.ExpectedRadiusPx), 0d, 1d)
            : 1d;

        double score = (0.5d * contrastScore) + (0.5d * sizeScore);

        return Task.FromResult<VisionMark?>(new VisionMark(centroid, score, radius, (int)darkCount));
    }

    /// <summary>Otsu 最大类间方差法自动阈值。</summary>
    public static int ComputeOtsuThreshold(byte[] pixels)
    {
        ArgumentNullException.ThrowIfNull(pixels);

        int[] histogram = new int[256];
        foreach (byte pixel in pixels)
        {
            histogram[pixel]++;
        }

        long total = pixels.Length;
        double sumAll = 0d;
        for (int i = 0; i < 256; i++)
        {
            sumAll += i * (double)histogram[i];
        }

        long backgroundWeight = 0;
        double backgroundSum = 0d;
        double bestVariance = -1d;
        int bestThreshold = 128;

        for (int t = 0; t < 256; t++)
        {
            backgroundWeight += histogram[t];
            if (backgroundWeight == 0)
            {
                continue;
            }

            long foregroundWeight = total - backgroundWeight;
            if (foregroundWeight == 0)
            {
                break;
            }

            backgroundSum += t * (double)histogram[t];

            double backgroundMean = backgroundSum / backgroundWeight;
            double foregroundMean = (sumAll - backgroundSum) / foregroundWeight;
            double difference = backgroundMean - foregroundMean;
            double variance = backgroundWeight * (double)foregroundWeight * difference * difference;

            if (variance > bestVariance)
            {
                bestVariance = variance;
                bestThreshold = t;
            }
        }

        return bestThreshold;
    }
}
