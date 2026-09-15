using MotionCore.Abstractions.Geometry;

namespace MotionCore.Calibration;

/// <summary>
/// 一个标定点：同一时刻的"图像像素坐标"与"平台机械坐标"。
/// </summary>
public sealed record CalibrationPoint(string Label, Point2D Image, Point2D Machine)
{
    public override string ToString() => $"{Label}: 像素 {Image} ↔ 机械 {Machine}";
}

/// <summary>
/// 标定点集合。它是标定的唯一输入，因此所有"数据是否可用"的判断都放在这里，
/// 而不是散在调用方。
/// </summary>
public sealed class CalibrationPointSet
{
    private readonly List<CalibrationPoint> _points = new();

    public CalibrationPointSet()
    {
    }

    public CalibrationPointSet(IEnumerable<CalibrationPoint> points)
    {
        ArgumentNullException.ThrowIfNull(points);
        _points.AddRange(points);
    }

    public IReadOnlyList<CalibrationPoint> Points => _points;

    public int Count => _points.Count;

    public void Add(string label, Point2D image, Point2D machine) =>
        _points.Add(new CalibrationPoint(label, image, machine));

    public void Add(CalibrationPoint point) => _points.Add(point);

    public void Clear() => _points.Clear();

    public Point2D ImageCentroid => new(
        _points.Average(point => point.Image.X),
        _points.Average(point => point.Image.Y));

    public Point2D MachineCentroid => new(
        _points.Average(point => point.Machine.X),
        _points.Average(point => point.Machine.Y));

    /// <summary>图像侧点的离散程度（平均到质心的距离，像素）。太小说明 9 点挤在一起。</summary>
    public double ImageSpread => _points.Average(point => point.Image.DistanceTo(ImageCentroid));

    /// <summary>机械侧标定行程（平均到质心的距离，mm）。</summary>
    public double MachineSpread => _points.Average(point => point.Machine.DistanceTo(MachineCentroid));

    /// <summary>任意两点之间的最小距离（mm）。过小说明有重复点，会拖垮条件数。</summary>
    public double MinimumInterPointDistanceMm()
    {
        double minimum = double.MaxValue;
        for (int i = 0; i < _points.Count; i++)
        {
            for (int j = i + 1; j < _points.Count; j++)
            {
                minimum = Math.Min(minimum, _points[i].Machine.DistanceTo(_points[j].Machine));
            }
        }

        return minimum == double.MaxValue ? 0d : minimum;
    }

    /// <summary>去掉第 <paramref name="index"/> 个点，用于留一交叉验证。</summary>
    public CalibrationPointSet Without(int index)
    {
        CalibrationPointSet subset = new();
        for (int i = 0; i < _points.Count; i++)
        {
            if (i != index)
            {
                subset.Add(_points[i]);
            }
        }

        return subset;
    }

    public IReadOnlyList<string> FindDuplicateLabels() =>
        _points
            .GroupBy(point => point.Label, StringComparer.OrdinalIgnoreCase)
            .Where(group => group.Count() > 1)
            .Select(group => group.Key)
            .ToList();

    public override string ToString() => $"{Count} 个标定点，行程范围 {MachineSpread * 2:F2} mm";
}
