using MotionCore.Abstractions.Geometry;
using MotionCore.Calibration;
using MotionCore.Calibration.Models;
using MotionCore.Calibration.Persistence;
using Xunit;

namespace MotionCore.Tests.Calibration;

/// <summary>标定文件（CSV）往返测试。</summary>
public class CalibrationFileTests
{
    private static CalibrationResult CreateResult()
    {
        AffineTransform2D truth = AffineCalibrationTests.CreateGroundTruth();
        CalibrationPointSet set = new();

        foreach (Point2D machine in TestHarness.BuildGrid(AffineCalibrationTests.MachineCenter, span: 6d))
        {
            set.Add($"P{set.Count + 1}", truth.MachineToImage(machine), machine);
        }

        return NinePointCalibrator.Calibrate(set, new CalibrationOptions
        {
            Model = CalibrationModelKind.Affine,
            RunLeaveOneOutValidation = false,
        });
    }

    [Fact]
    public void 序列化再反序列化_标定结果应完全一致()
    {
        CalibrationResult original = CreateResult();
        string csv = CalibrationFile.ToCsv(original);

        Assert.Contains("MotionCalibrationStudio-Calibration", csv, StringComparison.Ordinal);
        Assert.Contains("Label,ImageU,ImageV,MachineX,MachineY", csv, StringComparison.Ordinal);

        CalibrationResult reloaded = CalibrationFile.FromCsv(csv);

        Assert.Equal(original.Kind, reloaded.Kind);
        Assert.Equal(original.Residuals.Count, reloaded.Residuals.Count);

        // 文件里的标定点按 1e-6 精度保存（远超任何机构的实际重复定位精度），
        // 重新解算后参数会有 1e-8 量级的差异，因此按 6 位小数比对
        Assert.Equal(original.RmseMm, reloaded.RmseMm, 6);

        double[] originalCoefficients = original.Model.ToCoefficients();
        double[] reloadedCoefficients = reloaded.Model.ToCoefficients();
        for (int i = 0; i < originalCoefficients.Length; i++)
        {
            Assert.Equal(originalCoefficients[i], reloadedCoefficients[i], 6);
        }
    }

    [Fact]
    public void 写入文件再读回_标定点与模型均应保持一致()
    {
        CalibrationResult original = CreateResult();
        string path = Path.Combine(Path.GetTempPath(), $"calibration-{Guid.NewGuid():N}.csv");

        try
        {
            CalibrationFile.Save(original, path);
            Assert.True(File.Exists(path));

            CalibrationResult reloaded = CalibrationFile.Load(path);

            Assert.Equal(original.RmseMm, reloaded.RmseMm, 6);
            Assert.Equal(CalibrationQuality.Excellent, reloaded.Quality);
        }
        finally
        {
            if (File.Exists(path))
            {
                File.Delete(path);
            }
        }
    }

    [Fact]
    public void 文件里声明的模型应优先于调用方默认值()
    {
        AffineTransform2D truth = AffineCalibrationTests.CreateGroundTruth();
        CalibrationPointSet set = new();

        foreach (Point2D machine in TestHarness.BuildGrid(AffineCalibrationTests.MachineCenter, span: 6d))
        {
            set.Add($"P{set.Count + 1}", truth.MachineToImage(machine), machine);
        }

        CalibrationResult homography = NinePointCalibrator.Calibrate(set, new CalibrationOptions
        {
            Model = CalibrationModelKind.Homography,
            RunLeaveOneOutValidation = false,
        });

        string csv = CalibrationFile.ToCsv(homography);

        // 不显式指定模型时应沿用文件里声明的单应模型，而不是悄悄退回仿射
        CalibrationResult reloaded = CalibrationFile.FromCsv(csv);
        Assert.Equal(CalibrationModelKind.Homography, reloaded.Kind);
    }

    [Fact]
    public void 解析非法文件应抛出格式异常()
    {
        const string broken = """
            # MotionCalibrationStudio-Calibration
            # Version,1
            Label,ImageU,ImageV,MachineX,MachineY
            P1,abc,240,0,0
            """;

        Assert.Throws<FormatException>(() => CalibrationFile.FromCsv(broken));
    }

    [Fact]
    public void 空文件应抛出格式异常()
    {
        Assert.Throws<FormatException>(() => CalibrationFile.FromCsv("# MotionCalibrationStudio-Calibration\n"));
    }
}
