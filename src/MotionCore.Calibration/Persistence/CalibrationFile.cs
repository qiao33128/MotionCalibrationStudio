using System.Globalization;
using System.Text;
using MotionCore.Abstractions.Geometry;
using MotionCore.Calibration.Models;

namespace MotionCore.Calibration.Persistence;

/// <summary>
/// 标定文件（CSV）。
/// <para>
/// <b>设计取舍：文件里存的是"原始标定点 + 模型种类"，加载时重新解算，而不是直接存矩阵系数。</b>
/// 原因是可追溯性：现场交接时能一眼看到 9 个点的原始数据、能改一个点重算看影响，
/// 而只存 6 个系数的话出了问题根本查不出来是哪一步错了。
/// 已解算出的系数与精度指标同时以注释形式写入文件，便于人工核对与版本比对。
/// </para>
/// </summary>
public static class CalibrationFile
{
    public const string FileSignature = "MotionCalibrationStudio-Calibration";
    public const int FileVersion = 1;

    private static readonly UTF8Encoding Utf8WithBom = new(encoderShouldEmitUTF8Identifier: true);

    /// <summary>序列化为 CSV 文本（含 UTF-8 BOM 由 <see cref="Save"/> 处理，便于 Excel 直接打开）。</summary>
    public static string ToCsv(CalibrationResult result)
    {
        ArgumentNullException.ThrowIfNull(result);
        CultureInfo culture = CultureInfo.InvariantCulture;
        StringBuilder builder = new();

        builder.AppendLine($"# {FileSignature}");
        builder.AppendLine($"# Version,{FileVersion.ToString(culture)}");
        builder.AppendLine($"# Model,{result.Kind}");
        builder.AppendLine($"# CreatedAt,{result.CreatedAt:O}");
        builder.AppendLine($"# PointCount,{result.Residuals.Count.ToString(culture)}");
        builder.AppendLine($"# MachineSpreadMm,{result.MachineSpreadMm.ToString("F6", culture)}");
        builder.AppendLine($"# NormalizeCoordinates,{result.Options.NormalizeCoordinates}");
        builder.AppendLine($"# RmseMm,{result.RmseMm.ToString("F8", culture)}");
        builder.AppendLine($"# MeanErrorMm,{result.MeanErrorMm.ToString("F8", culture)}");
        builder.AppendLine($"# MaxErrorMm,{result.MaxErrorMm.ToString("F8", culture)}");
        builder.AppendLine($"# ConditionNumber,{result.ConditionNumber.ToString("E6", culture)}");
        builder.AppendLine($"# Quality,{result.Quality}");
        builder.AppendLine(
            $"# ModelParameters,\"{result.Model.DescribeParameters().Replace("\"", "'", StringComparison.Ordinal)}\"");

        string coefficients = string.Join(
            "|",
            result.Model.ToCoefficients().Select(value => value.ToString("R", culture)));
        builder.AppendLine($"# Coefficients,{coefficients}");

        builder.AppendLine("Label,ImageU,ImageV,MachineX,MachineY,PredictedX,PredictedY,ErrorMm");

        foreach (CalibrationResidual residual in result.Residuals)
        {
            builder.AppendLine(string.Join(
                ",",
                Escape(residual.Label),
                residual.Image.X.ToString("F6", culture),
                residual.Image.Y.ToString("F6", culture),
                residual.MachineMeasured.X.ToString("F6", culture),
                residual.MachineMeasured.Y.ToString("F6", culture),
                residual.MachinePredicted.X.ToString("F6", culture),
                residual.MachinePredicted.Y.ToString("F6", culture),
                residual.ErrorMagnitude.ToString("F8", culture)));
        }

        return builder.ToString();
    }

    /// <summary>写入文件（UTF-8 with BOM，Excel 双击不乱码）。</summary>
    public static void Save(CalibrationResult result, string path)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(path);
        string? directory = Path.GetDirectoryName(Path.GetFullPath(path));
        if (!string.IsNullOrEmpty(directory))
        {
            Directory.CreateDirectory(directory);
        }

        File.WriteAllText(path, ToCsv(result), Utf8WithBom);
    }

    /// <summary>
    /// 读取标定文件并重新解算。
    /// <para>重新解算是刻意的：加载得到的 <see cref="CalibrationResult"/> 一定与当前算法版本一致，
    /// 不会出现"文件里的系数是旧版本算的、现在代码已经改了"这种隐性不一致。</para>
    /// </summary>
    public static CalibrationResult Load(string path, CalibrationOptions? options = null)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(path);
        return FromCsv(File.ReadAllText(path), options);
    }

    public static CalibrationResult FromCsv(string content, CalibrationOptions? options = null)
    {
        ArgumentNullException.ThrowIfNull(content);

        CalibrationPointSet pointSet = ParsePointSet(content, out CalibrationModelKind? declaredModel);

        CalibrationOptions effective = options ?? new CalibrationOptions();
        if (options is null && declaredModel is not null)
        {
            // 文件里声明的模型优先于默认值：加载旧标定文件不该悄悄换成别的模型
            effective = effective.WithModel(declaredModel.Value);
        }

        return NinePointCalibrator.Calibrate(pointSet, effective);
    }

    /// <summary>只解析标定点，不做解算（供 UI 展示与手工修正）。</summary>
    public static CalibrationPointSet ParsePointSet(string content, out CalibrationModelKind? declaredModel)
    {
        declaredModel = null;
        CalibrationPointSet pointSet = new();
        bool headerSeen = false;

        foreach (string rawLine in content.Split('\n'))
        {
            string line = rawLine.TrimEnd('\r').Trim();
            if (line.Length == 0)
            {
                continue;
            }

            if (line.StartsWith('#'))
            {
                string body = line.TrimStart('#').Trim();
                int separator = body.IndexOf(',', StringComparison.Ordinal);
                if (separator <= 0)
                {
                    continue;
                }

                string key = body[..separator].Trim();
                string value = body[(separator + 1)..].Trim();

                if (key.Equals("Model", StringComparison.OrdinalIgnoreCase)
                    && Enum.TryParse(value, ignoreCase: true, out CalibrationModelKind model))
                {
                    declaredModel = model;
                }

                continue;
            }

            if (!headerSeen)
            {
                headerSeen = true;
                continue;
            }

            string[] fields = SplitCsv(line);
            if (fields.Length < 5)
            {
                throw new FormatException($"标定文件格式错误，数据行至少需要 5 列：{line}");
            }

            if (!double.TryParse(fields[1], NumberStyles.Float, CultureInfo.InvariantCulture, out double u)
                || !double.TryParse(fields[2], NumberStyles.Float, CultureInfo.InvariantCulture, out double v)
                || !double.TryParse(fields[3], NumberStyles.Float, CultureInfo.InvariantCulture, out double x)
                || !double.TryParse(fields[4], NumberStyles.Float, CultureInfo.InvariantCulture, out double y))
            {
                throw new FormatException($"标定文件数值解析失败：{line}");
            }

            pointSet.Add(fields[0], new Point2D(u, v), new Point2D(x, y));
        }

        if (pointSet.Count == 0)
        {
            throw new FormatException("标定文件中没有有效的标定点");
        }

        return pointSet;
    }

    private static string Escape(string value) =>
        value.Contains(',', StringComparison.Ordinal) || value.Contains('"', StringComparison.Ordinal)
            ? $"\"{value.Replace("\"", "\"\"", StringComparison.Ordinal)}\""
            : value;

    private static string[] SplitCsv(string line)
    {
        List<string> fields = new();
        StringBuilder current = new();
        bool inQuotes = false;

        for (int i = 0; i < line.Length; i++)
        {
            char character = line[i];
            if (inQuotes)
            {
                if (character == '"')
                {
                    if (i + 1 < line.Length && line[i + 1] == '"')
                    {
                        current.Append('"');
                        i++;
                    }
                    else
                    {
                        inQuotes = false;
                    }
                }
                else
                {
                    current.Append(character);
                }
            }
            else if (character == '"')
            {
                inQuotes = true;
            }
            else if (character == ',')
            {
                fields.Add(current.ToString());
                current.Clear();
            }
            else
            {
                current.Append(character);
            }
        }

        fields.Add(current.ToString());
        return fields.ToArray();
    }
}
