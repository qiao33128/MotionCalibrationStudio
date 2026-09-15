using MotionCore.Abstractions.Geometry;
using MotionCore.Calibration.LinearAlgebra;
using MotionCore.Calibration.Models;
using MotionCore.Calibration.Validation;

namespace MotionCore.Calibration;

/// <summary>
/// <b>九点标定解算核心</b>
/// <para>
/// 输入：9 组（平台机械坐标 ↔ 相机像素坐标）。输出：像素 → 机械 的映射模型 + 精度证据。
/// </para>
/// <para>处理流程（每一步都有存在的理由）：</para>
/// <list type="number">
///   <item>数据校验：点数、重复点、行程是否过小 —— 这些是现场最常见的"标了但不准"的根因。</item>
///   <item>Hartley 归一化：把像素与毫米两组坐标各自平移到质心、缩放到平均距离 √2，压低条件数。</item>
///   <item>构造超定方程组，用 Householder QR 求最小二乘解。</item>
///   <item>反归一化，还原到物理坐标系。</item>
///   <item>逐点残差 + RMSE + 条件数 → 质量分级。</item>
///   <item>（可选）留一交叉验证，区分"真的拟合得好"和"参数太多把噪声也拟合进去了"。</item>
/// </list>
/// </summary>
public static class NinePointCalibrator
{
    /// <summary>执行标定。</summary>
    /// <exception cref="ArgumentException">标定点数量不足、有重复点或行程过小。</exception>
    /// <exception cref="InvalidOperationException">设计矩阵秩亏（标定点共线 / 退化）。</exception>
    public static CalibrationResult Calibrate(CalibrationPointSet pointSet, CalibrationOptions? options = null)
    {
        ArgumentNullException.ThrowIfNull(pointSet);
        CalibrationOptions effective = options ?? new CalibrationOptions();
        effective.Validate();
        ValidatePointSet(pointSet, effective);

        // ── 1. 坐标归一化（Hartley）───────────────────────────────
        Point2D imageCentroid = pointSet.ImageCentroid;
        Point2D machineCentroid = pointSet.MachineCentroid;

        double imageScale = 1d;
        double machineScale = 1d;
        if (effective.NormalizeCoordinates)
        {
            imageScale = ComputeNormalizationScale(pointSet.Points.Select(point => point.Image), imageCentroid);
            machineScale = ComputeNormalizationScale(pointSet.Points.Select(point => point.Machine), machineCentroid);
        }

        Matrix3x3 imageNormalization = Matrix3x3.Normalization(imageScale, imageCentroid);
        Matrix3x3 machineNormalization = Matrix3x3.Normalization(machineScale, machineCentroid);

        Point2D[] normalizedImages = pointSet.Points
            .Select(point => imageNormalization.Transform(point.Image))
            .ToArray();
        Point2D[] normalizedMachines = pointSet.Points
            .Select(point => machineNormalization.Transform(point.Machine))
            .ToArray();

        // ── 2. 构造设计矩阵与右端项 ────────────────────────────────
        (double[][] design, double[] rightHandSide) = BuildEquationSystem(
            effective.Model,
            normalizedImages,
            normalizedMachines);

        // ── 3. 最小二乘求解 ────────────────────────────────────────
        double[] solution = LeastSquaresSolver.Solve(design, rightHandSide);
        double conditionNumber = SymmetricEigenSolver.ConditionNumber(design);

        // ── 4. 反归一化，还原到物理坐标系 ───────────────────────────
        ICalibrationModel model = BuildModel(
            effective.Model,
            solution,
            imageNormalization,
            machineNormalization,
            imageCentroid,
            machineCentroid,
            imageScale,
            machineScale);

        // ── 5. 精度评估 ────────────────────────────────────────────
        List<CalibrationResidual> residuals = new(pointSet.Count);
        double sumOfSquares = 0d;
        double sumOfErrors = 0d;
        double maxError = 0d;

        foreach (CalibrationPoint point in pointSet.Points)
        {
            Point2D predicted = model.ImageToMachine(point.Image);
            CalibrationResidual residual = new(point.Label, point.Image, point.Machine, predicted);
            residuals.Add(residual);

            double error = residual.ErrorMagnitude;
            sumOfSquares += error * error;
            sumOfErrors += error;
            maxError = Math.Max(maxError, error);
        }

        double rmse = Math.Sqrt(sumOfSquares / pointSet.Count);
        double meanError = sumOfErrors / pointSet.Count;

        // ── 6. 留一交叉验证（区分"真拟合"与"拟合噪声"）──────────────
        CalibrationLeaveOneOutResult? leaveOneOut = effective.RunLeaveOneOutValidation
            ? LeaveOneOutValidator.Validate(pointSet, effective)
            : null;

        return new CalibrationResult
        {
            Model = model,
            Kind = effective.Model,
            Options = effective,
            Residuals = residuals,
            RmseMm = rmse,
            MeanErrorMm = meanError,
            MaxErrorMm = maxError,
            ConditionNumber = conditionNumber,
            ImageCentroid = imageCentroid,
            MachineCentroid = machineCentroid,
            MachineSpreadMm = pointSet.MachineSpread,
            LeaveOneOut = leaveOneOut,
        };
    }

    /// <summary>依次用三种模型标定同一组数据并返回结果，用于模型选型对比。</summary>
    public static IReadOnlyDictionary<CalibrationModelKind, CalibrationResult> CompareAllModels(
        CalibrationPointSet pointSet,
        CalibrationOptions? options = null)
    {
        CalibrationOptions effective = options ?? new CalibrationOptions();
        Dictionary<CalibrationModelKind, CalibrationResult> results = new();

        foreach (CalibrationModelKind kind in Enum.GetValues<CalibrationModelKind>())
        {
            if (pointSet.Count < CalibrationOptions.GetMinimumPointCount(kind))
            {
                continue;
            }

            try
            {
                results[kind] = Calibrate(pointSet, effective.WithModel(kind));
            }
            catch (Exception ex) when (ex is InvalidOperationException or ArgumentException)
            {
                // 数据不支持该模型（例如点太少），跳过即可，不影响其它模型
            }
        }

        return results;
    }

    // ───────────────────────── 内部实现 ─────────────────────────

    private static void ValidatePointSet(CalibrationPointSet pointSet, CalibrationOptions options)
    {
        int required = options.EffectiveMinimumPointCount;

        if (pointSet.Count < required)
        {
            throw new ArgumentException(
                $"标定点数量不足：{options.Model} 模型需要至少 {required} 个点，当前只有 {pointSet.Count} 个",
                nameof(pointSet));
        }

        foreach (CalibrationPoint point in pointSet.Points)
        {
            if (!point.Image.IsFinite || !point.Machine.IsFinite)
            {
                throw new ArgumentException($"标定点 {point.Label} 含非法数值（NaN / Infinity）", nameof(pointSet));
            }
        }

        IReadOnlyList<string> duplicates = pointSet.FindDuplicateLabels();
        if (duplicates.Count > 0)
        {
            throw new ArgumentException(
                $"存在重复的标定点标签：{string.Join(", ", duplicates)}。标签重复意味着同一点被采了两次，会导致解算退化",
                nameof(pointSet));
        }

        double imageSpread = pointSet.ImageSpread;
        if (imageSpread < 10d)
        {
            throw new ArgumentException(
                $"标定点在图像上过于集中（平均半径仅 {imageSpread:F2} px）。9 个点必须铺开占满视野，否则参数不可辨识",
                nameof(pointSet));
        }

        double minimumDistance = pointSet.MinimumInterPointDistanceMm();
        if (minimumDistance < 1e-3)
        {
            throw new ArgumentException(
                $"存在两个几乎重合的标定点（最小间距 {minimumDistance:F6} mm）。请检查示教动作是否真的走了 9 个不同位置",
                nameof(pointSet));
        }
    }

    private static double ComputeNormalizationScale(IEnumerable<Point2D> points, Point2D centroid)
    {
        double meanDistance = points.Average(point => point.DistanceTo(centroid));
        return meanDistance > 1e-12 ? Math.Sqrt(2d) / meanDistance : 1d;
    }

    private static (double[][] Design, double[] RightHandSide) BuildEquationSystem(
        CalibrationModelKind kind,
        Point2D[] images,
        Point2D[] machines) => kind switch
        {
            CalibrationModelKind.Affine => BuildAffineSystem(images, machines),
            CalibrationModelKind.Homography => BuildHomographySystem(images, machines),
            CalibrationModelKind.QuadraticPolynomial => BuildQuadraticSystem(images, machines),
            _ => throw new ArgumentOutOfRangeException(nameof(kind), kind, "未知的标定模型"),
        };

    /// <summary>仿射：每个点贡献 2 个方程，共 6 个未知数。</summary>
    private static (double[][], double[]) BuildAffineSystem(Point2D[] images, Point2D[] machines)
    {
        double[][] design = new double[images.Length * 2][];
        double[] rhs = new double[images.Length * 2];

        for (int i = 0; i < images.Length; i++)
        {
            design[i * 2] = [images[i].X, images[i].Y, 1d, 0d, 0d, 0d];
            rhs[i * 2] = machines[i].X;

            design[(i * 2) + 1] = [0d, 0d, 0d, images[i].X, images[i].Y, 1d];
            rhs[(i * 2) + 1] = machines[i].Y;
        }

        return (design, rhs);
    }

    /// <summary>
    /// 单应（DLT）：固定 h8 = 1，每个点贡献 2 个方程，共 8 个未知数。
    /// 固定 h8 = 1 要求 h8 ≠ 0，在相机倾角不大的场景下总成立；
    /// 且因为前面已经做过 Hartley 归一化，这个简化不会带来数值问题。
    /// </summary>
    private static (double[][], double[]) BuildHomographySystem(Point2D[] images, Point2D[] machines)
    {
        double[][] design = new double[images.Length * 2][];
        double[] rhs = new double[images.Length * 2];

        for (int i = 0; i < images.Length; i++)
        {
            double u = images[i].X;
            double v = images[i].Y;
            double x = machines[i].X;
            double y = machines[i].Y;

            design[i * 2] = [u, v, 1d, 0d, 0d, 0d, -u * x, -v * x];
            rhs[i * 2] = x;

            design[(i * 2) + 1] = [0d, 0d, 0d, u, v, 1d, -u * y, -v * y];
            rhs[(i * 2) + 1] = y;
        }

        return (design, rhs);
    }

    /// <summary>二次多项式：每个点贡献 2 个方程，共 12 个未知数。</summary>
    private static (double[][], double[]) BuildQuadraticSystem(Point2D[] images, Point2D[] machines)
    {
        double[][] design = new double[images.Length * 2][];
        double[] rhs = new double[images.Length * 2];

        for (int i = 0; i < images.Length; i++)
        {
            double u = images[i].X;
            double v = images[i].Y;
            double[] features = [1d, u, v, u * v, u * u, v * v];

            design[i * 2] = [.. features, 0d, 0d, 0d, 0d, 0d, 0d];
            rhs[i * 2] = machines[i].X;

            design[(i * 2) + 1] = [0d, 0d, 0d, 0d, 0d, 0d, .. features];
            rhs[(i * 2) + 1] = machines[i].Y;
        }

        return (design, rhs);
    }

    private static ICalibrationModel BuildModel(
        CalibrationModelKind kind,
        double[] solution,
        Matrix3x3 imageNormalization,
        Matrix3x3 machineNormalization,
        Point2D imageCentroid,
        Point2D machineCentroid,
        double imageScale,
        double machineScale)
    {
        switch (kind)
        {
            case CalibrationModelKind.Affine:
            {
                Matrix3x3 normalizedModel = Matrix3x3.Affine(
                    solution[0], solution[1], solution[2],
                    solution[3], solution[4], solution[5]);

                // machine = T_mach⁻¹ · M_n · T_img · image
                Matrix3x3 physical = machineNormalization.Inverse() * normalizedModel * imageNormalization;
                return AffineTransform2D.FromMatrix(physical);
            }

            case CalibrationModelKind.Homography:
            {
                Matrix3x3 normalizedModel = new(
                    solution[0], solution[1], solution[2],
                    solution[3], solution[4], solution[5],
                    solution[6], solution[7], 1d);

                Matrix3x3 physical = machineNormalization.Inverse() * normalizedModel * imageNormalization;

                // 归一化为 h8 = 1，便于与其它实现比对
                double h8 = physical[2, 2];
                if (Math.Abs(h8) > 1e-14)
                {
                    physical = new Matrix3x3(
                        physical[0, 0] / h8, physical[0, 1] / h8, physical[0, 2] / h8,
                        physical[1, 0] / h8, physical[1, 1] / h8, physical[1, 2] / h8,
                        physical[2, 0] / h8, physical[2, 1] / h8, 1d);
                }

                return new Homography2D(physical);
            }

            case CalibrationModelKind.QuadraticPolynomial:
            {
                // 归一化空间：machine_n = g(image_n)
                // 物理空间：machine = T_mach⁻¹(g(T_img(image)))
                // 因为 T 是相似变换，只需要把系数按 s_img / s_mach 的幂次缩放，
                // 再把自变量从归一化坐标平移回原始像素坐标。
                double[] x = RestorePolynomial(
                    solution[0], solution[1], solution[2], solution[3], solution[4], solution[5],
                    imageScale, machineScale, imageCentroid, machineCentroid.X);

                double[] y = RestorePolynomial(
                    solution[6], solution[7], solution[8], solution[9], solution[10], solution[11],
                    imageScale, machineScale, imageCentroid, machineCentroid.Y);

                return new QuadraticPolynomialTransform2D(x, y);
            }

            default:
                throw new ArgumentOutOfRangeException(nameof(kind), kind, "未知的标定模型");
        }
    }

    /// <summary>
    /// 把归一化空间解出来的多项式还原成物理坐标系下的标准形式。
    /// <para>
    /// 推导：拟合得到的是 <c>machine_n = g(image_n)</c>，
    /// 而物理关系是 <c>machine = T_mach⁻¹(g(T_img(image)))</c>。
    /// 由于 T 是相似变换（等比缩放 + 平移），只需要：
    /// ① 把自变量从归一化坐标换回"以图像质心为原点的 mm 无关像素差"（乘 imageScale）；
    /// ② 输出整体除以 machineScale 并加上机械侧质心偏移。
    /// ③ 最后做一次自变量平移展开，把 (u − cx, v − cy) 的基底换成标准的 (u, v) 基底。
    /// </para>
    /// <code>
    /// 原式：c0 + c1·U + c2·V + c3·U·V + c4·U² + c5·V²   （U = u − cx, V = v − cy）
    /// 展开后：
    ///   const: c0 − c1·cx − c2·cy + c3·cx·cy + c4·cx² + c5·cy²
    ///   u    : c1 − c3·cy − 2·c4·cx
    ///   v    : c2 − c3·cx − 2·c5·cy
    ///   uv   : c3      u²: c4      v²: c5
    /// </code>
    /// </summary>
    private static double[] RestorePolynomial(
        double a0, double a1, double a2, double a3, double a4, double a5,
        double imageScale,
        double machineScale,
        Point2D imageCentroid,
        double machineOffset)
    {
        // ① 自变量换回原始像素（乘 imageScale），一次项乘 s、二次项乘 s²
        double scaled2 = imageScale * imageScale;
        double c1 = a1 * imageScale;
        double c2 = a2 * imageScale;
        double c3 = a3 * scaled2;
        double c4 = a4 * scaled2;
        double c5 = a5 * scaled2;

        // ② 输出量纲还原（除以 machineScale）并叠加机械侧平移
        double inverseMachineScale = 1d / machineScale;
        double c0 = (a0 * inverseMachineScale) + machineOffset;
        c1 *= inverseMachineScale;
        c2 *= inverseMachineScale;
        c3 *= inverseMachineScale;
        c4 *= inverseMachineScale;
        c5 *= inverseMachineScale;

        // ③ 自变量平移展开
        double cx = imageCentroid.X;
        double cy = imageCentroid.Y;

        double constant = c0 - (c1 * cx) - (c2 * cy) + (c3 * cx * cy) + (c4 * cx * cx) + (c5 * cy * cy);
        double linearU = c1 - (c3 * cy) - (2d * c4 * cx);
        double linearV = c2 - (c3 * cx) - (2d * c5 * cy);

        return [constant, linearU, linearV, c3, c4, c5];
    }
}
