using MotionCore.Abstractions.Geometry;

namespace MotionCore.Abstractions.Motion;

/// <summary>
/// 控制器语法糖。
/// <para>
/// 视觉相关的代码（标定、对位）天然只在 XY 平面上工作，
/// 每次都要写 <c>MotionPose.FromPlanar(point)</c> 既啰嗦又容易写错轴顺序，
/// 因此这里补一层薄扩展，让调用点直接表达意图。
/// </para>
/// </summary>
public static class MotionControllerExtensions
{
    /// <summary>XY 平面绝对定位（矢量合成）。</summary>
    public static Task MoveAbsoluteAsync(
        this IMotionController controller,
        Point2D planePosition,
        MoveProfile? profile = null,
        CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(controller);
        return controller.MoveAbsoluteAsync(MotionPose.FromPlanar(planePosition), profile, cancellationToken);
    }

    /// <summary>XY 平面相对运动（矢量合成）。</summary>
    public static Task MoveRelativeAsync(
        this IMotionController controller,
        Point2D delta,
        MoveProfile? profile = null,
        CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(controller);
        return controller.MoveRelativeAsync(delta.X, delta.Y, profile, cancellationToken);
    }

    /// <summary>等待 X / Y 两轴到位。</summary>
    public static Task WaitPlanarInPositionAsync(
        this IMotionController controller,
        TimeSpan? timeout = null,
        CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(controller);
        return controller.WaitInPositionAsync([AxisId.X, AxisId.Y], timeout, cancellationToken);
    }
}
