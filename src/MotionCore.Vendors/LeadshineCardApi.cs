using System.Runtime.InteropServices;
using MotionCore.Abstractions.Motion;
using MotionCore.Abstractions.Vendors;

namespace MotionCore.Vendors;

/// <summary>
/// <b>雷赛智能（Leadshine）DMC 系列运动控制卡适配器</b>
/// <para>依赖原生库：<c>LTDMC.dll</c>（随雷赛驱动安装，一般在 <c>C:\Program Files (x86)\Leadshine\</c>）。</para>
/// <para>
/// <b>雷赛的关键特点：</b>
/// </para>
/// <list type="bullet">
///   <item>几乎所有 API 的第一个参数都是卡号（CardNo），支持一台工控机插多张卡 ——
///         这一点和固高、正运动都不一样，需要在适配层显式保存卡号；</item>
///   <item>API 分成两套：<c>_unit</c> 后缀的按"用户单位"（mm）操作，
///         不带后缀的按脉冲操作。统一用 <c>_unit</c> 那套，配合 <c>dmc_set_equiv</c> 设脉冲当量；</item>
///   <item>软限位是原生支持的（<c>dmc_set_softlimit_unit</c>），这是三家里面最省事的；</item>
///   <item>停止分"减速停"和"急停"两种模式，分别对应正常停机和异常保护。</item>
/// </list>
/// </summary>
public sealed class LeadshineCardApi : VendorCardApiBase
{
    public const string LibraryName = "LTDMC.dll";

    /// <summary>轴硬件状态位（<c>dmc_axis_io_status</c> 返回值）。</summary>
    private const ushort IoBitAlarm = 1 << 1;

    private const ushort IoBitPositiveLimit = 1 << 2;

    private const ushort IoBitNegativeLimit = 1 << 3;

    private const ushort IoBitEmergencyStop = 1 << 4;

    private const ushort IoBitHome = 1 << 5;

    private const ushort IoBitSoftLimit = 1 << 6;

    private const ushort StopModeDecelerate = 0;

    private const ushort StopModeEmergency = 1;

    private bool _connected;

    private string _lastError = "正常";

    public LeadshineCardApi(ushort cardNumber = 0, IReadOnlyDictionary<AxisId, int>? axisIndices = null)
        : base(
            LibraryName,
            axisIndices ?? new Dictionary<AxisId, int>
            {
                [AxisId.X] = 0,
                [AxisId.Y] = 1,
                [AxisId.Z] = 2,
                [AxisId.R] = 3,
            }) => CardNumber = cardNumber;

    /// <summary>卡号（支持多卡）。</summary>
    public ushort CardNumber { get; }

    public override string VendorName => $"雷赛 DMC（Leadshine，卡号 {CardNumber}）";

    public override int InputCount => 32;

    public override int OutputCount => 32;

    public bool IsConnected => _connected;

    // ───────────────────────── 原生声明 ─────────────────────────

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern ushort dmc_board_init();

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern ushort dmc_board_close();

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern ushort dmc_set_equiv(ushort cardNo, ushort axis, double equivalent);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern ushort dmc_set_profile_unit(
        ushort cardNo, ushort axis,
        double minVelocity, double maxVelocity,
        double accelerationTime, double decelerationTime,
        double stopVelocity);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern ushort dmc_set_s_profile(ushort cardNo, ushort axis, ushort smoothMode, double smoothParameter);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern ushort dmc_set_softlimit_unit(
        ushort cardNo, ushort axis, ushort enable, ushort sourceSelect,
        short action, double negativeLimit, double positiveLimit);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern ushort dmc_set_son(ushort cardNo, ushort axis, ushort enable);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern ushort dmc_pmove_unit(ushort cardNo, ushort axis, double distance, ushort positionMode);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern ushort dmc_vmove(ushort cardNo, ushort axis, ushort direction);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern ushort dmc_stop(ushort cardNo, ushort axis, ushort stopMode);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern ushort dmc_set_position_unit(ushort cardNo, ushort axis, double position);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern ushort dmc_get_position_unit(ushort cardNo, ushort axis, out double position);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern ushort dmc_get_encoder_unit(ushort cardNo, ushort axis, out double position);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern ushort dmc_check_done_unit(ushort cardNo, ushort axis, out ushort done);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern ushort dmc_axis_io_status(ushort cardNo, ushort axis, out ushort status);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern ushort dmc_set_home_profile_unit(
        ushort cardNo, ushort axis, ushort homeMode,
        double lowVelocity, double highVelocity,
        double acceleration, double deceleration,
        double offset);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern ushort dmc_home_move(ushort cardNo, ushort axis);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern ushort dmc_get_home_result(ushort cardNo, ushort axis, out ushort done);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern ushort dmc_clear_axis_errcode(ushort cardNo, ushort axis);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern ushort dmc_read_inbit(ushort cardNo, ushort bitNumber);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern ushort dmc_read_outbit(ushort cardNo, ushort bitNumber);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern ushort dmc_write_outbit(ushort cardNo, ushort bitNumber, ushort onOff);

    // ───────────────────────── 接口实现 ─────────────────────────

    public override int Connect(string address, int port)
    {
        if (!IsSdkAvailable)
        {
            _lastError = $"未找到原生库 {LibraryName}，请先安装雷赛运动控制卡驱动";
            return NotAvailable();
        }

        if (dmc_board_init() == 0)
        {
            _lastError = "dmc_board_init 返回 0，未检测到雷赛控制卡";
            return VendorErrorCodes.NotConnected;
        }

        foreach (AxisId axis in Axes)
        {
            int index = IndexOf(axis);

            // 脉冲当量：1 个用户单位（mm）= 1000 脉冲，与减速比、丝杆导程解耦
            dmc_set_equiv(CardNumber, (ushort)index, 1000d);

            // 软限位：雷赛原生支持，开启后越限由卡直接拦截
            dmc_set_softlimit_unit(CardNumber, (ushort)index, 1, 1, 1, -200d, 200d);
        }

        _connected = true;
        _ = address;
        _ = port;
        return 0;
    }

    public override void Disconnect()
    {
        if (!_connected)
        {
            return;
        }

        dmc_board_close();
        _connected = false;
    }

    public override int SetServoEnabled(AxisId axis, bool enabled)
    {
        if (CheckReady(out int error, axis, out int index))
        {
            return error;
        }

        return dmc_set_son(CardNumber, (ushort)index, (ushort)(enabled ? 1 : 0));
    }

    public override int ConfigureProfile(AxisId axis, MoveProfile profile)
    {
        if (CheckReady(out int error, axis, out int index))
        {
            return error;
        }

        ushort ushortIndex = (ushort)index;

        // 雷赛用"加减速时间"而不是"加速度"，需要自己换算：
        //   t = v / a
        double accelerationTime = profile.Acceleration > 0 ? profile.Velocity / profile.Acceleration : 0.1d;
        double decelerationTime = profile.Deceleration > 0 ? profile.Velocity / profile.Deceleration : 0.1d;

        ushort result = dmc_set_profile_unit(
            CardNumber,
            ushortIndex,
            minVelocity: 100d,
            maxVelocity: profile.Velocity,
            accelerationTime: accelerationTime,
            decelerationTime: decelerationTime,
            stopVelocity: 100d);

        if (result != 0)
        {
            return result;
        }

        // s_mode 1 = S 曲线；s_para 为平滑时间（秒）
        return profile.SmoothTime is > 0
            ? dmc_set_s_profile(CardNumber, ushortIndex, 1, profile.SmoothTime.Value)
            : 0;
    }

    public override int ConfigureSoftLimit(AxisId axis, double negativeLimit, double positiveLimit, bool enabled)
    {
        if (CheckReady(out int error, axis, out int index))
        {
            return error;
        }

        return dmc_set_softlimit_unit(
            CardNumber,
            (ushort)index,
            (ushort)(enabled ? 1 : 0),
            sourceSelect: 1,
            action: 1,
            negativeLimit,
            positiveLimit);
    }

    public override int SetPosition(AxisId axis, double position)
    {
        if (CheckReady(out int error, axis, out int index))
        {
            return error;
        }

        return dmc_set_position_unit(CardNumber, (ushort)index, position);
    }

    public override int MoveAbsolute(AxisId axis, double position)
    {
        if (CheckReady(out int error, axis, out int index))
        {
            return error;
        }

        // positionMode = 1 表示绝对定位（0 = 相对）
        return dmc_pmove_unit(CardNumber, (ushort)index, position, 1);
    }

    public override int MoveRelative(AxisId axis, double distance)
    {
        if (CheckReady(out int error, axis, out int index))
        {
            return error;
        }

        return dmc_pmove_unit(CardNumber, (ushort)index, distance, 0);
    }

    public override int StartJog(AxisId axis, MotionDirection direction, double velocity)
    {
        if (CheckReady(out int error, axis, out int index))
        {
            return error;
        }

        ushort ushortIndex = (ushort)index;
        dmc_set_profile_unit(CardNumber, ushortIndex, 100d, Math.Abs(velocity), 0.1d, 0.1d, 100d);

        // direction: 0 = 负向，1 = 正向
        return dmc_vmove(CardNumber, ushortIndex, (ushort)(direction == MotionDirection.Positive ? 1 : 0));
    }

    public override int StopJog(AxisId axis)
    {
        if (CheckReady(out int error, axis, out int index))
        {
            return error;
        }

        return dmc_stop(CardNumber, (ushort)index, StopModeDecelerate);
    }

    public override int Home(AxisId axis, HomeMode mode, double speed, double offset)
    {
        if (CheckReady(out int error, axis, out int index))
        {
            return error;
        }

        ushort ushortIndex = (ushort)index;

        // 雷赛回零模式：1 = 负限位回零，2 = 正限位回零，3 = 原点开关回零，4 = 原点开关+Z 相
        ushort homeMode = mode switch
        {
            HomeMode.NegativeLimit => 1,
            HomeMode.PositiveLimit => 2,
            HomeMode.HomeSwitchWithIndex => 3,
            HomeMode.CurrentPosition => 4,
            _ => 1,
        };

        ushort result = dmc_set_home_profile_unit(
            CardNumber,
            ushortIndex,
            homeMode,
            lowVelocity: speed * 0.2d,
            highVelocity: speed,
            acceleration: 0.1d,
            deceleration: 0.1d,
            offset);

        return result != 0 ? result : dmc_home_move(CardNumber, ushortIndex);
    }

    public override int Stop(AxisId axis, bool emergency)
    {
        if (CheckReady(out int error, axis, out int index))
        {
            return error;
        }

        return dmc_stop(CardNumber, (ushort)index, emergency ? StopModeEmergency : StopModeDecelerate);
    }

    public override int ResetAlarm(AxisId axis)
    {
        if (CheckReady(out int error, axis, out int index))
        {
            return error;
        }

        return dmc_clear_axis_errcode(CardNumber, (ushort)index);
    }

    public override VendorAxisSnapshot ReadAxis(AxisId axis)
    {
        if (!_connected || !TryIndexOf(axis, out int index))
        {
            return default;
        }

        ushort ushortIndex = (ushort)index;

        dmc_get_position_unit(CardNumber, ushortIndex, out double commandPosition);
        dmc_get_encoder_unit(CardNumber, ushortIndex, out double actualPosition);
        dmc_axis_io_status(CardNumber, ushortIndex, out ushort ioStatus);
        dmc_check_done_unit(CardNumber, ushortIndex, out ushort done);
        dmc_get_home_result(CardNumber, ushortIndex, out ushort homed);

        bool alarm = (ioStatus & IoBitAlarm) != 0;

        return new VendorAxisSnapshot(
            CommandPosition: commandPosition,
            ActualPosition: actualPosition,
            Velocity: 0d, // 雷赛无直接的速度读取指令，需要按位置差分计算；此处省略
            ServoEnabled: true,
            InPosition: done != 0,
            Homed: homed != 0,
            PositiveLimit: (ioStatus & IoBitPositiveLimit) != 0,
            NegativeLimit: (ioStatus & IoBitNegativeLimit) != 0,
            HomeSwitch: (ioStatus & IoBitHome) != 0,
            Alarm: alarm || (ioStatus & IoBitEmergencyStop) != 0,
            AlarmCode: alarm || (ioStatus & IoBitEmergencyStop) != 0 ? ioStatus : 0);
    }

    public override bool ReadInput(int index) =>
        _connected && dmc_read_inbit(CardNumber, (ushort)index) != 0;

    public override bool ReadOutput(int index) =>
        _connected && dmc_read_outbit(CardNumber, (ushort)index) != 0;

    public override bool WriteOutput(int index, bool value) =>
        _connected && dmc_write_outbit(CardNumber, (ushort)index, (ushort)(value ? 1 : 0)) == 0;

    public override string GetErrorMessage(int errorCode) => errorCode switch
    {
        VendorErrorCodes.Ok => "正常",
        VendorErrorCodes.NotAvailable => _lastError,
        VendorErrorCodes.NotConnected => _lastError,
        VendorErrorCodes.InvalidAxis => "轴号非法",
        _ => $"雷赛错误码 {errorCode}",
    };

    public override void Dispose() => Disconnect();

    private bool CheckReady(out int errorCode, AxisId axis, out int index)
    {
        index = 0;

        if (!_connected)
        {
            errorCode = VendorErrorCodes.NotConnected;
            return true;
        }

        if (!TryIndexOf(axis, out index))
        {
            errorCode = VendorErrorCodes.InvalidAxis;
            return true;
        }

        errorCode = 0;
        return false;
    }
}
