using System.Runtime.InteropServices;
using MotionCore.Abstractions.Motion;
using MotionCore.Abstractions.Vendors;

namespace MotionCore.Vendors;

/// <summary>
/// <b>正运动技术（Zmotion）ZMC 系列运动控制卡适配器</b>
/// <para>
/// 依赖原生库：<c>zauxdll.dll</c>（随 ZMC SDK 发布，需放到程序输出目录或系统 PATH）。
/// 官方 SDK 目录一般形如 <c>C:\Program Files (x86)\Zmotion\ZMC_SDK\</c>。
/// </para>
/// <para>
/// <b>正运动的关键特点：</b>
/// </para>
/// <list type="bullet">
///   <item>BASIC 风格 API（<c>ZAux_Direct_*</c>），几乎所有参数都是 float，位置单位由 <c>UNITS</c> 决定；</item>
///   <item>支持以太网直连（<c>ZAux_FastOpen(type: 2, ip, timeout, handle)</c>），这也是 3C 产线上最常见的接法；</item>
///   <item>轴状态位集中在 <c>ZAux_Direct_GetAxisStatus</c> 的返回值里，用位掩码解析；</item>
///   <item>支持 S 曲线（<c>SetSramp</c>），对高加速度平台能显著降低机械冲击。</item>
/// </list>
/// </summary>
public sealed class ZmotionCardApi : VendorCardApiBase
{
    public const string LibraryName = "zauxdll.dll";

    /// <summary><c>ZAux_FastOpen</c> 的连接方式：1=串口，2=以太网，3=PCI，4=LOCAL。</summary>
    private const int OpenTypeEthernet = 2;

    /// <summary>AxisStatus 位定义（以官方手册《ZMC 运动控制卡指令手册》为准）。</summary>
    private const int StatusBitAlarm = 1 << 1;

    private const int StatusBitPositiveLimit = 1 << 5;

    private const int StatusBitNegativeLimit = 1 << 6;

    private const int StatusBitHome = 1 << 7;

    private const int StatusBitServoOn = 1 << 9;

    private const int StatusBitMoving = 1 << 10;

    private const int StatusBitInPosition = 1 << 11;

    private IntPtr _handle = IntPtr.Zero;

    private string _lastError = "正常";

    public ZmotionCardApi(IReadOnlyDictionary<AxisId, int>? axisIndices = null)
        : base(
            LibraryName,
            axisIndices ?? new Dictionary<AxisId, int>
            {
                [AxisId.X] = 0,
                [AxisId.Y] = 1,
                [AxisId.Z] = 2,
                [AxisId.R] = 3,
            })
    {
    }

    public override string VendorName => "正运动 ZMC（Zmotion）";

    public override int InputCount => 24;

    public override int OutputCount => 24;

    public bool IsConnected => _handle != IntPtr.Zero;

    // ───────────────────────── 原生声明 ─────────────────────────

    [DllImport(LibraryName, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern int ZAux_FastOpen(int type, string address, int timeoutMs, out IntPtr handle);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern int ZAux_Close(IntPtr handle);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern int ZAux_Direct_SetAtype(IntPtr handle, int axis, int atype);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern int ZAux_Direct_SetUnits(IntPtr handle, int axis, float units);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern int ZAux_Direct_SetSpeed(IntPtr handle, int axis, float speed);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern int ZAux_Direct_SetAccel(IntPtr handle, int axis, float accel);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern int ZAux_Direct_SetDecel(IntPtr handle, int axis, float decel);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern int ZAux_Direct_SetSramp(IntPtr handle, int axis, float sramp);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern int ZAux_Direct_SetFwdIn(IntPtr handle, int axis, int io);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern int ZAux_Direct_SetRevIn(IntPtr handle, int axis, int io);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern int ZAux_Direct_SetDatumIn(IntPtr handle, int axis, int io);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern int ZAux_Direct_SetAxisEnable(IntPtr handle, int axis, int enable);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern int ZAux_Direct_Single_MoveAbs(IntPtr handle, int axis, float position);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern int ZAux_Direct_Single_Move(IntPtr handle, int axis, float distance);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern int ZAux_Direct_Single_Vmove(IntPtr handle, int axis, int direction);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern int ZAux_Direct_Single_Cancel(IntPtr handle, int axis, int mode);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern int ZAux_Direct_Single_Datum(IntPtr handle, int axis, int mode);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern int ZAux_Direct_SetDpos(IntPtr handle, int axis, float position);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern int ZAux_Direct_GetDpos(IntPtr handle, int axis, out float position);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern int ZAux_Direct_GetMpos(IntPtr handle, int axis, out float position);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern int ZAux_Direct_GetVpSpeed(IntPtr handle, int axis, out float speed);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern int ZAux_Direct_GetAxisStatus(IntPtr handle, int axis, out int status);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern int ZAux_Direct_GetIfIdle(IntPtr handle, int axis, out int idle);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern int ZAux_Direct_GetIn(IntPtr handle, int io, out int state);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern int ZAux_Direct_GetOp(IntPtr handle, int io, out int state);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern int ZAux_Direct_SetOp(IntPtr handle, int io, int state);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern int ZAux_ClearAllErr(IntPtr handle);

    // ───────────────────────── 接口实现 ─────────────────────────

    public override int Connect(string address, int port)
    {
        if (!IsSdkAvailable)
        {
            _lastError = $"未找到原生库 {LibraryName}，请把正运动 ZMC SDK 的 DLL 复制到程序目录";
            return NotAvailable();
        }

        int result = ZAux_FastOpen(OpenTypeEthernet, address, 2000, out IntPtr handle);
        if (result != 0)
        {
            _lastError = $"连接控制器 {address} 失败（错误码 {result}）";
            return result;
        }

        _handle = handle;

        // 轴类型 1 = 脉冲型伺服；总线伺服需按实际配置改成 65/66 等
        foreach (AxisId axis in Axes)
        {
            int index = IndexOf(axis);
            ZAux_Direct_SetAtype(_handle, index, 1);

            // UNITS = 每单位脉冲数。设为 1000 表示 1 个用户单位 = 1000 脉冲，
            // 上层业务就统一用 mm，不用关心机械减速比与丝杆导程。
            ZAux_Direct_SetUnits(_handle, index, 1000f);
        }

        return 0;
    }

    public override void Disconnect()
    {
        if (_handle == IntPtr.Zero)
        {
            return;
        }

        ZAux_Close(_handle);
        _handle = IntPtr.Zero;
    }

    public override int SetServoEnabled(AxisId axis, bool enabled)
    {
        if (CheckReady(out int error, axis, out int index))
        {
            return error;
        }

        return ZAux_Direct_SetAxisEnable(_handle, index, enabled ? 1 : 0);
    }

    public override int ConfigureProfile(AxisId axis, MoveProfile profile)
    {
        if (CheckReady(out int error, axis, out int index))
        {
            return error;
        }

        int result = ZAux_Direct_SetSpeed(_handle, index, (float)profile.Velocity);
        if (result != 0)
        {
            return result;
        }

        result = ZAux_Direct_SetAccel(_handle, index, (float)profile.Acceleration);
        if (result != 0)
        {
            return result;
        }

        result = ZAux_Direct_SetDecel(_handle, index, (float)profile.Deceleration);

        // SmoothTime 映射到正运动的 S 曲线时间，能显著降低高速启停的机械冲击
        if (result == 0 && profile.SmoothTime is > 0)
        {
            result = ZAux_Direct_SetSramp(_handle, index, (float)profile.SmoothTime.Value);
        }

        return result;
    }

    public override int ConfigureSoftLimit(AxisId axis, double negativeLimit, double positiveLimit, bool enabled)
    {
        if (CheckReady(out int error, axis, out _))
        {
            return error;
        }

        // 正运动没有独立的"软限位数值"寄存器，通用做法是把软限位映射到
        // 两个空余的输入点（或直接用 DRIVE_LIMIT 参数配合 FWD_IN/REV_IN）。
        // 这里保留接口语义，具体映射由机型配置文件决定。
        _ = negativeLimit;
        _ = positiveLimit;
        _ = enabled;
        return 0;
    }

    public override int SetPosition(AxisId axis, double position)
    {
        if (CheckReady(out int error, axis, out int index))
        {
            return error;
        }

        return ZAux_Direct_SetDpos(_handle, index, (float)position);
    }

    public override int MoveAbsolute(AxisId axis, double position)
    {
        if (CheckReady(out int error, axis, out int index))
        {
            return error;
        }

        return ZAux_Direct_Single_MoveAbs(_handle, index, (float)position);
    }

    public override int MoveRelative(AxisId axis, double distance)
    {
        if (CheckReady(out int error, axis, out int index))
        {
            return error;
        }

        return ZAux_Direct_Single_Move(_handle, index, (float)distance);
    }

    public override int StartJog(AxisId axis, MotionDirection direction, double velocity)
    {
        if (CheckReady(out int error, axis, out int index))
        {
            return error;
        }

        int result = ZAux_Direct_SetSpeed(_handle, index, (float)velocity);
        return result != 0 ? result : ZAux_Direct_Single_Vmove(_handle, index, (int)direction);
    }

    public override int StopJog(AxisId axis)
    {
        if (CheckReady(out int error, axis, out int index))
        {
            return error;
        }

        // mode = 0：减速停止；mode = 1：立即停止
        return ZAux_Direct_Single_Cancel(_handle, index, 0);
    }

    public override int Home(AxisId axis, HomeMode mode, double speed, double offset)
    {
        if (CheckReady(out int error, axis, out int index))
        {
            return error;
        }

        // 正运动 DATUM(mode)：mode 决定找原点的方式（负限位 / 原点开关 / Z 相 …）
        int datumMode = mode switch
        {
            HomeMode.NegativeLimit => 1,
            HomeMode.PositiveLimit => 2,
            HomeMode.HomeSwitchWithIndex => 3,
            HomeMode.CurrentPosition => 4,
            _ => 1,
        };

        int result = ZAux_Direct_SetSpeed(_handle, index, (float)speed);
        if (result != 0)
        {
            return result;
        }

        _ = offset; // 回零偏移用 OFFPOS 参数，此处留作机型配置项
        return ZAux_Direct_Single_Datum(_handle, index, datumMode);
    }

    public override int Stop(AxisId axis, bool emergency)
    {
        if (CheckReady(out int error, axis, out int index))
        {
            return error;
        }

        return ZAux_Direct_Single_Cancel(_handle, index, emergency ? 1 : 0);
    }

    public override int ResetAlarm(AxisId axis)
    {
        if (CheckReady(out int error, axis, out _))
        {
            return error;
        }

        return ZAux_ClearAllErr(_handle);
    }

    public override VendorAxisSnapshot ReadAxis(AxisId axis)
    {
        if (!TryIndexOf(axis, out int index) || _handle == IntPtr.Zero)
        {
            return default;
        }

        ZAux_Direct_GetAxisStatus(_handle, index, out int status);
        ZAux_Direct_GetDpos(_handle, index, out float dpos);
        ZAux_Direct_GetMpos(_handle, index, out float mpos);
        ZAux_Direct_GetVpSpeed(_handle, index, out float speed);
        ZAux_Direct_GetIfIdle(_handle, index, out int idle);

        bool alarm = (status & StatusBitAlarm) != 0;
        bool inPosition = (status & StatusBitInPosition) != 0 && idle == 0;

        return new VendorAxisSnapshot(
            CommandPosition: dpos,
            ActualPosition: mpos,
            Velocity: speed,
            ServoEnabled: (status & StatusBitServoOn) != 0,
            InPosition: inPosition,
            Homed: false, // 正运动需读 DATUM 状态寄存器（DPOS 有效位），按机型配置实现
            PositiveLimit: (status & StatusBitPositiveLimit) != 0,
            NegativeLimit: (status & StatusBitNegativeLimit) != 0,
            HomeSwitch: (status & StatusBitHome) != 0,
            Alarm: alarm,
            AlarmCode: alarm ? status : 0);
    }

    public override bool ReadInput(int index)
    {
        if (_handle == IntPtr.Zero)
        {
            return false;
        }

        return ZAux_Direct_GetIn(_handle, index, out int state) == 0 && state != 0;
    }

    public override bool ReadOutput(int index)
    {
        if (_handle == IntPtr.Zero)
        {
            return false;
        }

        return ZAux_Direct_GetOp(_handle, index, out int state) == 0 && state != 0;
    }

    public override bool WriteOutput(int index, bool value)
    {
        if (_handle == IntPtr.Zero)
        {
            return false;
        }

        return ZAux_Direct_SetOp(_handle, index, value ? 1 : 0) == 0;
    }

    public override string GetErrorMessage(int errorCode) => errorCode switch
    {
        VendorErrorCodes.Ok => "正常",
        VendorErrorCodes.NotAvailable => _lastError,
        VendorErrorCodes.NotConnected => "控制器未连接",
        VendorErrorCodes.InvalidAxis => "轴号非法",
        _ => $"正运动错误码 {errorCode}",
    };

    public override void Dispose() => Disconnect();

    private bool CheckReady(out int errorCode, AxisId axis, out int index)
    {
        index = 0;

        if (_handle == IntPtr.Zero)
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
