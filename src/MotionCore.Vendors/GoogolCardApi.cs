using System.Runtime.InteropServices;
using MotionCore.Abstractions.Motion;
using MotionCore.Abstractions.Vendors;

namespace MotionCore.Vendors;

/// <summary>
/// <b>固高（Googol）GTS 系列运动控制卡适配器</b>
/// <para>依赖原生库：<c>gts.dll</c>（随 GTS 卡驱动安装，一般在 <c>C:\Program Files (x86)\Googol\</c>）。</para>
/// <para>
/// <b>固高的关键特点（和另两家差异最大的地方）：</b>
/// </para>
/// <list type="bullet">
///   <item><b>轴号从 1 开始</b>，且轴号与"规划器(profile)"是两套编号：规划器 = 轴号 × 2 − 1（梯形）、轴号 × 2（Jog）；</item>
///   <item>参数必须与"运动模式"成对配置：先 <c>GT_PrfTrap</c> 再 <c>GT_SetPos</c>，最后 <c>GT_Update</c> 才真正下发；</item>
///   <item>多轴同时启动靠 <c>GT_Update(mask)</c> 的位掩码，天然支持联动启动；</item>
///   <item>状态集中在 <c>GT_GetSts</c> 的位掩码里，并且复位报警必须调用 <c>GT_ClrSts</c>。</item>
/// </list>
/// <para>
/// 这段代码本身就说明了"为什么必须做适配层"：同样是"走到 X=10mm"，
/// 正运动一次 <c>MoveAbs</c> 就够了，固高要"切模式 → 设位置 → Update"三步。
/// 如果把这套差异漏到业务层，换卡就等于重写整个上位机。
/// </para>
/// </summary>
public sealed class GoogolCardApi : VendorCardApiBase
{
    public const string LibraryName = "gts.dll";

    /// <summary>GT_GetSts 状态位定义。</summary>
    private const int StatusBitAlarm = 1 << 1;

    private const int StatusBitPositiveLimit = 1 << 2;

    private const int StatusBitNegativeLimit = 1 << 3;

    private const int StatusBitServoOn = 1 << 6;

    private const int StatusBitProfileRunning = 1 << 7;

    private const int StatusBitProfileInPosition = 1 << 8;

    private const int StatusBitServoInPosition = 1 << 9;

    private bool _connected;

    private string _lastError = "正常";

    public GoogolCardApi(IReadOnlyDictionary<AxisId, int>? axisIndices = null)
        : base(
            LibraryName,
            axisIndices ?? new Dictionary<AxisId, int>
            {
                // 注意：固高轴号从 1 开始
                [AxisId.X] = 1,
                [AxisId.Y] = 2,
                [AxisId.Z] = 3,
                [AxisId.R] = 4,
            })
    {
    }

    public override string VendorName => "固高 GTS（Googol）";

    public override int InputCount => 16;

    public override int OutputCount => 16;

    public bool IsConnected => _connected;

    // ───────────────────────── 原生声明 ─────────────────────────

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern short GT_Open(short channel, short param);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern short GT_Close();

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern short GT_Reset();

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern short GT_LoadConfig([MarshalAs(UnmanagedType.LPStr)] string fileName);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern short GT_AxisOn(short axis);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern short GT_AxisOff(short axis);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern short GT_ClrSts(short core, short axis);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern short GT_PrfTrap(short profile);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern short GT_PrfJog(short profile);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern short GT_SetVel(short profile, double velocity);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern short GT_SetAcc(short profile, double acceleration);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern short GT_SetDec(short profile, double deceleration);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern short GT_SetPos(short profile, double position);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern short GT_SetJogVel(short profile, double velocity);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern short GT_Update(short mask);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern short GT_Stop(short mask, short maskAxis);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern short GT_SetPrfPos(short profile, double position);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern short GT_GetPrfPos(short profile, out double position);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern short GT_GetEncPos(short encoder, out double position);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern short GT_GetPrfVel(short profile, out double velocity);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern short GT_GetSts(short axis, out int status);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern short GT_ZeroPos(short profile);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern short GT_GoHome(short profile);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern short GT_GetDi(short diType, out int value);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern short GT_GetDo(short doType, out int value);

    [DllImport(LibraryName, CallingConvention = CallingConvention.StdCall)]
    private static extern short GT_SetDo(short doType, int value);

    // ───────────────────────── 接口实现 ─────────────────────────

    /// <summary>梯形规划器编号：固高约定 profile = axis × 2 − 1。</summary>
    private static short TrapProfile(int axis) => (short)((axis * 2) - 1);

    /// <summary>Jog 规划器编号：profile = axis × 2。</summary>
    private static short JogProfile(int axis) => (short)(axis * 2);

    public override int Connect(string address, int port)
    {
        if (!IsSdkAvailable)
        {
            _lastError = $"未找到原生库 {LibraryName}，请先安装固高 GTS 驱动";
            return NotAvailable();
        }

        // channel 1 = PCI/ISA 卡；网口卡（GTS-800-ETH）用 GT_Open(2, 0) + GT_SetIp 配置
        short result = GT_Open(1, 0);
        if (result != 0)
        {
            _lastError = $"GT_Open 失败（返回 {result}）";
            return result;
        }

        GT_Reset();
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

        GT_Close();
        _connected = false;
    }

    public override int SetServoEnabled(AxisId axis, bool enabled)
    {
        if (CheckReady(out int error, axis, out int index))
        {
            return error;
        }

        return enabled ? GT_AxisOn((short)index) : GT_AxisOff((short)index);
    }

    public override int ConfigureProfile(AxisId axis, MoveProfile profile)
    {
        if (CheckReady(out int error, axis, out int index))
        {
            return error;
        }

        short trap = TrapProfile(index);
        GT_PrfTrap(trap);

        short result = GT_SetVel(trap, profile.Velocity);
        if (result != 0)
        {
            return result;
        }

        result = GT_SetAcc(trap, profile.Acceleration);
        return result != 0 ? result : GT_SetDec(trap, profile.Deceleration);
    }

    public override int ConfigureSoftLimit(AxisId axis, double negativeLimit, double positiveLimit, bool enabled)
    {
        if (CheckReady(out int error, axis, out _))
        {
            return error;
        }

        // 固高软限位通过 GT_SetSoftLimit(axis, positive, negative) 配置（部分固件版本可用）。
        // 未启用时保持接口语义，由上位机侧的软限位护栏兜底。
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

        return GT_SetPrfPos(TrapProfile(index), position);
    }

    public override int MoveAbsolute(AxisId axis, double position)
    {
        if (CheckReady(out int error, axis, out int index))
        {
            return error;
        }

        short trap = TrapProfile(index);

        // 固高的"三步走"：切梯形模式 → 设目标位置 → Update 下发
        short result = GT_PrfTrap(trap);
        if (result != 0)
        {
            return result;
        }

        result = GT_SetPos(trap, position);
        return result != 0 ? result : GT_Update(trap);
    }

    public override int MoveRelative(AxisId axis, double distance)
    {
        if (!_connected || !TryIndexOf(axis, out int index))
        {
            return _connected ? VendorErrorCodes.InvalidAxis : VendorErrorCodes.NotConnected;
        }

        // 固高梯形模式本身就是"给定绝对目标"，
        // 因此相对运动必须先把规划位置读回来自己加 —— 这正是适配层要吸收的差异。
        short trap = TrapProfile(index);
        short result = GT_GetPrfPos(trap, out double current);
        return result != 0 ? result : MoveAbsolute(axis, current + distance);
    }

    public override int StartJog(AxisId axis, MotionDirection direction, double velocity)
    {
        if (CheckReady(out int error, axis, out int index))
        {
            return error;
        }

        short jog = JogProfile(index);
        short result = GT_PrfJog(jog);
        if (result != 0)
        {
            return result;
        }

        result = GT_SetJogVel(jog, Math.Abs(velocity) * (int)direction);
        return result != 0 ? result : GT_Update(jog);
    }

    public override int StopJog(AxisId axis)
    {
        if (CheckReady(out int error, axis, out int index))
        {
            return error;
        }

        short jog = JogProfile(index);

        // 先把 Jog 速度置 0，再 Update 让它平滑停下；直接 GT_Stop 会走急停逻辑、冲击较大
        short result = GT_SetJogVel(jog, 0d);
        return result != 0 ? result : GT_Update(jog);
    }

    public override int Home(AxisId axis, HomeMode mode, double speed, double offset)
    {
        if (CheckReady(out int error, axis, out int index))
        {
            return error;
        }

        short trap = TrapProfile(index);
        GT_PrfTrap(trap);
        GT_SetVel(trap, speed);

        _ = mode;
        _ = offset;

        // GT_GoHome 使用 GT_LoadConfig 里配置的回零参数（回零方式/速度/偏移都在配置文件里）
        return GT_GoHome(trap);
    }

    public override int Stop(AxisId axis, bool emergency)
    {
        if (CheckReady(out int error, axis, out int index))
        {
            return error;
        }

        // maskAxis 第 0 位对应轴 1
        return GT_Stop(TrapProfile(index), (short)(1 << (index - 1)));
    }

    public override int ResetAlarm(AxisId axis)
    {
        if (CheckReady(out int error, axis, out int index))
        {
            return error;
        }

        return GT_ClrSts(1, (short)index);
    }

    public override VendorAxisSnapshot ReadAxis(AxisId axis)
    {
        if (!_connected || !TryIndexOf(axis, out int index))
        {
            return default;
        }

        GT_GetSts((short)index, out int status);
        GT_GetPrfPos(TrapProfile(index), out double commandPosition);
        GT_GetEncPos((short)index, out double actualPosition);
        GT_GetPrfVel(TrapProfile(index), out double velocity);

        bool alarm = (status & StatusBitAlarm) != 0;
        bool running = (status & StatusBitProfileRunning) != 0;
        bool inPosition = !running
                          && (status & StatusBitProfileInPosition) != 0
                          && (status & StatusBitServoInPosition) != 0;

        return new VendorAxisSnapshot(
            CommandPosition: commandPosition,
            ActualPosition: actualPosition,
            Velocity: velocity,
            ServoEnabled: (status & StatusBitServoOn) != 0,
            InPosition: inPosition,
            Homed: false,
            PositiveLimit: (status & StatusBitPositiveLimit) != 0,
            NegativeLimit: (status & StatusBitNegativeLimit) != 0,
            HomeSwitch: false,
            Alarm: alarm,
            AlarmCode: alarm ? status : 0);
    }

    public override bool ReadInput(int index)
    {
        if (!_connected)
        {
            return false;
        }

        return GT_GetDi(1, out int value) == 0 && ((value >> index) & 1) != 0;
    }

    public override bool ReadOutput(int index)
    {
        if (!_connected)
        {
            return false;
        }

        return GT_GetDo(1, out int value) == 0 && ((value >> index) & 1) != 0;
    }

    public override bool WriteOutput(int index, bool value)
    {
        if (!_connected)
        {
            return false;
        }

        GT_GetDo(1, out int current);
        int target = value ? current | (1 << index) : current & ~(1 << index);
        return GT_SetDo(1, target) == 0;
    }

    public override string GetErrorMessage(int errorCode) => errorCode switch
    {
        VendorErrorCodes.Ok => "正常",
        VendorErrorCodes.NotAvailable => _lastError,
        VendorErrorCodes.NotConnected => "控制器未连接",
        VendorErrorCodes.InvalidAxis => "轴号非法",
        _ => $"固高错误码 {errorCode}",
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
