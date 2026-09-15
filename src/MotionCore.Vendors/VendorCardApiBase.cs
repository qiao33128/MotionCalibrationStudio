using MotionCore.Abstractions.Motion;
using MotionCore.Abstractions.Vendors;

namespace MotionCore.Vendors;

/// <summary>
/// 厂商适配器公共基类。
/// <para>
/// 只承担三件"每家都一样"的事：
/// ① SDK 可用性探测与统一报错；
/// ② <see cref="AxisId"/> → 厂商轴号 的映射（各家的轴号起点不同）；
/// ③ 轴号合法性校验。
/// </para>
/// <para>所有<b>厂商特有的差异</b>都留在各子类里，不做过度抽象 —— 那才是适配层该有的样子。</para>
/// </summary>
public abstract class VendorCardApiBase : IVendorCardApi
{
    private readonly Dictionary<AxisId, int> _axisIndices = new();

    protected VendorCardApiBase(string libraryName, IReadOnlyDictionary<AxisId, int> axisIndices)
    {
        SdkLibraryName = libraryName;
        ArgumentNullException.ThrowIfNull(axisIndices);
        _axisIndices = new Dictionary<AxisId, int>(axisIndices);

        if (_axisIndices.Count == 0)
        {
            throw new ArgumentException("必须至少配置一根轴的映射", nameof(axisIndices));
        }
    }

    /// <summary>原生库文件名（各子类以 <c>const LibraryName</c> 对外暴露，供 DllImport 使用）。</summary>
    public string SdkLibraryName { get; }

    public abstract string VendorName { get; }

    public IReadOnlyList<AxisId> Axes => _axisIndices.Keys.ToList();

    public abstract int InputCount { get; }

    public abstract int OutputCount { get; }

    /// <summary>SDK 是否就绪。</summary>
    public bool IsSdkAvailable => VendorSdkAvailability.IsAvailable(SdkLibraryName);

    /// <summary>把标准轴号翻译成厂商轴号。</summary>
    protected int IndexOf(AxisId axis)
    {
        if (!_axisIndices.TryGetValue(axis, out int index))
        {
            throw new MotionException($"厂商 {VendorName} 未配置轴 {axis.ToShortName()} 的轴号映射")
            {
                Category = AlarmCategory.Communication,
                AlarmCode = "MC-VENDOR-002",
            };
        }

        return index;
    }

    protected bool TryIndexOf(AxisId axis, out int index) => _axisIndices.TryGetValue(axis, out index);

    /// <summary>SDK 缺失时统一返回的错误码（由 <see cref="MotionControllerBase"/> 翻译成可读信息）。</summary>
    protected int NotAvailable() => VendorErrorCodes.NotAvailable;

    public abstract string GetErrorMessage(int errorCode);

    public abstract int Connect(string address, int port);

    public abstract void Disconnect();

    public abstract int SetServoEnabled(AxisId axis, bool enabled);

    public abstract int ConfigureProfile(AxisId axis, MoveProfile profile);

    public abstract int ConfigureSoftLimit(AxisId axis, double negativeLimit, double positiveLimit, bool enabled);

    public abstract int SetPosition(AxisId axis, double position);

    public abstract int MoveAbsolute(AxisId axis, double position);

    public abstract int MoveRelative(AxisId axis, double distance);

    public abstract int StartJog(AxisId axis, MotionDirection direction, double velocity);

    public abstract int StopJog(AxisId axis);

    public abstract int Home(AxisId axis, HomeMode mode, double speed, double offset);

    public abstract int Stop(AxisId axis, bool emergency);

    public abstract int ResetAlarm(AxisId axis);

    public abstract VendorAxisSnapshot ReadAxis(AxisId axis);

    public abstract bool ReadInput(int index);

    public abstract bool ReadOutput(int index);

    public abstract bool WriteOutput(int index, bool value);

    public abstract void Dispose();
}
