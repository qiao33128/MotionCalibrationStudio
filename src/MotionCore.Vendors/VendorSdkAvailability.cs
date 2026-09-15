using System.Runtime.InteropServices;

namespace MotionCore.Vendors;

/// <summary>厂商适配层统一错误码。</summary>
public static class VendorErrorCodes
{
    public const int Ok = 0;

    /// <summary>厂商 SDK 动态库未随程序发布（最常见的"现场一上机就崩"的根因）。</summary>
    public const int NotAvailable = 900;

    public const int NotConnected = 901;

    public const int InvalidAxis = 902;
}

/// <summary>
/// 厂商动态库探测。
/// <para>
/// <b>为什么需要它：</b> 雷赛/固高/正运动的 SDK 都是原生 DLL，
/// 如果程序启动时直接静态绑定，缺任何一个 DLL 都会让整个上位机连"参数配置界面"都打不开 ——
/// 而现场往往只装了一种卡。所以必须做"惰性探测 + 明确报错"。
/// </para>
/// <para>
/// 更进一步的做法是把每个厂商适配器拆成独立的 Plugin DLL，由宿主按配置动态加载；
/// 本示例为了保持单文件可读性，用 <see cref="NativeLibrary.TryLoad"/> 做运行期探测，
/// 效果等价。
/// </para>
/// </summary>
public static class VendorSdkAvailability
{
    private static readonly Dictionary<string, bool> Cache = new(StringComparer.OrdinalIgnoreCase);
    private static readonly object Gate = new();

    /// <summary>探测某个原生库是否可以加载（结果会缓存）。</summary>
    public static bool IsAvailable(string libraryName)
    {
        if (string.IsNullOrWhiteSpace(libraryName))
        {
            return false;
        }

        lock (Gate)
        {
            if (Cache.TryGetValue(libraryName, out bool cached))
            {
                return cached;
            }

            bool available;
            try
            {
                available = NativeLibrary.TryLoad(libraryName, out nint handle);
                if (available && handle != nint.Zero)
                {
                    NativeLibrary.Free(handle);
                }
            }
            catch (Exception)
            {
                available = false;
            }

            Cache[libraryName] = available;
            return available;
        }
    }

    /// <summary>列出各厂商 SDK 的探测结果，用于"关于 / 诊断"界面。</summary>
    public static IReadOnlyDictionary<string, bool> ProbeAll() => new Dictionary<string, bool>
    {
        [ZmotionCardApi.LibraryName] = IsAvailable(ZmotionCardApi.LibraryName),
        [GoogolCardApi.LibraryName] = IsAvailable(GoogolCardApi.LibraryName),
        [LeadshineCardApi.LibraryName] = IsAvailable(LeadshineCardApi.LibraryName),
    };
}
