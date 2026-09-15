using System.Collections.ObjectModel;
using System.Globalization;
using System.Text;
using System.Windows;
using System.Windows.Media.Imaging;
using MotionCore.Abstractions.Vision;

namespace MotionCore.App.Services;

/// <summary>日志级别。</summary>
public enum LogLevel
{
    Debug,
    Info,
    Success,
    Warning,
    Error,
}

/// <summary>一条日志。</summary>
public sealed record LogEntry(DateTimeOffset Timestamp, LogLevel Level, string Message)
{
    public string Display =>
        $"{Timestamp.ToString("HH:mm:ss.fff", CultureInfo.InvariantCulture)}  [{Level switch
        {
            LogLevel.Debug => "调试",
            LogLevel.Info => "信息",
            LogLevel.Success => "成功",
            LogLevel.Warning => "警告",
            LogLevel.Error => "错误",
            _ => "??",
        }}]  {Message}";

    /// <summary>用于界面上的颜色区分。</summary>
    public string LevelKey => Level.ToString();
}

/// <summary>
/// 应用日志。
/// <para>
/// 设备软件的日志必须满足两个要求：① 带毫秒时间戳（排查时序问题全靠它）；
/// ② 能在后台线程安全地写（状态轮询、报警都是后台线程触发的）。
/// </para>
/// </summary>
public sealed class AppLogger
{
    private const int MaxEntries = 2000;
    private readonly object _gate = new();

    /// <summary>界面绑定的日志集合（必须在 UI 线程上更新）。</summary>
    public ObservableCollection<LogEntry> Entries { get; } = [];

    public event EventHandler<LogEntry>? EntryAdded;

    public void Log(LogLevel level, string message)
    {
        LogEntry entry = new(DateTimeOffset.Now, level, message);

        // ObservableCollection 绑定到界面后只能在 UI 线程上修改。
        // 这里用 CheckAccess 区分两条路径：已经在 UI 线程就直连（避免多余的 Invoke 开销与潜在死锁），
        // 否则显式 Invoke 到 UI 线程 —— 状态轮询、报警都是后台线程触发的，这一步不能省。
        if (Application.Current is { } application && !application.Dispatcher.CheckAccess())
        {
            application.Dispatcher.Invoke(() => AddCore(entry));
        }
        else
        {
            AddCore(entry);
        }

        EntryAdded?.Invoke(this, entry);
    }

    private void AddCore(LogEntry entry)
    {
        lock (_gate)
        {
            Entries.Add(entry);
            while (Entries.Count > MaxEntries)
            {
                Entries.RemoveAt(0);
            }
        }
    }

    public void Debug(string message) => Log(LogLevel.Debug, message);

    public void Info(string message) => Log(LogLevel.Info, message);

    public void Success(string message) => Log(LogLevel.Success, message);

    public void Warn(string message) => Log(LogLevel.Warning, message);

    public void Error(string message) => Log(LogLevel.Error, message);

    /// <summary>把当前日志导出成文本（现场排查时直接发给供应商）。</summary>
    public string Export()
    {
        lock (_gate)
        {
            StringBuilder builder = new();
            foreach (LogEntry entry in Entries)
            {
                builder.AppendLine(entry.Display);
            }

            return builder.ToString();
        }
    }
}

/// <summary>图像互操作工具。</summary>
public static class ImageInterop
{
    /// <summary>
    /// 灰度帧 → WPF 位图。
    /// <para>返回前必须 <c>Freeze()</c>，否则跨线程绑定会抛 InvalidOperationException ——
    /// 这是 WPF 上位机里最常踩的坑之一。</para>
    /// </summary>
    public static BitmapSource ToBitmapSource(CameraFrame frame)
    {
        ArgumentNullException.ThrowIfNull(frame);
        frame.Validate();

        BitmapSource source = BitmapSource.Create(
            frame.Width,
            frame.Height,
            96d,
            96d,
            System.Windows.Media.PixelFormats.Gray8,
            null,
            frame.Pixels,
            frame.Width);

        source.Freeze();
        return source;
    }
}
