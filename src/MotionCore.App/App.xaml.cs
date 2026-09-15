using System.Windows;
using System.Windows.Threading;
using MotionCore.App.ViewModels;

namespace MotionCore.App;

/// <summary>
/// 应用入口。
/// <para>
/// 刻意不用 <c>StartupUri</c>：上位机的 ViewModel 持有运动卡与相机的连接，
/// 必须在窗口关闭时<b>确定地</b>释放（否则真机上会出现"卡被占用、下次启动连不上"的经典问题）。
/// 因此这里手工创建窗口并在 <see cref="OnExit"/> 里做释放。
/// </para>
/// <para>
/// 另外接入了未处理异常兜底：设备软件宁可弹框让人看到错误，也不能"界面还在、逻辑已经死了"。
/// 现场最怕的就是操作工不知道程序已经异常，继续点按钮。
/// </para>
/// </summary>
public partial class App : Application
{
    private MainViewModel? _viewModel;

    protected override void OnStartup(StartupEventArgs e)
    {
        base.OnStartup(e);

        DispatcherUnhandledException += OnDispatcherUnhandledException;
        AppDomain.CurrentDomain.UnhandledException += OnDomainUnhandledException;
        TaskScheduler.UnobservedTaskException += OnUnobservedTaskException;

        _viewModel = new MainViewModel();
        MainWindow window = new() { DataContext = _viewModel };
        MainWindow = window;
        window.Show();

        // 命令行传入 --demo：启动后自动跑一遍「示教 → 标定 → 模型对比 → 对位」全链路，
        // 用于演示与回归自检。
        if (e.Args.Any(argument => argument.Equals("--demo", StringComparison.OrdinalIgnoreCase)))
        {
            _ = _viewModel.RunAutoDemoAsync();
        }
    }

    protected override async void OnExit(ExitEventArgs e)
    {
        if (_viewModel is not null)
        {
            await _viewModel.DisposeAsync();
            _viewModel = null;
        }

        base.OnExit(e);
    }

    private void OnDispatcherUnhandledException(object sender, DispatcherUnhandledExceptionEventArgs e)
    {
        _viewModel?.Logger.Error($"界面线程未处理异常：{e.Exception.Message}");
        MessageBox.Show(
            $"发生未处理异常：\n\n{e.Exception.Message}\n\n程序已记录日志，建议排查后重新初始化控制器。",
            "运动控制上位机",
            MessageBoxButton.OK,
            MessageBoxImage.Error);

        // 标记为已处理，避免整个上位机被一个界面异常直接关掉
        e.Handled = true;
    }

    private void OnDomainUnhandledException(object sender, UnhandledExceptionEventArgs e)
    {
        if (e.ExceptionObject is Exception exception)
        {
            MessageBox.Show(
                $"发生严重错误，程序即将退出：\n\n{exception.Message}",
                "运动控制上位机",
                MessageBoxButton.OK,
                MessageBoxImage.Stop);
        }
    }

    private void OnUnobservedTaskException(object? sender, UnobservedTaskExceptionEventArgs e)
    {
        _viewModel?.Logger.Warn($"后台任务异常（已被观察，不影响主流程）：{e.Exception.Message}");
        e.SetObserved();
    }
}
