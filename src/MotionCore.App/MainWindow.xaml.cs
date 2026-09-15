using System.Collections.Specialized;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Threading;
using MotionCore.App.ViewModels;

namespace MotionCore.App;

/// <summary>
/// 主窗口。
/// <para>
/// 代码后置里只留一件事：日志自动滚动到底部。
/// 任何"业务"都不应该出现在这里 —— ViewModel 才是唯一的行为来源。
/// </para>
/// </summary>
public partial class MainWindow : Window
{
    private INotifyCollectionChanged? _logCollection;

    public MainWindow()
    {
        InitializeComponent();
        DataContextChanged += OnDataContextChanged;
    }

    private void OnDataContextChanged(object sender, DependencyPropertyChangedEventArgs e)
    {
        if (_logCollection is not null)
        {
            _logCollection.CollectionChanged -= OnLogCollectionChanged;
            _logCollection = null;
        }

        if (e.NewValue is MainViewModel viewModel)
        {
            _logCollection = viewModel.Logger.Entries;
            _logCollection.CollectionChanged += OnLogCollectionChanged;
        }
    }

    private void OnLogCollectionChanged(object? sender, NotifyCollectionChangedEventArgs e)
    {
        if (e.Action != NotifyCollectionChangedAction.Add)
        {
            return;
        }

        // 🔴 自动滚动必须延后一帧执行，不能在 CollectionChanged 里同步调用 ScrollIntoView：
        // 同步调用会迫使 ItemContainerGenerator 在"正在处理集合变更"的过程中重新生成容器，
        // 造成可重入，进而抛出「某个 ItemsControl 与它的项源不一致」并直接把进程干掉。
        // 这是 WPF 上位机里非常常见、又非常难查的一个坑。
        Dispatcher.BeginInvoke(
            DispatcherPriority.Background,
            new Action(() =>
            {
                if (LogList.Items.Count > 0)
                {
                    LogList.ScrollIntoView(LogList.Items[^1]);
                }
            }));
    }
}
