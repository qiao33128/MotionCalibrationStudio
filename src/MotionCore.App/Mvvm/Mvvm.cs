using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Windows.Input;

namespace MotionCore.App.Mvvm;

/// <summary>
/// 极简 MVVM 基类。
/// <para>
/// 刻意不引入 CommunityToolkit.Mvvm / Prism 之类的框架：
/// 这套上位机只有 3 个界面，<see cref="ObservableObject"/> + <see cref="RelayCommand"/>
/// 总共不到 120 行就够用，多一个依赖只会增加现场部署和升级的成本。
/// </para>
/// </summary>
public abstract class ObservableObject : INotifyPropertyChanged
{
    public event PropertyChangedEventHandler? PropertyChanged;

    protected void OnPropertyChanged([CallerMemberName] string? propertyName = null) =>
        PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));

    /// <summary>设置字段并在值变化时触发通知，返回是否真的发生了变化。</summary>
    protected bool SetProperty<T>(ref T field, T value, [CallerMemberName] string? propertyName = null)
    {
        if (EqualityComparer<T>.Default.Equals(field, value))
        {
            return false;
        }

        field = value;
        OnPropertyChanged(propertyName);
        return true;
    }
}

/// <summary>
/// 可主动通知可用性变化的命令。
/// <para>设备软件的按钮可用性完全由状态机决定（未就绪时不能运动、无报警时不能复位），
/// 因此必须有统一的"重新求值"入口。</para>
/// </summary>
public interface IAppCommand : ICommand
{
    void RaiseCanExecuteChanged();
}

/// <summary>同步命令。</summary>
public sealed class RelayCommand : IAppCommand
{
    private readonly Action<object?> _execute;
    private readonly Func<object?, bool>? _canExecute;

    public RelayCommand(Action execute, Func<bool>? canExecute = null)
        : this(_ => execute(), canExecute is null ? null : _ => canExecute())
    {
    }

    public RelayCommand(Action<object?> execute, Func<object?, bool>? canExecute = null)
    {
        _execute = execute ?? throw new ArgumentNullException(nameof(execute));
        _canExecute = canExecute;
    }

    public event EventHandler? CanExecuteChanged;

    public bool CanExecute(object? parameter) => _canExecute?.Invoke(parameter) ?? true;

    public void Execute(object? parameter) => _execute(parameter);

    public void RaiseCanExecuteChanged() => CanExecuteChanged?.Invoke(this, EventArgs.Empty);
}

/// <summary>
/// 异步命令。
/// <para>
/// 关键点是 <c>_isRunning</c> 互斥：设备软件里"同一个动作被连点两次"
/// （例如连点两次回零）是非常典型的事故来源，必须在命令层就挡住。
/// </para>
/// </summary>
public sealed class AsyncRelayCommand : IAppCommand
{
    private readonly Func<object?, Task> _execute;
    private readonly Func<object?, bool>? _canExecute;
    private bool _isRunning;

    public AsyncRelayCommand(Func<Task> execute, Func<bool>? canExecute = null)
        : this(_ => execute(), canExecute is null ? null : _ => canExecute())
    {
    }

    public AsyncRelayCommand(Func<object?, Task> execute, Func<object?, bool>? canExecute = null)
    {
        _execute = execute ?? throw new ArgumentNullException(nameof(execute));
        _canExecute = canExecute;
    }

    public event EventHandler? CanExecuteChanged;

    public bool IsRunning => _isRunning;

    public bool CanExecute(object? parameter) => !_isRunning && (_canExecute?.Invoke(parameter) ?? true);

    public async void Execute(object? parameter)
    {
        if (!CanExecute(parameter))
        {
            return;
        }

        _isRunning = true;
        RaiseCanExecuteChanged();

        try
        {
            await _execute(parameter).ConfigureAwait(true);
        }
        finally
        {
            _isRunning = false;
            RaiseCanExecuteChanged();
        }
    }

    public void RaiseCanExecuteChanged() => CanExecuteChanged?.Invoke(this, EventArgs.Empty);
}
