using System.Globalization;
using MotionCore.Abstractions.Motion;
using MotionCore.App.Mvvm;

namespace MotionCore.App.ViewModels;

/// <summary>轴状态行。</summary>
public sealed class AxisRowViewModel : ObservableObject
{
    private double _commandPosition;
    private double _actualPosition;
    private double _velocity;
    private bool _servoEnabled;
    private bool _inPosition;
    private bool _homed;
    private bool _positiveLimit;
    private bool _negativeLimit;
    private bool _alarm;
    private string _alarmText = string.Empty;

    public AxisRowViewModel(AxisId axis)
    {
        Axis = axis;
    }

    public AxisId Axis { get; }

    public string Name => Axis.ToShortName();

    public string Unit => Axis.Unit();

    public double CommandPosition
    {
        get => _commandPosition;
        set => SetProperty(ref _commandPosition, value);
    }

    public double ActualPosition
    {
        get => _actualPosition;
        set => SetProperty(ref _actualPosition, value);
    }

    public double Velocity
    {
        get => _velocity;
        set => SetProperty(ref _velocity, value);
    }

    /// <summary>跟随误差 = 指令位置 − 实际位置。数值异常增大说明伺服整定有问题或机构卡滞。</summary>
    public double FollowingError => _commandPosition - _actualPosition;

    public bool ServoEnabled
    {
        get => _servoEnabled;
        set => SetProperty(ref _servoEnabled, value);
    }

    public bool InPosition
    {
        get => _inPosition;
        set => SetProperty(ref _inPosition, value);
    }

    public bool Homed
    {
        get => _homed;
        set => SetProperty(ref _homed, value);
    }

    public bool PositiveLimit
    {
        get => _positiveLimit;
        set => SetProperty(ref _positiveLimit, value);
    }

    public bool NegativeLimit
    {
        get => _negativeLimit;
        set => SetProperty(ref _negativeLimit, value);
    }

    public bool Alarm
    {
        get => _alarm;
        set => SetProperty(ref _alarm, value);
    }

    public string AlarmText
    {
        get => _alarmText;
        set => SetProperty(ref _alarmText, value);
    }

    /// <summary>允许联动的轴（Z、R 由机型配置决定，这里全部开放）。</summary>
    public bool IsLinear => Axis.IsLinear();

    public void Update(AxisStatus status)
    {
        CommandPosition = status.CommandPosition;
        ActualPosition = status.ActualPosition;
        Velocity = status.Velocity;
        ServoEnabled = status.ServoEnabled;
        InPosition = status.InPosition;
        Homed = status.Homed;
        PositiveLimit = status.PositiveLimit;
        NegativeLimit = status.NegativeLimit;
        Alarm = status.Alarm;
        AlarmText = status.Alarm ? $"错误码 {status.AlarmCode}" : string.Empty;

        OnPropertyChanged(nameof(FollowingError));
    }

    public string Describe() => string.Format(
        CultureInfo.InvariantCulture,
        "{0}: {1:F4} {2}",
        Name,
        ActualPosition,
        Unit);
}
