using System.Collections.ObjectModel;
using System.Globalization;
using System.IO;
using System.Text;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using MotionCore.Abstractions.Geometry;
using MotionCore.Abstractions.Motion;
using MotionCore.Abstractions.Vision;
using MotionCore.App.Mvvm;
using MotionCore.App.Services;
using MotionCore.Calibration;
using MotionCore.Calibration.Alignment;
using MotionCore.Calibration.Models;
using MotionCore.Calibration.Persistence;
using MotionCore.Calibration.Teaching;
using MotionCore.Simulation;
using MotionCore.Simulation.Vision;

namespace MotionCore.App.ViewModels;

/// <summary>
/// 主界面 ViewModel。
/// <para>
/// 它把四个东西串起来：仿真运动卡、仿真相机、标定解算器、视觉对位服务。
/// <b>业务逻辑一行都不在这里</b> —— 这里只负责把用户操作翻译成服务调用、把结果投影到界面。
/// </para>
/// </summary>
public sealed class MainViewModel : ObservableObject, IAsyncDisposable
{
    /// <summary>仿真相机里固定 Mark 的真实机械坐标。</summary>
    private static readonly Point2D SimulatedMarkPosition = new(12.5d, -7.25d);

    private readonly SimulatedMotionController _controller;
    private readonly SimulatedCamera _camera;
    private readonly BlobCentroidLocator _locator = new();
    private readonly List<CalibrationPoint> _taughtPoints = [];

    private CardState _cardState = CardState.Disconnected;
    private string _statusText = "未初始化";
    private string _lastAlarmText = "无";
    private CalibrationResult? _calibration;
    private string _calibrationReport = "尚未执行标定。请先在「运动控制台」完成初始化，再点击「开始九点示教」。";
    private string _alignmentTrace = "尚未执行对位。";
    private string _modelComparisonReport = "尚未比较模型。";
    private BitmapSource? _cameraImage;
    private string _cameraInfo = "未采集";
    private Point2D? _referencePixel;
    private string _referenceText = "尚未建立（默认使用图像中心）";
    private double _teachingSpanMm = 3d;
    private CalibrationModelKind _selectedModel = CalibrationModelKind.Affine;
    private bool _runLeaveOneOut = true;
    private double _alignmentToleranceMm = 0.01d;
    private int _alignmentMaxIterations = 4;
    private double _alignmentMaxInitialErrorMm = 1.0d;
    private double _disturbanceX = 0.6d;
    private double _disturbanceY = -0.45d;
    private double _targetX = 12.5d;
    private double _targetY = -7.25d;
    private double _jogVelocity = 30d;
    private AxisId _selectedAxis = AxisId.X;
    private bool _isBusy;
    private int _selectedTabIndex;
    private string _ioStatusText = "未读取（点击「刷新 IO」）";

    public MainViewModel()
    {
        Logger = new AppLogger();

        _controller = new SimulatedMotionController(
            new SimulatedCardOptions
            {
                PositioningNoise = 0.004d, // 与真实丝杆平台的重复定位精度同量级
                FollowingLagSeconds = 0.003d,
                ConnectDelayMs = 300,
            },
            new MotionControllerOptions
            {
                Address = "192.168.0.11（仿真）",
                StatusPollIntervalMs = 30,
                MotionTimeout = TimeSpan.FromSeconds(15),
                Axes =
                [
                    new AxisConfiguration { Axis = AxisId.X, Profile = DemoProfile, FineProfile = MoveProfile.Fine },
                    new AxisConfiguration { Axis = AxisId.Y, Profile = DemoProfile, FineProfile = MoveProfile.Fine },
                    new AxisConfiguration { Axis = AxisId.Z, SoftLimitMin = -100d, SoftLimitMax = 50d, Profile = DemoProfile },
                    new AxisConfiguration
                    {
                        Axis = AxisId.R,
                        SoftLimitMin = -180d,
                        SoftLimitMax = 180d,
                        Profile = new MoveProfile(90d, 720d, 720d),
                    },
                ],
            });

        _camera = new SimulatedCamera(
            _controller,
            new SimulatedCameraOptions
            {
                Mount = new CameraMount
                {
                    PixelsPerMmU = 24d,
                    PixelsPerMmV = 24d,
                    RotationDeg = 1.7d, // 模拟现场真实存在的安装角偏差
                    FlipVertical = true,
                },
                NoiseAmplitude = 4d,
                ExposureDelayMs = 15,
            },
            SimulatedMarkPosition);

        foreach (AxisId axis in _controller.Axes)
        {
            Axes.Add(new AxisRowViewModel(axis));
        }

        _controller.StateChanged += OnCardStateChanged;
        _controller.StatusUpdated += OnStatusUpdated;
        _controller.AlarmRaised += OnAlarmRaised;

        InitializeCommand = new AsyncRelayCommand(InitializeAsync);
        ShutdownCommand = new AsyncRelayCommand(ShutdownAsync, () => CardState != CardState.Disconnected);
        HomeAllCommand = new AsyncRelayCommand(HomeAllAsync, () => IsReady);
        HomeSelectedCommand = new AsyncRelayCommand(HomeSelectedAsync, () => IsReady);
        MoveAbsoluteCommand = new AsyncRelayCommand(MoveAbsoluteAsync, () => IsReady);
        EnableCommand = new AsyncRelayCommand(EnableAsync, () => IsReady);
        DisableCommand = new AsyncRelayCommand(DisableAsync, () => IsReady);
        StopCommand = new AsyncRelayCommand(StopAsync, () => CardState is CardState.Ready or CardState.Moving);
        EmergencyStopCommand = new AsyncRelayCommand(EmergencyStopAsync, () => CardState != CardState.Disconnected);
        ResetAlarmCommand = new AsyncRelayCommand(ResetAlarmAsync, () => CardState == CardState.Alarm);
        JogPositiveCommand = new AsyncRelayCommand(() => StartJogAsync(MotionDirection.Positive), () => IsReady);
        JogNegativeCommand = new AsyncRelayCommand(() => StartJogAsync(MotionDirection.Negative), () => IsReady);
        StopJogCommand = new AsyncRelayCommand(() => _controller.StopJogAsync(SelectedAxis));
        InjectAlarmCommand = new AsyncRelayCommand(InjectAlarmAsync, () => CardState != CardState.Disconnected);

        TeachCommand = new AsyncRelayCommand(TeachAsync, () => IsReady && !IsBusy);
        CalibrateCommand = new AsyncRelayCommand(CalibrateAsync, () => _taughtPoints.Count >= 9);
        CompareModelsCommand = new AsyncRelayCommand(CompareModelsAsync, () => _taughtPoints.Count >= 9);
        SaveCalibrationCommand = new RelayCommand(SaveCalibration, () => _calibration is not null);
        LoadCalibrationCommand = new RelayCommand(LoadCalibration);

        CaptureCommand = new AsyncRelayCommand(CaptureAsync);
        SetReferenceCommand = new AsyncRelayCommand(SetReferenceAsync);
        DisturbCommand = new AsyncRelayCommand(DisturbAsync, () => IsReady);
        AlignCommand = new AsyncRelayCommand(AlignAsync, () => IsReady && _calibration is not null);

        ClearLogCommand = new RelayCommand(() => Logger.Entries.Clear());
        RefreshIoCommand = new RelayCommand(RefreshIo);
        AutoDemoCommand = new AsyncRelayCommand(RunAutoDemoAsync, () => !IsBusy);

        _initializeTask = InitializeAsync();
    }

    private readonly Task _initializeTask;

    /// <summary>
    /// 一键自动演示：初始化 → 九点示教 → 标定 → 模型对比 → 建立对位基准 → 故意偏移 → 自动对位。
    /// <para>
    /// 存在的意义：把"整条链路是通的"这件事变成一条命令就能复现，
    /// 而不是靠人对着文档一步步点。面试演示、客户演示、回归自检都用它。
    /// 命令行传入 <c>--demo</c> 也会自动执行。
    /// </para>
    /// </summary>
    public async Task RunAutoDemoAsync()
    {
        await _initializeTask.ConfigureAwait(true);

        Logger.Info("════ 一键自动演示开始 ════");

        // 停在「九点标定」页，让人看清标定画布与精度报告
        SelectedTabIndex = 1;
        await TeachAsync();
        await CompareModelsAsync();
        await Task.Delay(2500);

        SelectedTabIndex = 2;
        await SetReferenceAsync();
        await DisturbAsync();
        await AlignAsync();

        Logger.Success("════ 一键自动演示结束 ════");
    }

    public static MoveProfile DemoProfile => new(45d, 900d, 900d);

    // ───────────────────────── 数据 ─────────────────────────

    public AppLogger Logger { get; }

    public ObservableCollection<AxisRowViewModel> Axes { get; } = [];

    public ObservableCollection<CalibrationPointRowViewModel> CalibrationRows { get; } = [];

    public IReadOnlyList<CalibrationModelKind> AvailableModels { get; } =
        Enum.GetValues<CalibrationModelKind>();

    public IReadOnlyList<AxisId> AvailableAxes => _controller.Axes;

    public string SimulatedMarkPositionText =>
        $"Mark 真实机械坐标：X = {SimulatedMarkPosition.X:F3} mm，Y = {SimulatedMarkPosition.Y:F3} mm（仿真相机设定值）";

    // ───────────────────────── 状态 ─────────────────────────

    public CardState CardState
    {
        get => _cardState;
        private set
        {
            if (SetProperty(ref _cardState, value))
            {
                OnPropertyChanged(nameof(IsReady));
                OnPropertyChanged(nameof(StateBrush));
                RaiseCommandStates();
            }
        }
    }

    public bool IsReady => CardState == CardState.Ready;

    private static readonly IReadOnlyDictionary<CardState, Brush> StateBrushes = BuildStateBrushes();

    /// <summary>状态指示灯颜色（直接暴露 Brush，避免 XAML 侧做字符串到画刷的隐式转换）。</summary>
    public Brush StateBrush =>
        StateBrushes.TryGetValue(CardState, out Brush? brush) ? brush : Brushes.Gray;

    private static IReadOnlyDictionary<CardState, Brush> BuildStateBrushes()
    {
        Dictionary<CardState, Brush> map = new()
        {
            [CardState.Disconnected] = Make("#607D8B"),
            [CardState.Initializing] = Make("#FFB300"),
            [CardState.Ready] = Make("#4CC97A"),
            [CardState.Moving] = Make("#2E9BFF"),
            [CardState.Hold] = Make("#FF9800"),
            [CardState.Alarm] = Make("#FF5A5A"),
            [CardState.EmergencyStop] = Make("#D81B60"),
            [CardState.Disposed] = Make("#455A64"),
        };

        return map;

        static Brush Make(string hex)
        {
            SolidColorBrush brush = new((Color)ColorConverter.ConvertFromString(hex));
            brush.Freeze();
            return brush;
        }
    }

    public string StatusText
    {
        get => _statusText;
        private set => SetProperty(ref _statusText, value);
    }

    public string ControllerName => _controller.Name;

    public string LastAlarmText
    {
        get => _lastAlarmText;
        private set => SetProperty(ref _lastAlarmText, value);
    }

    public bool IsBusy
    {
        get => _isBusy;
        private set
        {
            if (SetProperty(ref _isBusy, value))
            {
                RaiseCommandStates();
            }
        }
    }

    public CalibrationResult? Calibration
    {
        get => _calibration;
        private set
        {
            if (SetProperty(ref _calibration, value))
            {
                OnPropertyChanged(nameof(CalibrationSummary));
                OnPropertyChanged(nameof(HasCalibration));
            }
        }
    }

    public bool HasCalibration => _calibration is not null;

    public string CalibrationSummary => _calibration is null
        ? "尚未标定"
        : string.Format(
            CultureInfo.InvariantCulture,
            "RMSE {0:F6} mm ｜ 最大误差 {1:F6} mm ｜ 条件数 {2:E2} ｜ {3}",
            _calibration.RmseMm,
            _calibration.MaxErrorMm,
            _calibration.ConditionNumber,
            _calibration.Quality);

    public string CalibrationReport
    {
        get => _calibrationReport;
        private set => SetProperty(ref _calibrationReport, value);
    }

    public string ModelComparisonReport
    {
        get => _modelComparisonReport;
        private set => SetProperty(ref _modelComparisonReport, value);
    }

    public string AlignmentTrace
    {
        get => _alignmentTrace;
        private set => SetProperty(ref _alignmentTrace, value);
    }

    public BitmapSource? CameraImage
    {
        get => _cameraImage;
        private set => SetProperty(ref _cameraImage, value);
    }

    public string CameraInfo
    {
        get => _cameraInfo;
        private set => SetProperty(ref _cameraInfo, value);
    }

    public string ReferenceText
    {
        get => _referenceText;
        private set => SetProperty(ref _referenceText, value);
    }

    // ───────────────────────── 参数 ─────────────────────────

    public double TeachingSpanMm
    {
        get => _teachingSpanMm;
        set => SetProperty(ref _teachingSpanMm, value);
    }

    public CalibrationModelKind SelectedModel
    {
        get => _selectedModel;
        set => SetProperty(ref _selectedModel, value);
    }

    public bool RunLeaveOneOut
    {
        get => _runLeaveOneOut;
        set => SetProperty(ref _runLeaveOneOut, value);
    }

    public double AlignmentToleranceMm
    {
        get => _alignmentToleranceMm;
        set => SetProperty(ref _alignmentToleranceMm, value);
    }

    public int AlignmentMaxIterations
    {
        get => _alignmentMaxIterations;
        set => SetProperty(ref _alignmentMaxIterations, value);
    }

    public double AlignmentMaxInitialErrorMm
    {
        get => _alignmentMaxInitialErrorMm;
        set => SetProperty(ref _alignmentMaxInitialErrorMm, value);
    }

    public double DisturbanceX
    {
        get => _disturbanceX;
        set => SetProperty(ref _disturbanceX, value);
    }

    public double DisturbanceY
    {
        get => _disturbanceY;
        set => SetProperty(ref _disturbanceY, value);
    }

    public double TargetX
    {
        get => _targetX;
        set => SetProperty(ref _targetX, value);
    }

    public double TargetY
    {
        get => _targetY;
        set => SetProperty(ref _targetY, value);
    }

    public double JogVelocity
    {
        get => _jogVelocity;
        set => SetProperty(ref _jogVelocity, value);
    }

    public AxisId SelectedAxis
    {
        get => _selectedAxis;
        set => SetProperty(ref _selectedAxis, value);
    }

    // ───────────────────────── 命令 ─────────────────────────

    public AsyncRelayCommand InitializeCommand { get; }

    public AsyncRelayCommand ShutdownCommand { get; }

    public AsyncRelayCommand HomeAllCommand { get; }

    public AsyncRelayCommand HomeSelectedCommand { get; }

    public AsyncRelayCommand MoveAbsoluteCommand { get; }

    public AsyncRelayCommand EnableCommand { get; }

    public AsyncRelayCommand DisableCommand { get; }

    public AsyncRelayCommand StopCommand { get; }

    public AsyncRelayCommand EmergencyStopCommand { get; }

    public AsyncRelayCommand ResetAlarmCommand { get; }

    public AsyncRelayCommand JogPositiveCommand { get; }

    public AsyncRelayCommand JogNegativeCommand { get; }

    public AsyncRelayCommand StopJogCommand { get; }

    public AsyncRelayCommand InjectAlarmCommand { get; }

    public AsyncRelayCommand TeachCommand { get; }

    public AsyncRelayCommand CalibrateCommand { get; }

    public AsyncRelayCommand CompareModelsCommand { get; }

    public RelayCommand SaveCalibrationCommand { get; }

    public RelayCommand LoadCalibrationCommand { get; }

    public AsyncRelayCommand CaptureCommand { get; }

    public AsyncRelayCommand SetReferenceCommand { get; }

    public AsyncRelayCommand DisturbCommand { get; }

    public AsyncRelayCommand AlignCommand { get; }

    public RelayCommand ClearLogCommand { get; }

    public RelayCommand RefreshIoCommand { get; }

    public AsyncRelayCommand AutoDemoCommand { get; }

    /// <summary>当前选中的页签（演示模式会自动切页）。</summary>
    public int SelectedTabIndex
    {
        get => _selectedTabIndex;
        set => SetProperty(ref _selectedTabIndex, value);
    }

    public string IoStatusText
    {
        get => _ioStatusText;
        private set => SetProperty(ref _ioStatusText, value);
    }

    private void RefreshIo()
    {
        StringBuilder builder = new();
        builder.Append("输入 DI0-15 : ");
        for (int i = 0; i < _controller.Io.InputCount; i++)
        {
            builder.Append(_controller.Io.ReadInput(i) ? '1' : '0');
        }

        builder.AppendLine();
        builder.Append("输出 DO0-15 : ");
        for (int i = 0; i < _controller.Io.OutputCount; i++)
        {
            builder.Append(_controller.Io.ReadOutput(i) ? '1' : '0');
        }

        IoStatusText = builder.ToString();
    }

    // ───────────────────────── 实现 ─────────────────────────

    private async Task InitializeAsync()
    {
        IsBusy = true;
        try
        {
            Logger.Info($"开始初始化控制器：{_controller.Name}");

            await _controller.InitializeAsync();
            await _camera.OpenAsync();

            Point2D current = (await _controller.GetPositionAsync()).Planar;
            Logger.Success($"控制器就绪；相机已打开（{_camera.Width}×{_camera.Height}）");
            Logger.Info(SimulatedMarkPositionText);
            Logger.Info($"当前平台位置：{current}");

            await _controller.MoveAbsoluteAsync(SimulatedMarkPosition, DemoProfile);
            Logger.Info("已移动到标定中心（Mark 成像于视野中心）");

            await CaptureAsync();
        }
        catch (Exception ex)
        {
            Logger.Error($"初始化失败：{ex.Message}");
        }
        finally
        {
            IsBusy = false;
        }
    }

    private async Task ShutdownAsync()
    {
        Logger.Info("关闭控制器连接");
        await _controller.ShutdownAsync();
        CardState = _controller.State;
    }

    private async Task HomeAllAsync()
    {
        IsBusy = true;
        try
        {
            Logger.Info("开始回零（依次进行，避免多轴同时冲向原点造成机械干涉）");
            await _controller.HomeAllAsync();
            Logger.Success("全部轴回零完成，用户坐标系已建立");
            await _controller.MoveAbsoluteAsync(SimulatedMarkPosition, DemoProfile);
        }
        catch (Exception ex)
        {
            Logger.Error($"回零失败：{ex.Message}");
        }
        finally
        {
            IsBusy = false;
        }
    }

    private async Task HomeSelectedAsync()
    {
        try
        {
            Logger.Info($"轴 {SelectedAxis.ToShortName()} 回零");
            await _controller.HomeAsync(SelectedAxis);
            Logger.Success($"轴 {SelectedAxis.ToShortName()} 回零完成");
        }
        catch (Exception ex)
        {
            Logger.Error($"回零失败：{ex.Message}");
        }
    }

    private async Task MoveAbsoluteAsync()
    {
        try
        {
            Logger.Info($"联动定位 → X = {TargetX:F4}，Y = {TargetY:F4}");
            await _controller.MoveAbsoluteAsync(new MotionPose(TargetX, TargetY), DemoProfile);
            Logger.Success("定位完成");
        }
        catch (MotionSoftLimitException ex)
        {
            Logger.Warn(ex.Message);
        }
        catch (Exception ex)
        {
            Logger.Error($"定位失败：{ex.Message}");
        }
    }

    private async Task EnableAsync()
    {
        await _controller.EnableAsync(SelectedAxis);
        Logger.Info($"轴 {SelectedAxis.ToShortName()} 已使能");
    }

    private async Task DisableAsync()
    {
        await _controller.DisableAsync(SelectedAxis);
        Logger.Warn($"轴 {SelectedAxis.ToShortName()} 已关闭使能（机构将失去保持力矩）");
    }

    private async Task StopAsync()
    {
        await _controller.StopAsync();
        Logger.Warn("已发送停止指令");
    }

    private async Task EmergencyStopAsync()
    {
        await _controller.StopAsync(emergency: true);
        Logger.Error("急停已触发！排除风险后需重新初始化");
    }

    private async Task ResetAlarmAsync()
    {
        await _controller.ResetAlarmAsync();
        LastAlarmText = "无";
        Logger.Success("报警已复位，伺服已重新使能");
    }

    private async Task StartJogAsync(MotionDirection direction)
    {
        try
        {
            await _controller.StartJogAsync(SelectedAxis, direction, new MoveProfile(JogVelocity, 1000d, 1000d));
            Logger.Info($"轴 {SelectedAxis.ToShortName()} 点动（{direction}，{JogVelocity:F1} mm/s）");
        }
        catch (Exception ex)
        {
            Logger.Warn($"点动被拒绝：{ex.Message}");
        }
    }

    private Task InjectAlarmAsync()
    {
        // 用于演示"报警传播 → 状态机切换 → 复位后重新使能"这一整条链路
        _controller.Card.InjectAlarm(SelectedAxis, SimulatedErrorCodes.ServoDisabled);
        Logger.Warn($"已向轴 {SelectedAxis.ToShortName()} 注入一次伺服报警（演示用），等待状态轮询上报");
        return Task.CompletedTask;
    }

    // ───────────────────────── 标定 ─────────────────────────

    private async Task TeachAsync()
    {
        IsBusy = true;
        try
        {
            Logger.Info($"开始九点示教：行程半幅 ±{TeachingSpanMm:F2} mm，共 9 点");

            NinePointTeachingService teaching = new(
                _controller,
                _camera,
                _locator,
                new NinePointTeachingOptions
                {
                    SpanMm = TeachingSpanMm,
                    SettleDelayMs = 80,
                    Center = SimulatedMarkPosition,
                    DemoProfile = DemoProfile,
                });

            Progress<TeachingProgress> progress = new(item =>
            {
                if (item.Mark is null)
                {
                    Logger.Error(item.Message);
                }
                else
                {
                    Logger.Info($"[{item.Index}/{item.Total}] {item.Message}");
                }
            });

            CalibrationPointSet pointSet = await teaching.TeachAsync(progress);

            _taughtPoints.Clear();
            _taughtPoints.AddRange(pointSet.Points);

            CalibrationRows.Clear();
            foreach (CalibrationPoint point in pointSet.Points)
            {
                CalibrationRows.Add(new CalibrationPointRowViewModel(point));
            }

            Logger.Success($"示教完成，共采集 {pointSet.Count} 个标定点");
            await CaptureAsync();
            await CalibrateAsync();
        }
        catch (Exception ex)
        {
            Logger.Error($"示教失败：{ex.Message}");
        }
        finally
        {
            IsBusy = false;
        }
    }

    private async Task CalibrateAsync()
    {
        if (_taughtPoints.Count < 9)
        {
            Logger.Warn("标定点不足 9 个，无法解算");
            return;
        }

        try
        {
            CalibrationOptions options = new()
            {
                Model = SelectedModel,
                RunLeaveOneOutValidation = RunLeaveOneOut,
            };

            CalibrationResult result = NinePointCalibrator.Calibrate(new CalibrationPointSet(_taughtPoints), options);
            Calibration = result;
            CalibrationReport = result.BuildReport();

            // 单点残差超过 3 倍 RMSE 时在表格里标红：现场判据就是"哪个点采坏了"
            double outlierThreshold = Math.Max(result.RmseMm * 3d, options.ExcellentRmseMm);
            for (int i = 0; i < CalibrationRows.Count && i < result.Residuals.Count; i++)
            {
                CalibrationRows[i].Apply(result.Residuals[i], outlierThreshold);
            }

            ReferenceText = "标定已更新，请重新建立对位基准";
            Logger.Success(
                $"标定完成：{result.Kind}，RMSE {result.RmseMm:F6} mm，最大误差 {result.MaxErrorMm:F6} mm，"
                + $"质量 {result.Quality}");

            if (result.IsIllConditioned)
            {
                Logger.Warn($"条件数 {result.ConditionNumber:E2} 偏大，标定点布局可能退化，建议增大示教行程");
            }

            if (result.IsOverFitting)
            {
                Logger.Warn("留一验证误差远大于训练 RMSE，疑似过拟合（模型参数过多），建议改用仿射模型");
            }

            await CaptureAsync();
        }
        catch (Exception ex)
        {
            Logger.Error($"解算失败：{ex.Message}");
            CalibrationReport = $"解算失败：{ex.Message}";
        }
    }

    private async Task CompareModelsAsync()
    {
        try
        {
            CalibrationOptions options = new() { RunLeaveOneOutValidation = true };
            IReadOnlyDictionary<CalibrationModelKind, CalibrationResult> results =
                NinePointCalibrator.CompareAllModels(new CalibrationPointSet(_taughtPoints), options);

            System.Text.StringBuilder builder = new();
            builder.AppendLine("========== 模型选型对比（同一组标定数据）==========");
            builder.AppendLine("模型                      训练RMSE(mm)   留一RMSE(mm)   条件数      结论");
            builder.AppendLine("--------------------------------------------------------------------------");

            foreach ((CalibrationModelKind kind, CalibrationResult result) in results)
            {
                string loocv = result.LeaveOneOut is null
                    ? "—"
                    : result.LeaveOneOut.MeanErrorMm.ToString("F6", CultureInfo.InvariantCulture);

                builder.AppendLine(string.Format(
                    CultureInfo.InvariantCulture,
                    "{0,-24}  {1,10:F6}   {2,10}   {3,9:E2}   {4}",
                    kind,
                    result.RmseMm,
                    loocv,
                    result.ConditionNumber,
                    result.Model is Homography2D homography && homography.PerspectiveMagnitude < 1e-3
                        ? "透视可忽略，用仿射即可"
                        : result.IsOverFitting ? "疑似过拟合" : "可用"));
            }

            builder.AppendLine();
            builder.AppendLine("判据：训练 RMSE 只说明拟合得好不好；留一 RMSE 才说明对没见过的点准不准。");
            builder.AppendLine("      参数更多的模型训练 RMSE 一定不会更差，所以必须以留一误差为准。");

            ModelComparisonReport = builder.ToString();
            Logger.Success($"模型对比完成，共比较 {results.Count} 种模型");
            await Task.CompletedTask;
        }
        catch (Exception ex)
        {
            Logger.Error($"模型对比失败：{ex.Message}");
        }
    }

    private void SaveCalibration()
    {
        if (_calibration is null)
        {
            return;
        }

        Microsoft.Win32.SaveFileDialog dialog = new()
        {
            Title = "保存标定文件",
            Filter = "标定文件 (*.csv)|*.csv|所有文件 (*.*)|*.*",
            FileName = $"calibration-{DateTime.Now:yyyyMMdd-HHmmss}.csv",
        };

        if (dialog.ShowDialog() != true)
        {
            return;
        }

        CalibrationFile.Save(_calibration, dialog.FileName);
        Logger.Success($"标定文件已保存：{dialog.FileName}");
    }

    private void LoadCalibration()
    {
        Microsoft.Win32.OpenFileDialog dialog = new()
        {
            Title = "载入标定文件",
            Filter = "标定文件 (*.csv)|*.csv|所有文件 (*.*)|*.*",
        };

        if (dialog.ShowDialog() != true)
        {
            return;
        }

        try
        {
            CalibrationPointSet pointSet = CalibrationFile.ParsePointSet(File.ReadAllText(dialog.FileName), out _);

            _taughtPoints.Clear();
            _taughtPoints.AddRange(pointSet.Points);

            CalibrationRows.Clear();
            foreach (CalibrationPoint point in pointSet.Points)
            {
                CalibrationRows.Add(new CalibrationPointRowViewModel(point));
            }

            CalibrationResult result = CalibrationFile.Load(dialog.FileName);
            Calibration = result;
            CalibrationReport = result.BuildReport();

            double outlierThreshold = Math.Max(result.RmseMm * 3d, result.Options.ExcellentRmseMm);
            for (int i = 0; i < CalibrationRows.Count && i < result.Residuals.Count; i++)
            {
                CalibrationRows[i].Apply(result.Residuals[i], outlierThreshold);
            }

            Logger.Success($"已载入标定文件：{dialog.FileName}（{pointSet.Count} 个点，RMSE {result.RmseMm:F6} mm）");
            RaiseCommandStates();
        }
        catch (Exception ex)
        {
            Logger.Error($"载入标定文件失败：{ex.Message}");
        }
    }

    // ───────────────────────── 视觉对位 ─────────────────────────

    private async Task CaptureAsync()
    {
        try
        {
            CameraFrame frame = await _camera.CaptureAsync();
            CameraImage = ImageInterop.ToBitmapSource(frame);

            VisionMark? mark = await _locator.FindMarkAsync(frame, MarkTemplate.Default);

            CameraInfo = mark is null
                ? $"第 {frame.Index} 帧：未定位到 Mark"
                : $"第 {frame.Index} 帧：Mark 像素 {mark.Pixel}（置信度 {mark.Score:P1}，半径 {mark.RadiusPx:F1} px，面积 {mark.PixelCount} px）";

            // 图像中心画十字标记，便于人眼判断 Mark 有没有居中
            Logger.Debug(CameraInfo);
        }
        catch (Exception ex)
        {
            Logger.Error($"拍照失败：{ex.Message}");
        }
    }

    private async Task SetReferenceAsync()
    {
        try
        {
            AlignmentService service = CreateAlignmentService();
            MarkMeasurement? measurement = await service.MeasureAsync();

            if (measurement is null)
            {
                Logger.Error("未定位到 Mark，无法建立对位基准");
                return;
            }

            _referencePixel = measurement.Mark.Pixel;
            ReferenceText = $"参考像素 {_referencePixel}（置信度 {measurement.Mark.Score:P1}）";
            Logger.Success($"对位基准已建立：像素 {_referencePixel}");
        }
        catch (Exception ex)
        {
            Logger.Error($"建立对位基准失败：{ex.Message}");
        }
    }

    private async Task DisturbAsync()
    {
        try
        {
            Point2D target = SimulatedMarkPosition.Offset(DisturbanceX, DisturbanceY);
            await _controller.MoveAbsoluteAsync(target, DemoProfile);
            Logger.Warn($"已故意把平台挪开 ({DisturbanceX:F3}, {DisturbanceY:F3}) mm → {target}，模拟来料位置偏差");
            await CaptureAsync();
        }
        catch (Exception ex)
        {
            Logger.Error($"偏移失败：{ex.Message}");
        }
    }

    private async Task AlignAsync()
    {
        if (_calibration is null)
        {
            Logger.Warn("尚未完成标定，无法执行对位");
            return;
        }

        try
        {
            AlignmentService service = CreateAlignmentService();

            Point2D referencePixel = _referencePixel
                ?? new Point2D(_camera.Width / 2d, _camera.Height / 2d);

            Logger.Info($"开始自动对位，目标像素 {referencePixel}，容差 {AlignmentToleranceMm:F6} mm");

            Progress<string> progress = new(line => Logger.Info(line));
            AlignmentResult result = await service.AlignToPixelAsync(referencePixel, progress);

            AlignmentTrace = result.BuildTrace();

            if (result.Success)
            {
                Logger.Success($"对位成功：{result.Iterations} 次迭代，残差 {result.FinalErrorMm:F6} mm，收敛比 {result.ConvergenceRatio:F1}");
            }
            else
            {
                Logger.Error($"对位失败：{result.Message}");
            }

            await CaptureAsync();

            MotionPose pose = await _controller.GetPositionAsync();
            Logger.Info($"对位后平台位置：X = {pose.X:F4} mm，Y = {pose.Y:F4} mm");
        }
        catch (Exception ex)
        {
            Logger.Error($"对位异常：{ex.Message}");
        }
    }

    private AlignmentService CreateAlignmentService()
    {
        if (_calibration is null)
        {
            throw new InvalidOperationException("尚未完成标定");
        }

        return new AlignmentService(
            _controller,
            _camera,
            _locator,
            _calibration,
            new AlignmentOptions
            {
                ToleranceMm = AlignmentToleranceMm,
                MaxIterations = AlignmentMaxIterations,
                MaxInitialErrorMm = AlignmentMaxInitialErrorMm,
                SettleDelayMs = 60,
            },
            MarkTemplate.Default,
            MoveProfile.Fine);
    }

    // ───────────────────────── 事件处理 ─────────────────────────

    private void OnCardStateChanged(object? sender, CardStateChangedEventArgs e)
    {
        Application.Current?.Dispatcher.BeginInvoke(
            () =>
            {
                CardState = e.NewState;
                StatusText = string.IsNullOrEmpty(e.Reason)
                    ? e.NewState.ToString()
                    : $"{e.NewState}：{e.Reason}";

                Logger.Info($"状态机：{e.OldState} → {e.NewState}（{e.Reason}）");
            });
    }

    private void OnStatusUpdated(object? sender, AxisStatus status)
    {
        // 注意：状态轮询是在后台线程触发的，必须切回 UI 线程再更新绑定
        Application.Current?.Dispatcher.BeginInvoke(
            () =>
            {
                AxisRowViewModel? row = Axes.FirstOrDefault(item => item.Axis == status.Axis);
                row?.Update(status);
            });
    }

    private void OnAlarmRaised(object? sender, MotionAlarm alarm)
    {
        Application.Current?.Dispatcher.BeginInvoke(
            () =>
            {
                LastAlarmText = $"[{alarm.Severity}] {alarm.Code} {alarm.Message}";
                Logger.Log(alarm.Severity switch
                {
                    AlarmSeverity.Info => LogLevel.Info,
                    AlarmSeverity.Warning => LogLevel.Warning,
                    _ => LogLevel.Error,
                }, $"报警 {alarm.Code}：{alarm.Message}" + (alarm.Advice is null ? string.Empty : $"｜建议：{alarm.Advice}"));
            });
    }

    /// <summary>所有受状态机影响的命令。状态一变就统一重新求值，避免出现"按钮该灰没灰"。</summary>
    private IEnumerable<IAppCommand> AllCommands =>
    [
        InitializeCommand, ShutdownCommand, HomeAllCommand, HomeSelectedCommand, MoveAbsoluteCommand,
        EnableCommand, DisableCommand, StopCommand, EmergencyStopCommand, ResetAlarmCommand,
        JogPositiveCommand, JogNegativeCommand, StopJogCommand, InjectAlarmCommand,
        TeachCommand, CalibrateCommand, CompareModelsCommand, SaveCalibrationCommand,
        DisturbCommand, AlignCommand, CaptureCommand, SetReferenceCommand, AutoDemoCommand,
    ];

    private void RaiseCommandStates()
    {
        foreach (IAppCommand command in AllCommands)
        {
            command.RaiseCanExecuteChanged();
        }
    }

    public async ValueTask DisposeAsync()
    {
        _controller.StatusUpdated -= OnStatusUpdated;
        _controller.AlarmRaised -= OnAlarmRaised;
        _controller.StateChanged -= OnCardStateChanged;

        _camera.Dispose();
        await _controller.DisposeAsync();
    }
}
