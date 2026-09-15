using System.Diagnostics;
using MotionCore.Abstractions.Geometry;
using MotionCore.Abstractions.Motion;
using MotionCore.Simulation;
using Xunit;

namespace MotionCore.Tests.Motion;

/// <summary>
/// 通用编排层（<see cref="MotionControllerBase"/>）+ 仿真卡的行为测试。
/// <para>
/// 这些用例覆盖的正是设备软件最容易出事故的地方：
/// 状态机、软限位拦截、多轴同步、回零超时、报警传播、急停后的状态约束。
/// </para>
/// </summary>
public class SimulatedMotionControllerTests
{
    private static SimulatedMotionController CreateController(double positioningNoise = 0d)
    {
        return new SimulatedMotionController(
            new SimulatedCardOptions
            {
                PositioningNoise = positioningNoise,
                FollowingLagSeconds = 0.002d,
                ConnectDelayMs = 0,
                Seed = 20260915,
            },
            TestHarness.ControllerOptions());
    }

    [Fact]
    public async Task 初始化后应进入就绪态()
    {
        await using SimulatedMotionController controller = CreateController();

        List<CardState> states = [];
        controller.StateChanged += (_, args) => states.Add(args.NewState);

        await controller.InitializeAsync();

        Assert.Equal(CardState.Ready, controller.State);
        Assert.True(controller.IsReady);
        Assert.Contains(CardState.Initializing, states);
        Assert.Contains(CardState.Ready, states);
    }

    [Fact]
    public async Task 连接失败时应抛出带明确建议的异常()
    {
        await using SimulatedMotionController controller = new(
            new SimulatedCardOptions { FailOnConnect = true, ConnectDelayMs = 0 },
            new MotionControllerOptions { StatusPollIntervalMs = 2 });

        MotionAlarm? captured = null;
        controller.AlarmRaised += (_, alarm) => captured = alarm;

        MotionException exception = await Assert.ThrowsAsync<MotionException>(() => controller.InitializeAsync());

        Assert.Contains("连接控制器失败", exception.Message, StringComparison.Ordinal);
        Assert.Equal(CardState.Alarm, controller.State);
        Assert.NotNull(captured);
        Assert.Equal(AlarmCategory.Communication, captured!.Category);
        Assert.NotNull(captured.Advice); // 报警必须带排查建议
    }

    [Fact]
    public async Task 未初始化就下发运动指令应被状态机拦截()
    {
        await using SimulatedMotionController controller = CreateController();

        await Assert.ThrowsAsync<MotionStateException>(
            () => controller.MoveAbsoluteAsync(AxisId.X, 5d));
    }

    [Fact]
    public async Task 绝对定位应到达目标位置()
    {
        await using SimulatedMotionController controller = CreateController();
        await controller.InitializeAsync();

        await controller.MoveAbsoluteAsync(AxisId.X, 23.5d, TestHarness.FastProfile);

        AxisStatus status = await controller.GetStatusAsync(AxisId.X);
        Assert.Equal(23.5d, status.ActualPosition, 6);
        Assert.True(status.InPosition);
        Assert.Equal(CardState.Ready, controller.State);
    }

    [Fact]
    public async Task 超出软限位应在上位机侧被拦截_且不改变控制器状态()
    {
        await using SimulatedMotionController controller = CreateController();
        await controller.InitializeAsync();

        List<MotionAlarm> alarms = [];
        controller.AlarmRaised += (_, alarm) => alarms.Add(alarm);

        MotionSoftLimitException exception = await Assert.ThrowsAsync<MotionSoftLimitException>(
            () => controller.MoveAbsoluteAsync(AxisId.X, 500d));

        Assert.Contains("软限位", exception.Message, StringComparison.Ordinal);

        // 关键点：一次误操作不能把整台设备打成报警停机
        Assert.Equal(CardState.Ready, controller.State);
        Assert.Single(alarms);
        Assert.Equal(AlarmSeverity.Warning, alarms[0].Severity);
        Assert.Equal(AlarmCategory.Limit, alarms[0].Category);
    }

    [Fact]
    public async Task 回零后应建立用户坐标系()
    {
        await using SimulatedMotionController controller = CreateController();
        await controller.InitializeAsync();

        await controller.MoveAbsoluteAsync(AxisId.X, 30d, TestHarness.FastProfile);
        await controller.HomeAsync(AxisId.X);

        AxisStatus status = await controller.GetStatusAsync(AxisId.X);
        Assert.True(status.Homed);
        Assert.Equal(0d, status.ActualPosition, 6);
    }

    [Fact]
    public async Task 多轴联动过程中_各轴行程完成比例应始终一致()
    {
        await using SimulatedMotionController controller = CreateController();
        await controller.InitializeAsync();

        // 行程比 3 : 1。若不按比例缩放速度，两轴的完成比例会差 3 倍，轨迹就不是直线了。
        Task moveTask = controller.MoveAbsoluteAsync(new MotionPose(12d, 4d), new MoveProfile(20d, 500d, 500d));

        await Task.Delay(200);

        AxisStatus x = await controller.GetStatusAsync(AxisId.X);
        AxisStatus y = await controller.GetStatusAsync(AxisId.Y);

        double completedX = x.ActualPosition / 12d;
        double completedY = y.ActualPosition / 4d;

        Assert.True(x.ActualPosition > 0d && x.ActualPosition < 12d, "取值时刻应处于运动过程中");
        Assert.True(
            Math.Abs(completedX - completedY) < 0.05d,
            $"联动轨迹偏离直线：X 完成 {completedX:P2}，Y 完成 {completedY:P2}");

        await moveTask;

        Assert.Equal(12d, (await controller.GetStatusAsync(AxisId.X)).ActualPosition, 6);
        Assert.Equal(4d, (await controller.GetStatusAsync(AxisId.Y)).ActualPosition, 6);
    }

    [Fact]
    public async Task 仿真卡也应返回与真实卡一致的定位重复性误差()
    {
        await using SimulatedMotionController controller = CreateController(positioningNoise: 0.01d);
        await controller.InitializeAsync();

        // 在正反两个落点之间来回走，每次记录与目标的偏差
        List<double> deviations = [];
        for (int i = 0; i < 20; i++)
        {
            double target = i % 2 == 0 ? 7.5d : -7.5d;
            await controller.MoveAbsoluteAsync(AxisId.Y, target, TestHarness.UltraFastProfile);
            deviations.Add((await controller.GetStatusAsync(AxisId.Y)).ActualPosition - target);
        }

        double meanDeviation = deviations.Average();
        double standardDeviation = Math.Sqrt(deviations.Select(value => (value - meanDeviation) * (value - meanDeviation)).Average());

        Assert.True(Math.Abs(meanDeviation) < 0.05d, $"定位系统性偏差过大：{meanDeviation:F4} mm");
        Assert.True(standardDeviation > 0d, "仿真卡必须带有定位重复性误差，否则标定测试会失真");
        Assert.True(standardDeviation < 0.05d, $"定位误差量级不合理：{standardDeviation:F4} mm");
    }

    [Fact]
    public async Task 注入伺服报警应传播到报警事件并切换状态()
    {
        await using SimulatedMotionController controller = CreateController();
        await controller.InitializeAsync();

        List<MotionAlarm> alarms = [];
        controller.AlarmRaised += (_, alarm) => alarms.Add(alarm);

        controller.Card.InjectAlarm(AxisId.Y, SimulatedErrorCodes.ServoDisabled);

        // 等状态轮询把报警捞上来
        await Task.Delay(80);

        Assert.Equal(CardState.Alarm, controller.State);
        Assert.Contains(alarms, alarm => alarm.Axis == AxisId.Y && alarm.Category == AlarmCategory.Servo);

        // 复位后应恢复就绪，并且伺服被重新使能
        await controller.ResetAlarmAsync();

        Assert.Equal(CardState.Ready, controller.State);
        Assert.True((await controller.GetStatusAsync(AxisId.Y)).ServoEnabled);
    }

    [Fact]
    public async Task 急停后必须重新初始化才能恢复()
    {
        await using SimulatedMotionController controller = CreateController();
        await controller.InitializeAsync();

        List<MotionAlarm> alarms = [];
        controller.AlarmRaised += (_, alarm) => alarms.Add(alarm);

        await controller.StopAsync(emergency: true);

        Assert.Equal(CardState.EmergencyStop, controller.State);
        Assert.Contains(alarms, alarm => alarm.Category == AlarmCategory.Safety && alarm.Severity == AlarmSeverity.Fatal);

        // 急停态下必须拒绝一切运动指令
        await Assert.ThrowsAsync<MotionStateException>(() => controller.MoveAbsoluteAsync(AxisId.X, 1d));
    }

    [Fact]
    public async Task 点动与停止点动应改变并使轴停下()
    {
        await using SimulatedMotionController controller = CreateController();
        await controller.InitializeAsync();

        await controller.StartJogAsync(AxisId.Y, MotionDirection.Positive, new MoveProfile(30d, 1000d, 1000d));

        await Task.Delay(120);
        AxisStatus duringJog = await controller.GetStatusAsync(AxisId.Y);
        Assert.True(duringJog.ActualPosition > 0.5d, $"点动应产生位移，实际 {duringJog.ActualPosition:F4}");

        await controller.StopJogAsync(AxisId.Y);
        await Task.Delay(30);

        AxisStatus afterStop = await controller.GetStatusAsync(AxisId.Y);
        Assert.Equal(CardState.Ready, controller.State);
        Assert.True(afterStop.ActualPosition >= duringJog.ActualPosition);
    }

    [Fact]
    public async Task 数字量IO应可读写()
    {
        await using SimulatedMotionController controller = CreateController();
        await controller.InitializeAsync();

        Assert.True(controller.Io.InputCount > 0);
        Assert.True(controller.Io.ReadInput(0)); // 仿真卡默认置位"气压正常"

        Assert.True(controller.Io.WriteOutput(3, true));
        Assert.True(controller.Io.ReadOutput(3));

        // 批量写再逐个回读，验证 IO 时序是否真的落到了卡上
        controller.Io.WriteOutputs(5, [true, false, true]);
        Assert.True(controller.Io.ReadOutput(5));
        Assert.False(controller.Io.ReadOutput(6));
        Assert.True(controller.Io.ReadOutput(7));

        IReadOnlyList<bool> inputs = controller.Io.ReadInputs(0, 4);
        Assert.Equal(4, inputs.Count);
        Assert.True(inputs[0]);
    }

    [Fact]
    public async Task 状态轮询应在后台触发状态广播()
    {
        await using SimulatedMotionController controller = CreateController();
        await controller.InitializeAsync();

        int notifications = 0;
        controller.StatusUpdated += (_, _) => Interlocked.Increment(ref notifications);

        await Task.Delay(150);

        Assert.True(notifications > 10, $"状态广播次数过少：{notifications}");
    }

    [Fact]
    public async Task 释放后应拒绝所有操作()
    {
        SimulatedMotionController controller = CreateController();
        await controller.InitializeAsync();
        await controller.DisposeAsync();

        await Assert.ThrowsAnyAsync<ObjectDisposedException>(
            () => controller.GetStatusAsync(AxisId.X));
    }

    [Fact]
    public async Task 跟随误差应随速度产生并在停止后归零()
    {
        await using SimulatedMotionController controller = CreateController();
        await controller.InitializeAsync();

        Task moveTask = controller.MoveAbsoluteAsync(AxisId.X, 10d, new MoveProfile(20d, 500d, 500d));

        await Task.Delay(100);
        AxisStatus moving = await controller.GetStatusAsync(AxisId.X);

        await moveTask;
        AxisStatus stopped = await controller.GetStatusAsync(AxisId.X);

        // 运动中指令位置应超前实际位置（跟随误差 > 0）
        Assert.True(moving.FollowingError > 0d, "运动过程中应存在跟随误差");
        Assert.Equal(0d, stopped.FollowingError, 9);
    }
}
