namespace MotionCore.Abstractions.Motion;

/// <summary>报警类别。设备软件的报警一定要分类，否则现场排查只能靠猜。</summary>
public enum AlarmCategory
{
    /// <summary>通讯类（断线、超时、丢包）。</summary>
    Communication,

    /// <summary>伺服 / 驱动类（过流、过载、编码器异常）。</summary>
    Servo,

    /// <summary>限位类（硬限位、软限位、超程）。</summary>
    Limit,

    /// <summary>回零类（找不到原点、回零超时）。</summary>
    Homing,

    /// <summary>超时类（等待到位超时）。</summary>
    Timeout,

    /// <summary>工艺 / 流程类（对位不收敛、精度不达标）。</summary>
    Process,

    /// <summary>安全类（急停、安全门、气压不足）。</summary>
    Safety,
}

/// <summary>报警级别。</summary>
public enum AlarmSeverity
{
    /// <summary>提示，不阻断。</summary>
    Info,

    /// <summary>警告，需要关注但可继续。</summary>
    Warning,

    /// <summary>错误，阻断当前动作。</summary>
    Error,

    /// <summary>致命，必须停机并人工介入。</summary>
    Fatal,
}

/// <summary>一条标准报警。</summary>
public sealed record MotionAlarm
{
    public required AlarmCategory Category { get; init; }

    public required AlarmSeverity Severity { get; init; }

    /// <summary>厂商原始错误码（0 表示非厂商错误）。</summary>
    public int VendorCode { get; init; }

    /// <summary>标准化报警码，格式如 MC-LIMIT-001，便于现场查表。</summary>
    public required string Code { get; init; }

    public required string Message { get; init; }

    public AxisId? Axis { get; init; }

    public string? Advice { get; init; }

    public DateTimeOffset Timestamp { get; init; } = DateTimeOffset.Now;

    public override string ToString() =>
        Axis is null
            ? $"[{Severity}] {Code} {Message}"
            : $"[{Severity}] {Code} 轴{Axis.Value.ToShortName()} {Message}";
}
