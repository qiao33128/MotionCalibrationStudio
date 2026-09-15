namespace MotionCore.Abstractions.Motion;

/// <summary>
/// 运动控制统一异常。厂商 SDK 的错误码在这里被翻译成人能看懂的话，
/// 并且带上标准报警码，现场可以直接查表。
/// </summary>
public class MotionException : Exception
{
    public MotionException(string message)
        : base(message)
    {
    }

    public MotionException(string message, int vendorErrorCode)
        : base(message) => VendorErrorCode = vendorErrorCode;

    public MotionException(string message, Exception innerException)
        : base(message, innerException)
    {
    }

    /// <summary>厂商原始错误码。</summary>
    public int VendorErrorCode { get; }

    /// <summary>标准化报警码。</summary>
    public string AlarmCode { get; init; } = "MC-GENERIC-000";

    public AlarmCategory Category { get; init; } = AlarmCategory.Communication;
}

/// <summary>等待到位 / 回零超时。</summary>
public sealed class MotionTimeoutException : MotionException
{
    public MotionTimeoutException(string message, int vendorErrorCode = 0)
        : base(message, vendorErrorCode)
    {
        AlarmCode = "MC-TIMEOUT-001";
        Category = AlarmCategory.Timeout;
    }
}

/// <summary>软限位拦截（在指令下发前就被上位机挡住）。</summary>
public sealed class MotionSoftLimitException : MotionException
{
    public MotionSoftLimitException(string message)
        : base(message)
    {
        AlarmCode = "MC-LIMIT-001";
        Category = AlarmCategory.Limit;
    }
}

/// <summary>控制器当前状态不允许该操作。</summary>
public sealed class MotionStateException : MotionException
{
    public MotionStateException(string message)
        : base(message) => AlarmCode = "MC-STATE-001";
}

/// <summary>厂商 SDK 动态库未就绪。</summary>
public sealed class VendorSdkNotAvailableException : MotionException
{
    public VendorSdkNotAvailableException(string message)
        : base(message) => AlarmCode = "MC-VENDOR-001";

    public VendorSdkNotAvailableException(string message, Exception innerException)
        : base(message, innerException) => AlarmCode = "MC-VENDOR-001";
}
