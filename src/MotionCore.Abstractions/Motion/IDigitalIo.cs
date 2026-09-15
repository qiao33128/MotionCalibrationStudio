namespace MotionCore.Abstractions.Motion;

/// <summary>
/// 数字量 IO。设备软件里 IO 一定要按“输入只读、输出可写”分开建模，
/// 否则现场很容易出现把输入点当输出写、导致误动作的事故。
/// </summary>
public interface IDigitalIo
{
    int InputCount { get; }

    int OutputCount { get; }

    /// <summary>读输入点（只读）。</summary>
    bool ReadInput(int index);

    /// <summary>读输出点回读（用于校验输出是否真的写进去）。</summary>
    bool ReadOutput(int index);

    /// <summary>写输出点。</summary>
    bool WriteOutput(int index, bool value);

    IReadOnlyList<bool> ReadInputs(int start, int count);

    void WriteOutputs(int start, IReadOnlyList<bool> values);
}
