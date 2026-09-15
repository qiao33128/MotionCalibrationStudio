# MotionCore.Vendors —— 厂商卡适配层

这里放 `IVendorCardApi` 的三家实现，**每个文件都写清了真实的 SDK 入口、错误码约定和常用函数**，上机时按下面两步就能接通。

## 一、把厂商 DLL 放到输出目录

工程里已经预留了 `x64/` 拷贝目标，把 SDK 的运行时 DLL 放进去即可（它们不会随代码提交）：

| 厂商 | 需要的文件 |
|---|---|
| 正运动 ZMC | `zauxdll.dll`、`zmotion.dll` |
| 固高 GTS | `gts.dll`、`gts800.dll`（视卡型） |
| 雷赛 DMC | `ltdmc.dll` |

## 二、替换仿真卡

```csharp
// 仿真（默认，无需硬件）
var api = new SimulatedCardApi(new SimulatedCardOptions { ... });

// 上机：换成对应适配器即可，上层一行都不用改
var api = new ZmotionCardApi();
var api = new GoogolCardApi();
var api = new LeadshineCardApi();

IMGmotionController controller = new MyMachineController(api, options);
```

## 三、各家的差异点（适配器里已经处理）

| 项目 | 正运动 ZMC | 固高 GTS | 雷赛 DMC |
|---|---|---|---|
| 轴号起点 | 0 起，支持虚拟轴 | 0 起 | 0 起 |
| 使能 | `ZAux_Direct_SetAxisEnable` | `GT_AxisOn` | `dmc_set_axis_enable` |
| 回零 | `ZAux_Direct_Home`（配置项 `DATUM_IN` 等） | `GT_GoHome`（`GT_SetHomeMode`） | `dmc_set_home_profile` |
| 绝对定位 | `ZAux_Direct_MoveAbs` | `GT_AxisOn` + `GT_SetPos` + `GT_Update` | `dmc_set_position` + `dmc_start_move` |
| 状态 | `ZAux_Direct_GetAxisStatus` / `GetDpos` / `GetMpos` | `GT_GetSts` / `GT_GetAxisPrfPos` | `dmc_get_axis_status` |
| 错误码 | 正数，`ZAux_GetMaxErrCode` 取最新 | 0 = 成功，非 0 为 `MC_xxx` 宏 | 0 = 成功，负数为错误 |
| 模拟量 | `ZAux_Direct_SetAout` | `GT_SetDac` | `dmc_set_dac` |

> ⚠️ 这里的函数名以各家公开文档为准，实际项目请对照**对应版本的 SDK 头文件**核对签名与参数顺序 —— 厂商在不同大版本之间改过参数（尤其是回零和插补相关）。适配器里每个方法都做了错误码 → 统一异常/报警的翻译。
