# 各型号能力说明

本文面向 C++ SDK `0.3.0`，按手型说明控制、遥测、配置和专有功能。
先用[能力总览](#能力总览)对照型号，再查看对应章节：
[L6](#l6)、[O6](#o6)、[L20 Lite](#l20-lite)、[L20](#l20)、[L30](#l30)、[O30](#o30)。

连接、命令完成判断、轮询和恢复的通用写法见 [API 使用指南](usage.md)；
操作系统、适配器和实机验证范围见 [平台与硬件验证范围](COMPATIBILITY.md)。

## 如何阅读能力表

- **统一接口**指 `HandDevice`；**型号接口**指 `l6::L6`、`o30::O30` 等对象的 Manager。
  统一接口未提供的操作，可能通过型号接口提供；Manager 中有方法声明也不代表当前设备支持。
- **查询**会请求设备数据；**写回复缓存**来自设备最近一次完整写回复；
  **目标缓存**只记录 SDK 最近发送的目标。后两者不能当成独立配置读回或实时测量。
- 除明确标注外，范围与单位均为协议原始值。统一数值接口使用原始计数，
  `set_percent_angles()` 单独使用 `0..100`，不应将原始速度、电流或扭矩直接当作 RPM、mA 或 N·m。
- 表中的支持状态描述 SDK 已实现的入口。实际可用性还取决于协议、固件、传感器和连接后能力；
  实机通过范围单独列在[验证记录](#验证记录)，不由接口存在推导。
- `—` 表示当前统一接口不支持。所有总览表中的 L6 均指**默认 Classic CAN**；
  CAN FD 是[显式选择的可选协议](#l6-可选-can-fd-协议)，不能套用 Classic 的全部功能。

## 能力总览

### 控制指令与参数设置

本表仅列 `HandDevice`。数值写入和离散操作均返回 `CommandHandle`，
具体终态与 ACK 要求见 [命令完成与错误处理](usage.md#命令完成与错误处理)。

| 功能 | L6 | O6 | L20 Lite | L20 | L30 | O30 |
| --- | --- | --- | --- | --- | --- | --- |
| 原始角度 / 百分比角度设置 | 支持 | 支持 | 支持 | 支持 | 支持 | 支持 |
| 速度设置 | 支持 | 支持 | 支持 | 支持 | 支持 | 支持 |
| 加速度设置 | — | 支持 | 支持 | — | — | 支持 |
| 电流上限设置 | 支持 | 支持 | 支持 | 支持 | 支持 | 支持 |
| 堵转时间 / 阈值 / 保持电流设置 | 支持 | 支持 | 支持 | — | — | 支持 |
| 清故障 | 支持 | — | — | 支持 | — | — |
| 使能 / 失能 | — | — | — | — | 支持 | — |
| 保存参数 | 支持 | 支持 | — | — | — | — |

L20 Lite 的保存配置、O30 的 Flash 保存等入口由型号 Manager 提供，见各型号的专有功能。
L6 的三项增益仅通过专有 Manager 读写；O30 的 `pid` Manager 是另一套型号专有入口，不能直接互换。

### 状态与配置读取

本表说明可获取哪些设备信息及其读取方式。状态包括实际角度、速度、电流、温度和故障；
配置包括设定的速度、加速度和电流上限等参数。能设置参数不代表能读回配置，例如 L20
支持设置电流上限，但不能读取限流配置。表中分别标明设备查询、周期上报、写回复缓存和
SDK 目标缓存，便于区分实际反馈与设定值。

| 功能 | L6 | O6 | L20 Lite | L20 | L30 | O30 |
| --- | --- | --- | --- | --- | --- | --- |
| 身份 / 能力 | 支持 | 支持 | 支持 | 支持 | 支持 | 支持 |
| 角度反馈 | 查询 | 查询 | 查询 | 查询 | 查询 / 上报 | 查询 |
| 实时速度 `get_speeds()` | — | — | 查询 | 查询 | 查询 / 上报 | 查询 |
| 配置速度 `get_configured_speeds()` | 写回复缓存 | 写回复缓存 | 写回复缓存 | — | 目标缓存 | 查询 |
| 配置加速度 | — | 查询 | 写回复缓存 | — | — | 查询 |
| 实时电流 | 查询 | 查询 | 查询 | 查询 | 查询 / 上报 | 查询 |
| 电流上限配置 | 查询 | 写回复缓存 | 写回复缓存 | — | 目标缓存 | 查询 |
| 扭矩 | 查询输出 / 模式 | — | — | — | — | 查询实际值 |
| 温度 | 查询 | 查询 | 查询 | 查询 | 查询 / 上报 | 查询 |
| 电压 | — | — | — | — | — | 查询 |
| 故障 | 查询 | 查询 | 查询 | 查询 | 查询 / 上报 | 查询 |
| 五指触觉 | 固定矩阵 | 固定矩阵 | 固定矩阵及固件相关标量 | 固定矩阵 | 固定矩阵 | 按传感器描述解码 |
| 堵转时间 / 阈值 / 保持电流 | 查询 | 查询 | 写回复缓存 | — | — | 查询 |

所有型号均提供主机轮询、快照、事件流和恢复接口，但只适用于各自支持的通道。
快照与事件流不等于设备主动上报；L30 的设备周期上报需单独配置。
缓存读取前检查 `meta.valid`、`meta.stale`、`meta.origin` 和连接代次，首次写入前可能没有配置缓存，
恢复连接后需重新建立。API 名称中的 `get_*` 不保证会发送查询。

## L6

### 默认 Classic CAN

6 关节，统一顺序为 `thumb_flex, thumb_abd, index, middle, ring, pinky`。
默认地址为左手 `0x28`、右手 `0x27`；型号入口为 `l6::L6`。

| 功能 | 读取（HandDevice） | 设置（HandDevice） | 原始范围与语义 |
| --- | --- | --- | --- |
| 角度 | `get_angles()` | `set_angles()` / `set_percent_angles()` | 原始 `0..255`；百分比 `0..100` |
| 配置速度 | `get_configured_speeds()`，写回复缓存 | `set_speeds()` | `0..255`，步进模式速度设置，不是实测 RPM |
| 实时电流 | `get_currents()` | — | `0..255`；协议换算为 `1400/255 mA` 每计数，绝对值 |
| 电流上限 | `get_current_limits()`，查询 | `set_current_limits()` | `0..255`；协议换算同上 |
| 扭矩输出 / 模式 | `get_torques()` | — | `0..255`；`0` 选择位置模式，`1..255` 为扭矩模式输出，物理扭矩未确认 |
| 堵转时间 | `get_stall_detection_times()` | `set_stall_detection_times()` | 写入 `1..255`，每计数 10 ms；读取保留设备原始值 |
| 堵转阈值 / 保持电流 | `get_stall_thresholds()` / `get_stall_hold_currents()` | 对应 `set_stall_*()` | `0..255`；保持电流沿用协议“堵转力矩”字段，物理比例未确认 |
| 温度 / 故障 | `get_temperatures()` / `get_faults()` | `clear_faults()` | 温度为 `1°C` 每计数；故障为逐关节位掩码 |
| 五指触觉 | `get_force()` | — | `fn` 为 `5 × 12 × 6` 原始矩阵，不能直接当作牛顿值 |
| 参数保存 | — | `save_parameters()` | 持久化操作，与普通参数写入区分 |

型号接口通过 `angle`、`speed`、`current`、`current_limit`、`torque`、`temperature`、
`fault`、`force_sensor`、`stall`、`version`、`config` 等 Manager 提供对应操作；
三项增益仅通过 `position_kp`、`position_kd`、`torque_kd` Manager 的
`set_values()` / `get_blocking()` / `get_snapshot()` 读写，不进入 `HandDevice` 能力表、快照或事件流。

| 专有 Manager | 写入原始范围 | 语义 |
| --- | --- | --- |
| `position_kp` | `0..200` | 协议说明为实际 Kp = raw × 10，增益解释仍有冲突记录 |
| `position_kd` | `0..255` | 建议值不等于 SDK 硬限制 |
| `torque_kd` | `0..255` | 物理比例未确认 |

写入均使用六关节 `std::array<uint8_t, 6>`，读取保留设备返回的原始值，包括超出文档写入范围的值。
统一接口的旧增益读写方法、样本与字段已移除；迁移方式见[使用指南](usage.md#通过-manager-使用专有功能)。
扭矩目标设置仅通过 `torque.set_torques()` 提供；限位补偿读写仅通过
`limit_compensation.set_values()` / `get_blocking()` 提供，不进入统一快照或事件流。

专有配置包括 `config.set_id()`、`set_transmit_id()`、`set_receive_id()`、
`set_reply_enabled()`、`restore_factory()` 和 `save()`。修改地址后需按新地址重新连接；
关闭回复会影响需要回复的读取或命令。

限制与使用要点：

- 当前统一接口不提供 L6 的实时速度；读取配置速度使用 `get_configured_speeds()`。
- 原生 `stall.set_times()` 使用毫秒，统一接口使用 10 ms 原始计数，不能直接传同一组数值。
- 不支持统一加速度、扭矩目标设置、限位补偿、电压、使能和失能。
- 调整增益、扭矩模式、补偿和持久参数前确认设备状态；常用功能实机通过不代表这些高级操作已验收。

### L6 可选 CAN FD 协议

仅在设备固件支持且需要 CAN FD 时设置 `config.l6_protocol = L6Protocol::kHopV004`。
默认值仍为 `kClassic`，SDK 不发送设备协议切换命令。型号入口为 `l6::HopL6`。

| 项目 | HOP CAN FD 与默认 Classic 的差异 |
| --- | --- |
| 连接 | CAN FD 标准帧、无 BRS；请求 ID 默认 `0x001`，回复 ID 为 `request_id \| 0x400`；不参与当前自动发现 |
| 角度 | 原始 `0..65535`，百分比线性映射到该范围；不支持 Classic 的角度映射文件 |
| 速度 / 限流 | 支持设置及主动查询配置，`0..255` 原始计数 |
| 电流 / 温度 | 支持查询原始计数，物理换算未确认 |
| 故障 | 支持读取，使用独立的 `l6::HopFaultCode`，不能套用 Classic 故障枚举 |
| 触觉 | 按传感器 1..5 的元数据解码；字段和形状需在读取后确认，不读取手掌传感器 |
| 参数保存 | 支持 `save_parameters()`，型号接口为 `config.save()` |
| 不支持项 | 加速度、实时速度、扭矩目标、增益、限位补偿、堵转配置、电压、使能 / 失能和清故障 |
| 验证边界 | 已有离线协议向量和 FakeTransport 验证，尚未完成 HOP 实机验收 |

完整连接示例、ACK 语义和待实机确认项见 [L6 可选 CAN FD 接入](usage.md#l6-可选-can-fd-协议)。

## O6

6 关节，顺序与 L6 的六关节顺序一致；Classic CAN，默认左手 `0x28`、右手 `0x27`。
型号入口为 `o6::O6`。

| 功能 | 读取（HandDevice） | 设置（HandDevice） | 原始范围与语义 |
| --- | --- | --- | --- |
| 角度 | `get_angles()` | `set_angles()` / `set_percent_angles()` | 原始 `0..255`；百分比 `0..100` |
| 配置速度 | `get_configured_speeds()`，写回复缓存 | `set_speeds()` | `0..255`；物理比例存在文档冲突 |
| 配置加速度 | `get_configured_accelerations()`，查询 | `set_accelerations()` | `0..254`；`0` 为特殊最大加速度编码，物理比例有冲突 |
| 实时电流 | `get_currents()` | — | `0..255`，物理比例有冲突 |
| 电流上限 | `get_current_limits()`，写回复缓存 | `set_current_limits()` | `0..255`，不是电流实测值 |
| 堵转时间 | `get_stall_detection_times()`，查询 | `set_stall_detection_times()` | `0..255`，每计数 10 ms |
| 堵转阈值 | `get_stall_thresholds()`，查询 | `set_stall_thresholds()` | `0..255`；协议说明为 `1000/255 mA` 每计数 |
| 堵转保持电流 | `get_stall_hold_currents()`，查询 | `set_stall_hold_currents()` | `0..255`，协议旧名称与物理含义有冲突 |
| 温度 / 故障 | `get_temperatures()` / `get_faults()` | — | 温度为 `1°C` 每计数；故障为位掩码 |
| 五指触觉 | `get_force()` | — | `fn` 为 `5 × 10 × 4` 原始矩阵 |
| 参数保存 | — | `save_parameters()` | 型号接口为 `config.save()` |

型号接口包含 `angle`、`speed`、`acceleration`、`current`、`current_limit`、`temperature`、
`fault`、`force_sensor`、`stall`、`version` 和 `config`。
专有配置包括 `config.set_id()`、`set_device_id()`、`set_transmit_id()`、`restore_factory()` 和 `save()`。

限制与使用要点：

- **固件 V1.3.2 不应调用原生 `speed.get_blocking()` 查询速度。** 该请求可能将设备速度改成 0。
  统一 `get_configured_speeds()` 只读写回复缓存，首次写入前可能不可用，恢复连接后缓存失效。
- 不支持统一实时速度、扭矩、增益、限位补偿、电压、清故障、使能 / 失能。
- 速度、限流和实时电流没有可靠统一的物理换算结论，保留原始值和单位置信度。

## L20 Lite

10 关节，Classic CAN，默认左手 `0x28`、右手 `0x27`；型号入口为 `l20lite::L20lite`。
向量按 SDK 的关节顺序填写，不能因型号名称包含“20”而传入 20 个值。

| 功能 | 读取（HandDevice） | 设置（HandDevice） | 原始范围与语义 |
| --- | --- | --- | --- |
| 角度 | `get_angles()` | `set_angles()` / `set_percent_angles()` | 原始 `0..255`；百分比 `0..100` |
| 实时速度 | `get_speeds()`，查询 | — | `0..255`，独立于配置缓存 |
| 配置速度 | `get_configured_speeds()`，写回复缓存 | `set_speeds()` | `0..255` |
| 配置加速度 | `get_configured_accelerations()`，写回复缓存 | `set_accelerations()` | `0..254` |
| 实时电流 | `get_currents()`，查询 | — | `0..255`，不能当作限流设定 |
| 电流上限 | `get_current_limits()`，写回复缓存 | `set_current_limits()` | `0..255` |
| 堵转时间 | `get_stall_detection_times()`，写回复缓存 | `set_stall_detection_times()` | `0..255`，每计数 10 ms |
| 堵转阈值 / 保持电流 | `get_stall_thresholds()` / `get_stall_hold_currents()`，写回复缓存 | 对应 `set_stall_*()` | `0..255`，物理换算未确认 |
| 温度 / 故障 | `get_temperatures()` / `get_faults()` | — | 温度为 `1°C` 每计数；故障为位掩码 |
| 触觉矩阵 | `get_force({"fn"})` | — | `5 × 12 × 6` 原始矩阵 |
| 触觉标量 | `get_force({"fns", "fts", "fds", "px"})` | — | 固件相关；`fds=255` 表示方向未定义 |
| 法向合力 | `get_normal_force_resultant()` | — | 需要可用的五指 `fns`，不是对矩阵任意求和 |

型号接口包含 `angle`、`speed`、`acceleration`、`current`、`current_limit`、`temperature`、
`fault`、`force_sensor`、`tip_force`、`stall`、`version` 和 `config`。
`tip_force.get_blocking()` / `get_finger_blocking()` 用于标量通道；同样受固件实现限制。
专有配置包括 `config.set_id()`、`set_device_id()`、`set_transmit_id()`、`set_reply_enabled()`、
`restore_factory()` 和 `save()`；统一 `save_parameters()` 当前不支持该型号。

限制与使用要点：

- **已验证固件 V4.6.10 不回复 `0x20..0x23` 标量查询。** 打开设备后，统一能力会移除
  `fns/fts/fds/px`，保留 `fn` 矩阵，并拒绝这些标量及法向合力调用。不要用打开前的声明能力代替运行能力。
- 实时速度查询不填充或覆盖配置速度缓存；速度、加速度、限流、堵转参数 getter
  读取最近完整写回复，恢复连接后需重新建立。
- 不支持统一扭矩、增益、限位补偿、电压、清故障、使能 / 失能。
- 速度、加速度和限流保留原始计数，物理换算未确认。
- 实机报告保留关节 7/8 的到位与联动失败；单关节接入检查通过不表示全关节运动精度通过。

## L20

16 关节，Classic CAN，默认左手 `0x28`、右手 `0x27`；型号入口为 `l20::L20`。

| 功能 | 读取（HandDevice） | 设置（HandDevice） | 原始范围与语义 |
| --- | --- | --- | --- |
| 角度 | `get_angles()` | `set_angles()` / `set_percent_angles()` | 原始 `0..255`；百分比 `0..100` |
| 实时速度 | `get_speeds()`，查询 | — | `0..255` |
| 配置速度 | 不支持配置读回 | `set_speeds()` | `0..255`；不能用实时速度证明配置已应用 |
| 实时电流 | `get_currents()`，查询 | — | `0..255` |
| 电流上限 | 不支持配置读回 | `set_current_limits()` | `0..255`；写入应答不能当作限流设定值 |
| 温度 / 故障 | `get_temperatures()` / `get_faults()` | `clear_faults()` | 温度为 `1°C` 每计数；故障为位掩码 |
| 五指触觉 | `get_force()` | — | `fn` 为 `5 × 12 × 6` 原始矩阵 |

型号接口包含 `angle`、`speed`、`current`、`current_limit`、`temperature`、`fault`、
`force_sensor` 和 `version`。`current_limit` 与实时 `current` 共用相关实现，
不意味着存在可用的限流配置 getter。

限制与使用要点：

- `get_configured_speeds()` 和 `get_current_limits()` 不支持；SDK 不伪造配置缓存。
- 角度、速度和限流命令不要求独立 ACK，成功发送不等于配置读回或机械到位。
  需要确认位置时单独读取角度反馈。
- 不支持统一加速度、扭矩、增益、限位补偿、堵转配置、电压、参数保存、使能 / 失能。
- 实机报告记录过拇指电机通信异常，原因未确定；不把一次故障清零当作永久排除设备问题。

## L30

17 关节（含腕关节），CAN FD 扩展帧；默认 NodeID=`1`、HostID=`0`。
型号入口为 `l30::L30`，向量按 J1..J17 顺序填写。

| 功能 | 读取（HandDevice） | 设置（HandDevice） | 原始范围与语义 |
| --- | --- | --- | --- |
| 角度 | `get_angles()`，查询或周期上报 | `set_angles()` / `set_percent_angles()` | 各关节范围不同，含负数；百分比 `0..100` |
| 实时速度 | `get_speeds()`，查询或上报 | — | 有符号 16 位计数；协议说明为 `0.732 RPM` 每计数 |
| 配置速度 | `get_configured_speeds()`，目标缓存 | `set_speeds()` | `1..250`，协议说明为 `0.732 RPM` 每计数 |
| 实时电流 | `get_currents()`，查询或上报 | — | 有符号 16 位计数，`6.5 mA` 每计数，符号表示输出方向 |
| 电流上限 | `get_current_limits()`，目标缓存 | `set_current_limits()` | `60..800`，`6.5 mA` 每计数 |
| 温度 / 故障 | `get_temperatures()` / `get_faults()`，查询或上报 | — | 温度为 `1°C` 每计数；故障为位掩码 |
| 五指触觉 | `get_force()`，查询 | — | `fn` 为 `5 × 12 × 6` 原始矩阵，不支持设备周期上报 |
| 使能 / 失能 | — | `enable()` / `disable()` | 运动前使能，结束后失能，并检查命令结果 |

角度范围和方向由各关节能力声明：例如拇指根部为 `0..900`、侧摆轴可为 `-200..200`、
腕关节为 `-1000..1000`。这些是 SDK 指令范围，不保证设备实际达到每个端点。
屈曲关节的百分比方向通常与原始值递增方向相反，应读取 `joints` 中的 `percent_mapping`，
不能把所有轴按同一公式换算。

型号接口包含 `angle`、`speed`、`current`、`current_limit`、`temperature`、`fault`、
`force_sensor`、`version`、`control` 和 `report`。

| 专有功能 | 型号接口 | 条件与限制 |
| --- | --- | --- |
| 设备周期上报 | `report.configure()` / `disable()` / `disable_all()` | 周期 `20..600000 ms`，当前只支持全关节掩码；限流与触觉矩阵不支持上报 |
| 初始角度周期上报 | 构造参数 `auto_start_periodic` | 型号对象可用 `false` 关闭默认上报；与主机轮询分别配置 |
| 底层协议请求 | `Client::send_no_ack()` / `request_ack()` / `read()` | 调用方负责字段、请求和回复语义 |

限制与使用要点：

- 配置速度、限流仅有 SDK 目标缓存，没有独立设备配置读回；不能用缓存证明设备已应用。
- 不支持统一加速度、扭矩、增益、限位补偿、堵转配置、电压、清故障和参数保存。

## O30

20 关节，HOP / CAN FD 标准帧、无 BRS；请求 ID 默认 `0x001`。
型号入口为 `o30::O30`；SDK 的 20 关节向量映射到协议槽位，不能把槽位总数当作向量长度。

| 功能 | 读取（HandDevice） | 设置（HandDevice） | 原始范围与语义 |
| --- | --- | --- | --- |
| 角度 | `get_angles()` | `set_angles()` / `set_percent_angles()` | 原始 `0..255`；百分比 `0..100`，部分轴方向相反 |
| 实时 / 配置速度 | `get_speeds()` / `get_configured_speeds()`，分别查询 | `set_speeds()` | `0..255`，实际值与设定值分开 |
| 配置加速度 | `get_configured_accelerations()`，查询 | `set_accelerations()` | `0..255`，物理比例未确认 |
| 实时电流 / 电流上限 | `get_currents()` / `get_current_limits()`，分别查询 | `set_current_limits()` | `0..255`，MI `0x05` 的实际值与设定值 |
| 实际扭矩 | `get_torques()` | 不支持扭矩目标写入 | `0..255`，不能直接当作 N·m |
| 堵转时间 / 阈值 / 保持电流 | 对应 `get_stall_*()`，查询 | 对应 `set_stall_*()` | `0..255`，物理单位未确认，不套用 Classic 手型的换算 |
| 温度 / 电压 | `get_temperatures()` / `get_voltages()` | 无统一 setter | `0..255` 原始计数，物理单位未确认 |
| 故障 | `get_faults()` | 不支持清故障 | 逐关节位掩码 |
| 触觉 | `get_force()` / 按字段读取 | — | 由运行时传感器描述确定字段、形状、编码和单位 |

统一接口不提供 O30 的增益、限位补偿、使能 / 失能及参数保存。
专有功能通过以下 Manager 提供：

| 功能 | 型号接口 | 条件与限制 |
| --- | --- | --- |
| 配置与通信地址 | `config.get()`、`set_standard_id()`、`set_extended_id()`、`set_control_mode()` | 标准 ID 写成功后 client 跟随新 ID；ACK 丢失时不能假定地址已确定 |
| 指示灯 / 蜂鸣器 | `config.set_led()` / `set_buzzer()` | 按设备固件语义配置 |
| 恢复与 Flash 保存 | `config.restore_config(true)`、`restore_all(true)`、`save_to_flash(true)` | 需显式确认；统一 `save_parameters()` 不替代这些操作 |
| PID 配置 | `pid.get()` / `pid.set()` | 使用 `PidTerm` 选择增益项，原始值不等同于 L6 的增益含义 |
| 预设动作 | `preset.get()`、`set_non_trigger()`、`set_raw()` | 索引 `1..5`；区分非触发字段与完整预设写入 |
| 关节映射 | `mapping.get_map()`、`set_map()`、`get_length_and_enable()`、`set_length_and_enable()` | 协议槽位配置，与 SDK 百分比映射不同；当前固件不回复 `get_map()` |
| 传感器描述与数据 | `sensor.select()`、`get_info()`、`read_channel()`、`read_data()`、`read_all_data()` | 按传感器描述解码，保留元数据和原始数据 |
| 单位与量程 | `unit.query()` / `get_range()` | 查询目标 MI 的解释，不预设所有数值已有固定物理单位 |
| 通信错误历史 | `comm_error.get_latest_index()` / `get_history()` | 错误历史不能直接当作其他事务的即时 ACK |
| 工程服务 | `service.get_status()`、`read_field()`、`write_field()` 及专用 setter | 涉及侧别、传感器类型、接口、口令、UID 等字段，按已确认设备语义使用 |

限制与使用要点：

- 原生 `enable.set_masks()` / `enable.get_blocking()` 和 `fault.set_faults()` 明确抛出不支持异常；
  方法存在不代表可以使能或清故障，也不能把有效关节掩码解释为使能反馈。
- 原生温度、电压 setter 虽然存在，其配置用途尚未确认，不作为常规调参入口推荐。
- 触觉字段取决于传感器描述；声明能力中的 `supported=nullopt` 表示待观测，不能当作 `true`。
  `get_normal_force_resultant()` 需要有效的五指 `fns`，不能保证任意传感器配置都满足。
- 常用接口实机验收不等于所有关节机械到位；报告保留关节 4、8、19 的偏差或不一致结果。
- Flash 保存、恢复出厂、PID 和工程字段写入不在当前常用功能实机验收范围内。

## 运行时确认能力

离线可用 `declared_capabilities(config)` 查看所选型号和协议声明。
设备打开后用 `hand.get_capabilities()` 获取当前连接的能力，必要时结合 `get_verified_capabilities()`。
统一接口在发送前拒绝已知不支持的操作，业务代码仍应先检查能力并处理运行时错误。

下面的完整程序仅查看 O30 声明能力，不连接硬件：

```cpp
#include <iostream>
#include <linkerbot/hand/api/config.hpp>
#include <linkerbot/hand/capabilities.hpp>

int main() {
    using namespace linkerbot::hand;
    HandDeviceConfig config;
    config.model = Model::kO30;
    config.side = HandSide::kRight;
    const auto caps = declared_capabilities(config);

    std::cout << "joints=" << caps.joints.size() << '\n';
    std::cout << "read torque=" << caps.supports(TelemetrySemantic::kTorques) << '\n';
    std::cout << "write current limit=" << caps.supports(CommandSemantic::kCurrentLimits) << '\n';
    if (const auto *speed = caps.telemetry_channel(TelemetrySemantic::kConfiguredSpeeds)) {
        std::cout << "speed can_query=" << speed->can_query << '\n';
    }
    std::cout << "save parameters=" << caps.supports(DeviceOperation::kSaveParameters) << '\n';
    return 0;
}
```

本例 O30 可读实际扭矩、可写电流上限，但不可写扭矩目标，也不支持统一参数保存；
原生 Flash 保存能力需查 O30 的 Manager 表。
构建和链接方式与 [API 使用指南](usage.md)相同。

可检查的能力字段包括：

| 需求 | 能力入口 | 注意事项 |
| --- | --- | --- |
| 能否写某类命令 | `supports(CommandSemantic)` / `command()` | 扭矩目标设置仅通过型号 Manager 提供 |
| 能否读取某类数据 | `supports(TelemetrySemantic)` / `telemetry_channel()` | 检查 `read_mode`、`source`、`can_query`，区分设备查询和缓存 |
| 能否执行离散操作 | `supports(DeviceOperation)` / `operations` | 原生配置保存不自动等于统一 `kSaveParameters` |
| 关节顺序、范围和方向 | `joints` / `command()->range_for()` | 向量按能力给定的顺序；L30 范围按关节区分 |
| 原始值单位 | `raw_unit`、`raw_scale_note`、`unit_confidence` | `kUnknown` / `kConflicting` 不作为已确认物理换算 |
| 触觉字段 | `force_fields()` / `force_field()` | 字段名使用小写 `fn/fns/ft/fts/fd/fds/px`；动态传感器需读取后确认 |

`get_capabilities()` 本身不发送硬件查询，也不是对所有高级功能逐项执行的自检。
身份、固件或传感器不支持某项功能时，不能用离线能力声明强行绕过限制。

## 验证记录

接口表说明当前 SDK 实现；下表说明源码仓库现有实机记录中的设备和主要边界。
完整平台、适配器和测试范围见 [兼容性说明](COMPATIBILITY.md)，
各型号原始报告位于源码仓库的 `docs/hardware-acceptance/<型号>/report.md`。

| 型号 / 协议 | 已记录设备固件 | 主要验证边界 |
| --- | --- | --- |
| L6 Classic | `V2.4.5` | 常用读取、角度、速度 / 限流及恢复有实机记录；高级增益、扭矩输出和持久化不在本轮验收范围 |
| L6 HOP CAN FD（可选） | 尚无实机验收记录 | 离线协议向量和 FakeTransport，不据此宣称设备接入或运动通过 |
| O6 | `V1.3.2` | 常用接口和恢复有实机记录；保留原生速度查询会改变配置的限制 |
| L20 Lite | `V4.6.10` | 常用接口、缓存及恢复有实机记录；标量触觉不支持，关节 7/8 运动问题保留 |
| L20 | `V3.0.6` | 常用接口和恢复有实机记录；速度 / 限流写入只验证发送，无配置读回；偶发故障记录保留 |
| L30 | `V6.1.6` | 常用接口、使能 / 失能及恢复有实机记录；历史场次有 1 项跳过，不能记为实机通过 |
| O30 | `1.1.0` | 常用命令、配置读回和恢复有实机记录；机械到位问题保留，高级配置与持久化未验收 |

Linux 常用功能结果与 Windows 适配接入结果分别记录；单关节 Windows 验证不覆盖全部关节、
所有 Release 实机、长期负载或定位精度。更换固件、传感器或适配器后按实际能力和对应报告重新确认。
