# API 使用指南

构建和首次连接见 [Linux 快速开始](../../../README.zh-CN.md#快速开始linux)或 [Windows 使用指南](windows.md)。
本文说明两种控制策略、常用与高级功能、命令完成判断、后台轮询和生命周期，以及型号可选协议。
按手型查询完整的功能、读写方式、数值范围和固件限制，见[各型号能力说明](../reference/model-capabilities.md)。

## 两种控制策略

C++ SDK 提供两条调用路径，两者都可以读取和控制灵巧手：

```text
应用程序 → HandDevice → 按型号适配 → 型号 Manager → 通信库 → 设备
应用程序 → L6 / O6 / L30 / O30 等型号对象 → 型号 Manager → 通信库 → 设备
```

`HandDevice` 根据配置选择型号实现，并统一参数、样本、能力检查和命令句柄。
型号对象直接暴露 `angle`、`speed`、`config` 等 Manager，保留型号专有功能和执行方式。
Manager 通常由型号对象创建和管理，业务代码通过 `hand.angle` 等成员调用。

| 对比项 | 策略一：统一 `HandDevice` | 策略二：型号专有 Manager |
| --- | --- | --- |
| 适用场景 | 同一套业务支持多型号、运行时切换型号 | 固定型号、专有配置、协议调试与精细控制 |
| 入口 | 从自动发现的候选创建，或用 `HandDeviceConfig` 手动配置 | 直接创建 `l6::L6`、`l30::L30` 等 |
| 角度控制 | `hand.set_angles()`、`hand.set_percent_angles()` | 如 `hand.angle.set_angles()`、`set_angles_percent()`，具体名称随型号变化 |
| 数据类型 | 统一的 vector、样本与快照结构 | 型号专有结构和固定长度 array |
| 执行结果 | 统一 setter 返回 `CommandHandle` | 可能发送后返回，或同步等待回复；以各方法为准 |
| 生命周期 | 构造后 `open()`，结束时 `close()` | 通常在构造时建立资源，结束时 `stop()` |
| 高级功能 | 暴露已统一的限流、堵转等能力 | 还可访问地址、预设、服务字段等专有功能 |

需要统一业务接口时选择 `HandDevice`；需要专有操作时选择型号对象及其 Manager。
同一设备选择一条控制路径，不要同时创建两个对象竞争其请求与回复。
统一接口接收 factory 或共享 Session，型号类接收 Transport 或共享 Session，见[连接与共享总线](#连接与共享总线)。

## 连接 CAN 总线

Linux 先配置 SocketCAN，再打开设备。接口名和速率必须与实际设备一致：

| 设备 | 关闭接口后执行的配置命令 |
| --- | --- |
| Classic 固件 L6、O6、L20 Lite、L20 | `sudo ip link set can0 type can bitrate 1000000 fd off` |
| L30、O30 | `sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on` |

配置前执行 `sudo ip link set can0 down`，配置后执行 `sudo ip link set can0 up`。
L30 使用扩展 FD，配置 NodeID 和 HostID；O30 使用标准 FD、无 BRS，配置 HOP 请求 ID。
L6 默认使用 Classic CAN；支持 CAN FD 的设备可显式选择[可选 HOP 协议](#l6-可选-can-fd-协议)，
并按设备实际速率配置 FD 接口。仅开启接口的 FD 模式不会改变 SDK 选择的 L6 协议。
Windows 使用[适配器 factory](../connections/windows-adapters.md#3-在-c-程序中连接)或[设备发现接口](../reference/discovery.md)。

## 策略一：HandDevice 统一接口

### 读取与控制

自动连接按 [README 快速开始](../../../README.zh-CN.md#快速开始linux)，由发现结果提供型号、侧别、通道和地址。
部署中已知设备配置时，也可以手动填写 `HandDeviceConfig`，调用 `HandDevice hand(config)`；
下文的统一接口示例使用这种手动连接方式。自动发现和手动配置创建的设备使用同一套控制 API。

先调用 `open()`，SDK 完成身份和基础状态检查后才进入可操作状态。
不同型号的功能和数值范围由 `get_capabilities()` 查询；不支持的操作会抛出 `UnsupportedOperationError`。

| 用途 | 常用入口 |
| --- | --- |
| 身份、能力 | `get_device_info()`、`get_identity()`、`get_capabilities()` |
| 主动查询 | `get_angles()`、`get_currents()`、`get_temperatures()`、`get_faults()`、`get_force()` |
| 角度控制 | `set_angles()` 使用协议原始整数；`set_percent_angles()` 使用 `0..100` |
| 缓存、状态 | `get_snapshot()`、`get_health()` |
| 后台采集 | `start_polling()`、`stop_polling()` |
| 事件流 | `stream_handle()`、`stop_stream()` |

向量长度必须等于设备关节数。`0..100` 表示 SDK 的归一化位置，运动方向取决于型号和角度映射。
传感器样本先检查 `meta.valid` 和 `meta.stale`；原始计数不应直接当作物理单位。
触觉字段由能力表声明，缺失值使用 `std::nullopt`，不能用零补齐。

### 型号差异

- L30 运动前需要使能`enable()`，结束后失能，并检查操作结果。
- Classic L6 和 O6 的统一 `get_configured_speeds()` 读取速度写回复缓存；首次写入前可能不可用。
  两者的实时速度接口 `get_speeds()` 均不支持。
  O6 V1.3.2 固件不要用原生 `speed.get_blocking()` 读取配置速度，该查询可能改变设备速度。
- L20 没有速度与电流限制的配置读回接口；L30 的配置速度缓存表示已发送目标，不证明设备已应用。
- L20 Lite 的配置速度、限流和堵转参数以写回复缓存为准，恢复连接后需要重新建立缓存。
  已验证的 V4.6.10 固件不支持部分触觉标量查询，以连接后的能力表为准。
- Classic L6 堵转时间的统一接口使用 `1..255` 原始计数，每个计数为 10 ms；原生接口使用毫秒，写入均拒绝编码为 0。

### 命令完成与错误处理

从 `0.3.0` 起，统一 `set_*` 在本地验证和队列接纳后返回 `CommandHandle`，发送及 ACK 等待在后台进行。
方法返回不表示设备已执行。原生 manager 和离散操作保持各自的执行契约。

| 情况 | 判断方式 |
| --- | --- |
| 参数、能力、设备状态或队列容量错误 | setter 同步抛异常 |
| 接纳后的通信、ACK 或执行超时错误 | 从 `handle.error()` 获取异常 |
| `wait_for()` 返回 `false` | 本次等待超时；命令未取消，不要自动重发 |
| `wait_for()` 返回 `true` | 命令已终结，继续检查错误与终态 |
| 无 ACK 命令完成 | `kDispatched`，并且不要求 ACK |
| 有 ACK 命令完成 | `kAcknowledged`；不表示机械到位 |
| `kSuperseded`、`kCancelled`、`kFailed` | 不能当作成功 |

角度、速度和电流上限会替换同类尚未发送的目标，其他 setter 使用有界 FIFO。
`command_timeout` 覆盖接纳后的排队、发送与 ACK；`wait_for()` 的等待时长是另一份预算。

下面读取并原样提交 L6 当前的原始角度，演示如何检查命令完成；运行时会发送角度目标。
实际应用中，先确认目标姿态，再替换 `target`：

```cpp
#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <linkerbot/hand/api/device.hpp>
#include <stdexcept>
#include <vector>

using namespace linkerbot::hand;
using namespace std::chrono_literals;

bool wait_for_command(const CommandHandle &command, std::chrono::milliseconds timeout) {
    if (!command.wait_for(timeout)) {
        return false;
    }
    if (const auto error = command.error()) {
        std::rethrow_exception(error);
    }
    return command.stage() == CommandStage::kAcknowledged ||
           (command.stage() == CommandStage::kDispatched && command.ack_expectation() == AckExpectation::kNone);
}

int main() try {
    HandDeviceConfig config;
    config.model = Model::kL6;
    config.side = HandSide::kLeft;
    HandDevice hand(config);
    hand.open();
    const auto angles = hand.get_angles();
    if (!angles.meta.valid) {
        throw std::runtime_error("Invalid angle response");
    }
    std::vector<int32_t> target(angles.raw.begin(), angles.raw.end());
    const auto command = hand.set_angles(target);
    const bool completed = wait_for_command(command, 2s);
    if (!completed) {
        std::cerr << "Command did not complete successfully; do not resend automatically\n";
    }
    hand.close();
    return completed ? 0 : 1;
} catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
}
```

## 策略二：型号专有 Manager

以 Classic 固件 L6 左手为例，直接创建 `l6::L6`，通过 `angle` 和 `version` Manager 读取设备，
再原样提交当前原始角度。这个示例会发送角度目标，结束时调用 `stop()`，不需要另调 `open()`：

```cpp
#include <chrono>
#include <exception>
#include <iostream>
#include <linkerbot/hand/l6/l6.hpp>

int main() try {
    namespace l6 = linkerbot::hand::l6;
    using namespace std::chrono_literals;
    l6::L6 hand(l6::Side::kLeft, "can0");
    const auto info = hand.version.get_verified_device_info(2s);
    const auto angles = hand.angle.get_blocking(500ms);
    std::cout << "angles (%):";
    for (const auto value : angles.angle.to_array()) {
        std::cout << ' ' << value;
    }
    std::cout << '\n';
    hand.angle.set_angles(angles.device_raw);
    hand.stop();
    return 0;
} catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
}
```

这里的 `angle.set_angles()` 返回 `void`，没有统一的 `CommandHandle`；此调用发送目标后返回。
需要等待角度反馈时，L6 提供 `set_angles_blocking()` 和 `set_angles_percent_blocking()`，
前提是设备已开启相应回复。等待反馈也不表示机械已到位。
`get_verified_device_info()` 显式核验 L6 型号与侧别；原生对象不自动套用统一接口的整套 `open()` 基线检查。

### 常用操作对照

下表的 Manager 写法以 Classic L6 为例，其他型号需使用各自头文件声明：

| 操作 | `HandDevice` | L6 Manager |
| --- | --- | --- |
| 主动读角度 | `get_angles()` | `angle.get_blocking()` |
| 写协议原始角度 | `set_angles(vector<int32_t>)` | `angle.set_angles(array<uint8_t, 6>)` |
| 写百分比角度 | `set_percent_angles(vector<float>)` | `angle.set_angles_percent(l6::Angle)` |
| 读取缓存 | `get_snapshot()` | `get_snapshot()` 或 `angle.get_snapshot()` |
| 读身份 | `get_device_info()` | `version.get_device_info()`；核验身份用 `get_verified_device_info()` |
| 后台轮询 | `start_polling(PollingPlan)` | `start_polling(L6PollingPlan)` 或按 `SensorSource` 选择 |
| 事件与状态 | `stream_handle()`、`get_health()` | `stream_handle()`、`health()` |

L6 的 `AngleData::angle` 是百分比结构，`device_raw` 才是设备原始字节。
保持当前原始目标时使用 `device_raw`，避免重复应用角度映射。
原生 Manager 的 `_blocking` 通常表示等待业务回复，但部分配置 getter 只读缓存；
如 L20 Lite 的堵转 getter，不能仅凭方法名判断是否会发送查询。

### 各型号 Manager 入口

| 型号类 | 常用 Manager | 专有或高级入口 |
| --- | --- | --- |
| `l6::L6` | `angle`、`speed`、`current`、`current_limit`、`force_sensor`、`fault`、`version` | `torque`、`stall`、`config`、`position_kp`、`position_kd`、`torque_kd`、`limit_compensation` |
| `o6::O6` | `angle`、`speed`、`current`、`current_limit`、`force_sensor`、`fault`、`version` | `acceleration`、`stall`、`config` |
| `l20lite::L20lite` | `angle`、`speed`、`current`、`current_limit`、`force_sensor`、`fault`、`version` | `acceleration`、`tip_force`、`stall`、`config` |
| `l20::L20` | `angle`、`speed`、`current`、`current_limit`、`force_sensor`、`fault`、`version` | 速度、限流写入与型号快照 |
| `l30::L30` | `angle`、`speed`、`current`、`current_limit`、`temperature`、`force_sensor`、`fault`、`version` | `control`、`report` |
| `o30::O30` | `angle`、`speed`、`acceleration`、`current`、`temperature`、`sensor`、`fault`、`version` | `torque`、`stall`、`pid`、`preset`、`mapping`、`config`、`service`、`unit`、`comm_error`、`voltage` |

Manager 成员存在不代表当前固件支持其所有操作。统一接口检查 `get_capabilities()`，
型号接口检查 `capabilities()` 并结合具体 Manager 的协议说明。

## 高级功能

高级功能既有统一入口，也有仅由 Manager 提供的专有入口。下面的 L6 均指 Classic 固件；
可选 CAN FD 协议对应 `l6::HopL6`，能力范围见 [L6 可选 CAN FD 协议](#l6-可选-can-fd-协议)，
不能直接套用 Classic 的高级功能。

### 已通过 HandDevice 统一的能力

统一入口按设备能力检查支持范围；数值 setter 同样返回 `CommandHandle`。

| 功能 | `HandDevice` 入口 | 支持型号与对应 Manager |
| --- | --- | --- |
| 速度、电流上限 | `set_speeds()`、`set_current_limits()` | 六型号；`speed`、`current_limit`（O30 限流使用 `current`） |
| 加速度 | `set_accelerations()` | O6、L20 Lite、O30：`acceleration` |
| 扭矩读取 | `get_torques()` | L6：`torque`；O30：`torque.get_torques()` |
| 堵转保护 | `set_stall_detection_times()`、`set_stall_thresholds()`、`set_stall_hold_currents()` 及对应 getter | L6、O6、L20 Lite、O30：`stall`；Manager 参数单位随型号变化 |
| 清故障 | `clear_faults()` | L6、L20：`fault.clear_faults()` |
| 使能、失能 | `enable()`、`disable()` | L30：`control.enable()`、`control.disable()` |
| 保存参数 | `save_parameters()` | L6、O6：`config.save()`；L6 HOP 也提供统一保存入口 |

这些入口的 getter 可能主动读取，也可能读取写回复或已发送目标缓存；查看样本的
`meta.origin`、`meta.valid`、`meta.stale` 和[型号差异](#型号差异)。
L6 增益、扭矩输出和限位补偿现已通过统一接口提供。
O30 扭矩限制和运动时间方法须显式选择 `O30Layout::kPythonV25`；原生 Manager 保持挂载 V36 布局。
Classic L6/O6 使用 `save_parameters_confirmed()` 等待持久化 ACK；`save_parameters()` 保留原语义。
L30 `calibrate()` 按 ACK 执行失能、解锁、校准，不自动重新使能。
时序、兼容性和实机范围见 [Python 行为基准](python-behavior-baseline.md)。升级后重新编译全部消费者。

### 通过 Manager 使用专有功能

这些操作不通过 `HandDevice` 的通用 setter 提供，直接调用对应型号对象的 Manager：

| 功能 | 型号与入口 | 使用要点 |
| --- | --- | --- |
| L6 位置与扭矩增益 | `position_kp`、`position_kd`、`torque_kd` 的 `set_values()`、`get_blocking()`、`get_snapshot()` | 六关节 `std::array<uint8_t, 6>`；Kp 写入 `0..200`，两项 Kd 写入 `0..255` |
| L6 扭矩目标设置 | `torque.set_torques()` | 原始 `0..255`；`0` 选择位置模式，`1..255` 为扭矩模式输出 |
| L6 限位补偿 | `limit_compensation.set_values()`、`get_blocking()`、`get_snapshot()` | 原始 `0..255`；物理比例与方向未确认，仅型号接口提供 |
| CAN 地址配置 | L6：`config.set_id()`、`set_transmit_id()`、`set_receive_id()`；O6／L20 Lite：`set_id()`、`set_device_id()`、`set_transmit_id()` | 修改后按设备实际地址重新连接 |
| O30 地址与控制模式 | `config.set_standard_id()`、`set_extended_id()`、`set_control_mode()` | 标准 ID 写成功后 client 跟随新 ID；ACK 丢失时地址状态不确定 |
| 回复开关 | L6、L20 Lite：`config.set_reply_enabled()` | 关闭后，依赖回复的阻塞调用可能超时 |
| 恢复出厂与配置保存 | L6、O6、L20 Lite：`config.restore_factory()`、`save()` | 可能改变地址及设备配置 |
| O30 配置恢复与保存 | `config.restore_config(true)`、`restore_all(true)`、`save_to_flash(true)` | API 要求显式确认参数 |
| O30 PID | `pid.get()`、`pid.set()` | 通过 `PidTerm` 选择具体增益项 |
| O30 预设动作 | `preset.get()`、`set_non_trigger()`、`set_raw()` | 索引为 `1..5`；区分非触发字段和完整原始数据 |
| O30 关节映射 | `mapping.set_map()`、`set_length_and_enable()` | 修改协议槽位映射，不能与统一百分比映射混淆 |
| O30 传感器描述与原始数据 | `sensor.select()`、`get_info()`、`read_data()` | 按传感器描述符解码，保留元数据和原始数据 |
| O30 工程服务 | `service.get_status()`、`read_field()`、`write_field()` 及身份、传感器类型等专用 setter | 使用已确认的字段与固件语义 |
| O30 单位、量程与通信诊断 | `unit.query()`、`get_range()`、`comm_error` | 查询物理量解释和设备错误历史 |
| L30 设备周期上报 | `report.configure()`、`disable()`、`disable_all()` | 由设备周期发送；与主机后台轮询区分，部分通道不支持上报 |
| 底层协议调试 | `l30::Client::send_no_ack()`、`request_ack()`、`read()` | 直接按协议组织请求，调用方处理字段和回复 |

保存配置、恢复出厂和工程字段写入也可能持久改变设备。
这些操作应在明确设备状态和参数含义后调用，不应混入启动时的默认读取流程。

### 高级 Manager 示例：读取并写回 L6 位置增益

下面使用 `position_kp` Manager 读取六个原始增益计数，再写回相同值并等待协议回复。
实际调参时替换 `values`；原始计数的含义与写入范围以对应型号为准，不把它当作通用物理单位：

```cpp
#include <chrono>
#include <exception>
#include <iostream>
#include <linkerbot/hand/l6/l6.hpp>

int main() try {
    namespace l6 = linkerbot::hand::l6;
    using namespace std::chrono_literals;
    l6::L6 hand(l6::Side::kLeft, "can0");
    const auto info = hand.version.get_verified_device_info(2s);
    const auto gains = hand.position_kp.get_blocking(500ms);
    auto values = gains.values;
    hand.position_kp.set_values(values, 500ms);
    const auto updated = hand.position_kp.get_snapshot();
    if (updated) {
        std::cout << "gain replies: " << updated->sequence << '\n';
    }
    hand.stop();
    return 0;
} catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
}
```

这三个 Manager 的 `set_values()` 同步等待完整写回复，不返回 `CommandHandle`。
写回复可能包含修改前的值；`get_snapshot()` 保留实际回复，显式读回使用 `get_blocking()`。
`position_kd`、`torque_kd` 的用法相同；这些参数不进入统一快照或统一事件流。
协议回复仍不等于机械效果已验证。

### 当前限制

- O30 的 `enable.set_masks()`、`enable.get_blocking()` 和 `fault.set_faults()` 会抛出不支持异常；
  Manager 入口不能绕过这些限制。
- O30 当前固件不回复 `mapping.get_map()`；不要把这个读取加入连接时的必需步骤。
- O30 的温度、电压写接口虽然存在，其配置用途尚未确认，不作为常规调参入口推荐。
- 写入参数前确认单位差异：例如 L6 堵转时间的统一接口是 10 ms 原始计数，专有 Manager 使用毫秒。

## 连接与共享总线

通信后端和控制策略可以分别选择。Linux 的字符串便捷构造使用 SocketCAN；
Windows 先建立[适配器 factory](../connections/windows-adapters.md#3-在-c-程序中连接)，再注入 transport 或 Session：

| 场景 | 统一接口 | 以 L6 为例的型号接口 |
| --- | --- | --- |
| 独立适配器连接 | `HandDevice(config, factory)` | `L6(Side::kLeft, factory())` |
| 共用总线与路由 | `HandDevice(config, session)` | `L6(Side::kLeft, session)` |

`factory()` 生成 transport 时会建立适配器资源；`HandDevice` 仅保存 factory，延迟到 `open()` 使用。
用 `transport::Session::create(factory)` 创建共享 Session，不同设备使用不同的地址／路由。
停止一个设备不应关闭其他设备借用的 Session；外部共享对象应覆盖所有设备对象的使用周期。
L30 的原生构造还要填写 NodeID、HostID，并可用 `auto_start_periodic=false` 禁止默认周期上报。

## 后台轮询与快照

`get_angles()` 等主动查询会访问设备；部分配置 getter 按设备能力读取缓存。
`get_snapshot()` 只读取已有缓存，不发送帧。
后台轮询负责更新缓存，`wait_for_snapshot()` 等待缓存版本变化，超时可能返回未变化的快照。
读取缓存不会刷新采样时间，过期状态由通道能力表的 `freshness_ms` 判断。

以下使用统一接口，以 20 ms 周期轮询角度，等待一次缓存更新后停止：

```cpp
#include <chrono>
#include <exception>
#include <iostream>
#include <linkerbot/hand/api/device.hpp>

int main() try {
    using namespace linkerbot::hand;
    using namespace std::chrono_literals;
    HandDeviceConfig config;
    config.model = Model::kL6;
    config.side = HandSide::kLeft;
    HandDevice hand(config);
    hand.open();
    const auto previous = hand.get_snapshot();
    hand.start_polling({{TelemetrySemantic::kAngles, 20ms}});
    const auto snapshot = hand.wait_for_snapshot(previous, 500ms);
    if (snapshot.angles && snapshot.angles->meta.valid && !snapshot.angles->meta.stale) {
        std::cout << "angles: " << snapshot.angles->raw.size() << '\n';
    }
    hand.stop_polling();
    hand.close();
    return 0;
} catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
}
```

型号对象也支持后台采集：例如 L6 用 `L6PollingPlan` 配置各通道周期和角度反馈策略，
调用 `start_polling(plan)`；`angle.get_snapshot()` 返回该 Manager 的缓存。
L30 的 `report.configure()` 控制设备主动上报，不等同于 SDK 发起查询的主机轮询。

两条路径需要事件流时，都可在采集前调用 `stream_handle()`，由消费线程读取队列；结束时调用
`stop_stream()`。队列有容量上限，慢消费者应检查 `drop_count()`。
不要在 SDK 的同一运行时回调中调用 `close()` 或 `recover()`，应交给外部线程。

## 关闭与恢复

统一接口结束时显式 `close()`，型号对象使用 `stop()`。
统一接口检查 `get_health()`，型号对象检查 `health()`；需要时通过各自的 `recover()` 恢复。
统一接口的恢复会重新核验同一设备、建立基础状态并恢复轮询，不重放旧运动目标和配置。
型号对象的恢复按对应实现处理，不自动套用统一接口的身份与状态门禁。
统一接口的 `close()`、`recover()` 可能因 `shutdown_timeout` 超时而抛出异常；调用方应处理失败状态。
公开 hand 异常可按 `HandError` 或 `std::runtime_error` 捕获。

升级 SDK 后重新编译应用，使用同一次构建产生的头文件和库。
高级操作入口见[高级功能](#高级功能)，平台及实机边界见[验证范围](../reference/compatibility.md)。

## 型号与协议选项

### L6 可选 CAN FD 协议

L6 默认使用 Classic CAN。仅当设备固件支持 HOP v0.0.4 且需要使用 CAN FD 时，
才显式选择 `L6Protocol::kHopV004`；`l6::L6` 和未设置 `l6_protocol` 的 `HandDevice`
仍使用 Classic CAN。此选项不参与自动发现，也不发送设备协议切换命令。

HOP 接入基于 `docs/protocol/L6_协议帧格式0.0.4_CANFD.xlsx`，使用 CAN FD 标准帧、
无 BRS、16 位位置。当前仅通过离线协议向量和 FakeTransport 验证，尚未完成实机验收。

#### 显式连接

```cpp
#include <linkerbot/hand/api/device.hpp>

int main() {
    using namespace linkerbot::hand;
    HandDeviceConfig config;
    config.model = Model::kL6;
    config.side = HandSide::kRight;
    config.l6_protocol = L6Protocol::kHopV004;
    config.interface_name = "can0";
    config.address = 0x001;

    // No device I/O: reports CAN FD and the protocol-specific 0..65535 range.
    auto capabilities = declared_capabilities(config);
    HandDevice hand(config);
    hand.open(); // identity, angles, faults, then current limits
    const auto angles = hand.get_angles();
    hand.close();
}
```

SocketCAN 接口需事先配置为 FD，仲裁速率示例为 1 Mbps，实际速率需与设备一致。
SDK 不调整设备或系统的通信参数。Windows 可使用现有 PCAN/MiniCANFD/SLCAN FD
TransportFactory 或共享 Session；适配器必须支持 FD 标准帧。帧本身关闭 BRS。

请求 ID 默认 `0x001`，范围 `0..0x3FE`；回复 ID 为 `request_id | 0x400`。
地址与左右手解耦，同总线的两只手需要配置不同地址。左右手必须显式指定，并与设备身份一致；
广播地址不用于正常连接。

源码仓库提供只读示例，需先启用示例构建（README 快速开始默认关闭示例）：

```sh
cmake -S . -B build -DLINKERBOT_BUILD_EXAMPLES=ON
cmake --build build --target l6_hop_readonly
./build/examples/l6_hop_readonly --help
./build/examples/l6_hop_readonly can0 right 0x001
```

型号专用入口为 `<linkerbot/hand/l6/hop_l6.hpp>` 中的 `l6::HopL6`，可按
`(Side, interface, id)`、`(Side, unique_ptr<CANTransport>, id)`、
`(Side, shared_ptr<Session>, id)` 构造。`angle` 管理器接受 `std::array<uint16_t,6>`；
其他数值管理器使用 `std::array<uint8_t,6>`。管理器提供同步调用，统一 `HandDevice`
的 setter 继续返回异步 `CommandHandle`。

#### 能力和语义

| 项目 | Classic L6（默认） | HOP L6（可选 CAN FD） |
| --- | --- | --- |
| 选择方式 | 默认 / `kClassic` | 显式设置 `kHopV004` |
| 原始角度 | 0..255 | 0..65535，MI 0x20，小端 |
| 百分比角度 | 原有映射 | 0..100 线性映射到 0..65535 |
| 角度映射文件 | 原有 256 项映射 | 不支持，配置时拒绝 |
| 触觉 | 固定旧矩阵 | 按传感器描述动态解码 |
| 速度 / 限流 | 旧协议语义 | u8 原始计数，写入和查询设定值 |
| 电流 / 温度 | 原有换算 | u8 原始计数，物理换算未确认 |
| 故障 | 旧 FaultCode | 堵转、过流、过温、异常、离线，掩码 0xCF |

统一六关节顺序仍是 `thumb_flex, thumb_abd, index, middle, ring, pinky`；
HOP 协议槽位为 `[10,5,11,12,13,14]`。位置写入只覆盖两个已安装区间，不写入其余关节。
两帧均发送且两个完整回复均到达后才确认命令；ACK 表示协议完成，不表示机械已到位。
部分失败不回滚，也不重试运动目标。

使用 `hand.get_capabilities()` 或 `declared_capabilities(config)` 获取协议对应能力；
原有 `declared_capabilities(Model::kL6, side)` 始终表示 Classic L6。
打开前、关闭后和恢复后的查询均保留显式选择。

当前 HOP 实现支持角度、速度设置及查询、限流设置及查询、电流、温度、故障、五指触觉、
身份和参数保存。可对这些遥测启动轮询。启停、清故障、PID、堵转配置和控制模式切换
均在发送前拒绝。`l6::HopFaultCode` 与旧 `FaultCode` 分开；`FaultSample`
保留完整 raw、已定义位和保留位。

触觉按传感器 1..5 逐个选择、读取元数据和全部数据，整轮成功才发布快照或事件。
字段支持 U8/U16/U32、I8/I16/I32、F32，小端解码；`FNS/FTS/FDS` 归一为
`FnS/FtS/FdS`，`Rsv` 保留原始字节但不生成力值。缺失传感器标记 unavailable；
全部缺失返回不支持。描述不匹配、选择变化、无效浮点数或有效性掩码模式均拒绝发布。
当前不读取手掌传感器。

`stop()` 关闭自己的运行时；借用的共享 Session 保持可用。
统一接口 `recover()` 重新验证同一物理设备、读取基线并恢复轮询，不重放目标。
事务超时后保留最后完整业务样本，相关回复域隔离至显式恢复。

#### 待实机确认的解释

- 默认关闭 BRS；仲裁速率 1 Mbps，实际接入需与设备配置一致。
- 百分比按原始数值递增；HOP 固件的物理方向和端点没有标记为实测。
- 写入回复需匹配 MI、RTS、SI 和完整 EDL；实时状态回复无需等于写入目标。
- 三个触觉数据区分别按 255 字节解释，总上限 765 字节；长对象以最多 61 字节的独立请求读取，并共用总超时。
- MI 0x4F 是错误历史位掩码，不假定它是其他事务的即时错误 ACK。
- 文档版本 0.0.4 与设备返回的协议版本分别记录；设备返回非空 `1.0.0` 不会因文档版本不同而被拒绝。

实机验收需记录身份、地址、通信配置、完整 ACK、三数据区布局和传感器描述；
再验证单关节小幅运动、方向与恢复原位。离线测试通过不代表硬件已验收。

### 查询命令的物理发送结果

`CommandHandle::send_results()` 返回统一异步 `set_*` 命令已观察到的终态物理批次回执副本。
当前同步设备操作接口不收集这些回执。
复合命令可能包含多个批次，回执按观察顺序排列；应分别检查每个批次的
`outcome`、`sent_frames`、`total_frames`、`current_frame_uncertain` 和 `generation`。
空列表表示尚未观察到终态回执，不能据此推断设备没有收到数据。

命令的逻辑结果和物理发送结果独立：命令因超时、关闭或恢复进入失败状态后，
已经进入驱动的写入仍可能完成，后续查询可以看到它的回执。`wait()` 和
`wait_for()` 只等待原有命令终态，不保证晚到的物理回执已经收集完成。
新接口不会修改原有 `stage()`、`error()`、`sent_frames()` 或历史状态。
不要仅凭逻辑超时自动重发运动或写配置命令。

设备恢复后，旧句柄继续保留旧命令的发送证据；其回执不代表新连接上的执行。
恢复后的数据应使用新请求读取，并检查缓存有效性与连接代际。
