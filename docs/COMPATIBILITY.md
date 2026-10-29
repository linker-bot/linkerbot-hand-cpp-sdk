# Platform and hardware validation scope

The unreleased 0.3.0 API removes O30 motion-time and torque-limit getters, setters,
model managers, samples, events and snapshot fields. Remove those calls and rebuild
consumers. Actual torque reads and current-limit reads/writes remain available;
surviving public enum values retain their numeric IDs. Dated acceptance records
refer to the SDK revision recorded in each report, before this removal.

L6 position Kp, position Kd and torque Kd are available only through the native
`l6::L6` managers (`position_kp`, `position_kd`, `torque_kd`). Their unified setters,
getters, samples, snapshot fields, events and capability entries have been removed.
Migrate to each manager's `set_values()`, `get_blocking()` and `get_snapshot()` and
rebuild consumers. Native protocol and raw-value semantics are unchanged.

The 0.3.0 validation baseline is Ubuntu 24.04 x86_64 / GCC 13 / C++17 and
Windows 11 x64 / Visual Studio 2022 / MSVC v143 14.44 / C++17. Other operating
systems, architectures, toolchains and driver versions are not covered by this
baseline. Windows Release uses `/MD`; Debug uses `/MDd`. Match the configuration
and toolchain of a prebuilt package and rebuild when the SDK changes.

L6 hardware results below cover the default Classic CAN protocol. The optional HOP
CAN FD protocol has offline protocol-vector and FakeTransport coverage only; hardware acceptance is pending.

All six models have Linux SocketCAN hardware records for common SDK functions.
Windows Debug hardware records cover PCAN Classic for L6, O6, L20 Lite and L20,
and CANable-profile SLCAN for all six models. MiniCANFD hardware adaptation checks
passed for O30 and L30. O30 used Debug with x64 `HCanbus.dll 3.3.103.0`;
the L30 build configuration and DLL version are not recorded here.
Windows hardware checks cover discovery, opening, identity/angle reads, closing
and one joint command with restoration. PCAN FD remains unverified on hardware.
The Lawicel profile and native
Linux PCAN, MiniCANFD and SLCAN paths have no hardware pass claim in this release
record. SLCAN remains experimental and requires an explicit firmware profile.

Windows Release engineering and ZIP consumption passed on 2026-10-19; this is
separate from Debug hardware results. Release hardware, Windows USB hotplug,
long-running load, full-joint Windows motion and motor positioning accuracy are
not certified by those results. Factory reset, EEPROM persistence
and advanced tuning are outside common-function hardware acceptance.
Vendor drivers and DLLs are obtained separately. See the source repository's
`docs/hardware-acceptance/README.md` for dated records and device-specific limits.

---

# 平台与硬件验证范围

未发布的 0.3.0 已删除 O30 运动时间和扭矩上限的读写接口、型号入口、样本、事件及快照字段。
旧调用须移除并重新编译；实际扭矩读取与电流上限读写保留，其余公开枚举编号保持不变。
历史实机验收以各报告记录的 SDK 版本为准，不代表删除后的接口清单。

L6 位置 Kp、位置 Kd、扭矩 Kd 仅通过 `l6::L6` 的专有 Manager 提供。
对应的统一读写方法、样本、快照字段、事件及能力描述已删除；调用方须迁移到 Manager 的
`set_values()`、`get_blocking()`、`get_snapshot()` 并重新编译。原生协议及原始值语义保持不变。

`0.3.0` 验证基线为 Ubuntu 24.04 x86_64 / GCC 13 / C++17，以及 Windows 11 x64 /
Visual Studio 2022 / MSVC v143 14.44 / C++17。其他系统、架构、工具链及驱动版本不在
本轮验证范围。Windows Release 使用 `/MD`，Debug 使用 `/MDd`；使用预编译包时匹配
工具链和配置，更新 SDK 后配套重编译应用。

下表的 L6 实机结果均指默认 Classic CAN 协议。可选 HOP CAN FD 协议仅通过离线协议向量和
FakeTransport 验证，尚未完成实机验收。

| 通信路径 | 本轮实机验证范围 |
| --- | --- |
| Linux SocketCAN | L6、O6、L20 Lite、L20、L30、O30 的 SDK 常用功能通过 |
| Windows PCAN Classic | L6、O6、L20 Lite、L20，Debug 适配检查通过 |
| Windows PCAN FD | 后端与离线测试已实现，L30、O30 实机未测 |
| Windows MiniCANFD | O30、L30 实机适配检查通过；O30 使用 Debug 和 x64 `HCanbus.dll 3.3.103.0`，L30 的构建配置及 DLL 版本未记录 |
| Windows SLCAN | 六型号的 Debug 适配检查通过；仅覆盖报告记录的 CANable 配置，仍属实验性接入 |
| Linux 原生 PCAN、MiniCANFD、SLCAN | 本轮没有相应实机通过结论；串口 PTY、动态库测试桩不等于实机 |

Windows 适配检查覆盖发现、打开、身份/角度读取、关闭及一次单关节命令和恢复。
Release 工程、安装和 ZIP 解压接入已于 2026-10-19 通过，和 Debug 真机结果分别记录。
上述结果不覆盖 Release 实机、Windows USB 拔插、长期负载或全关节 Windows 运动；
不保证设备机械定位精度。恢复出厂、EEPROM 持久化和高级调参不属于常用功能
实机验收范围。SLCAN 的 Lawicel 固件不由 CANable 配置的结果覆盖，厂商驱动和 DLL 需
另行获取。详细设备、固件及限制见源码仓库 `docs/hardware-acceptance/README.md`。
