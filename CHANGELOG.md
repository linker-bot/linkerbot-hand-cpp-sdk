# 版本变更

## 0.3.0（未发布）

- L6 位置 Kp、位置 Kd、扭矩 Kd 仅保留专有 Manager 的读写能力，删除统一读写方法、
  样本、快照字段、事件及能力描述；原有调用须迁移到型号 Manager 并重新编译。
  其余公开枚举编号保持不变。

- 删除 O30 运动时间、扭矩上限的统一接口和型号接口，以及相关能力、类型、事件、快照与轮询支持。
  原有调用须移除并重新编译；保留实际扭矩读取及电流上限读写，其余公开枚举编号保持不变。

- 增加 Gitea/GitHub 开发包工作流，支持版本 tag 和手动触发，生成并验收 Linux TGZ、Windows ZIP、SHA256 和来源清单。
  TGZ/ZIP 共用版本、许可证、包内容和解压接入检查；不自动创建标签或发布 Release。
- 统一 `HandDevice` 全部 8 个 `set_*` 改为本地接纳后返回，由后台发送和处理 ACK。
  接纳前异常同步抛出，接纳后异常保存在 `CommandHandle::error()`。
- 增加有界 FIFO、待发目标替换及默认 250 Hz 批次启动上限；执行预算从接纳时
  开始，覆盖排队、发送和 ACK。等待超时不取消或重发。
- handle 记录真实发送与 ACK 阶段、已确认帧数和稳定终态。关闭／恢复隔离旧代次，
  使用总 shutdown 预算并安全保留未退出驱动／回调所需资源。
- 修正共同型号缓存新鲜度与 ACK 完成规则，拒绝 L6 堵转时间 raw 0 写入，保留读回 0。
- 配套 comm 升级为 0.3.0，源码及安装包要求同 minor 且 comm 不低于 hand 版本。
- 通信仓库现名为 `linkerbot-transport-cpp`；CMake 包名 `linkerbot-comm`、
  `linkerbot::comm` 系列目标和 `LINKERBOT_COMM_SOURCE_DIR` 保留兼容名称。
  当前 CI 与 README 的依赖提交以 README 的源码基线为准；各历史验收场次保留实际使用的 SHA。
- 更新 README 的错误处理示例；安装包附带兼容性策略和
  [0.3 迁移说明](docs/usage.md#命令完成与错误处理)。调用方必须重新编译。
- 增加可执行的跨语言异步行为向量，覆盖接纳、发送/ACK、替换、FIFO、超时、生命周期
  和 polling 进展；明确等待超时、执行预算、关闭已 claimed 命令及短 ACK 的语言差异。
  内部调度器支持测试注入时钟，验证 deadline 边界；生产默认仍使用 steady_clock。
- 安装包 consumer 编译检查全部统一 setter 的返回契约，实际验证统一接口的返回时机、
  缓存读取、ACK 和两类错误出口；公开指南的完整 C++ 示例纳入编译检查。

- 增加 Windows x64 的 PCAN Classic / FD、MiniCANFD、默认适配器发现、显式 factory
  和只读连接示例；实现 SLCAN Lawicel Classic、CANable FD 配置及 Linux 原生串口接入。
- 修复共享总线生命周期和恢复相关的间歇性崩溃，增加并发与故障回归覆盖。
- 补齐 Windows 构建、运行库、ZIP、安装及迁移接入验证。2026-10-19 的 Release
  验收 39 步通过，核心 CTest 143/143、安装及解压接入 4/4。
- 六型号已完成 Linux 常用接口实机验证；Windows Debug 适配实测和未测组合见
  [实机矩阵](docs/hardware-acceptance/README.md#windows-实测矩阵)。PCAN FD 仍待实机验收，
  SLCAN 仍为指定固件配置的实验性接入；不把离线测试或 Debug 结果扩展为 Release 实机通过。

原生型号 Manager、主动 getter 和离散操作保持原执行契约；公开平台及硬件边界见
[支持与兼容性策略](docs/COMPATIBILITY.md)。
