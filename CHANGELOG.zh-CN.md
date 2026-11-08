# 版本变更

[English](CHANGELOG.md) | [简体中文](CHANGELOG.zh-CN.md)

## 0.3.10

- 修正 Python 策略缺少命令反馈时的健康统计，保持实际发送结果及 Native 默认行为。
- 包含未公开发行的 v0.3.9 标签中的轮询、逻辑延迟和力字段健康投影修正。
- 保持 transport v0.3.4 最低依赖，SDK 和消费者需要重新编译，实机验收单独进行。

## 0.3.9

已创建标签，但未公开发行；公开发行由 0.3.10 接替。

- 统一显式 Python 策略的轮询计数和逻辑事务延迟，使用与 Python 一致的百分位取整及设备生命周期内的有界窗口。
- 补齐力字段健康投影和共享查询间隔合并，保留 Native 默认行为及 transport v0.3.4 最低依赖。
- SDK 和消费者需要重新编译，不新增实机验收结论，详见[发行说明](RELEASE_NOTES.md)。

## 0.3.8

- 修正显式 Python 策略下的有界停止轮询、健康逻辑代次和可选查询降级后的最终错误记录。
- 增加 Python 策略接收路由容量配置、发现传递、溢出及安装消费者回归，保留 Native 默认行为及 transport v0.3.4 最低依赖。
- SDK 和消费者需要重新编译，不新增实机验收结论，详见[发行说明](RELEASE_NOTES.md)。

## 0.3.7

- 为显式 Python 运行策略补齐按发布推进的通道序号、身份采样元数据、过滤反馈的缓存等待、固件可选查询降级和统一角度反馈策略。
- 修正匹配触觉响应的校验和力数据合并；在该策略下增加 L20Lite 独立标量查询。
- 保留原生默认行为和 transport v0.3.4 最低依赖。SDK 与消费者需要重新编译，实机验收单独进行。详见[发行说明](RELEASE_NOTES.md)。

## 0.3.6

- 新增显式 Python 运行时策略，覆盖读取合并、发送后的响应窗口、匹配响应校验、关闭终态及有界显式恢复。
- 保留原生默认行为及 transport v0.3.4 最低依赖，需要重新编译消费者。验证范围见 [发行说明](RELEASE_NOTES.md) 和 [行为基准](docs/zh/guides/python-behavior-baseline.md)。

## 0.3.5

- 补齐统一实测速度、电压轮询，修正 O30 V25 过期阈值，新增不撤销命令的身份刷新和全生命周期可读缓存。
- 增加失败边界、身份刷新与关闭并发、安装消费者及 hand/transport 独立补丁版本回归。transport 最低版本仍为 v0.3.4，不新增实机验收结论。
- 范围见 [发行说明](RELEASE_NOTES.md)，剩余差异见 [Python 行为基准](docs/zh/guides/python-behavior-baseline.md)。

## 0.3.4

- 新增 L6 统一增益、扭矩输出及限位补偿，Classic 显式保存 ACK，L30 校准，以及显式 O30 V25 布局的扭矩限制和运动时间。
- 增加固定 Python 发送帧向量、失败边界回归和安装消费者验证；保留原生默认行为和 transport v0.3.4 依赖。
- 升级后重新编译全部消费者；不新增实机验收结论。范围与剩余迁移要求见 [Python 行为基准](docs/zh/guides/python-behavior-baseline.md)。

## 0.3.0（未发布）

- 增加显式源码/预构建 transport 接入选项和独立 CMake presets。
- 发布目标与 transport 对齐：八份 Ubuntu Release、四份 Windows x64/ARM64 Debug/Release SDK，校验构建身份与完整产物集。
- 修正 Focal 旧版 libstdc++ 下 runtime 请求、轮询停止和故障监听的无限等待，保留有限 deadline 和异常传播语义。
- 配套源码包含 transport 的 Focal 系统调用故障注入测试修复；原生 PCAN 夹具增加 Windows 调度余量与 FD 发现失败诊断。
- 配套 transport 源码和预构建下载更新至 v0.3.3，包含 Focal 回执等待修复；发布校验支持新的包名元数据，继续使用受支持的 comm CMake 兼容接口。

- 默认文档改为英文，保留完整中文对应页、双向导航及旧英文入口兼容副本。
- 补充 C++17 贡献约定、构建/质量指南和历史实机证据英文摘要；不改写原始实测结果。
- 接入 full 模式 C++ Report Card，以分析完整性为门禁并保留诊断，增加文档和打包回归检查。

- 为跨型号角度编码覆盖测试设置独立发送预算，避免 Windows 调度延迟误判；生产超时不变。
- 修正中英文离线测试构建目标、Windows transport 基线和仓库地址，补充构建与验证指南。
- 移除受版本控制的 `.vscode` 配置并忽略整个目录；临时发布测试不再依赖个人 Git 签名配置。

- 更新 transport 固定基线至 `a46177e`，纳入发送诊断与统计先于终态回执发布的修复。
- 回执存储在物理发送接纳前预留；观察器异常不再重复处理已完成的发送统计。
  保持现有公开 API、命令终态及晚到回执查询语义。
- 发布基线解析支持仅含 GitHub 工作流的仓库，存在 Gitea 配置时仍校验两者一致。

- 新增兼容的 `CommandHandle::send_results()` 查询，保留物理批次回执、部分发送与不确定结果；
  超时、关闭或恢复后的晚到回执不修改原有命令终态。补充恢复失败重试和旧代际证据回归。

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
  [0.3 迁移说明](docs/zh/guides/usage.md#命令完成与错误处理)。调用方必须重新编译。
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
[支持与兼容性策略](docs/zh/reference/compatibility.md)。
