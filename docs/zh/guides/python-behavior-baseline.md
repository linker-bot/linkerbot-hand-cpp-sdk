# Python 行为基准

参考源码为 `linkerbot-hand-python-sdk` 提交
`3a63e723b9a0046e9a60a58339b9f064ef5b70fd`，源码版本为 2.0.0。
本文记录未来 Python 迁移所需的原生 SDK 准备，不代表 Python 绑定、hand/arm 接入或实机验收已经完成。
本次不包含 O20。

## 新增原生能力

- L6 统一接口新增位置 Kp/Kd、扭矩 Kd、扭矩输出和限位补偿。新增 L6 控制写入及新增配置读取严格检查匹配回复长度，原生 Manager 默认行为不变。配置回复保留设备实际返回字节，包括旧值及超出写入范围的值。
  Kp 写入为 `0..200`，其余新增向量为六个整数字节。扭矩输出按 FIFO 执行，零值切回位置模式也不会被 latest 替换。
  默认总预算 300 ms；发送前等待 20 ms，发送后等待 20 ms，任一关节为零时改为 150 ms。
  这些时序继承 Python 的协议假设，不是新增实机测量。ACK 可先完成回执，后续端点事务仍等待窗口结束；ACK 失败也保留该窗口，等待不占用通道发送器。后续命令仍受现有接受至完成预算约束。
- Classic L6/O6 新增 `save_parameters_confirmed(timeout=200ms)`：发送八个 CF 字节，要求精确的 `CF 01` 回复。
  缺少 ACK 超时，异常 ACK 报协议错误。原 `save_parameters()` 保留原帧和仅确认发送的行为；L6 HOP 为独立协议。
- L30 新增 `calibrate(timeout=300ms)`：按 ACK 顺序执行失能、密码解锁、校准，失败后不发送后续步骤，不自动重新使能。
  该超时是三个步骤的总预算，不会把 Python 逐阶段窗口默认为同一语义。
- 显式 `O30Layout::kPythonV25` 选择 Python 的整手布局：20 个公开值映射为 25 字节对象，读取和写入均要求完整回复覆盖。
  此布局提供扭矩限制和运动时间，默认挂载 V36 路径不声明这两项能力。
  配置写入 ACK 不作为配置读回，独立查询才更新对应缓存；角度 ACK 保存实际状态，不保存目标冒充测量。

`submit_operation(operation, timeout)` 将操作异步接纳到同一 FIFO，包含等待 ACK 的使能/失能和仅确认发送的清故障。身份读取在入队前拒绝，应使用 `get_device_info()`。未来绑定可使用此入口，避免单独建立 Python 执行器；原操作方法保留同步行为。

新增确认型操作共用现有每设备命令执行器和 Session 发送路径，没有引入 Python 发送队列或第二套原生物理发送器。
原生 O30 Manager 保持挂载布局，不要在同一设备混用两种布局。

## 显式配置

只有使用该布局的端点才设置 `HandDeviceConfig::o30_layout = O30Layout::kPythonV25`，不根据数值或短 ACK 推断协议。
如需空的显式轮询计划关闭轮询，设置 `empty_polling_plan_disables = true`；无参数 `start_polling()` 仍请求角度轮询。
发现设备入口的运行时配置也传递这两个选项。

现有枚举值、方法和原生默认值保持可用。新增结构体成员和 variant 类型要求重新编译 SDK 及全部消费者；不保证旧头文件的二进制兼容。
最低 transport 依赖仍为同一 0.3 次版本线的 v0.3.4。

## 验证边界

离线测试使用生产编解码和 FakeTransport，覆盖配置实际回复、发送前拒绝非法输入、持久化 ACK、校准失败边界、V25 映射和完整 ACK 要求。
原有异步行为向量继续记录差异；全部通过不代表所有操作和型号已跨语言等价。

替代已验收 Python 实现前仍需完成：

- 固定业务配置，针对部署型号、单位和范围边界、异常回复构建相同输入的跨语言验收用例。
- 继续验证部署业务所需的自适应反馈、自动重连和逐通道序号映射。第二批显式策略覆盖的响应窗口、异常响应、关闭终态和显式恢复不代表完整 Python 行为等价；原 C++ 默认值保持不变。
- Python 绑定验证：包装或分配失败不丢回执，GIL、回调、析构及解释器退出安全，异常与墙上时间转换正确。
  原生单调时间戳不能直接当 Unix 时间。
- 多手、多臂接入证明共用通道 Runtime，且设备生命周期相互独立；本次不接入 hand/arm。
- 在部署 OS、适配器及驱动、CAN 后端、固件和拓扑上实机验收。本次不新增实机验证结论；参数持久化、校准、扭矩模式切换和 V25 行为均单独待验证。

V25 要求显式指定左右手；默认请求 ID 为右手 `0x001`、左手 `0x002`，与 Python 一致；`address` 可显式覆盖。原生挂载路径的默认地址不变。

经典协议构造器显式选择经典 CAN，即使 Session 使用 FD 适配器也不继承后端默认 FD 格式。离线帧向量使用 FD 后端默认值解析标志；混合设备实机验收仍待完成。

## 第一批兼容补全（0.3.5）

`refresh_device_info(timeout)` 在当前代次读取并验证身份，不撤销命令、不重启轮询。超时和协议错误不改变准入；身份不匹配或传输连接丢失会关闭此门面的准入、取消未完成任务并使可读缓存失效。原有 `verify_identity()`、`get_device_info()` 生命周期语义不变。已观察到的能力证据保留在当前代次。

`get_cached_snapshot()` 在打开前、恢复/故障/关闭过程中及关闭后均可无 I/O 读取。初始受支持向量具有声明的长度且 `valid=false`；零填充原始存储表示不可用，不代表实测值。不支持的通道保持缺省，不合成依赖描述符的触觉载荷。代次失效后，保留成功捕获的原始证据并将元数据置为不可用，清空关节投影；原生 `DeviceIdentity` 没有有效性元数据，因此省略身份。生命周期转换会尝试捕获最新原生缓存；捕获失败不能阻止关闭，因此可能只保留此前证据。初始化失败不发布部分候选基线。

新增缓存使用本地成功打开/恢复代次和单调递增缓存修订号，与原有 `get_snapshot()`、健康数据的原生代次策略独立。绑定需要显式映射，不应混用。物理 Session/transport 失效或物理代次变化后，保留缓存持续无效，直到本设备再次成功初始化；原有门面就绪状态和健康策略独立保留。没有可见变化时，重复读取不会推进修订号。原有 `get_snapshot()` 仍要求 Ready。本批不包含反馈过滤等待及完整 Python 逐通道序列对齐。

统一轮询支持 L20Lite/L20/L30/O30 实测速度和 O30 电压。V25 力矩限制、运动时间过期阈值分别为 100 ms、300 ms，等于阈值时仍未过期。Mounted V36 默认路径不变。继续使用既有原生轮询和 Session 发送器，不增加 Python 发送调度。hand 0.3.5 的 transport 最低版本仍独立固定为 0.3.4，不随 hand 补丁号提高。

## 第二批兼容补全（0.3.6）

显式设置 `runtime_behavior = RuntimeBehavior::kPython`，启用 Classic L6、O6、L20Lite、L20、L30 和显式 O30 V25 的事务策略。不兼容的协议组合在创建 transport 前拒绝。默认仍为原生策略；不包含 Python 绑定及 hand/arm 接入。

每个设备合并同类主动读取、身份刷新和轮询。`read_waiter_capacity` 默认允许 256 个主动等待者，轮询不占该容量。调用者各自保留等待期限，单个等待超时不取消共享事务；未领取且已无主动读取者或轮询需求的任务丢弃，轮询需求则不受内部等待超时撤销；发布完成后加入的调用者需要下一份样本。关闭或恢复使旧等待者失效，旧结果不能跨代完成新请求。这里只协调设备意图，物理发送仍归既有 Session。读取占用设备事务通道时，尚未执行的 latest 命令仍可替换。

该策略下，命令入队不启动响应计时，必需响应窗口从物理批次完成时开始；多阶段操作各自计时。`response_timeout` 不指定时使用普通事务的协议窗口，显式正值可覆盖。身份初始化和刷新独立使用 `identity_timeout`，与 Python 的优先级一致；保留原生默认 1000 ms，需要匹配固定 Python 默认值时显式设为 200 ms。getter 的调用者等待期限独立，旧 Manager 内部的等待期限不缩短已选择的响应窗口策略。Classic 身份查询以一个批次发送 C0/C1/C2/C4，帧间隔 5 ms，共用一个响应窗口。匹配但格式错误的响应报 `ProtocolError`，无关路由、操作码保持未匹配。必需响应窗口耗尽报 `ResponseTimeoutError`，它继承 `TimeoutError`。失败保留成功缓存证据，并隔离有限时间内的迟到回复；提前到达的回复或错误需等待物理回执后才能终结事务。

关闭使未领取任务报 `ClosedError`。已领取的无 ACK 批次按真实物理结果终结，成功时可在关闭开始后变为 `Dispatched`；发送失败仍失败。需要 ACK 的命令不能仅因发送成功而被确认。回执保留，重新打开和恢复均不重放旧命令。C++ handle 等待超时仍返回 `false`，未来 Python 绑定需要转换为 Python 的 `TimeoutError`。

`recover_for(timeout)` 的调用者等待与共享恢复执行分离，并发调用等待同一轮恢复。准备阶段受首个调用者预算和 `shutdown_timeout` 约束；准备排空失败后不得延迟重连。恢复开始后，调用者超时不撤销该轮恢复。`recovery_timeout` 默认 5 秒；既有 `recovery_attempts` 默认值仍为 1，需要与 Python 默认一致时显式设为 3；`recovery_quiet` 默认 200 ms。仅回复丢失允许重试初始化，安静时间至少覆盖失败阶段的响应窗口和迟到保护；该设备通道来帧会重新计时，但不能延长总预算。重试复用已恢复连接，身份、协议、传输错误或总预算耗尽即停止。Faulted 后可以再次显式恢复；身份及基线验证完成后恢复轮询，不重放目标和离散命令。

容量、节拍、协议布局和超时覆盖应来自已验收业务配置。自动重连、自适应反馈一致性、逐通道序号映射、Python 内存分配/GIL/退出安全及部署硬件验收仍为独立工作。上面的 Python 源码固定版本仅用于比较，不表示已确认部署使用该提交。

下面的显式运行时参数与固定 Python 源码默认值一致，使用前仍应核对部署业务配置：

```cpp
using namespace linkerbot::hand;
using namespace std::chrono_literals;
HandRuntimeConfig runtime;
runtime.runtime_behavior = RuntimeBehavior::kPython;
runtime.command_rate_hz = 250.0;
runtime.discrete_queue_capacity = 16;
runtime.read_waiter_capacity = 256;
runtime.identity_timeout = 200ms;
runtime.response_timeout = std::nullopt;
runtime.shutdown_timeout = 2s;
runtime.recovery_attempts = 3;
runtime.recovery_timeout = 5s;
runtime.recovery_quiet = 200ms;
```
