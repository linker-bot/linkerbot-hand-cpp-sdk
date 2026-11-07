# 异步命令验收

[English](README-async.md) | [简体中文](README-async.zh-CN.md)

`tests/data/async_command_behavior_vectors.json` 为 C++ 和 Python 探针提供相同的场景参数。
两个探针都通过生产 L6 编解码器和脚本化 CAN 传输调用统一 HandDevice。
运行器按语言对应的期望值比较阶段、错误、帧数和顺序；未知场景、适配失败或结果不符均判失败。
这些是行为测试，不是方法名或 JSON 文本检查。

安装 Python 解释器后，C++ 离线覆盖会加入 CTest。指定 Python SDK 源码可同时验证两种语言：

```sh
cmake -S . -B build \
  -DLINKERBOT_BUILD_TESTS=ON \
  -DPython3_EXECUTABLE=/path/to/python-with-sdk-dependencies \
  -DLINKERBOT_PYTHON_SDK_DIR=/path/to/linkerbot-hand
cmake --build build --target tests protocol-tests
ctest --test-dir build --output-on-failure
```

解释器需要 Python SDK 的依赖，包括 python-can。Python 源码应包含 `src/` 与
`tests/hand/api_v1/runtime/classic_can_fake.py`。运行器不调用 pytest 或选择 SDK 测试用例。
存在相邻 transport 源码时使用它，否则导入已安装依赖。
Python SDK 不是 C++ 库或安装包消费者的构建依赖。
结果写入 `build/tests/async-behavior-report.json`。

C++ 探针用保留的 L6 堵转阈值命令验证 ACK/FIFO；Python 探针用其位置 Kp 命令。
比较的是命令行为，不要求两边提供相同的参数 API。

共享场景覆盖接纳、完整/短 ACK、无 ACK 发送、目标替换、已进入驱动的写入、FIFO
容量和顺序、调用方等待超时、执行超时、关闭、不重放，以及轮询/缓存进展。
C++ 完整回归还覆盖六种型号的八个 setter、恢复/析构、接纳预算、限速和仲裁。
`[clock]` 用例向内部调度器注入单调时钟，检查截止前 1ms、恰好截止和等待不重置预算。
原生协议响应窗口与驱动阻塞仍用同步屏障和有界等待验证；这不是完全虚拟化的设备时钟。

两种语言仍有明确差异：Python 等待会抛 TimeoutError，执行预算是响应窗口；关闭时已
进入驱动的成功写入仍可能成为 dispatched；匹配但过短的 ACK 会产生 ProtocolError。
C++ 预算从接纳覆盖到完成，关闭时冻结未完成请求，短 ACK 被忽略直至完整响应或超时。
两个 API 均不公开取消接口；Python 没有 cancelled 枚举。
不得修改期望值来隐藏回归，也不能把这些验证称为完全一致。

安装后验证导出的 SDK 目标：

```sh
cmake --install build --prefix /tmp/linkerbot-install
cmake -S validation/installed-consumer -B /tmp/linkerbot-consumer \
  -DCMAKE_PREFIX_PATH=/tmp/linkerbot-install
cmake --build /tmp/linkerbot-consumer
ctest --test-dir /tmp/linkerbot-consumer --output-on-failure
python3 validation/check_documented_examples.py \
  --prefix /tmp/linkerbot-install --build-dir /tmp/linkerbot-doc-examples
```

消费者检查八种 setter 签名，并验证发送屏障下的提前返回、独立缓存读取、等待超时不取消、
无 ACK 完成、配置 ACK、零帧同步校验及句柄保留异步传输错误。
文档检查编译完整示例，不在硬件上运行运动。

六型号实机验收单独记录型号/序列号、固件、驱动、源码版本、完整回复、连续目标、
断连恢复及安全观察。离线通过不证明机械到位、固件兼容或版本已发布。

CTest 也会以显式 Python 运行时策略执行 C++ 探针。运行器的
`--cpp-python-profile` 将该策略与固定样本的 Python 期望比较。C++ handle
接口不变：调用者等待超时仍由 `wait_for()` 返回 `false`，Python 则抛出
`TimeoutError`；运行器明确检查这项绑定差异。原生默认期望仍单独执行。
启用可选 Python 源码比较后，十个场景产生三十项观测，分别来自原生 C++、
显式兼容策略和 Python。这是 L6 离线验收语料，不能据此声称所有型号或硬件等价。
