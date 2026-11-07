# 构建与验证指南

[English](../../en/guides/validation.md) · [文档索引](../README.md)

## 明确选择依赖

默认 `LINKERBOT_COMM_PROVIDER=AUTO` 保留旧命令的行为；也可明确指定：

- `SOURCE`：使用 `LINKERBOT_COMM_SOURCE_DIR`，未指定路径时使用同级 transport 目录。
- `PREBUILT`：只查找已安装或解压的 `linkerbot-comm`，忽略源码目录。
- `AUTO`：优先使用非空 `LINKERBOT_COMM_PREBUILT_PREFIX`，其次使用源码路径，最后查找安装包。

父项目已经提供 `linkerbot::comm` 时，复用父项目的目标，并检查版本。
显式设置旧参数 `LINKERBOT_COMM_SOURCE_DIR=` 仍可在 AUTO 下通过 `CMAKE_PREFIX_PATH`
使用安装包。配置日志会显示实际选择的依赖；切换依赖时使用独立构建目录，避免旧 CMake 缓存。
hand 0.3.4 要求 transport 0.3.x 且不低于 0.3.4。源码、预构建及安装消费者测试均拒绝更低补丁版本和其他 minor 版本。
源码开发继续使用 README 与 CI 固定的提交，并在测试报告中记录实际版本和构建配置。

### 从源码构建

```bash
cmake --preset source -DLINKERBOT_COMM_SOURCE_DIR=../linkerbot-transport-cpp
cmake --build --preset source --config Release --parallel 4
cmake --install build/source --config Release --prefix "$PWD/install/hand"
```

Preset 需要 CMake 3.21+；使用 CMake 3.16+ 时可用
`cmake -S . -B build/source -DCMAKE_BUILD_TYPE=Release -DLINKERBOT_COMM_PROVIDER=SOURCE`
并附加相同的路径和构建开关。

### 使用预构建 transport

从 [transport v0.3.4](https://github.com/linker-bot/linkerbot-transport-cpp/releases/tag/v0.3.4)
下载匹配系统、架构、编译器与运行库的 SDK 及同名 `.sha256` 文件。以下以
Ubuntu 24.04 x86_64 为例，在下载目录执行：

```bash
archive=linkerbot-transport-0.3.4-Linux-x86_64-ubuntu24.04-GNU13.3.0-libstdcxx-abi1-Release.tar.gz
sha256sum --check "$archive.sha256"
mkdir -p comm-sdk
tar -xzf "$archive" -C comm-sdk --strip-components=1
```

回到 Hand SDK 根目录，将路径替换为上面解压目录的绝对路径：

```bash
cmake --preset prebuilt -DLINKERBOT_COMM_PREBUILT_PREFIX=/absolute/path/to/comm-sdk
cmake --build --preset prebuilt --config Release --parallel 4
cmake --install build/prebuilt --config Release --prefix "$PWD/install/hand"
```

无需克隆或编译 transport。前缀必须指向含 `include/`、`lib/` 等目录的 SDK 根目录，
不能指向归档文件或其父目录。也可省略专用前缀，使用 `CMAKE_PREFIX_PATH` 或 `linkerbot-comm_DIR`。
配置时不自动下载依赖；缺少预构建包会报错，不会退回源码构建。
Windows 解压 ZIP 后同样指定 SDK 根目录；Release 使用 `/MD`，Debug 使用 `/MDd`，
构建时分别使用 `--config Release` 或 `--config Debug`。Linux 包使用 libstdc++ ABI1。
不提供 macOS 发布包；本机编译的安装包也可通过此模式使用。
驱动和厂商库仍需单独安装。

仅安装 Hand SDK 不会复制预构建 transport。下游应用需要同时提供两个前缀：

```bash
cmake -S demo -B demo/build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$PWD/install/hand;/absolute/path/to/comm-sdk"
cmake --build demo/build --config Release --parallel 4
```

## 运行离线测试

在 SDK 源码根目录执行，前提是已安装 Catch2 3，且相邻目录是固定版本的 transport：

```bash
cmake -S . -B build-tests -DCMAKE_BUILD_TYPE=Release \
  -DLINKERBOT_COMM_SOURCE_DIR=../linkerbot-transport-cpp \
  -DLINKERBOT_BUILD_TESTS=ON -DLINKERBOT_BUILD_EXAMPLES=OFF \
  -DLINKERBOT_BUILD_TOOLS=OFF -DLINKERBOT_COMM_BUILD_TESTS=OFF \
  -DLINKERBOT_ALLOW_DEPENDENCY_DOWNLOADS=OFF
cmake --build build-tests --parallel 4 --target tests protocol-tests
ctest --test-dir build-tests --output-on-failure
```

测试程序不属于默认构建目标，运行 CTest 前必须显式构建。库编译成功不等于测试通过。
离线夹具验证协议、调度和生命周期约定，不能证明 USB 驱动、固件时序、机械精度或实机行为。
实际支持边界见[平台与硬件验证范围](../reference/compatibility.md)。

Windows 环境准备见 [Windows 指南](windows.md)。构建两个测试目标时增加
`--config Release`，运行 CTest 时增加 `-C Release`，不要混用 Debug 与 Release 库和运行库设置。

## 判断命令结果

setter 返回句柄表示本地接纳。逻辑执行结果应检查终态和错误；ACK 不代表机械到位。
物理批次证据通过 `send_results()` 查询，见 [API 指南](usage.md#查询命令的物理发送结果)。
超时或生命周期错误不能证明已进入驱动的写入没有发送。
结果不确定时，不要自动重发运动或持久化配置命令。

## 排查 CI 失败

先查失败步骤和第一条具体错误，记录 SDK 与 transport 提交、平台、编译器、构建配置、
测试名称及随机种子。Windows 附件包含构建日志、测试结果和打包证据。
区分编译失败、协议断言、专门的 deadline 测试，以及受主机调度影响的功能测试。
deadline 测试应保留严格边界；编码覆盖测试可以设置独立预算，不应为此修改生产默认值。

PR 作业执行离线测试和协议检查；main 分支还按工作流的平台条件执行安装、公共头文件、
文档示例和包检查。跳过不等于通过。历史实机报告只适用于其中记录的版本和环境。

## 本地编辑器配置

整个 `.vscode/` 目录均被忽略。机器路径和个人启动配置留在本地，项目维护的构建、
测试命令应放在 CMake、justfile 和文档中。
