# 发布产物

[English](../../en/guides/releases.md) · [文档索引](../README.md)

## 与 transport 对齐的目标

Hand 发布工作流使用与 transport v0.3.4 相同的目标矩阵：

- Ubuntu 20.04、22.04、24.04、26.04，各有 x86_64 和 aarch64（ARM64）的 Release TGZ，共八份。
- Windows x64、ARM64，各有 Debug `/MDd` 和 Release `/MD` 的 MSVC v143 ZIP，共四份。

Linux 在对应发行版的原生架构容器内构建，并记录镜像 digest；使用发行版默认 GCC、
libstdc++ ABI1 和 C++17。Windows 使用原生架构 runner 与 v143 14.44 工具集。
容器验证不代表原生内核或硬件验收；ARM64 SDK 通过不代表厂商提供了 ARM64 驱动。

产物是开发 SDK，Hand 包包含 hand 与配套 transport 静态库、头文件、CMake 包和文档，
保留只读命令行示例；不包含测试程序、厂商驱动或厂商库。
两个项目版本独立。hand 0.3.5 的源码、预构建及安装消费者均要求 transport 0.3.x 且不低于 0.3.4。


配套源码固定提交和预构建下载统一使用 v0.3.4 发行版。transport 公共结构有新增字段，
须使用匹配的头文件和库重新编译 hand 及下游应用。本次更新不计为新增实机验收。
transport 归档名现以 `linkerbot-transport-` 开头；Hand 继续使用受支持的
`linkerbot-comm` CMake 包及 CAN 目标别名。

## 命名与校验

以下示例中的 hand 版本为 `0.3.0`，编译器版本以实际构建结果为准：

```text
linkerbot-hand-cpp-0.3.0-Linux-aarch64-ubuntu24.04-GNU13.3.0-libstdcxx-abi1-Release.tar.gz
linkerbot-hand-cpp-0.3.0-Windows-ARM64-msvc143-19.44.35229.0-Debug-MDd.zip
```

每份归档旁有 `.sha256`；完整集合另有 `release-manifest.json` 和 `SHA256SUMS.txt`，
共 26 个文件。清单记录 hand/transport 源码 SHA、构建身份、文件大小与对象检查数量。
收集步骤要求十二个目标完整且不重复，核对文件名、元数据和每个静态库对象的实际架构，
拒绝缺包、校验不符、跨架构对象、错误 CRT 和不同源码来源。

构建/发布文件的 PR 变更、版本标签或手动 dispatch 会触发验证并上传 Actions 附件。
PR 产物记录测试用合并提交。

如需将完整集合附加到 GitHub Release，在已有且匹配的 `v<version>` 标签上手动运行
**Release packages**，填写完整 transport SHA 和版本，并启用 `create_draft`。
标签对应的 hand 提交须已有通过 A+ 门禁的主分支推送或手动触发的 **C++ Report Card** 运行。
十二个平台构建及集合校验全部通过后，可选任务才会创建含 26 个文件的 **Release 草稿**，
并下载核对上传字节。分支触发、移动或不匹配的标签、已有 Release 均被拒绝，不覆盖附件。
默认运行只上传 Actions 附件；正式发布草稿由维护者另行操作。
下载可用性以 [Releases](https://github.com/linker-bot/linkerbot-hand-cpp-sdk/releases) 的实际附件为准。
独立使用 transport 预编译包见[构建指南](validation.md)。
