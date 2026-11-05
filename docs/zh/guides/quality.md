# C++ 质量与文档检查

[English](../../en/guides/quality.md) | [简体中文](quality.md)

## Report Card

`C++ Report Card` 工作流在 PR、main/master 推送和手动触发时运行。
它从主 CI 读取同一个完整 transport 提交号，固定 Report Card 与 GitHub Action 版本，
在仓库的自托管 Linux x64 runner（`linkerhand-linux`，使用 Ubuntu 24.04 工具链）上
运行 Clang 18 和 Cppcheck。同一运行器生成和消费编译数据库。

full 模式运行 clang-format、Cppcheck、clang-tidy（`clang-analyzer-*`、`bugprone-*`）。
范围包括 SDK 源码和头文件、示例、工具、离线及硬件测试编译单元、安装包消费者。
硬件测试仅分析，不执行。依赖、生成文件和构建目录不纳入范围。
历史独立程序 `tests/hand/o30/hardware/verify_fixes.cc` 针对旧版 API，未纳入现行 CMake
构建，因此明确排除；不按诊断结果排除源码。格式以 `.clang-format` 为准。

CI 要求三个工具均完整分析、**A+、100/100 且零诊断**。
缺少工具或编译单元、分析失败、覆盖不完整，以及警告和信息级诊断都会使门禁失败。
Action 使用 `fail_on: findings` 与 `threshold: 100`，另用独立校验器核对等级和完整性。
此次门禁不新增排除项、抑制标记或关闭规则。分数不代表测试覆盖率或硬件兼容性。

每次上传 `cppreportcard-linux-x64` 附件，包含 `report.json`、`report.md`、
`badge.svg` 和合并编译数据库。报告不自动提交到仓库。
[报告索引](https://github.com/linker-bot/linkerbot-hand-cpp-sdk/blob/main/.github/cppreportcard/README.md)链接到工作流。

## 本地复现

使用 Linux、clang++/clang-format/clang-tidy 18、Cppcheck 和固定的 transport 源码。
先执行工作流中的配置、构建、安装和消费者配置步骤，再运行：

```bash
python3 validation/prepare_report_db.py --output build/report-compile-commands.json \
  build/report/compile_commands.json build/report-consumer/compile_commands.json \
  build/report-l6/compile_commands.json
cppreportcard-cli --config .cppreportcard.json --mode full --fail-on findings --threshold 100 \
  --output-dir .github/cppreportcard
python3 validation/check_report_complete.py .github/cppreportcard/report.json --require-aplus
```

CLI 应与 `.github/workflows/cppreportcard.yml` 固定的 Action 提交一致。
Cppcheck 包装器对 Catch2 宏建模、显式选择 Linux 目标，并保留 CMake 字符串宏转义；
普通编译与 clang-tidy 仍使用真实头文件和原始参数数组。
诊断应逐项审查，不能直接认定每个警告都是缺陷，也不能整批屏蔽。

## 文档约定

默认英文入口是 `README.md`、`CONTRIBUTING.md`、`CHANGELOG.md` 与 `docs/README.md`。
中文根文档使用 `.zh-CN.md`，专题文档采用对称的 `docs/en` 和 `docs/zh` 路径。
`README.en.md`、`docs/README.en.md` 保留为完全一致的兼容副本。
历史实机报告保留原文，由英文验收索引汇总范围并链接到原始证据。

`validation/check_docs.py` 检查本地 Markdown 目标和锚点、专题双语文件、语言链接、
默认页与兼容副本、快速开始依赖基线及测试命令。
`check_documented_examples.py` 另外编译安装 SDK 的完整 C++ 示例；两者都不执行运动。

示例使用 C++17，明确所有权、单位与超时/ACK 含义，命令必须构建所需目标。
历史结果应始终标注对应版本。

[诊断复核与基线台账](quality-review.md)。
