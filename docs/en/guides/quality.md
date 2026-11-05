# C++ quality and documentation checks

[English](quality.md) | [简体中文](../../zh/guides/quality.md)

## Report Card

The `C++ Report Card` workflow runs on PRs, main/master pushes and manual dispatch.
It uses the same full transport SHA as the main CI workflow, pinned versions of the
Report Card action and GitHub actions, and Clang 18/Cppcheck on the repository's
self-hosted Linux x64 runner (`linkerhand-linux`, with an Ubuntu 24.04 toolchain).
The compilation database is produced and consumed on the same runner.

Full mode runs clang-format, Cppcheck and clang-tidy (`clang-analyzer-*`, `bugprone-*`).
The scope includes SDK sources/headers, examples, tools, offline and hardware-test
translation units, and the installed consumer. Hardware tests are analyzed, not run.
Dependencies and generated/build files are excluded. The historical standalone
`tests/hand/o30/hardware/verify_fixes.cc` is excluded because it targets an older API
and is not part of the maintained CMake build. No diagnostic-based source exclusions
are used. `.clang-format` remains the formatting authority.

CI requires full analysis, **A+, 100/100 and zero diagnostics** from all three tools.
Missing tools, compile entries, analyzer failures, incomplete coverage, warnings and
informational findings all fail the gate. The action uses `fail_on: findings` and
`threshold: 100`; an independent validator checks the grade and completeness.
No new exclusions, suppressions or disabled rules are introduced by this gate.
The numeric score does not measure test coverage or hardware compatibility.

Every run uploads `report.json`, `report.md`, `badge.svg` and the merged compilation
database as `cppreportcard-linux-x64`. Reports are not automatically committed.
The [report index](https://github.com/linker-bot/linkerbot-hand-cpp-sdk/blob/main/.github/cppreportcard/README.md) links to the workflow.

## Reproduce

Use Linux with clang++/clang-format/clang-tidy 18, Cppcheck and the pinned transport
checkout. Follow the workflow's configure/build/install/consumer steps, then:

```bash
python3 validation/prepare_report_db.py --output build/report-compile-commands.json \
  build/report/compile_commands.json build/report-consumer/compile_commands.json \
  build/report-l6/compile_commands.json
cppreportcard-cli --config .cppreportcard.json --mode full --fail-on findings --threshold 100 \
  --output-dir .github/cppreportcard
python3 validation/check_report_complete.py .github/cppreportcard/report.json --require-aplus
```

Use the CLI from the action revision pinned in `.github/workflows/cppreportcard.yml`.
When running the CLI directly, add `--output-dir .github/cppreportcard` if its default
output directory differs. The wrapper models Catch2 macros, selects the actual Linux target and preserves
CMake string-macro quoting for Cppcheck. Ordinary C++ compilation and clang-tidy
still use the real headers and original argument vectors. Review tool diagnostics
rather than treating all warnings as defects or suppressing them wholesale.

## Documentation contract

English is the default (`README.md`, `CONTRIBUTING.md`, `CHANGELOG.md`, `docs/README.md`).
Chinese root documents use `.zh-CN.md`; topical docs use parallel `docs/en` and `docs/zh`
paths. `README.en.md` and `docs/README.en.md` remain exact compatibility copies.
Historical hardware reports retain their original language; the English acceptance
index summarizes scope and links to primary evidence.

`validation/check_docs.py` checks local Markdown targets and anchors, paired topical
files, language links, default/alias consistency, pinned quickstarts and maintained
example commands. `check_documented_examples.py` separately compiles complete C++
examples against an installed SDK; neither check executes motion.

Use C++17 syntax, explicit ownership and units, truthful timeout/ACK language and
commands that build required targets. Keep historical results labeled by revision.

[Diagnostic review and baseline ledger](quality-review.md).
