# Build and validation guide

[中文](../../zh/guides/validation.md) · [Documentation index](../README.md)

## Select the dependency deliberately

`LINKERBOT_COMM_PROVIDER=AUTO` preserves existing commands. Select explicitly with:

- `SOURCE`: use `LINKERBOT_COMM_SOURCE_DIR`, defaulting to the adjacent transport checkout.
- `PREBUILT`: find an installed or extracted `linkerbot-comm`; ignore source paths.
- `AUTO`: prefer a nonempty `LINKERBOT_COMM_PREBUILT_PREFIX`, then a source path, then package discovery.

A parent-provided `linkerbot::comm` target is reused and version-checked.
The legacy empty `LINKERBOT_COMM_SOURCE_DIR=` still selects package discovery through
`CMAKE_PREFIX_PATH` in AUTO mode. Configure logs identify the selected dependency.
Use separate build directories when switching dependencies to avoid stale CMake caches.
Hand 0.3.4 requires transport 0.3.4 or newer in the 0.3 minor line. Source,
prebuilt and installed-consumer checks reject older patches and other minor lines.
For paired development, keep the source commit pinned in README and CI; record the
actual revision and configuration in test reports.

### Build from source

```bash
cmake --preset source -DLINKERBOT_COMM_SOURCE_DIR=../linkerbot-transport-cpp
cmake --build --preset source --config Release --parallel 4
cmake --install build/source --config Release --prefix "$PWD/install/hand"
```

Presets require CMake 3.21+. With CMake 3.16+, use
`cmake -S . -B build/source -DCMAKE_BUILD_TYPE=Release -DLINKERBOT_COMM_PROVIDER=SOURCE`
and supply the same dependency path and build options.

### Use prebuilt transport

Download a matching SDK and its adjacent `.sha256` file from
[transport v0.3.4](https://github.com/linker-bot/linkerbot-transport-cpp/releases/tag/v0.3.4).
Match the OS, architecture, compiler and runtime. For Ubuntu 24.04 x86_64,
run in the download directory:

```bash
archive=linkerbot-transport-0.3.4-Linux-x86_64-ubuntu24.04-GNU13.3.0-libstdcxx-abi1-Release.tar.gz
sha256sum --check "$archive.sha256"
mkdir -p comm-sdk
tar -xzf "$archive" -C comm-sdk --strip-components=1
```

From the Hand SDK root, substitute the absolute extraction path:

```bash
cmake --preset prebuilt -DLINKERBOT_COMM_PREBUILT_PREFIX=/absolute/path/to/comm-sdk
cmake --build --preset prebuilt --config Release --parallel 4
cmake --install build/prebuilt --config Release --prefix "$PWD/install/hand"
```

No transport checkout or rebuild is needed. The prefix must contain `include/`,
`lib/`, etc.; do not pass an archive or its parent directory. Alternatively, omit
the dedicated prefix and use `CMAKE_PREFIX_PATH` or `linkerbot-comm_DIR`.
Configuration does not download transport; a missing package fails without falling back to sources.
On Windows, extract the ZIP and pass its SDK root; match Release `/MD` or Debug `/MDd`
and build with `--config Release` or `--config Debug`. Linux packages use libstdc++ ABI1.
There is no macOS release archive; locally compiled installations also work in PREBUILT mode.
Drivers and proprietary libraries remain separate requirements.

Installing hand does not copy a prebuilt transport dependency. Downstream applications
must supply both installation prefixes:

```bash
cmake -S demo -B demo/build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$PWD/install/hand;/absolute/path/to/comm-sdk"
cmake --build demo/build --config Release --parallel 4
```

## Run offline tests

From the SDK source root, with Catch2 3 installed and the pinned transport checkout
in the adjacent directory:

```bash
cmake -S . -B build-tests -DCMAKE_BUILD_TYPE=Release \
  -DLINKERBOT_COMM_SOURCE_DIR=../linkerbot-transport-cpp \
  -DLINKERBOT_BUILD_TESTS=ON -DLINKERBOT_BUILD_EXAMPLES=OFF \
  -DLINKERBOT_BUILD_TOOLS=OFF -DLINKERBOT_COMM_BUILD_TESTS=OFF \
  -DLINKERBOT_ALLOW_DEPENDENCY_DOWNLOADS=OFF
cmake --build build-tests --parallel 4 --target tests protocol-tests
ctest --test-dir build-tests --output-on-failure
```

Test executables are excluded from the default build: explicitly build the targets
before running CTest. A successful library build alone is not a test result.
Offline fixtures cover protocol behavior, scheduling and lifecycle contracts;
they do not certify USB drivers, firmware timing, motion accuracy or real hardware.
See [hardware validation scope](../reference/compatibility.md).

For Windows, use the [Windows guide](windows.md), build both test targets with
`--config Release`, and pass `-C Release` to CTest. Do not mix Debug and Release
libraries or runtime settings.

## Interpret command results

A returned setter handle means local admission. Check its terminal stage and error
for logical completion; an ACK is not proof of mechanical arrival. For physical
batch evidence, use `send_results()` as described in the
[API guide](usage.md#inspect-physical-command-delivery). A timeout or lifecycle
failure does not prove that a claimed write sent nothing. Do not automatically
replay movement or persistent configuration after an uncertain result.

## Investigate CI failures

Start with the failed step and its first concrete error. Record the SDK and
transport commits, platform, compiler, build configuration, test name and random
seed. Windows artifacts contain build logs, test results and package evidence.
Distinguish compilation failures, protocol assertions, explicit deadline tests and
functional tests affected by host scheduling. Keep deadline tests strict; give
encoding-only tests an explicit budget without changing production defaults.

PR jobs run offline tests and protocol checks. Main-branch jobs additionally run
installation, public-header, documented-example and package checks according to
the workflow's platform conditions. A skipped check is not a pass. Historical
hardware reports apply only to their recorded revision and environment.

## Repository-local editor settings

`.vscode/` is ignored in full. Keep machine paths and personal launch settings
local; maintained build and test commands belong in CMake, the justfile and docs.
