# LinkerBot Dexterous Hand C++ SDK

[![C++ Report Card](.github/cppreportcard/badge.svg)](.github/cppreportcard/README.md)
[![C++ Report Card CI](https://github.com/linker-bot/linkerbot-hand-cpp-sdk/actions/workflows/cppreportcard.yml/badge.svg?branch=main)](https://github.com/linker-bot/linkerbot-hand-cpp-sdk/actions/workflows/cppreportcard.yml)

[English](README.md) | [简体中文](README.zh-CN.md)

A C++17 SDK for connecting to LinkerBot dexterous hands, controlling joints, reading sensors,
and collecting telemetry in the background. CAN/CAN FD communication is provided by
[linkerbot-transport-cpp](https://github.com/linker-bot/linkerbot-transport-cpp).

- **Linux users:** follow the quick start below to connect through SocketCAN.
- **Windows users:** start with the [Windows guide](docs/en/guides/windows.md), then select your adapter.
- **Existing applications:** see the [API guide](docs/en/guides/usage.md) and [device discovery](docs/en/reference/discovery.md).

## Supported models

| Model | Joints | Default protocol | Address |
| --- | ---: | --- | --- |
| L6 | 6 | Classic CAN | Left `0x28`, right `0x27` |
| O6 | 6 | Classic CAN | Left `0x28`, right `0x27` |
| L20 Lite | 10 | Classic CAN | Left `0x28`, right `0x27` |
| L20 | 16 | Classic CAN | Left `0x28`, right `0x27` |
| L30 | 17 | CAN FD extended frames | NodeID defaults to `1`; HostID defaults to `0` |
| O30 | 20 | HOP / CAN FD standard frames, no BRS | Request ID defaults to `0x001` |

L6 defaults to Classic CAN. Devices with compatible CAN FD firmware can explicitly select the
[optional HOP protocol](docs/en/guides/usage.md#optional-l6-can-fd-protocol).
See [platform and hardware validation scope](docs/en/reference/compatibility.md#platform-and-hardware-validation-scope)
for tested platforms, adapters, and limitations.

## Quick start (Linux)

This example uses **Ubuntu, SocketCAN, and a left L6 with Classic firmware** to build the SDK,
discover the device automatically and read its identity and angles. You need Git, CMake 3.16+, and a C++17 compiler.
The validated Linux baseline is Ubuntu 24.04 / GCC 13.

### 1. Get and build the SDK

Keep both repositories in the same parent directory. The transport revision below matches CI.
Use paired `0.3.x` versions; the transport version must be at least the hand SDK version.

```bash
mkdir sdk-build && cd sdk-build
git clone https://github.com/linker-bot/linkerbot-transport-cpp.git
git -C linkerbot-transport-cpp checkout --detach 0e190032d65ed0b315337b45b3779cf4f650ed68
git clone https://github.com/linker-bot/linkerbot-hand-cpp-sdk.git linkerbot-hand-cpp
cd linkerbot-hand-cpp

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DLINKERBOT_COMM_SOURCE_DIR=../linkerbot-transport-cpp \
  -DLINKERBOT_BUILD_TESTS=OFF -DLINKERBOT_BUILD_TOOLS=OFF \
  -DLINKERBOT_BUILD_EXAMPLES=OFF -DLINKERBOT_COMM_BUILD_TESTS=OFF \
  -DLINKERBOT_COMM_BUILD_EXAMPLES=OFF -DLINKERBOT_ALLOW_DEPENDENCY_DOWNLOADS=OFF
cmake --build build --parallel 4
cmake --install build --prefix "$PWD/install"
```

This installs the SDK and communication library into the local `install/` directory.
Catch2 and vendor development packages are not required. Run the remaining commands from
this repository's root directory.

Hand SDK 0.3.7 requires transport 0.3.4 or newer in the 0.3 minor line,
for source builds, prebuilt packages and installed consumers.

To reuse a prebuilt transport v0.3.4 SDK, configure with
`cmake --preset prebuilt -DLINKERBOT_COMM_PREBUILT_PREFIX=/absolute/path/to/comm-sdk`.
See the [build guide](docs/en/guides/validation.md) for source/prebuilt modes, archive verification and downstream paths.

Release target types follow transport: eight Ubuntu Release SDKs and four Windows x64/ARM64 Debug/Release SDKs. See [release artifacts](docs/en/guides/releases.md).

### 2. Connect the CAN bus

Connect the adapter, device power, and CAN wiring. Adjust the interface name and bitrate
to match your hardware:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 fd off
sudo ip link set can0 up
ip -details link show can0
```

The interface should be `UP` with a bitrate of `1000000`. Discovery uses enabled interfaces;
it does not configure the system CAN interface. For CAN FD models, see
[CAN bus configuration](docs/en/guides/usage.md#connecting-the-can-bus).

### 3. Create a read-only application

Run `mkdir -p demo` and save the following as `demo/main.cpp`. The SDK scans SocketCAN channels,
identifies the model, side, and address, then creates a `HandDevice` without manual device configuration.
This example requires exactly one candidate with complete identity and only reads identity and angles;
it does not send motion targets.

```cpp
#include <exception>
#include <iostream>
#include <linkerbot/hand/discovery.hpp>
#include <stdexcept>

int main() try {
    using namespace linkerbot::hand;
    DiscoveryConfig scan;
    scan.backends = {"socketcan"};
    const auto report = discover_hands(scan);
    const auto &candidate = report.require_one();
    std::cout << "channel=" << candidate.channel().name() << '\n';
    auto hand = candidate.create_hand_device();
    hand->open();
    const auto identity = hand->get_device_info();
    const auto angles = hand->get_angles();
    if (!angles.meta.valid) {
        throw std::runtime_error("Invalid angle response");
    }
    std::cout << "product=" << identity.product_code.value_or("unreported") << '\n';
    std::cout << "angles:";
    for (const auto value : angles.raw) {
        std::cout << ' ' << value;
    }
    std::cout << '\n';
    hand->close();
    return 0;
} catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
}
```

Save the following as `demo/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
project(hand_demo LANGUAGES CXX)
find_package(linkerbot-hand-cpp 0.3 CONFIG REQUIRED)
add_executable(hand_demo main.cpp)
target_compile_features(hand_demo PRIVATE cxx_std_17)
target_link_libraries(hand_demo PRIVATE linkerbot::cpp-sdk)
```

### 4. Build and run

```bash
cmake -S demo -B demo/build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$PWD/install"
cmake --build demo/build --parallel 4
./demo/build/hand_demo
```

On success, the program prints the discovered channel, product code, and raw angles (six values for L6),
closes the connection, and exits with code `0`. If no device is found, check interface status, power, bitrate,
and whether another application is accessing the device. See [device discovery](docs/en/reference/discovery.md)
for multiple candidates or incomplete identity, and the [API guide](docs/en/guides/usage.md#reading-and-control) for manual configuration.

## Next steps

Use the unified `HandDevice` interface for new applications. Model classes such as `l6::L6`
provide model-specific features. See the [API guide](docs/en/guides/usage.md) for commands,
completion checks, polling, and recovery.
Unified setters return a `CommandHandle`: returning means the command was accepted;
inspect the handle to determine its outcome.

To run offline tests, install Catch2 3 and configure a separate build directory from the SDK root:

```bash
cmake -S . -B build-tests -DCMAKE_BUILD_TYPE=Release \
  -DLINKERBOT_BUILD_TESTS=ON -DLINKERBOT_BUILD_TOOLS=OFF \
  -DLINKERBOT_COMM_SOURCE_DIR=../linkerbot-transport-cpp \
  -DLINKERBOT_BUILD_EXAMPLES=OFF \
  -DLINKERBOT_ALLOW_DEPENDENCY_DOWNLOADS=OFF -DLINKERBOT_COMM_BUILD_TESTS=OFF
cmake --build build-tests --parallel 4 --target tests protocol-tests
ctest --test-dir build-tests --output-on-failure
```

Hardware test configuration and execution are documented in the
[hardware test guide](tests/hand/device/hardware/README.zh-CN.md).

## Documentation

- [C++ quality and documentation checks](docs/en/guides/quality.md)
- [Build and validation guide](docs/en/guides/validation.md)
- [Windows guide](docs/en/guides/windows.md): environment, build, installation, and application integration.
- [Windows adapters](docs/en/connections/windows-adapters.md): PCAN, MiniCANFD, and SLCAN configuration and troubleshooting.
- [API guide](docs/en/guides/usage.md): reading, control, background polling, and lifecycle.
- [Model capabilities](docs/en/reference/model-capabilities.md): feature matrix, read/write APIs, raw ranges, and firmware limitations.
- [Device discovery](docs/en/reference/discovery.md): discovery and configuration responsibilities.
- [Platform and hardware validation scope](docs/en/reference/compatibility.md): tested environments and communication paths.
- [Changelog](CHANGELOG.md), [contributing](CONTRIBUTING.md), and [documentation index](docs/en/README.md).

Licensed under the [MIT License](LICENSE). See [NOTICE](NOTICE) for third-party software information.
