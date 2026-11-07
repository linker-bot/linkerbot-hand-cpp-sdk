# API Guide

For building and initial connection, see the [Linux quick start](../../../README.md#quick-start-linux) or [Windows guide](windows.md).
This guide covers two control strategies, common and advanced features, command completion, background polling, lifecycle, and optional model protocols.
For complete features, read/write behavior, numeric ranges, and firmware limitations by model, see [Model capabilities](../reference/model-capabilities.md).

## Two control strategies

The C++ SDK offers two call paths, both supporting reads and control:

```text
Application → HandDevice → Model adaptation → Model Manager → Transport library → Device
Application → L6 / O6 / L30 / O30 model object → Model Manager → Transport library → Device
```

`HandDevice` selects a model implementation from configuration and unifies parameters, samples, capability checks, and command handles.
Model objects expose Managers such as `angle`, `speed`, and `config` directly, retaining model-specific features and execution behavior.
Managers are usually created and owned by the model object; application code calls members such as `hand.angle`.

| Aspect | Strategy 1: unified `HandDevice` | Strategy 2: model-specific Managers |
| --- | --- | --- |
| Use case | Multiple models in one application; runtime model selection | Fixed model, proprietary configuration, protocol debugging, and detailed control |
| Entry point | Create from a discovery candidate or manually configure `HandDeviceConfig` | Construct `l6::L6`, `l30::L30`, etc. directly |
| Angle control | `hand.set_angles()`, `hand.set_percent_angles()` | For example, `hand.angle.set_angles()` and `set_angles_percent()`; names vary by model |
| Data types | Unified vectors, samples, and snapshots | Model-specific structs and fixed-size arrays |
| Execution result | Unified setters return `CommandHandle` | May return after sending or synchronously wait for a response; check each method |
| Lifecycle | `open()` after construction, `close()` when finished | Usually acquires resources during construction; `stop()` when finished |
| Advanced features | Exposes unified current-limit, stall-protection, and other capabilities | Also provides model-specific addresses, presets, and service fields |

Choose `HandDevice` for a unified application API, or model objects and their Managers for model-specific operations.
Use one control path per device; do not create two objects competing for its requests and responses.
The unified API accepts a factory or shared Session; model classes accept a Transport or shared Session. See [Connections and shared buses](#connections-and-shared-buses).

## Connecting the CAN bus

On Linux, configure SocketCAN before opening a device. Match the interface name and bitrate to the actual device:

| Device | Configuration command after bringing the interface down |
| --- | --- |
| L6, O6, L20 Lite, L20 with Classic firmware | `sudo ip link set can0 type can bitrate 1000000 fd off` |
| L30, O30 | `sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on` |

Run `sudo ip link set can0 down` before configuration, then `sudo ip link set can0 up` afterward.
L30 uses extended FD with NodeID and HostID; O30 uses standard FD without BRS and a HOP request ID.
L6 defaults to Classic CAN. For CAN FD devices, explicitly select the [Optional HOP protocol](#optional-l6-can-fd-protocol)
and configure the FD interface to the device's actual bitrates. Enabling FD on the interface alone does not change the L6 protocol selected by the SDK.
On Windows, use an [Adapter factory](../connections/windows-adapters.md#3-connect-in-a-c-program) or [Device discovery](../reference/discovery.md).

## Strategy 1: unified HandDevice API

### Reading and control

For automatic connection, follow the [README quick start](../../../README.md#quick-start-linux); discovery supplies the model, side, channel, and address.
When deployment configuration is known, fill in `HandDeviceConfig` manually and construct `HandDevice hand(config)`;
the unified examples below use this manual approach. Devices created by discovery and manual configuration share the same control API.

Call `open()` first; the device becomes operable only after the SDK completes identity and baseline-state checks.
Use `get_capabilities()` for model-specific features and ranges; unsupported operations throw `UnsupportedOperationError`.

| Purpose | Common entry points |
| --- | --- |
| Identity and capabilities | `get_device_info()`, `get_identity()`, `get_capabilities()` |
| Active queries | `get_angles()`, `get_currents()`, `get_temperatures()`, `get_faults()`, `get_force()` |
| Angle control | Protocol integers with `set_angles()`; `0..100` with `set_percent_angles()` |
| Cache and state | `get_snapshot()`, `get_health()` |
| Background acquisition | `start_polling()`, `stop_polling()` |
| Event stream | `stream_handle()`, `stop_stream()` |

Vector length must match the device's joint count. `0..100` denotes SDK-normalized position; motion direction depends on the model and angle mapping.
Check sensor samples for `meta.valid` and `meta.stale`; do not interpret raw counts directly as physical units.
Tactile fields are declared by capabilities; missing values use `std::nullopt` and must not be filled with zeros.

### Model differences

- Enable L30 with `enable()` before motion, disable it when finished, and check both operation results.
- Unified `get_configured_speeds()` for Classic L6 and O6 reads cached speed-write responses; it may be unavailable before the first write.
  Neither supports the measured-speed API `get_speeds()`.
  On O6 V1.3.2, do not query configured speeds through native `speed.get_blocking()`; this query may change the device speed.
- L20 lacks configured-speed and current-limit readback. L30 configured-speed caches represent sent targets, not proof of device application.
- L20 Lite configured speeds, current limits, and stall parameters use write-response caches, which must be rebuilt after reconnecting.
  The validated V4.6.10 firmware lacks some tactile scalar queries; use the capabilities after connection.
- Unified Classic L6 stall times use raw counts `1..255`, each representing 10 ms; the native API uses milliseconds. Both reject writes that encode as 0.

### Command completion and error handling

From `0.3.0`, unified `set_*` methods return `CommandHandle` after local validation and queue admission; sending and ACK waits occur in the background.
A returned method does not imply device execution. Native Managers and discrete operations retain their individual execution contracts.

| Situation | How to check |
| --- | --- |
| Invalid parameters, capabilities, device state, or queue capacity | Setter throws synchronously |
| Communication, ACK, or execution-timeout errors after admission | Retrieve the exception through `handle.error()` |
| `wait_for()` returns `false` | This wait timed out; the command was not cancelled. Do not automatically resend. |
| `wait_for()` returns `true` | Command has terminated; continue checking its error and final stage |
| Command without ACK completes | `kDispatched`, with no ACK required |
| Command with ACK completes | `kAcknowledged`; does not imply mechanical positioning |
| `kSuperseded`, `kCancelled`, `kFailed` | Do not count as success |

Angle, speed, and current-limit targets replace pending unsent targets of the same kind; other setters use a bounded FIFO.
`command_timeout` covers post-admission queuing, sending, and ACK; the wait duration of `wait_for()` is a separate budget.

The following reads and resubmits the current L6 raw angles to demonstrate completion checking; running it sends angle targets.
In a real application, confirm the target pose before replacing `target`:

```cpp
#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <linkerbot/hand/api/device.hpp>
#include <stdexcept>
#include <vector>

using namespace linkerbot::hand;
using namespace std::chrono_literals;

bool wait_for_command(const CommandHandle &command, std::chrono::milliseconds timeout) {
    if (!command.wait_for(timeout)) {
        return false;
    }
    if (const auto error = command.error()) {
        std::rethrow_exception(error);
    }
    return command.stage() == CommandStage::kAcknowledged ||
           (command.stage() == CommandStage::kDispatched && command.ack_expectation() == AckExpectation::kNone);
}

int main() try {
    HandDeviceConfig config;
    config.model = Model::kL6;
    config.side = HandSide::kLeft;
    HandDevice hand(config);
    hand.open();
    const auto angles = hand.get_angles();
    if (!angles.meta.valid) {
        throw std::runtime_error("Invalid angle response");
    }
    std::vector<int32_t> target(angles.raw.begin(), angles.raw.end());
    const auto command = hand.set_angles(target);
    const bool completed = wait_for_command(command, 2s);
    if (!completed) {
        std::cerr << "Command did not complete successfully; do not resend automatically\n";
    }
    hand.close();
    return completed ? 0 : 1;
} catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
}
```

## Strategy 2: model-specific Managers

For a left-hand L6 with Classic firmware, construct `l6::L6` directly, read through the `angle` and `version` Managers,
then resubmit the current raw angles. This example sends angle targets and calls `stop()` when finished; no separate `open()` is needed:

```cpp
#include <chrono>
#include <exception>
#include <iostream>
#include <linkerbot/hand/l6/l6.hpp>

int main() try {
    namespace l6 = linkerbot::hand::l6;
    using namespace std::chrono_literals;
    l6::L6 hand(l6::Side::kLeft, "can0");
    const auto info = hand.version.get_verified_device_info(2s);
    const auto angles = hand.angle.get_blocking(500ms);
    std::cout << "angles (%):";
    for (const auto value : angles.angle.to_array()) {
        std::cout << ' ' << value;
    }
    std::cout << '\n';
    hand.angle.set_angles(angles.device_raw);
    hand.stop();
    return 0;
} catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
}
```

Here, `angle.set_angles()` returns `void`, not a unified `CommandHandle`; it returns after sending the targets.
To wait for angle feedback, L6 provides `set_angles_blocking()` and `set_angles_percent_blocking()`,
provided the corresponding device responses are enabled. Waiting for feedback still does not imply mechanical positioning.
`get_verified_device_info()` explicitly verifies L6 model and hand side; native objects do not automatically apply the unified API's full `open()` baseline checks.

### Common operation comparison

The Manager calls below use Classic L6 as an example; for other models, follow their own header declarations:

| Operation | `HandDevice` | L6 Manager |
| --- | --- | --- |
| Actively read angles | `get_angles()` | `angle.get_blocking()` |
| Write raw protocol angles | `set_angles(vector<int32_t>)` | `angle.set_angles(array<uint8_t, 6>)` |
| Write percentage angles | `set_percent_angles(vector<float>)` | `angle.set_angles_percent(l6::Angle)` |
| Read cache | `get_snapshot()` | `get_snapshot()` or `angle.get_snapshot()` |
| Read identity | `get_device_info()` | `version.get_device_info()`; verify with `get_verified_device_info()` |
| Background polling | `start_polling(PollingPlan)` | `start_polling(L6PollingPlan)` or select by `SensorSource` |
| Events and state | `stream_handle()`, `get_health()` | `stream_handle()`, `health()` |

L6 `AngleData::angle` contains percentages; `device_raw` contains the actual device bytes.
Use `device_raw` to preserve the current raw target and avoid applying angle mappings twice.
Native Manager `_blocking` usually means waiting for a response, but some configuration getters only read caches;
for example, L20 Lite stall getters. Do not infer whether a query is sent from the method name alone.

### Manager entry points by model

| Model class | Common Managers | Model-specific or advanced entry points |
| --- | --- | --- |
| `l6::L6` | `angle`, `speed`, `current`, `current_limit`, `force_sensor`, `fault`, `version` | `torque`, `stall`, `config`, `position_kp`, `position_kd`, `torque_kd`, `limit_compensation` |
| `o6::O6` | `angle`, `speed`, `current`, `current_limit`, `force_sensor`, `fault`, `version` | `acceleration`, `stall`, `config` |
| `l20lite::L20lite` | `angle`, `speed`, `current`, `current_limit`, `force_sensor`, `fault`, `version` | `acceleration`, `tip_force`, `stall`, `config` |
| `l20::L20` | `angle`, `speed`, `current`, `current_limit`, `force_sensor`, `fault`, `version` | Speed/current-limit writes and model snapshots |
| `l30::L30` | `angle`, `speed`, `current`, `current_limit`, `temperature`, `force_sensor`, `fault`, `version` | `control`, `report` |
| `o30::O30` | `angle`, `speed`, `acceleration`, `current`, `temperature`, `sensor`, `fault`, `version` | `torque`, `stall`, `pid`, `preset`, `mapping`, `config`, `service`, `unit`, `comm_error`, `voltage` |

A Manager member does not mean the current firmware supports every operation. For the unified API, check `get_capabilities()`;
for model-specific APIs, check `capabilities()` and the protocol documentation for each Manager.

## Advanced features

Advanced features include both unified operations and model-specific Manager operations. L6 below means Classic firmware;
the optional CAN FD protocol uses `l6::HopL6`. See [Optional L6 CAN FD protocol](#optional-l6-can-fd-protocol) for its capabilities;
do not directly apply Classic advanced features to it.

### Features unified through HandDevice

The unified API checks device capabilities; numeric setters also return `CommandHandle`.

| Feature | `HandDevice` entry point | Supported models and corresponding Managers |
| --- | --- | --- |
| Speed and current limit | `set_speeds()`, `set_current_limits()` | All six models; `speed`, `current_limit` (O30 uses `current` for limits) |
| Acceleration | `set_accelerations()` | O6, L20 Lite, O30: `acceleration` |
| Torque reads | `get_torques()` | L6: `torque`; O30: `torque.get_torques()` |
| Stall protection | `set_stall_detection_times()`, `set_stall_thresholds()`, `set_stall_hold_currents()` and corresponding getters | L6, O6, L20 Lite, O30: `stall`; Manager parameter units vary by model |
| Clear faults | `clear_faults()` | L6, L20: `fault.clear_faults()` |
| Enable/disable | `enable()`, `disable()` | L30: `control.enable()`, `control.disable()` |
| Save parameters | `save_parameters()` | L6, O6: `config.save()`; L6 HOP also provides unified saving |

These getters may actively query the device, read write-response caches, or read sent-target caches. Inspect sample fields
`meta.origin`, `meta.valid`, and `meta.stale`, and see [Model differences](#model-differences).
L6 gains, torque output and limit compensation are available through the unified API.
O30 torque-limit and motion-time methods require explicit `O30Layout::kPythonV25`;
native Managers retain their mounted V36 layout. Use `save_parameters_confirmed()`
for Classic L6/O6 persistence ACKs; `save_parameters()` retains its earlier semantics.
L30 `calibrate()` is an ACK-gated disable/unlock/calibrate sequence and never re-enables motion.
See [Python behavior baseline](python-behavior-baseline.md) for timing, compatibility
and hardware-validation boundaries. Rebuild all consumers after updating.

### Using model-specific features through Managers

These operations are not provided by generic `HandDevice` setters. Call the corresponding model object's Manager directly:

| Feature | Model and entry point | Usage notes |
| --- | --- | --- |
| L6 position and torque gains | `set_values()`, `get_blocking()`, and `get_snapshot()` on `position_kp`, `position_kd`, `torque_kd` | Six-joint `std::array<uint8_t, 6>`; Kp writes `0..200`, both Kd writes `0..255` |
| L6 torque targets | `torque.set_torques()` | Raw `0..255`; `0` selects position mode, `1..255` selects torque-mode output |
| L6 limit compensation | `limit_compensation.set_values()`, `get_blocking()`, `get_snapshot()` | Raw `0..255`; physical scale and direction unconfirmed; model API only |
| CAN address configuration | L6: `config.set_id()`, `set_transmit_id()`, `set_receive_id()`; O6/L20 Lite: `set_id()`, `set_device_id()`, `set_transmit_id()` | Reconnect at the device's actual address after changes |
| O30 addresses and control mode | `config.set_standard_id()`, `set_extended_id()`, `set_control_mode()` | Client follows the new standard ID after a successful write; lost ACK leaves address state uncertain |
| Response enablement | L6, L20 Lite: `config.set_reply_enabled()` | Disabling responses may cause blocking calls that require them to time out |
| Factory restore and configuration saving | L6, O6, L20 Lite: `config.restore_factory()`, `save()` | May change addresses and device configuration |
| O30 configuration restore and saving | `config.restore_config(true)`, `restore_all(true)`, `save_to_flash(true)` | API requires explicit confirmation arguments |
| O30 PID | `pid.get()`, `pid.set()` | Select the gain term with `PidTerm` |
| O30 presets | `preset.get()`, `set_non_trigger()`, `set_raw()` | Indices `1..5`; distinguish non-trigger fields from complete raw data |
| O30 joint mapping | `mapping.set_map()`, `set_length_and_enable()` | Changes protocol-slot mapping; not the unified percentage mapping |
| O30 sensor descriptions and raw data | `sensor.select()`, `get_info()`, `read_data()` | Decode by sensor descriptor; retain metadata and raw data |
| O30 engineering services | `service.get_status()`, `read_field()`, `write_field()` and dedicated identity/sensor-type setters | Use confirmed field and firmware semantics |
| O30 units, ranges, and communication diagnostics | `unit.query()`, `get_range()`, `comm_error` | Query physical interpretation and device error history |
| L30 periodic device reports | `report.configure()`, `disable()`, `disable_all()` | Device-initiated periodic sending; separate from host polling; some channels do not support reports |
| Low-level protocol debugging | `l30::Client::send_no_ack()`, `request_ack()`, `read()` | Construct protocol requests directly; caller handles fields and responses |

Saving configuration, restoring factory settings, and writing engineering fields may persistently change the device.
Call these operations only after confirming device state and parameter meanings; do not mix them into default startup reads.

### Advanced Manager example: read and rewrite L6 position gains

The following uses `position_kp` to read six raw gain counts, write the same values back, and wait for a protocol response.
Replace `values` when tuning. Raw-count meanings and write ranges are model-specific, not universal physical units:

```cpp
#include <chrono>
#include <exception>
#include <iostream>
#include <linkerbot/hand/l6/l6.hpp>

int main() try {
    namespace l6 = linkerbot::hand::l6;
    using namespace std::chrono_literals;
    l6::L6 hand(l6::Side::kLeft, "can0");
    const auto info = hand.version.get_verified_device_info(2s);
    const auto gains = hand.position_kp.get_blocking(500ms);
    auto values = gains.values;
    hand.position_kp.set_values(values, 500ms);
    const auto updated = hand.position_kp.get_snapshot();
    if (updated) {
        std::cout << "gain replies: " << updated->sequence << '\n';
    }
    hand.stop();
    return 0;
} catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
}
```

These three Managers' `set_values()` waits synchronously for a complete write response and does not return `CommandHandle`.
Write responses may contain the previous values; `get_snapshot()` retains the actual response. Use `get_blocking()` for explicit readback.
`position_kd` and `torque_kd` work the same way; these parameters do not appear in unified snapshots or event streams.
A protocol response still does not verify the mechanical effect.

### Current limitations

- O30 `enable.set_masks()`, `enable.get_blocking()`, and `fault.set_faults()` throw unsupported-operation exceptions;
  Manager entry points cannot bypass these restrictions.
- Current O30 firmware does not respond to `mapping.get_map()`; do not make this read a required connection step.
- O30 temperature and voltage write APIs exist, but their configuration purpose is unconfirmed; they are not recommended for routine tuning.
- Confirm unit differences before writing: for example, unified L6 stall times use raw 10 ms counts, while its native Manager uses milliseconds.

## Connections and shared buses

Choose the communication backend and control strategy separately. Convenience constructors taking interface strings use SocketCAN on Linux;
on Windows, create an [Adapter factory](../connections/windows-adapters.md#3-connect-in-a-c-program), then inject a transport or Session:

| Scenario | Unified API | Model API, using L6 as an example |
| --- | --- | --- |
| Dedicated adapter connection | `HandDevice(config, factory)` | `L6(Side::kLeft, factory())` |
| Shared bus and routing | `HandDevice(config, session)` | `L6(Side::kLeft, session)` |

Calling `factory()` to create a transport acquires adapter resources; `HandDevice` only stores the factory and defers its use until `open()`.
Create shared Sessions through `transport::Session::create(factory)`; use distinct addresses/routes for different devices.
Stopping one device must not close a Session borrowed by others; externally shared objects must outlive all device use.
Native L30 construction also requires NodeID and HostID; use `auto_start_periodic=false` to disable default periodic reports.

## Background polling and snapshots

Active queries such as `get_angles()` access the device; some configuration getters read caches according to device capabilities.
`get_snapshot()` only reads existing caches and sends no frames.
Background polling updates caches; `wait_for_snapshot()` waits for a cache-version change and may return an unchanged snapshot on timeout.
Cache reads do not refresh sampling timestamps; staleness follows the channel capability's `freshness_ms`.

The following polls angles every 20 ms through the unified API, waits for one cache update, then stops:

```cpp
#include <chrono>
#include <exception>
#include <iostream>
#include <linkerbot/hand/api/device.hpp>

int main() try {
    using namespace linkerbot::hand;
    using namespace std::chrono_literals;
    HandDeviceConfig config;
    config.model = Model::kL6;
    config.side = HandSide::kLeft;
    HandDevice hand(config);
    hand.open();
    const auto previous = hand.get_snapshot();
    hand.start_polling({{TelemetrySemantic::kAngles, 20ms}});
    const auto snapshot = hand.wait_for_snapshot(previous, 500ms);
    if (snapshot.angles && snapshot.angles->meta.valid && !snapshot.angles->meta.stale) {
        std::cout << "angles: " << snapshot.angles->raw.size() << '\n';
    }
    hand.stop_polling();
    hand.close();
    return 0;
} catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
}
```

Model objects also support background acquisition. For example, use `L6PollingPlan` for channel periods and angle-feedback policy,
then call `start_polling(plan)`; `angle.get_snapshot()` returns that Manager's cache.
L30 `report.configure()` controls device-initiated reports, not host polling that queries through the SDK.

For event streams, either path can call `stream_handle()` before acquisition and read the queue from a consumer thread; call
`stop_stream()` when finished. Queues are bounded; slow consumers should check `drop_count()`.
Do not call `close()` or `recover()` from the same SDK runtime's callback; delegate to an external thread.

## Closing and recovery

Explicitly call `close()` on the unified API when finished; use `stop()` on model objects.
Check `get_health()` for the unified API or `health()` for model objects; use their respective `recover()` methods as needed.
Unified recovery rechecks the same device, establishes baseline state, and restores polling without replaying old motion targets or configuration.
Model-object recovery follows each implementation; it does not automatically apply unified identity and state gates.
Unified `close()` and `recover()` may throw on `shutdown_timeout`; callers must handle the failed state.
Public hand exceptions can be caught as `HandError` or `std::runtime_error`.

Recompile applications after SDK upgrades, using headers and libraries from the same build.
For advanced operations, see [Advanced features](#advanced-features); for platform and hardware limits, see [Validation scope](../reference/compatibility.md).

## Model and protocol options

### Optional L6 CAN FD protocol

L6 defaults to Classic CAN. Only when firmware supports HOP v0.0.4 and CAN FD is required
should you explicitly select `L6Protocol::kHopV004`. `l6::L6` and `HandDevice` without `l6_protocol`
still use Classic CAN. This option is not included in automatic discovery and sends no device protocol-switch command.

HOP integration is based on `docs/protocol/L6_协议帧格式0.0.4_CANFD.xlsx` and uses standard CAN FD frames,
no BRS, and 16-bit positions. Only offline protocol-vector and FakeTransport validation is complete; hardware acceptance is pending.

#### Explicit connection

```cpp
#include <linkerbot/hand/api/device.hpp>

int main() {
    using namespace linkerbot::hand;
    HandDeviceConfig config;
    config.model = Model::kL6;
    config.side = HandSide::kRight;
    config.l6_protocol = L6Protocol::kHopV004;
    config.interface_name = "can0";
    config.address = 0x001;

    // No device I/O: reports CAN FD and the protocol-specific 0..65535 range.
    auto capabilities = declared_capabilities(config);
    HandDevice hand(config);
    hand.open(); // identity, angles, faults, then current limits
    const auto angles = hand.get_angles();
    hand.close();
}
```

Configure SocketCAN for FD first. The example nominal bitrate is 1 Mbps; actual bitrates must match the device.
The SDK does not change device or system communication parameters. On Windows, use existing PCAN/MiniCANFD/SLCAN FD
TransportFactory or shared Session support; the adapter must support standard FD frames. Frames have BRS disabled.

The request ID defaults to `0x001`, with range `0..0x3FE`; the response ID is `request_id | 0x400`.
Addresses are independent of hand side; two hands on one bus need different addresses. Explicitly specify hand side to match device identity;
do not use the broadcast address for normal connections.

The source repository provides a read-only example. Enable example builds first (the README quick start disables examples):

```sh
cmake -S . -B build -DLINKERBOT_BUILD_EXAMPLES=ON
cmake --build build --target l6_hop_readonly
./build/examples/l6_hop_readonly --help
./build/examples/l6_hop_readonly can0 right 0x001
```

The native model entry point is `l6::HopL6` in `<linkerbot/hand/l6/hop_l6.hpp>`, with constructors accepting
`(Side, interface, id)`, `(Side, unique_ptr<CANTransport>, id)`,
`(Side, shared_ptr<Session>, id)`. Its `angle` Manager takes `std::array<uint16_t,6>`;
other numeric Managers use `std::array<uint8_t,6>`. Managers offer synchronous calls; unified `HandDevice`
setters continue to return asynchronous `CommandHandle` objects.

#### Capabilities and semantics

| Aspect | Classic L6 (default) | HOP L6 (optional CAN FD) |
| --- | --- | --- |
| Selection | Default / `kClassic` | Explicit `kHopV004` |
| Raw angles | 0..255 | 0..65535, MI 0x20, little-endian |
| Percentage angles | Existing mapping | 0..100 linearly maps to 0..65535 |
| Angle-mapping file | Existing 256-entry mapping | Unsupported; rejected during configuration |
| Tactile data | Fixed legacy matrices | Dynamically decoded from sensor descriptors |
| Speed/current limit | Legacy protocol semantics | Raw u8 counts; write and query setpoints |
| Current/temperature | Existing conversions | Raw u8 counts; physical conversion unconfirmed |
| Faults | Legacy FaultCode | Stall, overcurrent, overtemperature, exception, offline; mask 0xCF |

The unified six-joint order remains `thumb_flex, thumb_abd, index, middle, ring, pinky`;
HOP protocol slots are `[10,5,11,12,13,14]`. Position writes cover only the two installed ranges, leaving other joints untouched.
Commands are acknowledged only after both frames are sent and both complete responses arrive. ACK means protocol completion, not mechanical positioning.
Partial failures neither roll back nor retry motion targets.

Use `hand.get_capabilities()` or `declared_capabilities(config)` for protocol-specific capabilities;
the original `declared_capabilities(Model::kL6, side)` always means Classic L6.
Queries before opening, after closing, and after recovery all retain the explicit selection.

The current HOP implementation supports angle and speed writes/queries, current-limit writes/queries, current, temperature, faults, five-finger tactile data,
identity, and parameter saving. These telemetry channels support polling. Enable/disable, fault clearing, PID, stall configuration, and control-mode switching
are rejected before sending. `l6::HopFaultCode` is separate from legacy `FaultCode`; `FaultSample`
retains the full raw value, defined bits, and reserved bits.

Tactile acquisition selects sensors 1..5 individually and reads metadata and all data; snapshots/events publish only after a complete successful cycle.
Fields support little-endian U8/U16/U32, I8/I16/I32, and F32 decoding; `FNS/FTS/FDS` normalize to
`FnS/FtS/FdS`. `Rsv` retains raw bytes without generating force values. Missing sensors are marked unavailable;
if all are missing, the result is unsupported. Descriptor mismatches, changed selections, invalid floats, or validity-mask modes prevent publication.
The palm sensor is not currently read.

`stop()` closes its own runtime; borrowed shared Sessions remain available.
Unified `recover()` verifies the same physical device again, reads baseline state, and restores polling without replaying targets.
Transaction timeout retains the last complete sample and quarantines the affected response domain until explicit recovery.

#### Interpretations awaiting hardware confirmation

- BRS is disabled by default; nominal bitrate is 1 Mbps. Actual connections must match device configuration.
- Percentages increase with raw values; HOP firmware physical directions and endpoints are not marked as hardware-tested.
- Write responses must match MI, RTS, SI, and complete EDL; measured-state replies need not equal write targets.
- Each of three tactile data regions is interpreted as 255 bytes, totaling at most 765 bytes. Long objects use separate requests of up to 61 bytes sharing one total timeout.
- MI 0x4F is an error-history bitmask; do not assume it is an immediate error ACK for other transactions.
- Record document version 0.0.4 separately from the protocol version reported by the device; a nonempty device version `1.0.0` is not rejected merely because it differs.

Hardware acceptance must record identity, address, communication settings, complete ACKs, the three-region layout, and sensor descriptors;
then verify small single-joint motions, direction, and return to the original position. Passing offline tests does not mean hardware acceptance is complete.

### Inspect physical command delivery

`CommandHandle::send_results()` returns a snapshot of terminal physical batch
receipts observed for unified asynchronous `set_*` commands, in observation order.
Legacy synchronous device operations do not collect these receipts. Composite commands can
have several receipts. Inspect each receipt's `outcome`, `sent_frames`,
`total_frames`, `current_frame_uncertain`, and `generation`. An empty list means
no terminal receipt has been observed; it does not prove zero delivery.

Logical completion and physical delivery are independent. A driver write already
in progress can complete after a command fails due to timeout, close, or recovery.
Later snapshots retain that receipt without changing the existing `stage()`,
`error()`, `sent_frames()`, or transition history. `wait()` and `wait_for()` wait
for logical completion, not for collection of late physical receipts. Do not
automatically replay motion or configuration writes solely because of a timeout.

Old handles retain old-generation evidence after recovery. Read fresh state with
new requests and check cache validity and connection generation.
