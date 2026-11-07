# Model Capabilities

This reference covers control, telemetry, configuration, and model-specific features in C++ SDK `0.3.4`.
Compare models in the [Capabilities overview](#capabilities-overview), then read the corresponding section:
[L6](#l6), [O6](#o6), [L20 Lite](#l20-lite), [L20](#l20), [L30](#l30), [O30](#o30).

For common connection, command-completion, polling, and recovery patterns, see the [API guide](../guides/usage.md);
for operating systems, adapters, and hardware validation, see [Platform and hardware validation scope](compatibility.md).

## Reading the capability tables

- **Unified API** means `HandDevice`; **model API** means Managers on objects such as `l6::L6` or `o30::O30`.
  An operation missing from the unified API may be available through a model API; a declared Manager method does not guarantee current device support.
- **Query** requests device data; **write-response cache** comes from the latest complete device write response;
  **target cache** only records the SDK's latest sent target. Neither cache constitutes independent configuration readback or live measurement.
- Unless explicitly stated otherwise, ranges and units are raw protocol values. Unified numeric APIs use raw counts;
  `set_percent_angles()` separately uses `0..100`. Do not interpret raw speed, current, or torque directly as RPM, mA, or N·m.
- Table support describes implemented SDK entry points. Actual availability also depends on protocol, firmware, sensors, and capabilities after connection;
  hardware-tested scope is listed separately in [Validation records](#validation-records), not inferred from API existence.
- `—` means unsupported by the current unified API. L6 in all overview tables means **default Classic CAN**;
  CAN FD is an [Explicitly selected optional protocol](#optional-l6-can-fd-protocol), without all Classic features.

## Capabilities overview

### Control commands and parameter settings

This table covers only `HandDevice`. Numeric writes and discrete operations return `CommandHandle`;
for final stages and ACK requirements, see [Command completion and error handling](../guides/usage.md#command-completion-and-error-handling).

| Feature | L6 | O6 | L20 Lite | L20 | L30 | O30 |
| --- | --- | --- | --- | --- | --- | --- |
| Raw/percentage angle settings | Supported | Supported | Supported | Supported | Supported | Supported |
| Speed settings | Supported | Supported | Supported | Supported | Supported | Supported |
| Acceleration settings | — | Supported | Supported | — | — | Supported |
| Current-limit settings | Supported | Supported | Supported | Supported | Supported | Supported |
| Stall time/threshold/hold-current settings | Supported | Supported | Supported | — | — | Supported |
| Clear faults | Supported | — | — | Supported | — | — |
| Enable/disable | — | — | — | — | Supported | — |
| Save parameters | Supported | Supported | — | — | — | — |
| L6 gains / torque output / compensation | Supported | — | — | — | — | — |
| Torque limits / motion time | — | — | — | — | — | Explicit V25 |
| Calibration | — | — | — | — | Supported | — |
| Confirmed parameter save | Supported | Supported | — | — | — | — |

Model Managers provide operations such as L20 Lite configuration saving and O30 Flash saving; see each model's dedicated features.
L6 gains are available through unified methods and native Managers. O30 `pid` is distinct; see [Python behavior baseline](../guides/python-behavior-baseline.md).

### State and configuration reads

This table describes available device information and how it is read. State includes measured angles, speed, current, temperature, and faults;
configuration includes speed, acceleration, and current-limit setpoints. Being able to set a parameter does not imply configuration readback: for example, L20
can set current limits but cannot read their configuration. Device queries, periodic reports, write-response caches, and
SDK target caches are identified separately to distinguish actual feedback from setpoints.

| Feature | L6 | O6 | L20 Lite | L20 | L30 | O30 |
| --- | --- | --- | --- | --- | --- | --- |
| Identity/capabilities | Supported | Supported | Supported | Supported | Supported | Supported |
| Angle feedback | Query | Query | Query | Query | Query/report | Query |
| Measured speed `get_speeds()` | — | — | Query | Query | Query/report | Query |
| Configured speed `get_configured_speeds()` | Write-response cache | Write-response cache | Write-response cache | — | Target cache | Query |
| Configured acceleration | — | Query | Write-response cache | — | — | Query |
| Measured current | Query | Query | Query | Query | Query/report | Query |
| Current-limit configuration | Query | Write-response cache | Write-response cache | — | Target cache | Query |
| Torque | Query output/mode | — | — | — | — | Query actual value |
| Temperature | Query | Query | Query | Query | Query/report | Query |
| Voltage | — | — | — | — | — | Query |
| Faults | Query | Query | Query | Query | Query/report | Query |
| Five-finger tactile data | Fixed matrix | Fixed matrix | Fixed matrix and firmware-dependent scalars | Fixed matrix | Fixed matrix | Sensor-descriptor decoding |
| Stall time/threshold/hold current | Query | Query | Write-response cache | — | — | Query |

All models provide host polling, snapshots, event streams, and recovery, limited to their supported channels.
Snapshots and event streams are not device-initiated reports; configure L30 periodic device reports separately.
Before reading caches, check `meta.valid`, `meta.stale`, `meta.origin`, and connection generation. Configuration caches may be absent before the first write
and must be rebuilt after reconnecting. An API name beginning with `get_*` does not guarantee a query is sent.

## L6

### Default Classic CAN

6 joints, in unified order `thumb_flex, thumb_abd, index, middle, ring, pinky`.
Default addresses are left `0x28`, right `0x27`; the native model entry point is `l6::L6`.

| Feature | Read (HandDevice) | Set (HandDevice) | Raw range and semantics |
| --- | --- | --- | --- |
| Angles | `get_angles()` | `set_angles()` / `set_percent_angles()` | Raw `0..255`; percentage `0..100` |
| Configured speed | `get_configured_speeds()`, write-response cache | `set_speeds()` | `0..255`, step-mode speed setting, not measured RPM |
| Measured current | `get_currents()` | — | `0..255`; protocol conversion `1400/255 mA` per count, absolute value |
| Current limit | `get_current_limits()`, query | `set_current_limits()` | `0..255`; same protocol conversion as above |
| Torque output/mode | `get_torques()` | — | `0..255`; `0` selects position mode, `1..255` torque-mode output; physical torque unconfirmed |
| Stall time | `get_stall_detection_times()` | `set_stall_detection_times()` | Write `1..255`, 10 ms per count; reads retain the device's raw value |
| Stall threshold/hold current | `get_stall_thresholds()` / `get_stall_hold_currents()` | Corresponding `set_stall_*()` | `0..255`; hold current uses the protocol's legacy "stall torque" field; physical scale unconfirmed |
| Temperature/faults | `get_temperatures()` / `get_faults()` | `clear_faults()` | Temperature `1°C` per count; per-joint fault bitmasks |
| Five-finger tactile data | `get_force()` | — | `fn`: raw `5 × 12 × 6` matrix; not directly a value in newtons |
| Save parameters | — | `save_parameters()` | Persistent operation, distinct from ordinary parameter writes |

The native model API provides corresponding operations through `angle`, `speed`, `current`, `current_limit`, `torque`, `temperature`,
`fault`, `force_sensor`, `stall`, `version`, `config`, and other Managers;
Unified gain methods share the native Managers' raw protocol semantics. Kp writes accept `0..200`, both Kd writes `0..255`;
replies retain returned bytes even outside the documented write range. Unified snapshots and streams include these channels and limit compensation.
`set_torques()` is FIFO with explicit mode-transition guards; ACKs are not measured torque.
`save_parameters_confirmed()` requires `CF 01`; legacy `save_parameters()` is dispatch-only.

Dedicated configuration includes `config.set_id()`, `set_transmit_id()`, `set_receive_id()`,
`set_reply_enabled()`, `restore_factory()`, and `save()`. Reconnect at the new address after changing it;
disabling replies affects reads and commands that require responses.

Limitations and usage notes:

- The unified API does not currently provide L6 measured speed; use `get_configured_speeds()` for configured speed.
- Native `stall.set_times()` uses milliseconds; the unified API uses raw 10 ms counts. Do not pass the same values directly.
- Unified acceleration, voltage, enable and disable remain unsupported.
- Confirm device state before adjusting gains, torque mode, compensation, or persistent parameters. Common-feature hardware passes do not validate these advanced operations.

### Optional L6 CAN FD protocol

Set `config.l6_protocol = L6Protocol::kHopV004` only when device firmware supports it and CAN FD is needed.
The default remains `kClassic`; the SDK sends no device protocol-switch command. The native model entry point is `l6::HopL6`.

| Aspect | HOP CAN FD differences from default Classic |
| --- | --- |
| Connection | Standard CAN FD frames, no BRS; default request ID `0x001`, response ID `request_id \| 0x400`; not included in current automatic discovery |
| Angles | Raw `0..65535`, linearly mapped percentages; Classic angle-mapping files unsupported |
| Speed/current limit | Writes and active configured-value queries; raw `0..255` |
| Current/temperature | Raw-count queries; physical conversion unconfirmed |
| Faults | Readable, using separate `l6::HopFaultCode`; do not apply Classic fault enums |
| Tactile data | Decodes sensors 1..5 by metadata; confirm fields/shapes after reads; palm sensor not read |
| Parameter saving | `save_parameters()` supported; native API `config.save()` |
| Unsupported | Acceleration, measured speed, torque targets, gains, limit compensation, stall configuration, voltage, enable/disable, and fault clearing |
| Validation boundary | Offline protocol-vector and FakeTransport coverage; HOP hardware acceptance pending |

For complete connection examples, ACK semantics, and pending hardware checks, see [Optional L6 CAN FD integration](../guides/usage.md#optional-l6-can-fd-protocol).

## O6

6 joints, with the same order as L6; Classic CAN, default left `0x28`, right `0x27`.
The native model entry point is `o6::O6`.

| Feature | Read (HandDevice) | Set (HandDevice) | Raw range and semantics |
| --- | --- | --- | --- |
| Angles | `get_angles()` | `set_angles()` / `set_percent_angles()` | Raw `0..255`; percentage `0..100` |
| Configured speed | `get_configured_speeds()`, write-response cache | `set_speeds()` | `0..255`; documented physical scales conflict |
| Configured acceleration | `get_configured_accelerations()`, query | `set_accelerations()` | `0..254`; `0` encodes special maximum acceleration; physical scales conflict |
| Measured current | `get_currents()` | — | `0..255`; physical scales conflict |
| Current limit | `get_current_limits()`, write-response cache | `set_current_limits()` | `0..255`; not measured current |
| Stall time | `get_stall_detection_times()`, query | `set_stall_detection_times()` | `0..255`, 10 ms per count |
| Stall threshold | `get_stall_thresholds()`, query | `set_stall_thresholds()` | `0..255`; protocol states `1000/255 mA` per count |
| Stall hold current | `get_stall_hold_currents()`, query | `set_stall_hold_currents()` | `0..255`; legacy protocol name and physical meaning conflict |
| Temperature/faults | `get_temperatures()` / `get_faults()` | — | Temperature `1°C` per count; fault bitmasks |
| Five-finger tactile data | `get_force()` | — | `fn`: raw `5 × 10 × 4` matrix |
| Save parameters | — | `save_parameters()` | Native API: `config.save()` |

The native model API includes `angle`, `speed`, `acceleration`, `current`, `current_limit`, `temperature`,
`fault`, `force_sensor`, `stall`, `version`, and `config`.
Dedicated configuration includes `config.set_id()`, `set_device_id()`, `set_transmit_id()`, `restore_factory()`, and `save()`.

Limitations and usage notes:

- **Do not query speed through native `speed.get_blocking()` on V1.3.2 firmware.** The request may set device speed to 0.
  Unified `get_configured_speeds()` reads only write-response caches; it may be unavailable before the first write, and recovery invalidates the cache.
- Unified measured speed, torque, gains, limit compensation, voltage, fault clearing, and enable/disable are unsupported.
- Speed, current limits, and measured current have no reliably established uniform physical conversion; retain raw values and unit confidence.

## L20 Lite

10 joints, Classic CAN, default left `0x28`, right `0x27`; native entry point `l20lite::L20lite`.
Fill vectors in SDK joint order; the "20" in the model name does not mean you should pass 20 values.

| Feature | Read (HandDevice) | Set (HandDevice) | Raw range and semantics |
| --- | --- | --- | --- |
| Angles | `get_angles()` | `set_angles()` / `set_percent_angles()` | Raw `0..255`; percentage `0..100` |
| Measured speed | `get_speeds()`, query | — | `0..255`, independent of configuration cache |
| Configured speed | `get_configured_speeds()`, write-response cache | `set_speeds()` | `0..255` |
| Configured acceleration | `get_configured_accelerations()`, write-response cache | `set_accelerations()` | `0..254` |
| Measured current | `get_currents()`, query | — | `0..255`; not a current-limit setpoint |
| Current limit | `get_current_limits()`, write-response cache | `set_current_limits()` | `0..255` |
| Stall time | `get_stall_detection_times()`, write-response cache | `set_stall_detection_times()` | `0..255`, 10 ms per count |
| Stall threshold/hold current | `get_stall_thresholds()` / `get_stall_hold_currents()`, write-response cache | Corresponding `set_stall_*()` | `0..255`; physical conversion unconfirmed |
| Temperature/faults | `get_temperatures()` / `get_faults()` | — | Temperature `1°C` per count; fault bitmasks |
| Tactile matrix | `get_force({"fn"})` | — | Raw `5 × 12 × 6` matrix |
| Tactile scalars | `get_force({"fns", "fts", "fds", "px"})` | — | Firmware-dependent; `fds=255` means undefined direction |
| Resultant normal force | `get_normal_force_resultant()` | — | Requires available five-finger `fns`; not an arbitrary matrix sum |

The native model API includes `angle`, `speed`, `acceleration`, `current`, `current_limit`, `temperature`,
`fault`, `force_sensor`, `tip_force`, `stall`, `version`, and `config`.
`tip_force.get_blocking()` / `get_finger_blocking()` access scalar channels and remain subject to firmware support.
Dedicated configuration includes `config.set_id()`, `set_device_id()`, `set_transmit_id()`, `set_reply_enabled()`,
`restore_factory()`, and `save()`; unified `save_parameters()` is currently unsupported for this model.

Limitations and usage notes:

- **Validated V4.6.10 firmware does not respond to `0x20..0x23` scalar queries.** After opening, unified capabilities remove
  `fns/fts/fds/px`, keep the `fn` matrix, and reject these scalar and resultant-normal-force calls. Do not substitute pre-open declared capabilities for runtime capabilities.
- Measured-speed queries do not populate or overwrite configured-speed caches. Speed, acceleration, current-limit, and stall-parameter getters
  read the latest complete write responses; rebuild these caches after reconnecting.
- Unified torque, gains, limit compensation, voltage, fault clearing, and enable/disable are unsupported.
- Speed, acceleration, and current limits retain raw counts; physical conversions are unconfirmed.
- Hardware reports retain joint 7/8 positioning and coupled-motion failures; passing a single-joint integration check does not validate positioning accuracy across all joints.

## L20

16 joints, Classic CAN, default left `0x28`, right `0x27`; native entry point `l20::L20`.

| Feature | Read (HandDevice) | Set (HandDevice) | Raw range and semantics |
| --- | --- | --- | --- |
| Angles | `get_angles()` | `set_angles()` / `set_percent_angles()` | Raw `0..255`; percentage `0..100` |
| Measured speed | `get_speeds()`, query | — | `0..255` |
| Configured speed | Configuration readback unsupported | `set_speeds()` | `0..255`; measured speed does not prove configuration application |
| Measured current | `get_currents()`, query | — | `0..255` |
| Current limit | Configuration readback unsupported | `set_current_limits()` | `0..255`; write responses are not current-limit setpoints |
| Temperature/faults | `get_temperatures()` / `get_faults()` | `clear_faults()` | Temperature `1°C` per count; fault bitmasks |
| Five-finger tactile data | `get_force()` | — | `fn`: raw `5 × 12 × 6` matrix |

The native model API includes `angle`, `speed`, `current`, `current_limit`, `temperature`, `fault`,
`force_sensor`, and `version`. `current_limit` and measured `current` share related implementation,
but this does not imply an available current-limit configuration getter.

Limitations and usage notes:

- `get_configured_speeds()` and `get_current_limits()` are unsupported; the SDK does not fabricate configuration caches.
- Angle, speed, and current-limit commands require no separate ACK. Successful sending does not mean configuration readback or mechanical positioning;
  read angle feedback separately when position confirmation is needed.
- Unified acceleration, torque, gains, limit compensation, stall configuration, voltage, parameter saving, and enable/disable are unsupported.
- Hardware reports record a thumb-motor communication fault with unresolved cause; one successful fault clear does not permanently rule out device problems.

## L30

17 joints including the wrist, extended CAN FD frames; default NodeID=`1`, HostID=`0`.
The native entry point is `l30::L30`; fill vectors in J1..J17 order.

| Feature | Read (HandDevice) | Set (HandDevice) | Raw range and semantics |
| --- | --- | --- | --- |
| Angles | `get_angles()`, query or periodic report | `set_angles()` / `set_percent_angles()` | Per-joint ranges, including negatives; percentage `0..100` |
| Measured speed | `get_speeds()`, query or report | — | Signed 16-bit counts; protocol states `0.732 RPM` per count |
| Configured speed | `get_configured_speeds()`, target cache | `set_speeds()` | `1..250`; protocol states `0.732 RPM` per count |
| Measured current | `get_currents()`, query or report | — | Signed 16-bit counts, `6.5 mA` per count; sign indicates output direction |
| Current limit | `get_current_limits()`, target cache | `set_current_limits()` | `60..800`, `6.5 mA` per count |
| Temperature/faults | `get_temperatures()` / `get_faults()`, query or report | — | Temperature `1°C` per count; fault bitmasks |
| Five-finger tactile data | `get_force()`, query | — | `fn`: raw `5 × 12 × 6` matrix; periodic device reports unsupported |
| Enable/disable | — | `enable()` / `disable()` | Enable before motion, disable when finished, and check command results |

Each joint's capabilities declare its angle range and direction: for example, thumb base `0..900`, abduction/adduction axes possibly `-200..200`,
and wrist `-1000..1000`. These are SDK command ranges, not guarantees that hardware reaches every endpoint.
Flexion-joint percentages generally increase opposite to raw values. Read `percent_mapping` in `joints`;
do not convert every axis with the same formula.

The native model API includes `angle`, `speed`, `current`, `current_limit`, `temperature`, `fault`,
`force_sensor`, `version`, `control`, and `report`.

| Model-specific feature | Model API | Conditions and limitations |
| --- | --- | --- |
| Periodic device reports | `report.configure()` / `disable()` / `disable_all()` | Period `20..600000 ms`; only full-joint masks currently supported; current limits/tactile matrices cannot be reported |
| Initial periodic angle reports | Constructor argument `auto_start_periodic` | Pass `false` to disable default reporting; configure separately from host polling |
| Low-level protocol requests | `Client::send_no_ack()` / `request_ack()` / `read()` | Caller owns field, request, and response semantics |

Limitations and usage notes:

- Configured speed/current limits only have SDK target caches, without independent device configuration readback; caches do not prove application by the device.
- Unified acceleration, torque, gains, limit compensation, stall configuration, voltage, fault clearing, and parameter saving are unsupported.

L30 also exposes `calibrate()` and native `control.calibrate()`: disable, unlock, then calibrate, each requiring ACK. It does not re-enable the device; hardware validation is pending.

## O30

Explicit `O30Layout::kPythonV25` adds torque-limit and motion-time reads/writes and uses the compact whole-hand protocol for unified vector operations. Default native mounted V36 behavior remains unchanged. See [Python behavior baseline](../guides/python-behavior-baseline.md).

20 joints, standard HOP/CAN FD frames without BRS; default request ID `0x001`.
The native entry point is `o30::O30`; the SDK's 20-joint vectors map to protocol slots. Do not use the total slot count as vector length.

| Feature | Read (HandDevice) | Set (HandDevice) | Raw range and semantics |
| --- | --- | --- | --- |
| Angles | `get_angles()` | `set_angles()` / `set_percent_angles()` | Raw `0..255`; percentage `0..100`, with some axis directions reversed |
| Measured/configured speed | `get_speeds()` / `get_configured_speeds()`, separate queries | `set_speeds()` | `0..255`; actual values and setpoints are distinct |
| Configured acceleration | `get_configured_accelerations()`, query | `set_accelerations()` | `0..255`; physical conversion unconfirmed |
| Measured current/current limit | `get_currents()` / `get_current_limits()`, separate queries | `set_current_limits()` | `0..255`; MI `0x05` actual values and setpoints |
| Actual torque | `get_torques()` | Torque-target writes unsupported | `0..255`; not directly N·m |
| Stall time/threshold/hold current | Corresponding `get_stall_*()`, queries | Corresponding `set_stall_*()` | `0..255`; physical units unconfirmed; do not apply Classic-model conversions |
| Temperature/voltage | `get_temperatures()` / `get_voltages()` | No unified setter | Raw `0..255`; physical units unconfirmed |
| Faults | `get_faults()` | Fault clearing unsupported | Per-joint bitmasks |
| Tactile data | `get_force()` / selected-field reads | — | Runtime sensor descriptors determine fields, shapes, encodings, and units |

The unified API provides no O30 gains, limit compensation, enable/disable, or parameter saving.
The following Managers provide model-specific features:

| Feature | Model API | Conditions and limitations |
| --- | --- | --- |
| Configuration and communication addresses | `config.get()`, `set_standard_id()`, `set_extended_id()`, `set_control_mode()` | Client follows a successfully written standard ID; lost ACK leaves the address uncertain |
| LED/buzzer | `config.set_led()` / `set_buzzer()` | Configure according to device firmware semantics |
| Restore and Flash saving | `config.restore_config(true)`, `restore_all(true)`, `save_to_flash(true)` | Explicit confirmation required; unified `save_parameters()` is not a substitute |
| PID configuration | `pid.get()` / `pid.set()` | Select gains through `PidTerm`; raw values do not have L6 gain semantics |
| Presets | `preset.get()`, `set_non_trigger()`, `set_raw()` | Indices `1..5`; distinguish non-trigger fields from complete preset writes |
| Joint mapping | `mapping.get_map()`, `set_map()`, `get_length_and_enable()`, `set_length_and_enable()` | Protocol-slot configuration, not SDK percentage mapping; current firmware does not answer `get_map()` |
| Sensor descriptions/data | `sensor.select()`, `get_info()`, `read_channel()`, `read_data()`, `read_all_data()` | Decode by sensor descriptor; retain metadata and raw data |
| Units and ranges | `unit.query()` / `get_range()` | Query the target MI's interpretation; do not assume fixed physical units for every value |
| Communication error history | `comm_error.get_latest_index()` / `get_history()` | Error history is not an immediate ACK for other transactions |
| Engineering services | `service.get_status()`, `read_field()`, `write_field()` and dedicated setters | Side, sensor type, interface, password, UID, etc.; follow confirmed device semantics |

Limitations and usage notes:

- Native `enable.set_masks()` / `enable.get_blocking()` and `fault.set_faults()` explicitly throw unsupported-operation exceptions;
  method existence does not imply enable or fault-clear support, and a valid-joint mask is not enable feedback.
- Native temperature/voltage setters exist, but their configuration purpose is unconfirmed; they are not recommended for routine tuning.
- Tactile fields depend on sensor descriptors; declared `supported=nullopt` means not yet observed, not `true`.
  `get_normal_force_resultant()` requires valid five-finger `fns`; arbitrary sensor configurations may not satisfy that requirement.
- Common-API hardware acceptance does not guarantee mechanical positioning on every joint; reports retain deviations/inconsistencies on joints 4, 8, and 19.
- Flash saving, factory restore, PID, and engineering-field writes are outside current common-feature hardware acceptance.

## Confirming capabilities at runtime

Use `declared_capabilities(config)` offline to inspect the selected model/protocol declarations.
After opening, use `hand.get_capabilities()` for current-connection capabilities, with `get_verified_capabilities()` as needed.
The unified API rejects known unsupported operations before sending; application code should still check capabilities and handle runtime errors.

The following complete program inspects declared O30 capabilities without connecting to hardware:

```cpp
#include <iostream>
#include <linkerbot/hand/api/config.hpp>
#include <linkerbot/hand/capabilities.hpp>

int main() {
    using namespace linkerbot::hand;
    HandDeviceConfig config;
    config.model = Model::kO30;
    config.side = HandSide::kRight;
    const auto caps = declared_capabilities(config);

    std::cout << "joints=" << caps.joints.size() << '\n';
    std::cout << "read torque=" << caps.supports(TelemetrySemantic::kTorques) << '\n';
    std::cout << "write current limit=" << caps.supports(CommandSemantic::kCurrentLimits) << '\n';
    if (const auto *speed = caps.telemetry_channel(TelemetrySemantic::kConfiguredSpeeds)) {
        std::cout << "speed can_query=" << speed->can_query << '\n';
    }
    std::cout << "save parameters=" << caps.supports(DeviceOperation::kSaveParameters) << '\n';
    return 0;
}
```

Here, O30 supports actual-torque reads and current-limit writes, but not torque-target writes or unified parameter saving;
see the O30 Manager table for native Flash-saving support.
Build and link as described in the [API guide](../guides/usage.md).

Capability fields to inspect include:

| Requirement | Capability entry point | Notes |
| --- | --- | --- |
| Can a command be written? | `supports(CommandSemantic)` / `command()` | Torque-target writes are model-Manager-only |
| Can data be read? | `supports(TelemetrySemantic)` / `telemetry_channel()` | Inspect `read_mode`, `source`, `can_query`; distinguish queries from caches |
| Can a discrete operation run? | `supports(DeviceOperation)` / `operations` | Native configuration saving does not automatically imply unified `kSaveParameters` |
| Joint order, range, direction | `joints` / `command()->range_for()` | Follow capability order; L30 ranges are per-joint |
| Raw-value units | `raw_unit`, `raw_scale_note`, `unit_confidence` | `kUnknown` / `kConflicting` do not establish a confirmed physical conversion |
| Tactile fields | `force_fields()` / `force_field()` | Lowercase `fn/fns/ft/fts/fd/fds/px`; confirm dynamic sensors after reading |

`get_capabilities()` itself sends no hardware queries and does not individually self-test all advanced functions.
Offline capability declarations cannot bypass restrictions from device identity, firmware, or sensors.

## Validation records

The API tables describe the current SDK implementation; the following lists device firmware and main boundaries from existing hardware records in the source repository.
For complete platforms, adapters, and test scope, see [Compatibility](compatibility.md);
original per-model reports are in the source repository at `docs/hardware-acceptance/<model>/report.md`.

| Model/protocol | Recorded device firmware | Main validation boundaries |
| --- | --- | --- |
| L6 Classic | `V2.4.5` | Common reads, angles, speed/current limits, and recovery have hardware records; advanced gains, torque output, and persistence are outside this acceptance scope |
| L6 HOP CAN FD (optional) | No hardware acceptance record yet | Offline protocol vectors and FakeTransport do not establish device integration or motion passes |
| O6 | `V1.3.2` | Common APIs and recovery have hardware records; native speed-query configuration changes remain a limitation |
| L20 Lite | `V4.6.10` | Common APIs, caches, and recovery have hardware records; scalar tactile fields unsupported; joint 7/8 motion issues retained |
| L20 | `V3.0.6` | Common APIs and recovery have hardware records; speed/current-limit writes verify sending only, without configuration readback; intermittent faults retained |
| L30 | `V6.1.6` | Common APIs, enable/disable, and recovery have hardware records; one historical skip cannot count as a hardware pass |
| O30 | `1.1.0` | Common commands, configuration readback, and recovery have hardware records; positioning issues retained; advanced configuration/persistence not accepted |

Linux common-feature results and Windows adapter-integration results are recorded separately. Single-joint Windows checks do not cover all joints,
all Release hardware, long-term load, or positioning accuracy. After firmware, sensor, or adapter changes, recheck actual capabilities and the corresponding reports.
