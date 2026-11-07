# Python behavior baseline

The reference source is `linkerbot-hand-python-sdk` commit
`3a63e723b9a0046e9a60a58339b9f064ef5b70fd` (source version 2.0.0).
This document records native SDK preparation for a future Python migration.
It does not claim that a Python binding, a hand/arm migration, or hardware
acceptance has been completed. O20 is outside this change.

## Added native capabilities

- L6 unified position Kp/Kd, torque Kd, torque output and limit compensation.
  Configuration replies retain returned bytes, including previous values and
  out-of-range evidence. Kp writes accept 0..200. Other added vectors accept
  six integer bytes. Torque output uses FIFO, including zero-valued transitions
  to position mode; it is never a latest target. Its default total budget is
  300 ms, with a 20 ms pre-send guard and a 20 ms post-dispatch guard, or 150 ms when
  any joint value is zero. These are timing assumptions inherited from Python,
  not new hardware measurements. The total budget remains bounded.
- `save_parameters_confirmed(timeout=200ms)` for Classic L6/O6. It writes eight
  CF bytes and requires exactly `CF 01`. Missing ACKs time out; malformed ACKs
  fail with ProtocolError. `save_parameters()` keeps its previous wire and
  dispatch-only behavior. L6 HOP remains a separate protocol.
- L30 `calibrate(timeout=300ms)` disables, unlocks with the protocol password,
  then requests calibration. Each step requires a successful ACK; no later
  step is sent after failure. It does not re-enable the hand. The timeout is a
  total budget; Python's per-stage windows are not silently substituted.
- `O30Layout::kPythonV25` explicitly selects the Python whole-hand layout.
  Twenty public values map into one 25-byte object. Reads and writes require
  full response coverage. Torque limits and motion time are available through
  this layout; they are not advertised on the default mounted V36 path.
  Configuration ACKs are not published as configuration readback. Independent
  queries populate those caches. Angle ACKs publish actual state, not targets.

The new acknowledged operations share the existing per-device command executor
and the existing Session send path. No Python send queue or second native
physical transmitter is introduced. Native O30 Managers keep their existing
mounted layout; do not mix those layouts on one device.

## Explicit configuration

Set `HandDeviceConfig::o30_layout = O30Layout::kPythonV25` only for endpoints
using that layout. There is no inference from values or a successful short ACK.
Set `empty_polling_plan_disables = true` when an empty explicit polling plan
must disable polling. The existing no-argument `start_polling()` still requests
angles. Discovery runtime configuration carries both options.

Existing enum values, method signatures and native defaults remain available.
New struct members and variant alternatives require rebuilding the SDK and all
consumers; this release does not promise binary compatibility with old headers.
The minimum transport dependency remains v0.3.4 in the 0.3 minor line.

## Verification boundary

Offline checks exercise production codecs and FakeTransport, including returned
configuration bytes, rejection before I/O, explicit persistence ACKs, calibration
failure boundaries, V25 mapping and full-object ACK requirements. Existing
async vectors continue to document differences; passing them does not establish
cross-language equivalence for every operation or model.

Still required before replacing the accepted Python implementation:

- A pinned business configuration and same-input cross-language acceptance
  corpus for every deployed model, unit/range boundary and fault response.
- An explicit policy for admission-to-completion versus per-response timeout,
  strict malformed-response handling, close/in-flight completion, adaptive
  feedback, automatic reconnect and recovery quiet windows. Existing C++
  defaults are retained rather than labeled Python-equivalent.
- A Python binding proving receipt retention on allocation/wrapping failure,
  GIL/callback/destructor/interpreter-exit safety, exception conversion and
  wall-clock timestamp conversion. Native steady timestamps are not Unix time.
- Multi-device hand/arm integration demonstrating one shared channel Runtime
  and independent device lifecycles. The current change does not integrate SDKs.
- Hardware acceptance on the deployed OS, adapter/driver, CAN backend, firmware
  and channel topology. No new hardware validation is claimed here, including
  persistence, calibration, torque-mode transitions and V25 device behavior.

V25 requires an explicit side. Default request IDs are right `0x001` and left `0x002`, matching Python; `address` explicitly overrides them. The native mounted default is unchanged.
