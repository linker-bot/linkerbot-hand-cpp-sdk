# Python behavior baseline

The reference source is `linkerbot-hand-python-sdk` commit
`3a63e723b9a0046e9a60a58339b9f064ef5b70fd` (source version 2.0.0).
This document records native SDK preparation for a future Python migration.
It does not claim that a Python binding, a hand/arm migration, or hardware
acceptance has been completed. O20 is outside this change.

## Added native capabilities

- L6 unified position Kp/Kd, torque Kd, torque output and limit compensation.
  New L6 control setters and the added configuration getters reject malformed matching vector lengths. Existing native Manager defaults remain unchanged. Configuration replies retain returned bytes, including previous values and
  out-of-range evidence. Kp writes accept 0..200. Other added vectors accept
  six integer bytes. Torque output uses FIFO, including zero-valued transitions
  to position mode; it is never a latest target. Its default total budget is
  300 ms, with a 20 ms endpoint-local pre-send guard and a 20 ms post-dispatch guard, or 150 ms when
  any joint value is zero. These are timing assumptions inherited from Python,
  not new hardware measurements. An ACK can complete before the post-send guard ends; subsequent endpoint transactions wait. The guard is retained after a failed ACK and does not reserve the channel transmitter. Existing admission budgets still apply to subsequent commands.
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

`submit_operation(operation, timeout)` admits supported operations asynchronously through the same FIFO, including ACK-gated enable/disable and dispatch-only clear-faults. Identity reads are rejected before admission; use `get_device_info()`. This entry point lets a future binding avoid a separate Python executor while legacy operation methods keep their synchronous behavior.

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
- Deployed-configuration acceptance for the opt-in transaction policy below,
  plus automatic reconnect parity and the remaining deployment-specific behavior corpus. Native defaults remain
  separate from the Python policy; passing offline cases is not complete parity.
- A Python binding proving receipt retention on allocation/wrapping failure,
  GIL/callback/destructor/interpreter-exit safety, exception conversion and
  wall-clock timestamp conversion. Native steady timestamps are not Unix time.
- Multi-device hand/arm integration demonstrating one shared channel Runtime
  and independent device lifecycles. The current change does not integrate SDKs.
- Hardware acceptance on the deployed OS, adapter/driver, CAN backend, firmware
  and channel topology. No new hardware validation is claimed here, including
  persistence, calibration, torque-mode transitions and V25 device behavior.

V25 requires an explicit side. Default request IDs are right `0x001` and left `0x002`, matching Python; `address` explicitly overrides them. The native mounted default is unchanged.

Classic protocol constructors explicitly select classic CAN, including when their Session uses an FD-capable adapter. Offline wire vectors resolve flags using the FD-backend default; mixed-device hardware acceptance remains pending.

## First compatibility patch (0.3.5)

`refresh_device_info(timeout)` reads and validates identity in the current generation without cancelling commands or restarting polling. Timeout and protocol errors leave admission unchanged. Identity mismatch or transport loss faults this facade, cancels outstanding work and invalidates its readable cache. The earlier `verify_identity()` and `get_device_info()` lifecycle semantics remain unchanged. Capability observations remain attached to their current generation.

`get_cached_snapshot()` is a zero-I/O cache read available before open, during recovery/fault/close and after close. Supported initial vector samples have their declared length and `valid=false`; zero-filled raw storage is unavailable data, not a measurement. Unsupported channels remain absent; descriptor-dependent tactile payloads are not synthesized. On generation loss, successfully retained raw evidence remains with unavailable metadata, joint projections are cleared and identity is omitted because native `DeviceIdentity` has no validity metadata. A transition attempts to retain the latest native cache; capture failure cannot prevent shutdown, so earlier retained evidence may remain. Failed initialization never exposes a partial candidate baseline.

The new cache uses local successful-open/recovery generations and a monotonic cache revision. These are independent of the retained native-generation policy of `get_snapshot()` and health. Bindings must not mix the two policies without explicit mapping. Physical Session/transport loss or a changed physical generation invalidates the retained cache until this device successfully initializes again. Legacy facade readiness/health policies remain separate. Repeated reads do not advance the revision without a visible change. Existing `get_snapshot()` still requires Ready. Feedback-filtered waiting and full per-channel Python sequence parity remain outside this patch.

Unified polling accepts measured speeds on L20Lite/L20/L30/O30 and voltages on O30. V25 torque-limit and motion-time freshness is 100 ms and 300 ms; equality at the threshold is still fresh. Mounted V36 defaults are unchanged. These capabilities use the existing native polling and Session transmitter; no Python send scheduler is introduced. Hand 0.3.5 retains the explicit transport 0.3.4 minimum independently of the hand patch number.

## Second compatibility patch (0.3.6)

Set `runtime_behavior = RuntimeBehavior::kPython` explicitly. This policy supports Classic L6, O6, L20Lite, L20, L30 and `O30Layout::kPythonV25`. Other native protocol selections keep their default policy; an incompatible explicit combination is rejected before transport construction. Discovery carries the same runtime fields. There is no Python binding or hand/arm integration in this patch.

Each device coordinates equivalent active reads, identity refreshes and polls. `read_waiter_capacity` defaults to 256 active callers; polls do not consume this capacity. Each active read keeps its own absolute caller deadline. Leaving a wait does not cancel a shared transaction. Unclaimed work with no remaining reader or poll demand is discarded; polling demand survives its internal wait timeout until the poll is cancelled. A caller arriving after publication needs a subsequent sample. Closing or recovering invalidates old waiters and prevents their results from serving a new generation. This is a device intent coordinator; the existing Session owns physical sending. Pending latest setters remain replaceable while another read owns the transaction lane. Results and errors published after an individual caller deadline remain timeouts for that caller, including when publication delays its wakeup. Admission and receipt retention are serialized with close. Already claimed physical batches, including reads, drain without truncation for both ACK policies; required-ACK commands still fail closed. Reopen and recovery resume the endpoint after draining. Retained closed profile runtimes cannot cancel the new device generation.

In this policy, setter admission does not start a response timer. Required response windows start at the physical batch receipt, and chained operations have separate stage windows. `response_timeout = std::nullopt` uses ordinary protocol windows; an explicit positive value overrides them. Identity uses `identity_timeout` independently, matching the Python initializer and refresh path. Its native default remains 1000 ms; explicitly use 200 ms to match the pinned Python default. Neither setting changes getter caller waits. Legacy Manager wait deadlines do not shorten the selected response-window policy. Classic identity sends C0/C1/C2/C4 in one batch with 5 ms gaps and one response window. Matching malformed responses fail with `ProtocolError`; unrelated frames remain unmatched. A sent request exhausting its required response window raises `ResponseTimeoutError`, a `TimeoutError` subtype. Failure retains successful cache evidence and fences late replies for the protocol guard. An early reply or error cannot precede the physical delivery decision. Native absolute timeout behavior remains the default.

Close rejects unclaimed work with `ClosedError`. A claimed dispatch-only batch may finish `Dispatched` after close starts, according to its actual physical receipt. Failed physical delivery remains failure. A command requiring an ACK fails closed when its generation ends; delivery alone cannot acknowledge it. Handle receipts remain available and old commands are never replayed. A C++ handle wait timeout still returns `false`; a future Python binding must translate that result to Python's `TimeoutError`.

`recover_for(timeout)` separates a caller's wait from the shared recovery execution. Concurrent callers join the running cycle. Preparation must drain within the first caller's preparation budget and `shutdown_timeout`; failed preparation cannot later reconnect. Once recovery starts, a caller timeout does not cancel it. `recovery_timeout` defaults to 5 s; `recovery_attempts` remains the existing explicit setting (native default 1, set 3 to match Python's default retry count). `recovery_quiet` defaults to 200 ms. Only response-window loss permits another initialization attempt. Quiet time is at least the failed stage's response window and late guard, and endpoint traffic restarts it without extending the total execution deadline. Initialization retries reuse the recovered transport. Identity, protocol, transport and total-budget failures stop the cycle. A subsequent explicit recovery can restore Ready from Faulted after identity and baseline verification; polling is restored only after initialization. No targets or discrete commands are replayed.

Configure capacity, command rate, protocol layout and timeout overrides from the accepted business configuration rather than inferring them from this policy name. Automatic reconnect, adaptive feedback equivalence, per-channel sequence mapping, Python allocation/GIL/exit behavior and deployed hardware acceptance remain separate work. The source pin above is a comparison reference, not evidence that the deployed Python build used that exact commit.

For example, these explicit runtime fields match the pinned reference defaults;
confirm the deployed business configuration before using them:

```cpp
using namespace linkerbot::hand;
using namespace std::chrono_literals;
HandRuntimeConfig runtime;
runtime.runtime_behavior = RuntimeBehavior::kPython;
runtime.command_rate_hz = 250.0;
runtime.discrete_queue_capacity = 16;
runtime.read_waiter_capacity = 256;
runtime.identity_timeout = 200ms;
runtime.response_timeout = std::nullopt;
runtime.shutdown_timeout = 2s;
runtime.recovery_attempts = 3;
runtime.recovery_timeout = 5s;
runtime.recovery_quiet = 200ms;
```


## Third compatibility patch (0.3.7)

The following additions apply to the explicit Python runtime policy; native defaults remain unchanged.

- Complete telemetry publications advance their own channel sequence even when no caller reads a snapshot. A coherent publication view supplies sample sequence, generation, receive time and response/report/command delivery metadata. Global snapshot revisions also include identity and observed capability changes. Repeated cache reads do not create acquisitions.
- `DeviceIdentity::meta` carries its own identity-read sequence and monotonic collection metadata. A successful `refresh_device_info()` advances identity without advancing telemetry channels. Retained caches preserve identity evidence with unavailable metadata after generation loss. This does not add the remaining Python raw/version detail fields.
- `wait_for_cached_snapshot(previous, timeout, wake_on_command_feedback = true)` reads retained state and performs no device I/O. Filtering suppresses command-only wakeups, not cache retention. A non-command update returns its coherent revision, including command data that preceded that revision. Timeout returns the current cache; close or recovery releases old-generation waiters. The original wait signature remains unambiguous; native wait behavior is retained.
- Matching Classic tactile rows validate length and compact/nibble row indices before assembly. Conflicting repeats fail the transaction without replacing the previous complete sample. Complete identical duplicates do not fabricate a new acquisition. Unrelated routes/types/opcodes stay unmatched.
- Force aggregation replaces all fields belonging to the completed query while retaining other queries and their original collection metadata. Scalar-only aggregate raw data follows the latest scalar query. L20Lite Python-policy scalar getters issue only their selected category query (0x20..0x23); the native aggregate query remains available on the native path.
- Firmware-optional observations belong to one device generation. Only a physically sent optional query that has not succeeded in that generation can degrade. Three timeout/protocol failures disable that query; an explicit unsupported response can disable it immediately. Transport failure and expired unsent callers do not establish unsupported firmware. Capability queries, force status and polling reflect the observation; other channels remain usable. Explicit recovery clears those observations. Required initialization remains required.
- `PollingChannel::angle_feedback` selects `kQueryOnly`, `kCommandFeedbackOnly` or `kAdaptive`; the Python-policy default is adaptive. Command-only mode requires a declared verified current-state response and never repeats motion targets. Query-only mode does not use command feedback to postpone queries. Adaptive mode postpones only redundant angle queries after verified feedback. Feedback, completed-read, timeout and missed-deadline metrics remain distinct. Reapplying an equivalent plan preserves deadlines and metrics. All physical frames still pass through the existing Session.

Python bindings must map retained snapshot and filtered wait APIs explicitly. GIL, callback, destructor/interpreter-exit and allocation-failure proof, wall-clock conversion and shared hand/arm channel integration are not provided by this native patch. Hardware acceptance remains separate. O20 and automatic reconnect are not included.
