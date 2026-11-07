# Release notes

## 0.3.6

- Add an explicit `RuntimeBehavior::kPython` policy for the unified facade. The native policy remains the default. Classic L6, O6, L20Lite, L20, L30 and explicit O30 V25 are in scope; L6 HOP and mounted O30 retain their existing native path.
- Coalesce equivalent active reads, identity refreshes and polling by device. Callers retain independent wait deadlines and bounded admission. A result or error published after a caller deadline does not replace that caller's timeout. Unclaimed reads without remaining demand are discarded; polling demand remains independent of internal waiter timeouts. A timed-out reader cannot cancel another reader's transaction. Latest setters remain replaceable until the device transaction lane is available. The existing transport Session remains the physical sender.
- Start required response windows after physical batch completion, retain a separate recovery execution deadline, and buffer early replies/errors until delivery is known. Classic identity uses one four-request batch with the Python 5 ms spacing. Protocol defaults and an explicit ordinary response-window override are available without changing native timeout defaults. Identity windows use `identity_timeout` independently; legacy Manager waits do not shorten the selected policy.
- Reject matching malformed Classic vector, bank and identity responses in the opt-in path. Preserve the last successful cache and apply finite late-response guards. Unrelated routes and opcodes remain unmatched.
- Preserve the physical outcome of a claimed dispatch-only command during close. Unclaimed commands fail closed; a required-ACK command cannot become successful from delivery alone. No command is replayed on reopen or recovery.
- Let concurrent callers join an explicit recovery cycle, with independent waits. Retry response-window loss only, after an endpoint quiet period, without reopening the transport for every initialization attempt. Protocol, identity and transport failures stop the cycle. Failed preparation cannot start a later reconnect; explicit recovery is also allowed from Faulted.

The minimum and paired transport remain v0.3.4 (`0e190032d65ed0b315337b45b3779cf4f650ed68`). Rebuild the SDK and downstream consumers with matching headers and static libraries. Existing signatures and native default behavior remain available; binary compatibility with earlier headers is not claimed.

Validation is offline unless a separate hardware record states otherwise. FakeTransport regressions and pinned Python behavior vectors cover the changed transaction and lifecycle paths. Native/source/prebuilt/installed consumers, sanitizers, static analysis and the twelve-target package matrix remain release gates. No new physical-device acceptance is claimed. Deployed adapters, drivers, firmware, timestamps and mixed-device channel behavior still require hardware acceptance. O20, Python bindings, automatic reconnect/adaptive feedback parity and hand/arm integration are not included.

## 0.3.5

- Add unified polling for measured speeds on L20Lite, L20, L30 and O30, and voltages on O30, using the existing native polling and Session send path.
- Correct explicit O30 V25 torque-limit and motion-time freshness to 100 ms and 300 ms. Apply these bounds to projected snapshots and health. Mounted V36 remains unchanged.
- Add `refresh_device_info()` for an identity read that preserves queued commands, polling and the current generation on success. Timeout/protocol failures leave readiness unchanged; identity mismatch or transport loss faults admission and invalidates the readable cache. Existing `get_device_info()` and `verify_identity()` retain their earlier lifecycle behavior.
- Add `get_cached_snapshot()` for zero-I/O reads in all lifecycle states. Initial supported vectors are shaped and invalid. Generation loss retains raw evidence with invalid metadata, clears joint projections and omits identity. Physical connection loss or a changed shared Session generation cannot revalidate old evidence. Successful open/recovery replaces the cache; failed initialization cannot publish a partial baseline. Existing `get_snapshot()` still requires Ready.
- Keep hand and transport patch versions independent within the supported minor line. Source, prebuilt, installed-consumer and release checks retain the explicit transport 0.3.4 minimum.

The paired transport remains v0.3.4 (`0e190032d65ed0b315337b45b3779cf4f650ed68`). Rebuild the SDK and downstream consumers with matching headers and static libraries. Existing C++ signatures and native defaults remain available; no new binary compatibility guarantee is made.

Validation covers native builds, FakeTransport lifecycle/concurrency regressions, source/prebuilt/installed consumers, sanitizers, static analysis and the twelve-target package matrix. No new physical-device acceptance is claimed. The new polling channels, identity refresh, recovery/cache transitions and deployed adapter/driver/firmware combinations require separate hardware validation. Existing hardware records retain their original scope. O20, Python bindings, hand/arm integration and complete Python runtime equivalence are outside this patch.

## 0.3.4

- Explicitly mark Classic L6/O6/L20Lite/L20 protocol frames as classic CAN so an FD-capable shared Session cannot promote them to FD through backend defaults. Existing classic-link behavior is retained.
- Add unified L6 position Kp/Kd, torque Kd, torque-output and limit-compensation commands and samples. Keep returned configuration bytes rather than substituting requested values. Torque output uses FIFO and bounded mode-transition guards derived from the Python reference.
- Add `submit_operation()` for nonblocking operation admission through the existing bounded FIFO; retain the earlier synchronous operation methods.
- Add asynchronous `save_parameters_confirmed()` for Classic L6/O6. Require the explicit `CF 01` reply. Keep the existing `save_parameters()` behavior unchanged.
- Add L30 calibration as an ACK-gated disable, unlock and calibrate sequence. A failed stage prevents later writes; calibration does not re-enable the hand.
- Add an explicit O30 Python V25 layout for compact whole-hand commands and telemetry, including torque limits and motion time. Keep the default mounted V36 path and native Managers unchanged. Require complete response coverage and distinguish configuration queries from write ACKs.
- Add an explicit empty-polling-plan disable option, propagated through discovery. Keep existing no-argument polling defaults.
- Add same-input outgoing-frame vectors from Python SDK commit `3a63e723b9a0046e9a60a58339b9f064ef5b70fd`, FakeTransport failure-boundary regressions and installed-consumer coverage. Record remaining migration requirements in the Python behavior baseline guide. O20 and Python hand/arm integration are not included.

The minimum transport dependency and bundled baseline remain v0.3.4 (`0e190032d65ed0b315337b45b3779cf4f650ed68`). Rebuild the SDK and every downstream consumer with matching headers and libraries. New public struct fields and variant alternatives do not preserve binary compatibility with older builds. Existing C++ method signatures, enum values and default protocol paths remain available.

Verification uses native builds, offline regressions, package consumers, sanitizers and static analysis. This release does not add physical-device acceptance. Persistence, calibration, torque-mode transitions, V25 behavior, deployment adapter/driver/firmware combinations and long-duration multi-device operation remain subject to hardware validation. Existing hardware records retain their original scope. The added interfaces do not establish complete Python behavioral equivalence or Python binding safety.

## 0.3.3

- Require transport 0.3.4 or newer in the 0.3 minor line for source builds, parent-provided targets, prebuilt selection and installed CMake consumers. Keep the version floor in one CMake definition, independent of the hand patch version. Existing same-minor compatibility and the requirement not to use a transport patch older than hand are retained.
- Reject an older transport target already loaded by the parent of an installed consumer, even when a compatible package file is found.
- Apply the same floor to release validation and package collection. Add source/prebuilt/parent-target rejection tests and installed-package minimum, older-patch and newer-patch regressions using the exported package template.
- Retain existing C++ API signatures, runtime defaults, protocol behavior and scheduling. The supported legacy transport package and target aliases remain available.

Rebuild hand and downstream applications with matching transport headers and libraries. The release bundles transport v0.3.4 (`0e190032d65ed0b315337b45b3779cf4f650ed68`); do not mix the new headers with earlier objects or libraries. Binary ABI compatibility with earlier transport SDKs is not promised.

Native SDK builds, offline regressions, installed consumers, sanitizers and static analysis do not establish new physical-device acceptance. No new hardware/firmware/driver combination, physical timing or device soak is qualified. Vendor drivers are not bundled. Existing hardware records retain their original scope.

## 0.3.2

- Run the compiler-configuration compatibility matrix as a separate required CTest item, with its own process budget; retain the fast package audit and device-test limits.
- Read the current SDK version in package-compatibility validation instead of assuming 0.3.0.
- Use transport v0.3.4 (`0e190032d65ed0b315337b45b3779cf4f650ed68`) as the source and prebuilt validation baseline. The bundled SDK pairs these exact sources; standalone consumers retain the existing same-minor, minimum-version package policy.
- Give the interstage deadline-selection regression sufficient host-scheduling headroom while retaining its delayed reply beyond the angle budget. Production deadlines are unchanged.
- Update build, installation and release documentation. Device protocols, public hand API signatures, runtime defaults and scheduling are unchanged by this dependency update.

Rebuild the hand library and downstream applications with matching transport headers and libraries. Transport v0.3.4 adds public structure fields; do not mix old objects or libraries with the new headers. Binary ABI compatibility with earlier transport SDKs is not promised.

The release matrix builds and consumes native Ubuntu 20.04/22.04/24.04/26.04 x86_64/ARM64 Release packages and Windows x64/ARM64 Debug/Release packages. Verify the release manifest and checksums and select the matching platform and runtime. Vendor drivers are not bundled.

Offline tests, installed consumers, sanitizers and static analysis do not establish new physical-device acceptance. No new hand/arm hardware combination, driver/firmware combination, physical timing or long-duration device run is qualified by this release. Existing hardware records retain their original scope.

The v0.3.1 tag was created during release validation, which subsequently found a stale package-test version. No v0.3.1 Release was published; its tag is retained and this release uses a new immutable tag.
