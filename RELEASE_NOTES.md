# Release notes

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
