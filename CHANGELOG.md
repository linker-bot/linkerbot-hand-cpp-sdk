# Changelog

[English](CHANGELOG.md) | [简体中文](CHANGELOG.zh-CN.md)

## 0.3.10

- Correct Python-policy missing-command-feedback health without changing physical dispatch outcomes or Native defaults.
- Include the polling, logical latency and force-field health updates from the unpublished v0.3.9 tag.
- Keep transport v0.3.4 minimum, rebuild requirements and separate hardware acceptance.

## 0.3.9

Tagged but not published; superseded for publication by 0.3.10.

- Align explicit Python-policy polling counters and logical transaction latency with the Python reference, including nearest-even percentiles and a bounded device-lifetime window.
- Add force-field health projection and shared-query interval resolution; preserve Native defaults and the transport v0.3.4 minimum.
- Rebuild SDK and consumers. No new hardware acceptance is claimed; see [release notes](RELEASE_NOTES.md).

## 0.3.8

- Correct bounded poll shutdown, logical health generations and final optional-query errors in the explicit Python policy.
- Add Python-only routed receive capacity configuration, discovery propagation, overflow and installed-consumer regressions. Native defaults and transport v0.3.4 minimum remain unchanged.
- Rebuild SDK and consumers. No new hardware acceptance is claimed; see [release notes](RELEASE_NOTES.md).

## 0.3.7

- Add publication-based channel sequences, identity sample metadata, filtered retained-cache waits, firmware-optional query degradation and unified angle feedback policies to the explicit Python runtime policy.
- Correct matching tactile response validation and force aggregation; add independent L20Lite scalar queries in that policy.
- Preserve native defaults and the transport v0.3.4 minimum. Rebuild SDK and consumers. Hardware acceptance remains separate; see [release notes](RELEASE_NOTES.md).

## 0.3.6

- Add an opt-in Python runtime policy for coalesced reads, response-stage windows, strict matching responses, close terminal outcomes and bounded explicit recovery.
- Preserve native defaults and the transport v0.3.4 minimum. Rebuild consumers. See [release notes](RELEASE_NOTES.md) and the [behavior baseline](docs/en/guides/python-behavior-baseline.md) for validation limits.

## 0.3.5

- Add missing unified speed/voltage polling, correct O30 V25 freshness, and add non-destructive identity refresh and lifecycle-readable caches.
- Cover lifecycle failure boundaries, concurrent refresh/close, installed consumers and independent hand/transport patch versions. Transport minimum remains v0.3.4; no new hardware acceptance is claimed.
- See [release notes](RELEASE_NOTES.md) for scope and [Python behavior baseline](docs/en/guides/python-behavior-baseline.md) for remaining differences.

## 0.3.4

- Add unified L6 gains, torque output and limit compensation; explicit Classic save ACKs; L30 calibration; and an opt-in O30 V25 path with torque-limit and motion-time support.
- Add pinned Python outgoing-frame vectors, failure-boundary regressions and installed-consumer coverage. Retain existing native defaults and transport v0.3.4. Rebuild downstream consumers; new hardware acceptance is not claimed.
- See [release notes](RELEASE_NOTES.md) and [Python behavior baseline](docs/en/guides/python-behavior-baseline.md) for scope and remaining migration requirements.

## 0.3.0 (unreleased)

### Build and platform compatibility

- Add explicit source/prebuilt transport providers and separate CMake presets.
- Align SDK release targets with transport: eight Ubuntu Release archives and four
  Windows x64/ARM64 Debug/Release archives, with build identities and complete-set validation.
- Fix unbounded runtime request, polling-stop and fault waits on Focal's older
  libstdc++; retain finite deadlines and error propagation.
- Update paired transport sources and prebuilt downloads to v0.3.3, including the
  Focal receipt-wait fix. Accept its renamed package metadata in release validation;
  retain the supported comm CMake compatibility interface.
- Include the transport Focal syscall fault-injection test fix in the source pin.
- Give native PCAN fixture discovery enough response time for Windows scheduling
  and report FD discovery failures before asserting identity.

### Documentation and quality tooling

- Make English the default documentation with Chinese counterparts, reciprocal
  navigation and compatibility aliases for the previous English landing pages.
- Add C++17 contribution guidance, validation/quality instructions and English
  summaries of unchanged historical hardware evidence.
- Add full-mode C++ Report Card analysis with an explicit completeness gate and
  preserved findings, plus documentation and packaging regression checks.

### Validation and delivery

- Give cross-model angle-encoding tests their own command budget to tolerate Windows
  scheduling. Production timeouts are unchanged.
- Correct bilingual offline test targets, Windows transport pins and repository URLs;
  add a build and validation guide.
- Remove tracked `.vscode` settings and ignore the entire directory. Temporary release
  tests no longer inherit personal Git signing requirements.
- Pin transport to `a46177e`, including the fix that completes send diagnostics and
  accounting before publishing terminal receipts.
- Reserve receipt storage before physical admission. A throwing observer no longer
  causes completed send accounting to be replayed. Existing public APIs, command
  terminal states and late-receipt queries retain their semantics.
- Allow release-pin resolution in GitHub-only repositories; when a Gitea workflow is
  present, its dependency pin must still match.
- Add `CommandHandle::send_results()` for physical batch receipts, partial sends and
  uncertain outcomes. Late receipts after timeout, close or recovery do not rewrite
  logical completion. Add recovery-retry and old-generation regression coverage.

### API changes and migration

- Keep L6 position Kp, position Kd and torque Kd in native Managers only. Remove the
  unified methods, samples, snapshots, events and capability entries for these controls.
  Migrate existing callers to native Managers and rebuild. Other public enum values remain unchanged.
- Remove O30 motion-time and torque-limit unified/native interfaces, associated types,
  events, snapshots and polling. Remove old calls and rebuild. Actual torque reads and
  current-limit reads/writes remain; surviving enum values are unchanged.
- Make all eight unified `HandDevice::set_*` methods return after local admission.
  Transmission and ACK handling run in the background. Pre-admission errors throw
  synchronously; subsequent errors are retained in `CommandHandle::error()`.
- Add bounded FIFO admission, replacement of pending targets and a default 250 Hz
  batch-start limit. The execution budget includes queueing, sending and ACK handling.
  A caller wait timeout neither cancels nor resubmits a command.
- Record actual dispatch/ACK stages, confirmed frame counts and stable terminal states.
  Close/recovery isolate old generations, share a shutdown budget and retain resources
  required by driver calls and callbacks that have not exited.
- Correct common-model cache freshness and ACK completion. Reject raw zero for L6
  stall-time writes while preserving reads that return zero.

### Dependencies and packaging

- Require transport 0.3.0 or newer within the same minor release line for both source
  checkouts and installed packages.
- The transport repository is `linkerbot-transport-cpp`. CMake package name
  `linkerbot-comm`, `linkerbot::comm` targets and `LINKERBOT_COMM_SOURCE_DIR` retain
  their compatibility names. Current README/CI pins identify the supported source
  baseline; historical reports retain the SHAs actually tested.
- Add development-package automation for version tags and manual runs, producing and
  validating Linux TGZ, Windows ZIP, SHA256 files and provenance manifests. Archive
  checks share version, license, content and extracted-consumer validation. Workflows
  do not automatically create tags or publish releases. Available forge workflows
  depend on the files present in the checkout.
- Include compatibility policy and the [0.3 migration guide](docs/en/guides/usage.md)
  in packages. Update README error handling. Rebuild callers for the new API.
- Add executable cross-language asynchronous behavior vectors covering admission,
  send/ACK, replacement, FIFO, timeouts, lifecycle and polling. Document differences in
  wait timeout, execution budget, claimed writes during close and short ACKs.
  Internal scheduler tests inject a clock; production uses `steady_clock`.
- Check every unified setter signature and runtime contract from an installed consumer.
  Compile complete C++ documentation examples.

### Platform and hardware evidence

- Add Windows x64 PCAN Classic/FD, MiniCANFD, default adapter discovery, explicit factories
  and read-only examples. Implement SLCAN Lawicel Classic, CANable FD and native Linux serial access.
- Fix intermittent shared-bus lifecycle/recovery crashes; add concurrency/fault regressions.
- Complete Windows build, runtime, ZIP, installation and migration validation. The historical
  2026-10-19 Release acceptance passed 39 steps, core CTest 143/143 and installed/extracted
  consumers 4/4. These are revision-specific historical counts, not current suite totals.
- Record common Linux hardware validation for six models. See the
  [English hardware index](docs/hardware-acceptance/README.en.md) for the recorded Windows
  Debug paths and gaps. PCAN FD hardware acceptance remains pending; SLCAN support is
  limited to the tested firmware profiles. Offline/Debug results do not certify Release hardware.

Native Managers, active getters and discrete operations retain their execution contracts.
See the [compatibility policy](docs/en/reference/compatibility.md) for platform and hardware limits.
