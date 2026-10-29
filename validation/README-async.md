# Asynchronous command acceptance checks

The JSON in `tests/data/async_command_behavior_vectors.json` drives the C++ and
Python probes with identical scenario parameters. Each probe calls the unified
HandDevice using the production L6 codec and a scripted CAN transport. The runner
compares observed stages, errors, frame counts and ordering with per-language
expectations. Unknown scenarios, adapter failures and unexpected results fail
acceptance. This checks behavior; it does not inspect method names or JSON text.

C++ offline coverage runs in CTest when a Python interpreter is available. The
optional Python source checkout enables both languages in the same CTest case:

```sh
cmake -S . -B build \
  -DLINKERBOT_BUILD_TESTS=ON \
  -DPython3_EXECUTABLE=/path/to/python-with-sdk-dependencies \
  -DLINKERBOT_PYTHON_SDK_DIR=/path/to/linkerbot-hand
cmake --build build --target tests protocol-tests example
ctest --test-dir build --output-on-failure
```

The selected interpreter needs the Python SDK dependencies, including python-can.
The Python checkout must contain `src/` and its reusable CAN test fixture under
`tests/hand/api_v1/runtime/classic_can_fake.py`. No pytest invocation or SDK test
case selection is used by the vector runner. A sibling linkerbot-transport source
checkout is used when present; otherwise the installed transport is imported.
The Python SDK is not a build dependency of the C++ library or installed consumer.
The JSON report is written to `build/tests/async-behavior-report.json`.

The C++ probe uses the retained L6 stall-threshold command for ACK and FIFO
scenarios. The Python probe continues to use its SDK's position-Kp command; the
vectors compare command behavior rather than requiring identical parameter APIs.

The shared scenarios cover L6 admission, complete/short ACK responses, no-ACK
sending, latest target replacement, claimed writes, FIFO capacity/order, caller
wait timeout, execution timeout, closing, no replay and polling/cache progress.
The full C++ regression additionally covers all eight setters across supported
models, recover/destruction, admission budgets, rate limiting and arbitration.
The `[clock]` test injects a monotonic clock into the internal scheduler: pending
work stays live at deadline minus 1 ms, expires exactly at the deadline without
sending, and a caller wait does not reset that budget. Native protocol response
windows and real driver blocking remain covered by barriers and bounded waits;
this is not a fully virtualized device/runtime clock.

Explicit differences remain in the JSON and migration guide: Python wait raises
TimeoutError; its execution budget is a response window; a successful claimed
write may finish dispatched on close; malformed matching short ACKs fail with
ProtocolError. C++ has admission-to-completion budgets and freezes unfinished
requests on close, while ignoring short ACKs until a complete reply or timeout.
Neither API exposes public cancellation; Python has no cancelled enum value.
Do not change expectations to hide regressions or describe these as full parity.

After installing, validate only the exported SDK target:

```sh
cmake --install build --prefix /tmp/linkerbot-install
cmake -S validation/installed-consumer -B /tmp/linkerbot-consumer \
  -DCMAKE_PREFIX_PATH=/tmp/linkerbot-install
cmake --build /tmp/linkerbot-consumer
ctest --test-dir /tmp/linkerbot-consumer --output-on-failure
python3 validation/check_documented_examples.py \
  --prefix /tmp/linkerbot-install --build-dir /tmp/linkerbot-doc-examples
```

The consumer compiles all eight setter signatures and actually checks early
return at a send barrier, independent cache reads, wait timeout without cancelling,
no-ACK completion, configuration ACK, synchronous zero-frame validation and an
asynchronous transport error retained on the handle. The documentation check
compiles every migration C++ block with the current README helper, plus README's
complete program and L30 snippet. Compilation does not run motion on hardware.

Six-model hardware acceptance remains separate. Record model/serial, firmware,
transport/driver, SDK revisions, complete replies, continuous target delivery,
disconnection/recovery and safety observations. Offline success is not evidence
of mechanical arrival, firmware compatibility or a published release.
