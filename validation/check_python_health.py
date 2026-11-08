"""Compare every bounded latency-window update with Python's rounding contract."""
import argparse
from collections import deque
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--cpp-probe', required=True)
parser.add_argument('--python-sdk', type=Path)
args = parser.parse_args()
actual = json.loads(subprocess.check_output([args.cpp_probe], text=True, timeout=30))
window = deque(maxlen=2048)
reference = None
if args.python_sdk:
    sys.path.insert(0, str(args.python_sdk.resolve() / 'src'))
    from linkerbot_hand.api.health import HealthStatus
    from linkerbot_hand.runtime.metrics import RuntimeMetrics
    reference = RuntimeMetrics(HealthStatus.initial())
values = [(i * 37) % 211 if i < 2048 else 0 if i < 4096 else 1000 for i in range(6144)]
assert len(actual) == len(values)
for i, value in enumerate(values):
    window.append(value)
    ordered = sorted(window)
    expected = [len(window), *(ordered[round((len(ordered) - 1) * p)] for p in (.50, .95, .99))]
    if reference is not None:
        reference.transaction_completed(timestamp=float(i), duration_s=value / 1e9)
        health = reference.health(
            now=float(i), captured_at=float(i), capabilities=None,
            commands=SimpleNamespace(metrics=lambda: (), discrete_depth=0),
            polling=SimpleNamespace(active=False, metrics=lambda **_: (), force_metrics=lambda **_: ()),
            snapshot_store=SimpleNamespace(data_ages=lambda **_: {}), transport=None, arbiter=None)
        m = health.transaction_latency
        projected = [m.count, *(round(v * 1e9) for v in (m.p50_s, m.p95_s, m.p99_s))]
        assert projected == expected, (i, projected, expected)
    assert actual[i] == expected, (i, actual[i], expected)
print('6144 rolling-window updates matched Python percentile selection, ties, duplicates and eviction'
      + (' and the reference RuntimeMetrics' if reference is not None else ''))
