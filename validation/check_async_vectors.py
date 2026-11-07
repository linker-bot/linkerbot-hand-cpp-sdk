"""Run one language-neutral scenario set against both SDK adapters."""
import argparse
import json
import subprocess
import sys
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--cpp-probe", required=True)
parser.add_argument("--python-sdk", type=Path)
parser.add_argument("--report", type=Path)
parser.add_argument("--vectors", type=Path, default=Path(__file__).resolve().parents[1] / "tests/data/async_command_behavior_vectors.json")
parser.add_argument("--cpp-python-profile", action="store_true",
                    help="Also validate the opt-in native Python runtime profile")
args = parser.parse_args()
vectors = json.loads(args.vectors.read_text())
assert vectors["schema_version"] == 2
report = {"schema_version": 2, "results": []}
for vector in vectors["scenarios"]:
    data = vector["input"]
    parameters = [vector["id"], str(data["capacity"]), str(data["targets"]), str(data["timeout_ms"])]
    commands = {"cpp": [args.cpp_probe, *parameters]}
    if args.cpp_python_profile:
        commands["cpp_python"] = [args.cpp_probe, *parameters, "python"]
    if args.python_sdk:
        commands["python"] = [sys.executable, str(Path(__file__).with_name("python_async_probe.py")),
                              "--python-sdk", str(args.python_sdk), *parameters]
    for language, command in commands.items():
        result = subprocess.run(command, text=True, capture_output=True, timeout=20)
        if result.returncode:
            raise RuntimeError(f"{language}/{vector['id']}: {result.stdout}\n{result.stderr}")
        actual = json.loads(result.stdout)
        if language == "cpp_python":
            expected = dict(vector["expected"]["python"])
            # Preserve the C++ handle API. A Python binding must translate false
            # into TimeoutError; this runtime profile does not change that API.
            if vector["id"] == "wait_timeout":
                expected["wait"] = "false"
        else:
            expected = vector["expected"][language]
        assert actual == expected, f"{language}/{vector['id']}: {actual} != {expected}"
        report["results"].append({"scenario": vector["id"], "language": language, "actual": actual})
        print(f"PASS {language}/{vector['id']}")
if args.report:
    args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
print(f"{len(report['results'])} executable behavior vectors passed")
