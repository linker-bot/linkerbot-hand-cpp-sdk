"""Compare native outgoing frames with a pinned Python protocol baseline.

No hardware is opened. Optional --python-sdk recomputes the same fixtures with
production Python transaction factories, rather than separate expectations.
"""
import argparse
import json
import subprocess
import sys
from pathlib import Path

BASELINE = "3a63e723b9a0046e9a60a58339b9f064ef5b70fd"
SCENARIOS = ("l6_position_kp", "l6_position_kd", "l6_torque_kd", "l6_torques",
             "l6_limit_compensations", "l6_save_confirmed", "o30_torque_limits",
             "o30_motion_times", "l30_calibrate")


def python_frames(sdk, scenario):
    sys.path.insert(0, str(sdk / "src"))
    from linkerbot_hand.api.enums import HandSide
    from linkerbot_hand.capabilities.registry import get_declared_capabilities
    from linkerbot_hand.capabilities.types import CommandSemantic
    from linkerbot_hand.protocols.l6.transactions import L6ProtocolBackend
    from linkerbot_hand.protocols.l30.operations import operation_transaction
    from linkerbot_hand.protocols.o30.control import command_transaction
    from linkerbot_hand.protocols.o30.framing import HOPAddressing
    if scenario.startswith("l6_"):
        backend = L6ProtocolBackend(HandSide.LEFT)
        caps = get_declared_capabilities("L6", side="left")
        if scenario == "l6_save_confirmed":
            transaction = backend.operation_transaction("save_parameters", caps)
        else:
            semantic = getattr(CommandSemantic, scenario.removeprefix("l6_").upper())
            transaction = backend.command_transaction(semantic, tuple(range(1, 7)), caps)
    elif scenario.startswith("o30_"):
        semantic = CommandSemantic.TORQUES_LIMITS if scenario.endswith("torque_limits") else CommandSemantic.MOTION_TIMES
        transaction = command_transaction(semantic, tuple(range(1, 21)),
                                          get_declared_capabilities("O30", side="left"), HOPAddressing(1, 0x401))
    else:
        transaction = operation_transaction("calibrate", get_declared_capabilities("L30"), node_id=1, host_id=0)
    frames = []
    while transaction:
        for message in transaction.request_messages:
            native_can = hasattr(message, "is_fd")
            fd = bool(message.is_fd) if native_can else True
            wire_length = message.dlc if native_can else (0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64)[message.dlc]
            frames.append({"id": message.arbitration_id, "fd": fd,
                           "extended": message.is_extended_id,
                           "brs": bool(getattr(message, "bitrate_switch", False)),
                           "data": list(bytes(message.data).ljust(wire_length, b"\0"))})
        transaction = transaction.next_transaction
    return frames


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cpp-probe")
    parser.add_argument("--python-sdk", type=Path)
    parser.add_argument("--write-baseline", action="store_true")
    parser.add_argument("--vectors", type=Path, default=Path(__file__).resolve().parents[1] / "tests/data/python_migration_vectors.json")
    args = parser.parse_args()
    if args.write_baseline:
        if not args.python_sdk:
            parser.error("--write-baseline requires --python-sdk")
        commit = subprocess.check_output(["git", "-C", str(args.python_sdk), "rev-parse", "HEAD"], text=True).strip()
        if commit != BASELINE:
            raise ValueError(f"Python checkout {commit} is not baseline {BASELINE}")
        vectors = {"schema_version": 1, "python_commit": BASELINE,
                   "scope": "outgoing physical CAN frames; no hardware validation",
                   "scenarios": {name: python_frames(args.python_sdk, name) for name in SCENARIOS}}
        args.vectors.write_text(json.dumps(vectors, indent=2) + "\n")
        return
    if not args.cpp_probe:
        parser.error("--cpp-probe is required")
    vectors = json.loads(args.vectors.read_text())
    if vectors["schema_version"] != 1 or vectors["python_commit"] != BASELINE:
        raise ValueError("unsupported migration baseline")
    for name, expected in vectors["scenarios"].items():
        result = subprocess.run([args.cpp_probe, name], capture_output=True, text=True, timeout=20, check=True)
        actual = json.loads(result.stdout)
        if actual != expected:
            raise AssertionError(f"C++/{name}: {actual} != {expected}")
        if args.python_sdk and python_frames(args.python_sdk, name) != expected:
            raise AssertionError(f"Python/{name}: differs from pinned baseline")
        print(f"PASS {name}")
    print(f"{len(vectors['scenarios'])} same-input wire vectors passed")


if __name__ == "__main__":
    main()
