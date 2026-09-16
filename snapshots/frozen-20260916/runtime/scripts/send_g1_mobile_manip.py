#!/usr/bin/env python3
"""Send live movement/manipulation commands to the MATRiX G1 runtime."""

import argparse
import json
from pathlib import Path
import socket
import sys


DEFAULT_PORT = 15151
POSES = ("home", "wave", "reach", "grasp", "pickup", "carry", "release", "auto")


def joint_value(value: str) -> tuple[str, float]:
    if "=" not in value:
        raise argparse.ArgumentTypeError("joint targets must use NAME=RADIANS")
    name, raw = value.split("=", 1)
    if not name:
        raise argparse.ArgumentTypeError("joint name must not be empty")
    try:
        target = float(raw)
    except ValueError as error:
        raise argparse.ArgumentTypeError(f"invalid joint target: {value}") from error
    return name, target


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser(description=__doc__)
    root.add_argument("--host", default="127.0.0.1")
    root.add_argument("--port", type=int, default=DEFAULT_PORT)
    subparsers = root.add_subparsers(dest="action", required=True)

    move = subparsers.add_parser("move", help="set persistent body velocity")
    move.add_argument("--vx", type=float, default=0.0)
    move.add_argument("--vy", type=float, default=0.0)
    move.add_argument("--yaw", type=float, default=0.0)
    move.add_argument("--course-hold", action="store_true")
    move.add_argument("--reference-y", type=float, default=0.0)
    move.add_argument("--reference-yaw", type=float, default=0.0)

    subparsers.add_parser("stop", help="set all body velocities to zero")

    pose = subparsers.add_parser("pose", help="select a built-in upper-body pose")
    pose.add_argument("name", choices=POSES)

    joints = subparsers.add_parser("joints", help="set arbitrary upper-body joints")
    joints.add_argument("targets", nargs="+", type=joint_value, metavar="NAME=RADIANS")
    joints.add_argument("--pose", choices=POSES, default="home")
    joints.add_argument("--speed", type=float, default=1.2)

    payload = subparsers.add_parser(
        "payload", help="attach nearby payload, reset-and-pickup, or release"
    )
    payload.add_argument("operation", choices=("attach", "pickup", "release"))

    mission = subparsers.add_parser("mission", help="load a JSON mission at runtime")
    mission.add_argument("path", type=Path)

    subparsers.add_parser("cancel", help="cancel the current mission and hold its commands")
    subparsers.add_parser("status", help="query the current runtime state")
    return root


def build_command(args: argparse.Namespace) -> dict:
    if args.action == "move":
        return {
            "command": "move",
            "vx": args.vx,
            "vy": args.vy,
            "yaw": args.yaw,
            "course_hold": args.course_hold,
            "reference_y": args.reference_y,
            "reference_yaw": args.reference_yaw,
        }
    if args.action == "stop":
        return {"command": "stop"}
    if args.action == "pose":
        return {"command": "upper", "pose": args.name}
    if args.action == "joints":
        return {
            "command": "upper",
            "pose": args.pose,
            "speed": args.speed,
            "joints": dict(args.targets),
        }
    if args.action == "payload":
        return {"command": "payload", "action": args.operation}
    if args.action == "mission":
        path = args.path.expanduser().resolve()
        with path.open("r", encoding="utf-8") as stream:
            mission = json.load(stream)
        return {"command": "mission", "mission": mission}
    if args.action == "cancel":
        return {"command": "cancel_mission"}
    return {"command": "status"}


def main() -> int:
    args = parser().parse_args()
    command = build_command(args)
    payload = json.dumps(command, ensure_ascii=False).encode("utf-8")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client:
        client.settimeout(1.0)
        client.sendto(payload, (args.host, args.port))
        try:
            response, _ = client.recvfrom(65535)
        except TimeoutError:
            print("[ERROR] G1 mobile-manipulation runtime did not reply", file=sys.stderr)
            return 1
    result = json.loads(response.decode("utf-8"))
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
