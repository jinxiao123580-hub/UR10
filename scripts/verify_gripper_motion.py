#!/usr/bin/env python3
"""验证 Robotiq 开合命令与实时 POS/PRE/OBJ/STA/FLT 反馈。"""

import argparse
import json
import os
import time

from rq_gripper import RobotiqGripper


def sample(gripper, phase, duration, interval):
    rows = []
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        stamp = time.time()
        row = {"time": stamp, "phase": phase}
        for key in ("POS", "PRE", "OBJ", "STA", "FLT", "ACT", "GTO", "SPE", "FOR"):
            row[key] = gripper.query(key)
        rows.append(row)
        print("[%s] POS=%s PRE=%s OBJ=%s STA=%s FLT=%s" %
              (phase, row["POS"], row["PRE"], row["OBJ"], row["STA"], row["FLT"]), flush=True)
        time.sleep(interval)
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--interval", type=float, default=0.10)
    parser.add_argument("--output", default="outputs/gripper/motion-verification.json")
    args = parser.parse_args()

    if args.duration <= 0 or args.interval <= 0:
        parser.error("duration 和 interval 必须为正数")
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    output = args.output if os.path.isabs(args.output) else os.path.join(root, args.output)
    rows = []
    gripper = RobotiqGripper()
    try:
        gripper.connect()
        initial = gripper.status()
        print("初始:", initial)
        if initial.get("FLT") not in (0, None):
            raise RuntimeError("初始夹爪故障 FLT=%s" % initial.get("FLT"))
        if not gripper.activate():
            raise RuntimeError("夹爪激活未确认")

        print("发送 OPEN：目标 POS=0")
        gripper.move(0, speed=255, force=255, wait=False)
        rows += sample(gripper, "open", args.duration, args.interval)

        print("发送 CLOSE：目标 POS=255")
        gripper.move(255, speed=255, force=255, wait=False)
        rows += sample(gripper, "close", args.duration, args.interval)

        print("恢复 OPEN：目标 POS=0")
        gripper.move(0, speed=255, force=255, wait=False)
        rows += sample(gripper, "restore_open", args.duration, args.interval)
    finally:
        gripper.close_socket()

    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "w", encoding="utf-8") as stream:
        json.dump({"method": "Robotiq ASCII realtime polling", "samples": rows},
                  stream, indent=2)
        stream.write("\n")
    print("JSON:", output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
