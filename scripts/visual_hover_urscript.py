#!/usr/bin/env python3
"""Move horizontally above a visually located object using controller movel."""
import argparse
import json
import math
import os
import socket
import time

from record_ur_trajectory import RealtimeReader


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate", default="outputs/vision/cube-candidate-20260917.json")
    parser.add_argument("--speed", type=float, default=0.03)
    parser.add_argument("--acceleration", type=float, default=0.05)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if not args.execute:
        parser.error("real motion requires --execute")
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = args.candidate if os.path.isabs(args.candidate) else os.path.join(root, args.candidate)
    with open(path, encoding="utf-8") as stream:
        candidate = json.load(stream)
    if candidate.get("status") != "candidate_not_motion_authorized":
        raise RuntimeError("unexpected candidate status")
    center = candidate["estimated_cube_center_base_m"]
    reader = RealtimeReader("192.168.1.3")
    _, before, _ = reader.read()
    target = [center[0], center[1], before[2], *before[3:]]
    distance = math.dist(before[:3], target[:3])
    if distance > 0.15:
        raise RuntimeError("horizontal move %.3f m exceeds 0.15 m gate" % distance)
    if target[2] < 0.25:
        raise RuntimeError("tool0 Z %.3f m below hover gate" % target[2])
    program = ("def visual_hover():\n"
               "  movel(p[%s], a=%.6f, v=%.6f)\n"
               "  sleep(1.0)\n"
               "end\n") % (", ".join("%.9f" % value for value in target),
                              args.acceleration, args.speed)
    print("当前 TCP:", [round(value, 6) for value in before])
    print("悬停目标:", [round(value, 6) for value in target])
    print("平移距离: %.1f mm；Z 保持不变；不操作夹爪" % (distance * 1000.0), flush=True)
    with socket.create_connection(("192.168.1.3", 30002), timeout=3.0) as command:
        command.sendall(program.encode("ascii"))
    deadline = time.monotonic() + max(8.0, distance / args.speed * 2.0 + 3.0)
    samples = []
    while time.monotonic() < deadline:
        _, tcp, velocity = reader.read()
        samples.append(list(tcp))
        position_error = math.dist(tcp[:3], target[:3])
        linear_speed = math.sqrt(sum(value * value for value in velocity[:3]))
        if position_error < 0.001 and linear_speed < 0.002:
            break
    reader.close()
    after = samples[-1]
    error_mm = math.dist(after[:3], target[:3]) * 1000.0
    result = {"executed_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
              "method": "URScript movel; controller-native kinematics",
              "before_tcp": list(before), "target_tcp": target,
              "after_tcp": after, "translation_m": distance,
              "final_position_error_mm": error_mm,
              "speed_m_s": args.speed, "acceleration_m_s2": args.acceleration,
              "gripper_command_sent": False}
    output = os.path.join(root, "outputs", "vision", "hover-result-20260917.json")
    with open(output, "w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2)
        stream.write("\n")
    print("到达后 TCP:", [round(value, 6) for value in after])
    print("最终位置误差: %.3f mm" % error_mm)
    print("OUTPUT:", output)
    if error_mm > 2.0:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
