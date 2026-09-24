#!/usr/bin/env python3
"""Create camera-orbit observation poses about a fixed gripper-centre hover.

The camera is offset from tool0, so rolling tool0 about its local Z axis makes
the camera orbit the cube while the gripper-centre hover point stays fixed.
This is a read-only plan: each pose must pass collision/J6 gating before a
future executor is allowed to visit it.
"""
import argparse
import json
import os

import cv2
import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", required=True)
    parser.add_argument("--grasp-center", default="config/gripper_grasp_center_20260922.yaml")
    parser.add_argument("--hover-mm", type=float, default=80.0)
    parser.add_argument("--roll-deg", type=float, nargs="+", default=[0.0, 25.0, -25.0])
    parser.add_argument("--output", default="outputs/vision/active-cube-view-plan.json")
    args = parser.parse_args()
    if not 40.0 <= args.hover_mm <= 150.0 or any(abs(x) > 70.0 for x in args.roll_deg):
        parser.error("hover must be 40..150 mm; each roll must be within +/-70 deg")
    path = args.plan if os.path.isabs(args.plan) else os.path.join(ROOT, args.plan)
    with open(path, encoding="utf-8") as stream: plan = json.load(stream)
    if plan.get("kind") != "auto_cube_pick_place_plan":
        raise SystemExit("expected auto cube pick/place plan")
    grasp_path = args.grasp_center if os.path.isabs(args.grasp_center) else os.path.join(ROOT, args.grasp_center)
    with open(grasp_path, encoding="utf-8") as stream:
        offset = np.asarray((yaml.safe_load(stream) or {})["translation_m"], dtype=float)
    initial = np.asarray(plan["pick"], dtype=float)
    base_rotation, _ = cv2.Rodrigues(initial[3:])
    normal = np.asarray(plan["board_normal_base"], dtype=float); normal /= np.linalg.norm(normal)
    # Use the physical cube centre, not the deliberately higher jaw pinch
    # level, so camera hover stays at the requested clearance from the object.
    cube_centre = np.asarray(plan.get("cube_center_base_m") or
                            plan["gripper_center_pick_base_m"], dtype=float)
    centre = cube_centre + normal * args.hover_mm / 1000.0
    views = []
    for index, degrees in enumerate(args.roll_deg, 1):
        angle = np.radians(degrees)
        roll = np.array([[np.cos(angle), -np.sin(angle), 0.0],
                         [np.sin(angle), np.cos(angle), 0.0], [0.0, 0.0, 1.0]])
        rotation = base_rotation @ roll
        rvec, _ = cv2.Rodrigues(rotation)
        tcp = np.r_[centre - rotation @ offset, rvec.reshape(3)]
        views.append({"slot": index, "tool_roll_deg": float(degrees),
                      "target_tcp_pose_m_rad": tcp.tolist()})
    output = args.output if os.path.isabs(args.output) else os.path.join(ROOT, args.output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    document = {"schema_version": 1, "kind": "active_cube_view_plan", "motion_sent": False,
                "authorization": "none: must pass per-segment collision/J6 gate before execution",
                "cube_hover_gripper_center_base_m": centre.tolist(),
                "views": views, "inputs": {"initial_plan": os.path.relpath(path, ROOT)}}
    with open(output, "w", encoding="utf-8") as stream:
        json.dump(document, stream, ensure_ascii=False, indent=2); stream.write("\n")
    print("active view plan:", output, "rolls:", args.roll_deg)


if __name__ == "__main__":
    main()
