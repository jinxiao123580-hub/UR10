#!/usr/bin/env python3
"""Read-only safety gate for the active cube-view rotations.

Checks the live-to-view and view-to-view Cartesian segments before any command
can be sent.  This is deliberately separate from the executor.
"""
import argparse
import datetime as dt
import json
import os
import sys

import numpy as np
import pinocchio as pin
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_move_plan import (BASE_FROM_URDF_ROOT, FK_POSITION_GATE_M,
                             FK_ROTATION_GATE_RAD, check_segment, robot_pose,
                             pose_from_tcp_target)
from record_ur_trajectory import RealtimeReader
from self_collision import SelfCollisionModel
from ur_pose_ik import UR10IK


def static_state(host, port, count=30):
    reader = RealtimeReader(host, port)
    try:
        rows = [reader.read() for _ in range(count)]
    finally:
        reader.close()
    q = np.mean([row[0] for row in rows], axis=0)
    tcp = np.mean([row[1] for row in rows], axis=0)
    tcp[3:] = np.asarray(rows[-1][1][3:], dtype=float)
    positions = np.asarray([row[1][:3] for row in rows])
    motion_mm = float(np.max(np.linalg.norm(positions - positions.mean(axis=0), axis=1)) * 1000.0)
    latest = robot_pose(rows[-1][1]).rotation
    rotation_deg = max(float(np.degrees(np.linalg.norm(pin.log3(latest.T @ robot_pose(row[1]).rotation))))
                       for row in rows)
    return q, tcp, {"frames": len(rows), "max_position_motion_mm": motion_mm,
                    "max_rotation_motion_deg": rotation_deg}


def rel(root, value):
    return value if os.path.isabs(value) else os.path.join(root, value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", default="outputs/vision/active-cube-view-plan.json")
    parser.add_argument("--host", default="192.168.1.3")
    parser.add_argument("--port", type=int, default=30013)
    parser.add_argument("--joint6-range-deg", type=float, nargs=2, default=None,
                        help="optional operational J6 range; omitted uses only UR10 model hard limits")
    parser.add_argument("--margin-mm", type=float, default=20.0,
                        help="preliminary CAD camera/arm clearance gate")
    parser.add_argument("--attachments", default="config/robot_attachments.preliminary-cad-20260921.yaml")
    parser.add_argument("--calibration", default="config/handeye_eye_in_hand_20260921.yaml")
    parser.add_argument("--output", default="outputs/vision/active-cube-view-gate.json")
    args = parser.parse_args()
    if args.joint6_range_deg is not None and args.joint6_range_deg[0] >= args.joint6_range_deg[1]:
        parser.error("joint6 range needs MIN < MAX")
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    with open(rel(root, args.plan), encoding="utf-8") as stream:
        plan = json.load(stream)
    if plan.get("kind") != "active_cube_view_plan" or not plan.get("views"):
        raise SystemExit("not an active_cube_view_plan with views")
    q, tcp, stationary = static_state(args.host, args.port)
    if stationary["max_position_motion_mm"] > 0.5 or stationary["max_rotation_motion_deg"] > 0.2:
        raise SystemExit("robot state not self-consistent: %.3f mm / %.3f deg" %
                         (stationary["max_position_motion_mm"], stationary["max_rotation_motion_deg"]))
    ik = UR10IK()
    measured = robot_pose(tcp)
    fk = BASE_FROM_URDF_ROOT * ik.pose(q)
    difference = pin.log6(fk.inverse() * measured).vector
    fk_position, fk_rotation = float(np.linalg.norm(difference[:3])), float(np.linalg.norm(difference[3:]))
    with open(rel(root, args.calibration), encoding="utf-8") as stream:
        calibration = yaml.safe_load(stream)
    tool0_from_camera = np.eye(4)
    tool0_from_camera[:3, :3] = np.asarray(calibration["rotation_matrix"], dtype=float)
    tool0_from_camera[:3, 3] = np.asarray(calibration["translation_m"], dtype=float)
    collision = SelfCollisionModel(tool0_from_camera, attachments_path=rel(root, args.attachments))
    current_pose, current_q = measured, q
    records = []
    for view in plan["views"]:
        target = np.asarray(view["target_tcp_pose_m_rad"], dtype=float)
        result = check_segment(ik, current_pose, pose_from_tcp_target(target), current_q, 80,
                               collision, args.margin_mm / 1000.0,
                               (None if args.joint6_range_deg is None
                                else tuple(np.radians(args.joint6_range_deg))))
        records.append({"slot": view["slot"], "tool_roll_deg": view["tool_roll_deg"],
                        "start_tcp_m_rad": list(current_pose.translation) + list(pin.log3(current_pose.rotation)),
                        "end_tcp_m_rad": target.tolist(), "end_q_rad": result["end_q"],
                        "passed": not result["reasons"], "reasons": result["reasons"],
                        "worst": result["worst"]})
        current_pose, current_q = pose_from_tcp_target(target), np.asarray(result["end_q"])
    passed = bool(fk_position <= FK_POSITION_GATE_M and fk_rotation <= FK_ROTATION_GATE_RAD and
                  all(record["passed"] for record in records))
    document = {"schema_version": 1, "kind": "active_cube_view_gate", "motion_sent": False,
                "passed": passed, "created_at": dt.datetime.now().astimezone().isoformat(),
                "plan": os.path.relpath(rel(root, args.plan), root),
                "current": {"q_rad": q.tolist(), "tcp_m_rad": tcp.tolist(), "stationary": stationary},
                "segments": records, "joint6_range_deg": args.joint6_range_deg, "margin_mm": args.margin_mm,
                "model": {"fk_position_agreement_m": fk_position, "fk_rotation_agreement_rad": fk_rotation,
                          "status": "preliminary CAD envelope; requires human workspace observation"}}
    output = rel(root, args.output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "w", encoding="utf-8") as stream:
        json.dump(document, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print("active cube view gate:", "PASS" if passed else "FAIL", output)
    if not passed:
        for record in records:
            if not record["passed"]:
                print("view", record["slot"], "reasons:", record["reasons"])
        raise SystemExit(2)


if __name__ == "__main__":
    main()
