#!/usr/bin/env python3
"""Create a live, read-only gate for one small descent in base Z.

It deliberately creates one direct segment instead of a lift/translate plan.
Use it for an already verified hover pose only; it never opens a motion port.
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
    # UR represents the same near-pi orientation with either rotvec branch
    # (+pi axis or -pi axis). Arithmetic averaging those vectors can fabricate
    # a 130-degree pose although the robot never moved. Position benefits from
    # averaging; for a static gate take orientation from one actual latest
    # controller frame instead.
    tcp[3:] = np.asarray(rows[-1][1][3:], dtype=float)
    positions = np.asarray([row[1][:3] for row in rows])
    motion_mm = float(np.max(np.linalg.norm(positions - positions.mean(axis=0), axis=1)) * 1000.0)
    latest_rotation = robot_pose(rows[-1][1]).rotation
    rotation_spread_deg = max(float(np.degrees(np.linalg.norm(
        pin.log3(latest_rotation.T @ robot_pose(row[1]).rotation)))) for row in rows)
    return q, tcp, {"frames": len(rows), "max_position_motion_mm": motion_mm,
                    "max_rotation_motion_deg": rotation_spread_deg}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.1.3")
    parser.add_argument("--port", type=int, default=30013)
    parser.add_argument("--down-mm", type=float, default=None,
                        help="legacy positive downward distance in mm")
    parser.add_argument("--delta-z-mm", type=float, default=None,
                        help="signed base-Z displacement in mm (+ is upward); enables a gated retrace")
    parser.add_argument("--delta-x-mm", type=float, default=0.0,
                        help="signed base-X displacement in mm (direct linear move)")
    parser.add_argument("--delta-y-mm", type=float, default=0.0,
                        help="signed base-Y displacement in mm (direct linear move)")
    parser.add_argument("--joint6-range-deg", type=float, nargs=2, default=[-270.0, -180.0])
    parser.add_argument("--margin-mm", type=float, default=40.0)
    parser.add_argument("--attachments", default="config/robot_attachments.preliminary-cad-20260921.yaml")
    parser.add_argument("--calibration", default="config/handeye_eye_in_hand_20260921.yaml")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    if args.down_mm is not None and args.delta_z_mm is not None:
        parser.error("--down-mm and --delta-z-mm are mutually exclusive")
    delta_z_mm = (-args.down_mm if args.down_mm is not None
                  else (args.delta_z_mm or 0.0))
    delta_mm = np.array([args.delta_x_mm, args.delta_y_mm, delta_z_mm], dtype=float)
    length_mm = float(np.linalg.norm(delta_mm))
    if not 1.0 <= length_mm <= 50.0:
        parser.error("base linear displacement magnitude must be within 1..50 mm")
    low, high = args.joint6_range_deg
    if low >= high:
        parser.error("joint6 range needs MIN < MAX")
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    q, tcp, stationary = static_state(args.host, args.port)
    if stationary["max_position_motion_mm"] > 0.5 or stationary["max_rotation_motion_deg"] > 0.2:
        raise SystemExit("robot state not self-consistent: %.3f mm / %.3f deg" %
                         (stationary["max_position_motion_mm"],
                          stationary["max_rotation_motion_deg"]))
    target = np.asarray(tcp, dtype=float).copy()
    target[:3] += delta_mm / 1000.0
    ik = UR10IK()
    measured = robot_pose(tcp)
    fk = BASE_FROM_URDF_ROOT * ik.pose(q)
    difference = pin.log6(fk.inverse() * measured).vector
    fk_position, fk_rotation = float(np.linalg.norm(difference[:3])), float(np.linalg.norm(difference[3:]))
    calibration_path = args.calibration if os.path.isabs(args.calibration) else os.path.join(root, args.calibration)
    with open(calibration_path, encoding="utf-8") as stream:
        calibration = yaml.safe_load(stream)
    tool0_from_camera = np.eye(4)
    tool0_from_camera[:3, :3] = np.asarray(calibration["rotation_matrix"], dtype=float)
    tool0_from_camera[:3, 3] = np.asarray(calibration["translation_m"], dtype=float)
    attachment_path = args.attachments if os.path.isabs(args.attachments) else os.path.join(root, args.attachments)
    collision = SelfCollisionModel(tool0_from_camera, attachments_path=attachment_path)
    result = check_segment(ik, measured, pose_from_tcp_target(target), q, 60,
                           collision, args.margin_mm / 1000.0,
                           tuple(np.radians([low, high])))
    passed = bool(fk_position <= FK_POSITION_GATE_M and fk_rotation <= FK_ROTATION_GATE_RAD and
                  not result["reasons"])
    if abs(delta_mm[0]) > 1e-9 or abs(delta_mm[1]) > 1e-9:
        segment_name = "base_linear_dx%+g_dy%+g_dz%+gmm" % tuple(delta_mm)
    else:
        direction = "up" if delta_z_mm > 0 else "down"
        segment_name = "base_%s_%gmm" % (direction, abs(delta_z_mm))
    record = {"slot": 1, "segment": segment_name,
              "order": "direct_base_z_descent", "start_tcp_m_rad": tcp.tolist(),
              "end_tcp_m_rad": target.tolist(), "end_q_rad": result["end_q"],
              "worst": result["worst"], "passed": passed, "reasons": result["reasons"]}
    document = {"schema_version": 1, "kind": "live_direct_base_descent_gate",
                "motion_sent": False, "passed": passed, "created_at": dt.datetime.now().astimezone().isoformat(),
                "current": {"q_rad": q.tolist(), "tcp_m_rad": tcp.tolist(),
                            "source": "live UR %d" % args.port, "stationary": stationary},
                "visit_order": [1], "segments": [record],
                "joint6_range_deg": [low, high], "margin_mm": args.margin_mm,
                "model": {"fk_position_agreement_m": fk_position,
                          "fk_rotation_agreement_rad": fk_rotation,
                          "attachments": os.path.relpath(attachment_path, root),
                          "status": "preliminary CAD envelope; not real-motion proof"}}
    output = args.output if os.path.isabs(args.output) else os.path.join(root, args.output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "w", encoding="utf-8") as stream:
        json.dump(document, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    print("direct base descent gate:", "PASS" if passed else "FAIL", output)
    print("current z %.4f -> target z %.4f m; clearance %.1f mm" %
          (tcp[2], target[2], result["worst"]["min_camera_clearance_m"] * 1000.0))
    if not passed:
        print("reasons:", result["reasons"])
        raise SystemExit(2)


if __name__ == "__main__":
    main()
