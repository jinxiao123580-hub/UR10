#!/usr/bin/env python3
"""Design and verify an OFFLINE-only UR10 wrist-orientation calibration path.

This program never opens a robot socket and never emits URScript.  It produces
a candidate set of static joint waypoints around a supplied measured seed and
checks the nominal URDF kinematics along linearly interpolated joint segments.
It is deliberately not a motion executor.
"""
import argparse
import datetime as dt
import itertools
import json
import math
import os

import numpy as np
import pinocchio as pin


URDF = os.path.expanduser("~/ur_learn/generated/ur10.urdf")
LOWER = np.deg2rad([-360, -360, -180, -360, -360, -360])
UPPER = -LOWER
LIMIT_MARGIN = math.radians(10.0)
G_BASE = np.array([0.0, 0.0, -1.0])

# Read-only 30003 measurement on 2026-09-16.  It is only a planning seed.
DEFAULT_SEED = np.array([-0.173080, -1.869437, -1.441179,
                         -4.532762, 4.796732, -3.384877])


def parse_seed(text):
    values = np.fromstring(text, sep=",")
    if values.shape != (6,):
        raise ValueError("seed 必须是 6 个逗号分隔的弧度值")
    return values


def pose(model, data, frame_id, q):
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    return data.oMf[frame_id].copy()


def gravity_direction(tool_pose):
    # Unit gravity direction represented in the tool/sensor coordinate system.
    return tool_pose.rotation.T @ G_BASE


def waypoint_pool(seed):
    """Small, wrist-first offsets; no Cartesian IK and no real robot command."""
    offsets = [(0.0, 0.0, 0.0)]
    for j4, j5, j6 in itertools.product((-0.40, 0.0, 0.40),
                                         (-0.50, 0.0, 0.50),
                                         (-0.45, 0.0, 0.45)):
        if (j4, j5, j6) != (0.0, 0.0, 0.0):
            offsets.append((j4, j5, j6))
    # Modest shoulder/elbow alternatives supply direction diversity that wrist
    # rotation alone may not provide; all retain the current base angle.
    for j2, j3, j4, j5, j6 in itertools.product((-0.18, 0.18), (-0.18, 0.18),
                                                   (-0.35, 0.35), (-0.45, 0.45),
                                                   (-0.35, 0.35)):
        offsets.append((j4, j5, j6, j2, j3))
    candidates = []
    for offset in offsets:
        q = seed.copy()
        if len(offset) == 3:
            q[3:] += offset
        else:
            q[3:] += offset[:3]
            q[1] += offset[3]
            q[2] += offset[4]
        if np.all(q > LOWER + LIMIT_MARGIN) and np.all(q < UPPER - LIMIT_MARGIN):
            candidates.append(q)
    return candidates


def greedy_select(model, data, frame_id, candidates, count):
    """Greedily improve isotropy of gravity directions, retaining the seed first."""
    directions = np.array([gravity_direction(pose(model, data, frame_id, q))
                           for q in candidates])
    selected = [0]
    remaining = set(range(1, len(candidates)))
    while len(selected) < min(count, len(candidates)):
        best, best_score = None, -np.inf
        for index in remaining:
            matrix = directions[selected + [index]]
            singular = np.linalg.svd(matrix, compute_uv=False)
            # Before rank 3 is possible, prefer angular spread via trace(M'M).
            score = (singular[-1] if len(selected) >= 2 else singular[0])
            score -= 0.015 * np.max(np.abs(candidates[index] - candidates[selected[-1]]))
            if score > best_score:
                best, best_score = index, score
        selected.append(best)
        remaining.remove(best)
    return [candidates[index] for index in selected]


def sampled_segment_metrics(model, data, frame_id, start, end, samples, attachment_radius):
    max_step = float(np.max(np.abs(end - start)))
    min_tool_z = float("inf")
    for alpha in np.linspace(0.0, 1.0, samples + 1):
        value = pose(model, data, frame_id, (1.0 - alpha) * start + alpha * end)
        min_tool_z = min(min_tool_z, float(value.translation[2]))
    return {"max_joint_change_rad": max_step, "samples": samples,
            "minimum_tool0_z_m": min_tool_z,
            "minimum_attachment_clearance_m": min_tool_z - attachment_radius}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", default=URDF)
    parser.add_argument("--seed", help="六关节初值（rad，以逗号分隔）")
    parser.add_argument("--waypoints", type=int, default=12)
    parser.add_argument("--segment-samples", type=int, default=50)
    parser.add_argument("--gripper-length-m", type=float, default=0.20,
                        help="夹爪从 tool0 起的最大长度包络（m）")
    parser.add_argument("--camera-radial-offset-m", type=float, default=0.30,
                        help="相机相对 tool0 的 XY 偏心距离（m）")
    parser.add_argument("--camera-z-offset-m", type=float, default=0.15,
                        help="相机相对 tool0 的 Z 偏心距离绝对值（m）")
    parser.add_argument("--plane-margin-m", type=float, default=0.05,
                        help="相对基座安装平面的额外净空（m）")
    parser.add_argument("--output")
    args = parser.parse_args()
    if (args.waypoints < 8 or args.waypoints > 24 or args.segment_samples < 2 or
            min(args.gripper_length_m, args.camera_radial_offset_m,
                args.camera_z_offset_m, args.plane_margin_m) < 0):
        parser.error("waypoints 应为 8..24；采样数和几何尺寸必须有效")
    seed = parse_seed(args.seed) if args.seed else DEFAULT_SEED
    model = pin.buildModelFromUrdf(args.urdf)
    data = model.createData()
    frame_id = model.getFrameId("tool0")
    if frame_id >= len(model.frames):
        raise RuntimeError("URDF 未包含 tool0")
    # Direction/sign of the camera offset is not yet measured.  Its norm is a
    # direction-independent, conservative sphere radius around tool0.
    camera_reach = math.hypot(args.camera_radial_offset_m, args.camera_z_offset_m)
    attachment_radius = max(args.gripper_length_m, camera_reach) + args.plane_margin_m
    candidates = [q for q in waypoint_pool(seed)
                  if pose(model, data, frame_id, q).translation[2] >= attachment_radius]
    if len(candidates) < args.waypoints:
        raise RuntimeError("平面净空门禁后候选不足：%d < %d" %
                           (len(candidates), args.waypoints))
    selected = greedy_select(model, data, frame_id, candidates, args.waypoints)
    directions = np.asarray([gravity_direction(pose(model, data, frame_id, q)) for q in selected])
    singular = np.linalg.svd(directions, compute_uv=False)
    condition = float(singular[0] / singular[-1]) if singular[-1] > 1e-12 else float("inf")
    direction_min = directions.min(axis=0)
    direction_max = directions.max(axis=0)
    # A low condition number alone can be misleading if all points lie in one
    # hemisphere.  Full-range gravity fitting needs an observable sign change
    # along every sensor axis; a local/downward study deliberately will not.
    full_range_axis_sign_coverage = bool(np.all(direction_min < -0.20) and
                                         np.all(direction_max > 0.20))
    segments = [sampled_segment_metrics(model, data, frame_id, start, end,
                                        args.segment_samples, attachment_radius)
                for start, end in zip(selected[:-1], selected[1:])]
    waypoint_rows = []
    for index, q in enumerate(selected, 1):
        tcp = pose(model, data, frame_id, q)
        waypoint_rows.append({"index": index, "q_rad": q.tolist(),
                              "tool0_xyz_m": tcp.translation.tolist(),
                              "gravity_direction_tool": gravity_direction(tcp).tolist()})
    report = {
        "schema_version": 1,
        "generated_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "purpose": "OFFLINE candidate only; not authorized for robot execution",
        "urdf": os.path.abspath(args.urdf),
        "seed_q_rad": seed.tolist(),
        "checks": {
            "joint_limit_margin_deg": 10.0,
            "all_waypoints_inside_margin": True,
            "gravity_design_matrix_condition_number": condition,
            "target_condition_number": 120.0,
            "gravity_direction_tool_component_min": direction_min.tolist(),
            "gravity_direction_tool_component_max": direction_max.tolist(),
            "full_range_axis_sign_coverage": full_range_axis_sign_coverage,
            "fit_scope": ("full-range candidate" if full_range_axis_sign_coverage
                          else "local/downward candidate only; do not fit or deploy a global model"),
            "minimum_nominal_tool0_z_m": min(item["minimum_tool0_z_m"] for item in segments),
            "attachment_conservative_radius_m": attachment_radius,
            "attachment_model": ("max(gripper %.3fm, hypot(camera_xy %.3fm, camera_z %.3fm)) "
                                 "+ plane margin %.3fm; camera offset direction and body size are unknown" %
                                 (args.gripper_length_m, args.camera_radial_offset_m,
                                  args.camera_z_offset_m, args.plane_margin_m)),
            "minimum_conservative_plane_clearance_m": min(
                item["minimum_attachment_clearance_m"] for item in segments),
            "plane_clearance_pass": all(item["minimum_attachment_clearance_m"] >= 0.0
                                        for item in segments),
            "collision_check": "Plane clearance checked conservatively; no self/cell/cable collision model",
            "dynamic_check": "NOT PERFORMED: RViz twin is kinematic only; no controller/dynamics model",
        },
        "waypoints": waypoint_rows,
        "segments": segments,
        "execution_boundary": [
            "No URScript is emitted by this tool.",
            "Before any physical replay, an operator must teach/verify each waypoint at reduced mode.",
            "Do not use test_ik_move.py --execute or stream IK for this path.",
            "A passing nominal-URDF report does not prove collision-free real motion.",
            "This default path is intentionally wrist-first and may only be used as a local repeatability study."
        ],
    }
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    output = args.output or os.path.join(root, "outputs", "ft_path_design",
                                         "candidate-%s.json" % dt.datetime.now().strftime("%Y%m%d-%H%M%S"))
    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    with open(output, "w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2)
        stream.write("\n")
    print("waypoints=%d condition=%.3f min_tool0_z=%.4fm conservative_clearance=%.4fm" %
          (len(selected), condition, report["checks"]["minimum_nominal_tool0_z_m"],
           report["checks"]["minimum_conservative_plane_clearance_m"]))
    print("max_segment_joint_change=%.4frad" %
          max(item["max_joint_change_rad"] for item in segments))
    print("report", output)
    if not math.isfinite(condition) or condition > 120.0:
        raise RuntimeError("candidate orientation coverage did not meet condition-number target")
    if not report["checks"]["plane_clearance_pass"]:
        raise RuntimeError("candidate violates conservative base-plane clearance")


if __name__ == "__main__":
    main()
