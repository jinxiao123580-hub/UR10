#!/usr/bin/env python3
"""Select static calibration waypoints from a recorded UR trajectory.

The selected configurations are actual recorded states, never interpolated or
IK-generated.  This tool is offline-only and emits no robot command.
"""
import argparse
import csv
import json
import math
import os

import numpy as np


def skew(v):
    x, y, z = v
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def rotvec_to_matrix(v):
    angle = np.linalg.norm(v)
    if angle < 1e-12:
        return np.eye(3)
    axis = v / angle
    cross = skew(axis)
    return np.eye(3) + math.sin(angle) * cross + (1.0 - math.cos(angle)) * (cross @ cross)


def gravity_direction(rotvec):
    return rotvec_to_matrix(rotvec).T @ np.array([0.0, 0.0, -1.0])


def load_csv(path):
    with open(path, newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
    if not rows:
        raise RuntimeError("轨迹 CSV 没有数据")
    result = []
    for row in rows:
        result.append({
            "elapsed_s": float(row["elapsed_s"]),
            "q_rad": [float(row["q%d_rad" % i]) for i in range(1, 7)],
            "tcp_pose": [float(row[name]) for name in
                         ("tcp_x_m", "tcp_y_m", "tcp_z_m", "tcp_rx_rad",
                          "tcp_ry_rad", "tcp_rz_rad")],
        })
    return result


def select(rows, count, min_time_separation, joint_margin_deg):
    # Downsample candidates to about 10 Hz; selection quality does not need all
    # 125-Hz duplicates and each selected point remains an exact recorded frame.
    joint_limit = math.radians(360.0 - joint_margin_deg)
    candidates = [row for row in rows[::12]
                  if np.max(np.abs(row["q_rad"])) <= joint_limit]
    if len(candidates) < count:
        raise RuntimeError("关节余量门禁后候选不足：%d < %d" % (len(candidates), count))
    directions = np.asarray([gravity_direction(np.asarray(row["tcp_pose"][3:]))
                             for row in candidates])
    # Anchor the first recorded state so the plan has an exact initial pose.
    selected = [0]
    while len(selected) < count:
        best_index = None
        best_score = -float("inf")
        for index, row in enumerate(candidates):
            if index in selected:
                continue
            if any(abs(row["elapsed_s"] - candidates[j]["elapsed_s"]) < min_time_separation
                   for j in selected):
                continue
            trial = directions[selected + [index]]
            design = np.hstack([trial, np.ones((len(trial), 1))])
            singular = np.linalg.svd(design, compute_uv=False)
            # Before rank 4 is available, angular maximin spreading dominates.
            nearest = min(math.acos(float(np.clip(directions[index] @ directions[j], -1, 1)))
                          for j in selected)
            score = nearest if len(selected) < 3 else math.log(max(singular[-1], 1e-12))
            if score > best_score:
                best_index, best_score = index, score
        if best_index is None:
            raise RuntimeError("无法在时间间隔门限下选出 %d 个点" % count)
        selected.append(best_index)
    selected.sort(key=lambda index: candidates[index]["elapsed_s"])
    return [(candidates[index], directions[index]) for index in selected]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory_csv")
    parser.add_argument("--count", type=int, default=12)
    parser.add_argument("--min-time-separation", type=float, default=2.0)
    parser.add_argument("--dwell", type=float, default=14.0)
    parser.add_argument("--joint-margin-deg", type=float, default=10.0)
    parser.add_argument("--output")
    args = parser.parse_args()
    if (args.count < 8 or args.min_time_separation < 0 or args.dwell < 3 or
            not 0 < args.joint_margin_deg < 180):
        parser.error("count 至少 8，时间间隔非负，dwell 至少 3 秒")
    rows = load_csv(args.trajectory_csv)
    chosen = select(rows, args.count, args.min_time_separation, args.joint_margin_deg)
    directions = np.asarray([direction for _, direction in chosen])
    design = np.hstack([directions, np.ones((len(directions), 1))])
    condition = float(np.linalg.cond(design))
    waypoints = []
    for index, (row, direction) in enumerate(chosen, 1):
        waypoints.append({
            "index": index,
            "source_elapsed_s": row["elapsed_s"],
            "q_rad": row["q_rad"],
            "tcp_pose": row["tcp_pose"],
            "gravity_direction_tool": direction.tolist(),
            "dwell_s": args.dwell,
        })
    document = {
        "schema_version": 1,
        "source_trajectory": os.path.abspath(args.trajectory_csv),
        "purpose": "offline waypoint plan only; no robot execution authorization",
        "selection": "exact recorded frames, greedy orientation coverage, chronological order",
        "joint_limit_margin_deg": args.joint_margin_deg,
        "design_condition": condition,
        "gravity_direction_component_min": directions.min(axis=0).tolist(),
        "gravity_direction_component_max": directions.max(axis=0).tolist(),
        "outbound_waypoints": waypoints,
        "return_waypoint_indices": [row["index"] for row in reversed(waypoints[:-1])],
        "final_exact_initial_q_rad": waypoints[0]["q_rad"],
        "safety_note": ("The source path was physically recorded, but stop-and-go replay changes dynamics. "
                        "Verify every waypoint on the teach pendant at reduced speed before execution."),
    }
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    output = args.output or os.path.join(root, "outputs", "ur_trajectory", "selected-waypoints.json")
    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    with open(output, "w", encoding="utf-8") as stream:
        json.dump(document, stream, indent=2)
        stream.write("\n")
    print("selected=%d condition=%.3f" % (len(waypoints), condition))
    print("gravity range min=%s max=%s" %
          (np.round(directions.min(axis=0), 3), np.round(directions.max(axis=0), 3)))
    print("source times:", [round(row["source_elapsed_s"], 2) for row in waypoints])
    print("output", output)


if __name__ == "__main__":
    main()
