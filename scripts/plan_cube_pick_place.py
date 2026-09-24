#!/usr/bin/env python3
"""Create a read-only top-down pick/place plan for one 50 mm cube.

Input is the JSON made by ``locate_cube_near_checkerboard.py``.  The cube is
picked at its measured centre and placed with its centre 25 mm above the
checkerboard inner-grid centre.  The gripper approach axis is normal to the
board; the jaw roll is aligned to its X grid axis.  This file deliberately
does not command the robot -- a human must inspect the generated JSON and use
an explicitly authorised executor afterwards.
"""
import argparse
import json
import os

import cv2
import numpy as np
import yaml


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def rotvec(rotation):
    value, _ = cv2.Rodrigues(np.asarray(rotation, dtype=float))
    return value.reshape(3)


def pose(rotation, translation):
    return np.r_[np.asarray(translation, dtype=float), rotvec(rotation)].tolist()


def load(path, reader):
    full = path if os.path.isabs(path) else os.path.join(ROOT, path)
    with open(full, encoding="utf-8") as stream:
        return reader(stream), full


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--observation", required=True)
    parser.add_argument("--grasp-center", default="config/gripper_grasp_center_20260922.yaml")
    parser.add_argument("--cube-edge-mm", type=float, default=50.0)
    parser.add_argument("--hover-mm", type=float, default=80.0)
    parser.add_argument("--max-board-tilt-deg", type=float, default=12.0,
                        help="refuse plans whose usable board normal is not near base +Z")
    parser.add_argument("--output", default="outputs/vision/auto-cube-pick-place-plan.json")
    args = parser.parse_args()
    if not 40.0 <= args.cube_edge_mm <= 60.0 or not 40.0 <= args.hover_mm <= 150.0:
        parser.error("cube edge must be 40..60 mm and hover must be 40..150 mm")
    observation, observation_path = load(args.observation, json.load)
    if observation.get("status") != "candidate_not_motion_authorized":
        raise SystemExit("observation did not pass the read-only geometry gate")
    grasp, grasp_path = load(args.grasp_center, yaml.safe_load)
    if grasp.get("parent_frame") != "tool0" or grasp.get("orientation") != "inherits_tool0":
        raise SystemExit("grasp-centre calibration must be a tool0 position-only measurement")
    offset = np.asarray(grasp.get("translation_m"), dtype=float)
    if offset.shape != (3,):
        raise SystemExit("grasp-centre translation_m must have 3 values")
    cube = np.asarray(observation["estimated_cube_center_base_m"], dtype=float)
    board = np.asarray(observation["board_center_base_m"], dtype=float)
    normal = np.asarray(observation["board_normal_base"], dtype=float)
    normal /= np.linalg.norm(normal)
    if normal[2] < 0:
        normal *= -1.0
    tilt = float(np.degrees(np.arccos(np.clip(normal[2], -1.0, 1.0))))
    if tilt > args.max_board_tilt_deg:
        raise SystemExit("board normal is %.2f deg from base +Z; top-down base-Z executor is unsafe" % tilt)
    # tool0 +Z is the physical outward gripper chain, therefore it points down
    # during a vertical pinch.  Align tool X to the board grid for placement yaw.
    z_axis = -normal
    x_axis = np.asarray(observation["board_x_axis_base"], dtype=float)
    x_axis -= z_axis * float(x_axis @ z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    rotation = np.column_stack((x_axis, y_axis, z_axis))
    half = args.cube_edge_mm / 2000.0
    hover = args.hover_mm / 1000.0
    pick_center, place_center = cube, board + normal * half
    pick_tool = pick_center - rotation @ offset
    place_tool = place_center - rotation @ offset
    document = {
        "schema_version": 1,
        "kind": "auto_cube_pick_place_plan",
        "motion_sent": False,
        "authorization": "none: inspect and gate before any --execute motion",
        "inputs": {"observation": os.path.relpath(observation_path, ROOT),
                   "grasp_center": os.path.relpath(grasp_path, ROOT)},
        "cube_edge_m": args.cube_edge_mm / 1000.0,
        "board_normal_base": normal.tolist(), "board_tilt_deg": tilt,
        "gripper_center_pick_base_m": pick_center.tolist(),
        "gripper_center_place_base_m": place_center.tolist(),
        "tool_orientation_source": "tool +Z down; tool X parallel to checkerboard X",
        "pick": pose(rotation, pick_tool), "place": pose(rotation, place_tool),
        "height": hover,
        "execution_limits": {"speed_m_s": 0.015, "acceleration_m_s2": 0.04,
                             "position_tolerance_m": 0.002},
    }
    output = args.output if os.path.isabs(args.output) else os.path.join(ROOT, args.output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "w", encoding="utf-8") as stream:
        json.dump(document, stream, ensure_ascii=False, indent=2); stream.write("\n")
    print("READ-ONLY plan:", output)
    print("cube -> board centre (m):", np.round(pick_center, 4).tolist(), "->", np.round(place_center, 4).tolist())


if __name__ == "__main__":
    main()
