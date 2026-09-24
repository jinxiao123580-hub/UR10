#!/usr/bin/env python3
"""Update only a pick target from a boardless tracked cube observation.

The board centre and the board-aligned tool orientation are frozen from the
first plan.  The second observation intentionally uses no image/chessboard;
it changes only the measured cube-centre translation before descent.
"""
import argparse
import json
import os

import cv2
import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load(path):
    full = path if os.path.isabs(path) else os.path.join(ROOT, path)
    with open(full, encoding="utf-8") as stream:
        return json.load(stream), full


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", required=True, help="initial board-aligned plan")
    parser.add_argument("--track", required=True, help="boardless tracked cube JSON")
    parser.add_argument("--grasp-center", default="config/gripper_grasp_center_20260922.yaml")
    parser.add_argument("--max-correction-mm", type=float, default=20.0)
    parser.add_argument("--grasp-depth-below-top-mm", type=float, default=-25.0,
                        help="closed-jaw level measured down from point-cloud cube top (negative: above top; default: +25 mm)")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    plan, plan_path = load(args.plan)
    track, track_path = load(args.track)
    if plan.get("kind") != "auto_cube_pick_place_plan":
        raise SystemExit("initial input is not an auto cube pick/place plan")
    if track.get("status") not in ("tracked_stable", "partial_fit_consistent"):
        raise SystemExit("boardless cube tracking / partial fit did not pass its gate")
    grasp_path = args.grasp_center if os.path.isabs(args.grasp_center) else os.path.join(ROOT, args.grasp_center)
    with open(grasp_path, encoding="utf-8") as stream:
        offset = np.asarray((yaml.safe_load(stream) or {})["translation_m"], dtype=float)
    old_pick = np.asarray(plan["pick"], dtype=float)
    rotation, _ = cv2.Rodrigues(old_pick[3:])
    updated_center = np.asarray(track.get("center_base_m") or
                                track.get("fitted_center_base_m"), dtype=float)
    old_center = np.asarray(plan.get("cube_center_base_m") or
                            plan["gripper_center_pick_base_m"], dtype=float)
    correction_mm = float(np.linalg.norm(updated_center - old_center) * 1000.0)
    if correction_mm > args.max_correction_mm:
        raise SystemExit("boardless correction %.2f mm exceeds %.2f mm limit" %
                         (correction_mm, args.max_correction_mm))
    if not -30.0 <= args.grasp_depth_below_top_mm <= 20.0:
        parser.error("grasp depth below top must be -30..20 mm")
    heights = [row.get("geometry", {}).get("measured_height_m")
               for row in track.get("observations", [])
               if row.get("geometry", {}).get("measured_height_m") is not None]
    height = float(np.median(heights)) if heights else float(plan.get("cube_edge_m", 0.05))
    top_z = float(updated_center[2] + height / 2.0)
    grasp_center = updated_center.copy()
    grasp_center[2] = top_z - args.grasp_depth_below_top_mm / 1000.0
    updated = dict(plan)
    updated["pick"] = np.r_[grasp_center - rotation @ offset, old_pick[3:]].tolist()
    # Keep the geometry centre distinct from the intentionally higher pinch
    # point.  Tracking/template fitting uses the former; the controller uses
    # the latter.
    updated["cube_center_base_m"] = updated_center.tolist()
    updated["gripper_center_pick_base_m"] = grasp_center.tolist()
    updated["pointcloud_grasp_level"] = {"cube_height_m": height,
        "cube_top_z_base_m": top_z,
        "depth_below_top_mm": args.grasp_depth_below_top_mm,
        "height_above_top_mm": -args.grasp_depth_below_top_mm,
        "gripper_center_z_base_m": float(grasp_center[2]),
        "policy": ("pointcloud_top_plus_25mm" if args.grasp_depth_below_top_mm == -25
                   else ("pointcloud_top_surface" if args.grasp_depth_below_top_mm == 0
                         else "custom_top_relative_grasp"))}
    updated["second_observation"] = {
        "mode": "boardless_cube_cloud_only",
        "board_usage": "none; board centre and orientation frozen from initial plan",
        "track": os.path.relpath(track_path, ROOT),
        "translation_correction_mm": correction_mm,
    }
    updated["motion_sent"] = False
    output = args.output if os.path.isabs(args.output) else os.path.join(ROOT, args.output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "w", encoding="utf-8") as stream:
        json.dump(updated, stream, ensure_ascii=False, indent=2); stream.write("\n")
    print("boardless pick refresh:", output, "correction mm:", round(correction_mm, 3))


if __name__ == "__main__":
    main()
