#!/usr/bin/env python3
"""P0-B: decompose the 2026-09-17 cube-centre error into a difference VECTOR.

Read-only, offline. It never talks to the camera or the robot: it reuses the
captured candidate JSON, the accepted eye-in-hand calibration and the manually
confirmed grasp point recorded in
``experiments/2026-09-17-视觉候选人工修正抓取.md``.

The question it answers: the candidate centre was 35.84 mm away from the
manually confirmed centre in the base XY plane.  Reporting only the norm hides
which step of the "single visible side + known edge length" inference was
wrong, so this tool prints the error in the base frame *and* in the
checkerboard (target) frame where that inference actually happens.
"""
import argparse
import json
import os
from datetime import datetime

import numpy as np
import yaml

from solve_handeye_checkerboard import rt


def load_frame_chain(candidate, calibration):
    """Return base_from_target and the camera_from_target matrix used on 2026-09-17."""
    tcp = np.asarray(candidate["robot_motion"]["base_to_tool0_tcp_mean"], dtype=np.float64)
    base_from_tool = rt(tcp[3:], tcp[:3])
    tool_from_camera = np.eye(4)
    tool_from_camera[:3, :3] = np.asarray(calibration["rotation_matrix"], dtype=np.float64)
    tool_from_camera[:3, 3] = np.asarray(calibration["translation_m"], dtype=np.float64)
    camera_from_target = np.asarray(candidate["checkerboard_target_to_camera"], dtype=np.float64)
    return base_from_tool @ tool_from_camera @ camera_from_target, camera_from_target


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate", default="outputs/vision/cube-candidate-20260917.json")
    parser.add_argument("--calibration", default="config/handeye_eye_in_hand_20260917.yaml")
    # Manually confirmed grasp TCP from experiments/2026-09-17-视觉候选人工修正抓取.md
    parser.add_argument("--confirmed-tcp", type=float, nargs=3,
                        default=[0.66161, -0.11659, 0.30468])
    parser.add_argument("--cube-edge", type=float, default=0.05)
    parser.add_argument("--output", default="outputs/vision/center-error-diagnosis-20260918.json")
    args = parser.parse_args()

    with open(os.path.join(root, args.candidate), encoding="utf-8") as stream:
        candidate = json.load(stream)
    with open(os.path.join(root, args.calibration), encoding="utf-8") as stream:
        calibration = yaml.safe_load(stream)

    base_from_target, camera_from_target = load_frame_chain(candidate, calibration)
    target_from_base = np.linalg.inv(base_from_target)

    estimated_base = np.asarray(candidate["estimated_cube_center_base_m"], dtype=np.float64)
    confirmed_base = np.asarray(args.confirmed_tcp, dtype=np.float64)

    delta_base = confirmed_base[:2] - estimated_base[:2]
    delta_norm = float(np.linalg.norm(delta_base))

    def to_target(point_xyz):
        return (target_from_base @ np.asarray([*point_xyz, 1.0]))[:3]

    estimated_target = to_target(estimated_base)
    # The confirmed grasp point fixes the cube centre in XY; its height above the
    # board plane is unknown, so the target-frame comparison is XY + signed Z
    # comment only.
    confirmed_target_xy = to_target(confirmed_base)[:2]

    inferred_target = np.asarray(candidate["inference"]["cube_center_target_m"], dtype=np.float64)
    best = candidate["best"]
    side_center_target = np.asarray(best["center_target_m"], dtype=np.float64)
    extent = np.asarray(best["extent_p5_p95_m"], dtype=np.float64)

    # Error of the inference in the frame where it was made (XY of the board plane).
    delta_target_xy = confirmed_target_xy - inferred_target[:2]

    # What the "single side + half edge" step predicts on each board-plane axis.
    half = args.cube_edge / 2.0
    lateral_axis = int(np.argmin(extent[:2]))
    measured_axis_gap = float(side_center_target[lateral_axis])

    report = {
        "schema_version": 1,
        "generated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "purpose": "P0-B difference-vector diagnosis of the 2026-09-17 cube centre error",
        "inputs": {
            "candidate": args.candidate,
            "calibration": args.calibration,
            "confirmed_grasp_tcp_base_m": confirmed_base.tolist(),
            "assumed_cube_edge_m": args.cube_edge,
        },
        "base_frame": {
            "estimated_center_xy_m": estimated_base[:2].tolist(),
            "confirmed_center_xy_m": confirmed_base[:2].tolist(),
            "difference_vector_xy_m": delta_base.tolist(),
            "difference_norm_mm": delta_norm * 1000.0,
            "confirmed_minus_estimated_direction": "X+ / Y-" if delta_base[0] > 0 > delta_base[1] else "other",
        },
        "target_frame": {
            "base_from_target_rotation": base_from_target[:3, :3].tolist(),
            "board_plane_tilt_deg": float(np.degrees(np.arccos(
                np.clip((base_from_target[:3, :3] @ np.array([0.0, 0.0, 1.0]))[2], -1.0, 1.0)))),
            "inferred_center_m": inferred_target.tolist(),
            "visible_side_center_m": side_center_target.tolist(),
            "confirmed_center_xy_m": confirmed_target_xy.tolist(),
            "difference_vector_xy_m": delta_target_xy.tolist(),
            "difference_norm_xy_mm": float(np.linalg.norm(delta_target_xy) * 1000.0),
            "visible_side_lateral_axis": "xy"[lateral_axis],
            "visible_side_extent_p5_p95_m": extent.tolist(),
        },
        "inference_steps_checked": {
            "side_plane_normal_offset_m": half,
            "side_plane_measured_axis_value_m": measured_axis_gap,
            "forced_vertical_center_m": float(inferred_target[2]),
            "vertical_center_sign_note": (
                "locator forces z = sign*0.025; sign=%s from the component search, so the "
                "cube centre was placed %s the board plane"
                % (best["sign"], "below" if inferred_target[2] < 0 else "above")),
            "board_center_constant_used_for_outward_sign_m": [0.024, 0.015],
            "expected_error_if_two_axes_each_off_by_half_edge_mm": float(
                np.hypot(half, half) * 1000.0),
            "expected_error_if_one_axis_off_by_half_edge_mm": half * 1000.0,
        },
        "camera_side_check": None,
        "sign_corrected_estimate": None,
        "ground_truth_caveats": None,
        "verdict": None,
    }

    # ---- which side of the visible face can the body be on? -----------------
    # A convex body only shows the face whose outward normal points at the camera, so the
    # cube centre must lie on the far side of the measured face plane.  The archived
    # locator instead used sign(side_centre - board_centre), which is meaningless here.
    camera_origin_target = np.linalg.inv(camera_from_target)[:3, 3]
    camera_axis_value = float(camera_origin_target[lateral_axis])
    camera_side = 1.0 if camera_axis_value > measured_axis_gap else -1.0
    correct_center = inferred_target.copy()
    correct_center[lateral_axis] = measured_axis_gap - camera_side * half
    correct_center_xy_base = (base_from_target @ np.asarray([*correct_center, 1.0]))[:2]
    residual_vs_confirmed = confirmed_base[:2] - correct_center_xy_base
    report["camera_side_check"] = {
        "lateral_axis": "xy"[lateral_axis],
        "camera_origin_target_m": camera_origin_target.tolist(),
        "camera_axis_value_m": camera_axis_value,
        "visible_face_axis_value_m": measured_axis_gap,
        "camera_is_on": "+%s" % "xy"[lateral_axis] if camera_side > 0 else "-%s" % "xy"[lateral_axis],
        "archived_locator_offset_sign": "+" if inferred_target[lateral_axis] > measured_axis_gap else "-",
        "sign_is_consistent": bool(
            (inferred_target[lateral_axis] - measured_axis_gap) * camera_side < 0),
        "consequence_mm": float(2 * half * 1000.0),
    }
    report["sign_corrected_estimate"] = {
        "note": "same archived cloud, only the offset sign corrected by the camera side",
        "center_target_m": correct_center.tolist(),
        "center_xy_base_m": correct_center_xy_base.tolist(),
        "residual_vs_confirmed_grasp_xy_mm": float(np.linalg.norm(residual_vs_confirmed) * 1000.0),
        "residual_vector_xy_mm": (residual_vs_confirmed * 1000.0).tolist(),
    }
    report["ground_truth_caveats"] = {
        "confirmed_point_uncertainty": (
            "the 35.84 mm reference is ONE by-eye pendant correction; the gripper can close "
            "with the cube several millimetres off centre, so its own uncertainty is not "
            "quantified and it cannot serve as an acceptance reference"),
        "visible_face_size_measured": {
            "y_span_p5_p95_mm": float(extent[1] * 1000.0),
            "z_span_p5_p95_mm": float(extent[2] * 1000.0),
            "nominal_edge_mm": args.cube_edge * 1000.0,
            "note": ("only %.1f mm of the 50 mm nominal face was seen in y, and the height "
                     "mask clipped the face to >=20 mm, so neither the 50 mm size prior nor "
                     "the half-edge value was verified against the measurement"
                     % (extent[1] * 1000.0)),
        },
        "conclusion": (
            "the sign error alone explains 50 mm along one axis; the residual ~%.1f mm against "
            "the manual point is inconsistent with any single-face reconstruction of a 50 mm "
            "cube, so the centre must be MEASURED (top face + footprint), not inferred, and "
            "accepted against an independent ground truth at several cube placements"
            % (np.linalg.norm(residual_vs_confirmed) * 1000.0)),
    }

    norm_mm = delta_norm * 1000.0
    two_axis_mm = float(np.hypot(half, half) * 1000.0)
    one_axis_mm = half * 1000.0
    axes = np.abs(delta_target_xy) * 1000.0
    report["verdict"] = {
        "measured_norm_mm": norm_mm,
        "matched_two_half_edge_hypothesis": bool(abs(norm_mm - two_axis_mm) < 2.0),
        "matched_one_half_edge_hypothesis": bool(abs(norm_mm - one_axis_mm) < 2.0),
        "target_frame_axis_errors_mm": axes.tolist(),
        "note": "a two-axis half-edge error would show BOTH target-frame axes near 25 mm",
    }

    output = args.output if os.path.isabs(args.output) else os.path.join(root, args.output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    print("OUTPUT:", output)


if __name__ == "__main__":
    main()
