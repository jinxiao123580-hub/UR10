#!/usr/bin/env python3
"""Offline design gate: does the planned shot list actually excite the solve?

Before any sample is taken, this builds the *predicted* sample set for each roll
mode from one reference sample plus a guidance calibration, then runs the same
observability diagnostic the solver reports.  It answers two questions with
numbers instead of opinion:

1. is the pose distribution non-degenerate (rank-8 rotation design, healthy
   null-space gap, relative rotations spread over more than one axis)?
2. does pinning the roll to the flange-clearance optimum cost observability
   compared with the spread-roll design?

Needs no camera and no robot.  The absolute transforms are guidance-grade; only
the *geometry of the distribution* is being judged here.
"""
import argparse
import datetime as dt
import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from handeye_pose_advisor import (build_plan, target_from_camera_from_spherical,
                                  choose_roll, load_reference,
                                  estimate_base_from_target,
                                  spherical_from_target_from_camera)
from solve_handeye_checkerboard import observability, rt  # noqa: E402

CLEARANCE_ROLLS_DEG = np.arange(0.0, 360.0, 2.0)


def predicted_samples(slots, base_from_target, tool0_from_camera, roll_mode):
    samples = []
    for slot in slots:
        if roll_mode == "design":
            roll = slot["roll_deg"]
        elif roll_mode == "clearance":
            selection = choose_roll(slot["distance_m"], slot["tilt_deg"],
                                    slot["azimuth_deg"], base_from_target,
                                    tool0_from_camera, slot["roll_deg"])
            if selection is None:
                raise RuntimeError("no clearance-feasible roll for slot %s" %
                                   slot["slot"])
            roll = selection["roll_deg"]
        else:
            raise ValueError("unknown roll mode: %s" % roll_mode)
        target_from_camera = target_from_camera_from_spherical(
            slot["distance_m"], slot["tilt_deg"], slot["azimuth_deg"], roll)
        camera_from_target = np.linalg.inv(target_from_camera)
        base_from_camera = base_from_target @ target_from_camera
        base_from_tool0 = base_from_camera @ np.linalg.inv(tool0_from_camera)
        tcp_rvec, _ = cv2.Rodrigues(base_from_tool0[:3, :3])
        target_rvec, _ = cv2.Rodrigues(camera_from_target[:3, :3])
        samples.append({
            "sample_id": "predicted-%02d" % slot["slot"],
            "accepted": True,
            "robot": {"base_to_tool0_tcp_mean":
                      base_from_tool0[:3, 3].tolist() + tcp_rvec.reshape(3).tolist(),
                      "max_position_motion_mm": 0.0,
                      "max_rotation_motion_deg": 0.0},
            "target_to_camera": {
                "rvec_rad": target_rvec.reshape(3).tolist(),
                "translation_m": camera_from_target[:3, 3].tolist(),
                "reprojection_rms_px": 0.0},
            "roll_used_deg": roll,
            "flange_z_m": float(base_from_tool0[2, 3]),
        })
    return samples


def summarise(samples):
    report = observability(samples)
    flange = np.asarray([s["flange_z_m"] for s in samples])
    rolls = {round(s["roll_used_deg"], 1) for s in samples}
    distances = []
    tilts = []
    azimuths = []
    for sample in samples:
        target = sample["target_to_camera"]
        camera_from_target = rt(np.asarray(target["rvec_rad"], dtype=np.float64),
                                np.asarray(target["translation_m"], dtype=np.float64))
        values = spherical_from_target_from_camera(
            np.linalg.inv(camera_from_target))
        distances.append(values["distance_m"])
        tilts.append(values["tilt_deg"])
        azimuths.append(values["azimuth_deg"])
    distances = np.asarray(distances)
    # Count exact 45-degree sectors instead of histogramming: the planned
    # azimuths sit exactly on the bin edges, where floating point decides which
    # side a value lands and the counts come out wrong.
    sectors = [0] * 8
    for azimuth in azimuths:
        sectors[int(round(azimuth / 45.0)) % 8] += 1
    return {
        "observability": report,
        "flange_z_m": {"min": float(flange.min()), "max": float(flange.max()),
                       "mean": float(flange.mean())},
        "distinct_rolls": sorted(rolls),
        "distance_m": {"min": float(distances.min()), "max": float(distances.max())},
        "tilt_deg": {"min": float(min(tilts)), "max": float(max(tilts))},
        "azimuth_sectors_45deg": sectors,
    }


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", required=True,
                        help="one accepted sample.json or dataset, for guidance geometry")
    parser.add_argument("--calibration",
                        default="config/handeye_eye_in_hand_20260917.yaml")
    parser.add_argument("--historical-tcp-z-floor", type=float, default=0.202,
                        help="lowest flange height used by the 2026-09-17 set that "
                             "physically worked; a design that goes below it needs "
                             "on-site clearance confirmation")
    parser.add_argument("--output", default=None)
    args = parser.parse_args()

    import yaml
    with open(os.path.join(root, args.calibration), encoding="utf-8") as stream:
        calibration = yaml.safe_load(stream)
    tool0_from_camera = np.eye(4)
    tool0_from_camera[:3, :3] = np.asarray(calibration["rotation_matrix"],
                                           dtype=np.float64)
    tool0_from_camera[:3, 3] = np.asarray(calibration["translation_m"],
                                          dtype=np.float64)

    sample, source = load_reference(args.reference, root)
    base_from_target = estimate_base_from_target(sample, tool0_from_camera)
    slots = build_plan()

    modes = {}
    for mode in ("design", "clearance"):
        samples = predicted_samples(slots, base_from_target, tool0_from_camera, mode)
        modes[mode] = summarise(samples)

    document = {
        "schema_version": 1,
        "created_at": dt.datetime.now(dt.timezone.utc).astimezone().isoformat(),
        "kind": "plan_observability_gate",
        "guidance_only": True,
        "reference_sample": sample["sample_id"],
        "reference_source": source,
        "guidance_calibration": args.calibration,
        "board_origin_base_estimate_m": base_from_target[:3, 3].tolist(),
        "historical_tcp_z_floor_m": args.historical_tcp_z_floor,
        "modes": modes,
    }

    print("%-10s %6s %14s %14s %9s %11s %8s %s" %
          ("roll mode", "rank", "null_space_gap", "cond", "axis_iso",
           "flange_min", "rolls", "azimuth sectors"))
    for mode, values in modes.items():
        rotation = values["observability"]["rotation_design"]
        isotropy = (values["observability"].get("relative_rotation_axis_isotropy")
                    or {}).get("min_over_max")
        gap = rotation["null_space_gap"]
        condition = rotation["condition_number"]
        gap_text = "exact (floor)" if gap is None else "%.2f" % gap
        cond_text = "exact (floor)" if condition is None else "%.1f" % condition
        print("%-10s %6d %14s %14s %9.3f %11.3f %8d %s" %
              (mode, rotation["rank_at_1e-9_relative"], gap_text, cond_text,
               isotropy, values["flange_z_m"]["min"],
               len(values["distinct_rolls"]),
               values["azimuth_sectors_45deg"]))

    clearance_min = modes["clearance"]["flange_z_m"]["min"]
    design_min = modes["design"]["flange_z_m"]["min"]
    print()
    print("法兰高度：clearance 模式最低 %.3f m，design 模式最低 %.3f m，"
          "09-17 实测地板 %.3f m" %
          (clearance_min, design_min, args.historical_tcp_z_floor))
    for mode, values in modes.items():
        risk = values["flange_z_m"]["min"] < args.historical_tcp_z_floor
        print("  %-10s %s" % (mode, "低于实测地板，需现场确认避障" if risk else "不低于实测地板"))

    output = args.output or os.path.join(
        root, "outputs", "handeye", "plan-observability-gate.json")
    with open(output, "w", encoding="utf-8") as stream:
        json.dump(document, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    print("OUTPUT:", output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
