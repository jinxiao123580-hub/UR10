#!/usr/bin/env python3
"""Fit archived partial cube clouds to an archived full-view cube template.

This is deliberately a *bounded* refinement, not an object detector: the full
template supplies cube size and the last reliable base-frame centre; each later
partial cloud may translate by at most ``--max-shift-mm``.  The output is useful
for checking drift while the camera no longer sees a complete top face, but it
does not authorize contact, grasp closure, or an unbounded re-localisation.
"""
import argparse
import json
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def path(value):
    return value if os.path.isabs(value) else os.path.join(ROOT, value)


def load_report(value):
    with open(path(value), encoding="utf-8") as stream:
        return json.load(stream)


def base_points(npz_path):
    data = np.load(path(npz_path))
    points = np.asarray(data["camera_points_m"], dtype=np.float64)
    transform = np.asarray(data["base_from_camera"], dtype=np.float64)
    return points @ transform[:3, :3].T + transform[:3, 3]


def saved_paths(report):
    values = [row.get("saved_cloud") for row in report.get("observations", [])]
    values = [value for value in values if value]
    if not values:
        raise RuntimeError("report has no archived raw clouds")
    return values


def sampled(points, count, rng):
    if len(points) <= count:
        return points
    return points[rng.choice(len(points), size=count, replace=False)]


def cube_only(points, centre, table_z, half=0.065):
    """Bounded crop: excludes table but retains any visible top/side surface."""
    keep = ((np.abs(points[:, 0] - centre[0]) < half) &
            (np.abs(points[:, 1] - centre[1]) < half) &
            (points[:, 2] > table_z + 0.001) &
            (points[:, 2] < table_z + 0.075))
    return points[keep]


def robust_icp(source, target, bound_m, iterations=12):
    """Translation-only, trimmed ICP with a hard prior bound around zero."""
    tree = cKDTree(target)
    delta = np.zeros(3)
    history = []
    for _ in range(iterations):
        distances, indices = tree.query(source + delta, k=1, workers=-1)
        cutoff = np.percentile(distances, 80.0)
        keep = distances <= cutoff
        residual = target[indices[keep]] - (source[keep] + delta)
        update = np.median(residual, axis=0)
        candidate = np.clip(delta + update, -bound_m, bound_m)
        history.append({"trimmed_p80_mm": float(cutoff * 1000.0),
                        "delta_m": candidate.tolist(), "points": int(keep.sum())})
        if np.linalg.norm(candidate - delta) < 0.00005:
            delta = candidate
            break
        delta = candidate
    distances, _ = tree.query(source + delta, k=1, workers=-1)
    kept = distances <= np.percentile(distances, 80.0)
    return delta, distances, kept, history


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--template", required=True, help="tracked_stable full-view report")
    parser.add_argument("--partial", required=True, help="archived partial-view report")
    parser.add_argument("--max-shift-mm", type=float, default=5.0)
    parser.add_argument("--max-p80-mm", type=float, default=2.0,
                        help="required 80th-percentile nearest-template residual")
    parser.add_argument("--output", default=None)
    args = parser.parse_args()
    if not 0.5 <= args.max_shift_mm <= 20.0:
        parser.error("--max-shift-mm must be within 0.5..20")
    template = load_report(args.template)
    partial = load_report(args.partial)
    if template.get("status") != "tracked_stable":
        raise RuntimeError("template must be a stable complete cube observation")
    centre = np.asarray(template["center_base_m"], dtype=float)
    geometry = [row["geometry"] for row in template["observations"]
                if row.get("status") == "measured"]
    if not geometry:
        raise RuntimeError("template has no measured geometry")
    height = float(np.median([row["measured_height_m"] for row in geometry]))
    table_z = float(centre[2] - height / 2.0)
    rng = np.random.default_rng(20260922)
    template_points = np.concatenate([base_points(value) for value in saved_paths(template)])
    partial_points = np.concatenate([base_points(value) for value in saved_paths(partial)])
    model = cube_only(template_points, centre, table_z)
    source = cube_only(partial_points, centre, table_z)
    if len(model) < 1000 or len(source) < 1000:
        raise RuntimeError("insufficient archived cube-surface points for fitting")
    model = sampled(model, 50000, rng)
    source = sampled(source, 50000, rng)
    delta, distances, kept, history = robust_icp(source, model,
                                                  args.max_shift_mm / 1000.0)
    p50, p80, p95 = np.percentile(distances, [50, 80, 95]) * 1000.0
    bounded = bool(np.max(np.abs(delta)) < args.max_shift_mm / 1000.0 - 0.00005)
    status = "partial_fit_consistent" if bounded and p80 <= args.max_p80_mm else "partial_fit_rejected"
    document = {
        "schema_version": 1,
        "mode": "bounded_translation_only_partial_cloud_fit",
        "status": status,
        "motion_authorization": "none: partial-cloud consistency/refinement only; never authorizes contact or grasp",
        "template": args.template, "partial": args.partial,
        "template_center_base_m": centre.tolist(),
        "fitted_center_base_m": (centre + delta).tolist(),
        "translation_correction_mm": (delta * 1000.0).tolist(),
        "max_shift_mm": args.max_shift_mm,
        "table_z_base_m": table_z,
        "template_cube_surface_points": int(len(model)),
        "partial_cube_surface_points": int(len(source)),
        "nearest_template_residual_mm": {"p50": float(p50), "p80": float(p80), "p95": float(p95)},
        "bound_not_hit": bounded, "iterations": history,
        "limitations": ["translation only; cube yaw is not observable from a symmetric partial view",
                        "template and partial clouds must be from an unchanged scene",
                        "a rejected strict full-cube observation remains rejected for contact planning"],
    }
    stamp = os.path.basename(path(args.partial)).replace("cube-track-", "").replace(".json", "")
    output = path(args.output or "outputs/vision/cube-partial-fit-%s.json" % stamp)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "w", encoding="utf-8") as stream:
        json.dump(document, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(json.dumps({key: document[key] for key in ("status", "fitted_center_base_m",
                                                      "translation_correction_mm",
                                                      "nearest_template_residual_mm", "bound_not_hit")},
                     ensure_ascii=False, indent=2))
    print("OUTPUT:", output)
    return 0 if status == "partial_fit_consistent" else 3


if __name__ == "__main__":
    sys.exit(main())
