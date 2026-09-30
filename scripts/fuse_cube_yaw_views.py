#!/usr/bin/env python3
"""Read-only consensus of archived cube top-face observations across camera views.

A rejected three-frame view may contain individually valid measurements.  This
uses them only when two separate camera viewpoints agree on the 50 mm top-face
orientation and position.  It never upgrades a translation-only partial fit.
"""
import argparse
import json
import os

import numpy as np

from plan_oriented_cube_transfer import cube_yaw_from_track

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def absolute(path):
    return path if os.path.isabs(path) else os.path.join(ROOT, path)


def build_consensus(initial, manifest):
    initial_rows = [row for row in initial.get("observations", [])
                    if row.get("status") == "measured" and row.get("center_base_m")]
    if initial.get("status") == "tracked_stable" and initial.get("center_base_m"):
        anchor = np.asarray(initial["center_base_m"], dtype=float)
    elif initial_rows:
        # The initial three-frame batch may contain dropouts.  Use its valid
        # measurement only as an identity cross-check; final centre and yaw
        # still come from independent active-view consensus below.
        anchor = np.median(np.asarray([row["center_base_m"] for row in initial_rows],
                                      dtype=float), axis=0)
    else:
        raise ValueError("initial hover has no measured cube centre to cross-check")
    if (manifest.get("kind") != "active_cube_view_capture_manifest" or
            not manifest.get("complete") or not manifest.get("motion_sent")):
        raise ValueError("active-view manifest is incomplete")
    rows = []
    for view in manifest.get("views", []):
        with open(absolute(view["report"]), encoding="utf-8") as stream:
            report = json.load(stream)
        for row in report.get("observations", []):
            if row.get("status") != "measured":
                continue
            footprint = ((row.get("geometry") or {}).get("top_face") or {}).get("footprint") or {}
            if float(footprint.get("coverage", 0)) < 0.55:
                continue
            selected = dict(row)
            selected["view_slot"] = view["slot"]
            rows.append(selected)
    result = {"schema_version": 1, "mode": "boardless_multi_view_yaw_consensus",
              "status": "tracked_stable", "motion_authorization": "none: read-only yaw evidence",
              "anchor_center_base_m": anchor.tolist(),
              "center_base_m": anchor.tolist(),
              "measured_captures": len(rows), "observations": rows,
              "source_manifest": manifest.get("plan")}
    yaw, spread, angles = cube_yaw_from_track(result)
    centres = np.asarray([row["center_base_m"] for row in rows], dtype=float)
    result["center_base_m"] = np.median(centres, axis=0).tolist()
    result["yaw_consensus"] = {"yaw_modulo_90_deg": yaw,
                                "spread_deg": spread, "angles_deg": angles,
                                "fused_center_method": "coordinatewise median of accepted multi-view measurements",
                                "high_coverage_captures": sum(
                                    row["geometry"]["top_face"]["footprint"]["coverage"] >= 0.85
                                    for row in rows)}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--initial-track", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    with open(absolute(args.initial_track), encoding="utf-8") as stream:
        initial = json.load(stream)
    with open(absolute(args.manifest), encoding="utf-8") as stream:
        manifest = json.load(stream)
    try:
        result = build_consensus(initial, manifest)
    except (ValueError, KeyError, TypeError, OSError) as exc:
        raise SystemExit("multi-view cube yaw rejected: %s" % exc)
    output = absolute(args.output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "w", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print("multi-view cube yaw: %.2f deg, spread %.2f deg, %d captures -> %s" %
          (result["yaw_consensus"]["yaw_modulo_90_deg"],
           result["yaw_consensus"]["spread_deg"], len(result["observations"]), output))


if __name__ == "__main__":
    main()
