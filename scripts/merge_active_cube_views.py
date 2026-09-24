#!/usr/bin/env python3
"""Merge archived boardless active-view clouds for bounded cube fitting."""
import argparse
import json
import os

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def absolute(value):
    return value if os.path.isabs(value) else os.path.join(ROOT, value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--output", default="outputs/vision/active-cube-views-merged.json")
    args = parser.parse_args()
    with open(absolute(args.manifest), encoding="utf-8") as stream:
        manifest = json.load(stream)
    if (manifest.get("kind") != "active_cube_view_capture_manifest" or
            not manifest.get("motion_sent") or not manifest.get("complete")):
        raise SystemExit("requires a complete real active-view capture manifest")
    observations = []
    centres = []
    for item in manifest["views"]:
        with open(absolute(item["report"]), encoding="utf-8") as stream:
            report = json.load(stream)
        observations.extend(report.get("observations", []))
        if report.get("status") != "tracked_stable" or not report.get("center_base_m"):
            raise SystemExit("view %d did not produce a stable cube centre" % item["slot"])
        centres.append(report["center_base_m"])
    saved = sum(bool(row.get("saved_cloud")) for row in observations)
    if saved < 3:
        raise SystemExit("fewer than three raw clouds were archived")
    centres = np.asarray(centres, dtype=float)
    mean = centres.mean(axis=0)
    spread = np.linalg.norm(centres - mean, axis=1) * 1000.0
    if float(spread.max()) > 3.0:
        raise SystemExit("view-to-view cube centre spread %.2f mm exceeds 3.00 mm" % spread.max())
    document = {"schema_version": 1, "kind": "active_cube_multi_view_cloud_archive",
                "motion_sent": False, "status": "archived_for_bounded_fit",
                "source_manifest": args.manifest, "observations": observations,
                "raw_cloud_count": saved,
                "view_centers_base_m": centres.tolist(), "view_center_mean_base_m": mean.tolist(),
                "view_center_spread_mm": {"max_from_mean": float(spread.max()),
                                           "per_view": spread.tolist(), "limit": 3.0},
                "runtime_board_usage": "none: only archived boardless clouds"}
    output = absolute(args.output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "w", encoding="utf-8") as stream:
        json.dump(document, stream, ensure_ascii=False, indent=2); stream.write("\n")
    print("merged active-view clouds:", saved, output)


if __name__ == "__main__":
    main()
