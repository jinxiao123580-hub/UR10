#!/usr/bin/env python3
"""Merge archived boardless active-view clouds for bounded cube fitting."""
import argparse
import json
import os
from datetime import datetime

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def absolute(value):
    return value if os.path.isabs(value) else os.path.join(ROOT, value)


def read_report(value):
    with open(absolute(value), encoding="utf-8") as stream:
        return json.load(stream)


def view_angle_deg(left, right):
    a, _ = cv2.Rodrigues(np.asarray(left["observations"][0]["tcp_base_tool0"][3:], dtype=float))
    b, _ = cv2.Rodrigues(np.asarray(right["observations"][0]["tcp_base_tool0"][3:], dtype=float))
    return float(np.linalg.norm(cv2.Rodrigues(a.T @ b)[0]) * 180.0 / np.pi)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--prior-track", default=None,
                        help="stable boardless hover report from this same run; used if an active view timed out")
    parser.add_argument("--output", default="outputs/vision/active-cube-views-merged.json")
    args = parser.parse_args()
    with open(absolute(args.manifest), encoding="utf-8") as stream:
        manifest = json.load(stream)
    if (manifest.get("kind") != "active_cube_view_capture_manifest" or
            not manifest.get("motion_sent") or not manifest.get("complete")):
        raise SystemExit("requires a complete real active-view capture manifest")
    accepted = []
    skipped = []
    if args.prior_track:
        prior = read_report(args.prior_track)
        if (prior.get("mode") != "boardless_local_cube_tracking" or
                prior.get("status") != "tracked_stable"):
            raise SystemExit("prior hover track is not a stable boardless cube observation")
        accepted.append(("prior_hover", args.prior_track, prior))
    for item in manifest["views"]:
        # A failed capture can leave last run's view-XX.json on disk.  Never
        # read it merely because that path exists.
        if item.get("tracker_returncode") != 0:
            skipped.append({"slot": item["slot"], "reason": "tracker failed in this run"})
            continue
        report = read_report(item["report"])
        if (report.get("mode") != "boardless_local_cube_tracking" or
                report.get("status") != "tracked_stable"):
            skipped.append({"slot": item["slot"], "reason": "no stable cube centre"})
            continue
        accepted.append(("view_%d" % item["slot"], item["report"], report))
    if len(accepted) < 2 or not any(label.startswith("view_") for label, _, _ in accepted):
        raise SystemExit("need two current stable viewpoints; timed-out/stale reports cannot be merged")
    observations = []
    centres = []
    times = []
    for label, source, report in accepted:
        rows = report.get("observations", [])
        if (int(report.get("measured_captures", 0)) < 3 or
                sum(bool(row.get("saved_cloud")) for row in rows) < 3 or
                not report.get("center_base_m")):
            raise SystemExit("%s lacks three measured, archived clouds" % label)
        observations.extend(rows)
        centres.append(report["center_base_m"])
        times.append(datetime.fromisoformat(report["measured_at"]))
    if (max(times) - min(times)).total_seconds() > 600:
        raise SystemExit("view reports are more than 10 minutes apart; reject stale evidence")
    angles = [view_angle_deg(accepted[i][2], accepted[j][2])
              for i in range(len(accepted)) for j in range(i + 1, len(accepted))]
    if max(angles) < 10.0:
        raise SystemExit("accepted views have less than 10 deg camera orientation separation")
    saved = sum(bool(row.get("saved_cloud")) for row in observations)
    centres = np.asarray(centres, dtype=float)
    mean = centres.mean(axis=0)
    spread = np.linalg.norm(centres - mean, axis=1) * 1000.0
    pairwise = max(float(np.linalg.norm(centres[i] - centres[j]) * 1000.0)
                   for i in range(len(centres)) for j in range(i + 1, len(centres)))
    if pairwise > 3.0:
        raise SystemExit("view-to-view cube centre distance %.2f mm exceeds 3.00 mm" % pairwise)
    document = {"schema_version": 1, "kind": "active_cube_multi_view_cloud_archive",
                "motion_sent": False, "status": "archived_for_bounded_fit",
                "source_manifest": args.manifest, "observations": observations,
                "raw_cloud_count": saved,
                "accepted_sources": [{"kind": label, "report": source} for label, source, _ in accepted],
                "skipped_views": skipped, "max_view_angle_deg": max(angles),
                "view_centers_base_m": centres.tolist(), "view_center_mean_base_m": mean.tolist(),
                "view_center_spread_mm": {"max_from_mean": float(spread.max()),
                                           "per_view": spread.tolist(),
                                           "max_pairwise": pairwise, "limit": 3.0},
                "runtime_board_usage": "none: only archived boardless clouds"}
    output = absolute(args.output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "w", encoding="utf-8") as stream:
        json.dump(document, stream, ensure_ascii=False, indent=2); stream.write("\n")
    print("merged active-view clouds:", saved, output)


if __name__ == "__main__":
    main()
