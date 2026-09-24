#!/usr/bin/env python3
"""Read-only cube tracking from the live point cloud, without a checkerboard.

The hand-eye result is used only as a fixed camera-to-tool transform.  At run
time this script does not request an image, detect a checkerboard, or use board
corners.  It maps the cloud into base coordinates from the simultaneous UR
30013 pose, measures the 50 mm cube against the local table plane, and repeats
the observation to reject an unstable result.  It never commands the robot.
"""
import argparse
import json
import os
import sys
import time
from datetime import datetime

import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

from measure_cube_geometry import (DEFAULTS, _jsonable, measure_cube_geometry,
                                   rotation_from_rotvec)
from record_ur_trajectory import RealtimeReader
from validate_checkerboard_pointcloud import BoardCloudCapture, organized_xyz


def absolute(root, path):
    return path if os.path.isabs(path) else os.path.join(root, path)


def transform_from_tcp(tcp):
    transform = np.eye(4)
    transform[:3, :3] = rotation_from_rotvec(np.asarray(tcp[3:], dtype=float))
    transform[:3, 3] = np.asarray(tcp[:3], dtype=float)
    return transform


def one_observation(node, reader, tool_from_camera, anchor, args, index, stamp):
    """Capture cloud once with poses immediately before/after; require stillness."""
    _, before_tcp, _, _ = reader.read_extended()
    cloud = node.capture_cloud(args.timeout)
    _, after_tcp, _, _ = reader.read_extended()
    before = np.asarray(before_tcp, dtype=float)
    after = np.asarray(after_tcp, dtype=float)
    motion_mm = float(np.linalg.norm(after[:3] - before[:3]) * 1000.0)
    if motion_mm > args.max_robot_motion_mm:
        return {"status": "rejected", "reason": "robot moved during cloud capture",
                "motion_mm": motion_mm}

    # Pair the cloud with the midpoint tool pose.  The static gate above bounds
    # the timing error; this avoids falsely treating an old cached cloud as live.
    tcp = (before + after) / 2.0
    # UR may emit either sign of an equivalent near-pi rotation vector across
    # two static frames. Averaging those three numbers fabricates a zero (or
    # otherwise unrelated) orientation and maps the cloud into the wrong base
    # frame. Use the post-capture controller orientation; position averaging
    # remains valid under the stillness gate above.
    tcp[3:] = after[3:]
    base_from_camera = transform_from_tcp(tcp) @ tool_from_camera
    camera_points = organized_xyz(cloud).reshape(-1, 3)
    finite = camera_points[np.all(np.isfinite(camera_points), axis=1)]
    saved_cloud = None
    if args.save_cloud_dir:
        directory = absolute(ROOT, args.save_cloud_dir)
        os.makedirs(directory, exist_ok=True)
        saved_cloud = os.path.join(directory, "cube-track-%s-%02d.npz" % (stamp, index + 1))
        # Keep the untransformed camera-frame points plus the synchronized
        # extrinsic. This is sufficient to replay any later model fit without
        # another robot or camera capture, and avoids silently discarding a
        # partial observation just because today's strict geometry gate rejects it.
        np.savez_compressed(saved_cloud,
                            camera_points_m=np.asarray(finite, dtype=np.float32),
                            base_from_camera=np.asarray(base_from_camera, dtype=np.float64),
                            tcp_before_m_rad=before,
                            tcp_after_m_rad=after,
                            anchor_base_m=np.asarray(anchor, dtype=np.float64))
    points_base = finite @ base_from_camera[:3, :3].T + base_from_camera[:3, 3]

    # Geometry core uses a local table-aligned frame whose x/y origin is the
    # last verified cube centre. This is only a bounded local search, not a
    # board observation or a re-localisation of the workcell.
    local_origin = np.array([anchor[0], anchor[1], 0.0])
    points_local = points_base - local_origin
    camera_local = base_from_camera[:3, 3] - local_origin
    cfg = {"expect_edge_m": args.expect_edge, "min_coverage": args.min_coverage,
           "allow_partial": False}
    measured = measure_cube_geometry(points_local, camera_local, cfg,
                                     board_half_extent=[0.0, 0.0])
    result = {
        "status": measured["status"], "geometry": measured,
        "tcp_base_tool0": tcp.tolist(), "robot_motion_during_capture_mm": motion_mm,
        "finite_cloud_points": int(len(finite)),
        "camera_origin_base_m": base_from_camera[:3, 3].tolist(),
        "saved_cloud": (os.path.relpath(saved_cloud, ROOT) if saved_cloud else None),
    }
    if measured.get("center_target_m") is not None:
        result["center_base_m"] = (np.asarray(measured["center_target_m"]) + local_origin).tolist()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--anchor", required=True, help="previous verified cube measurement JSON")
    parser.add_argument("--calibration", default="config/handeye_eye_in_hand_20260921.yaml")
    parser.add_argument("--port", type=int, default=30013)
    parser.add_argument("--captures", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--max-robot-motion-mm", type=float, default=0.5)
    parser.add_argument("--max-center-spread-mm", type=float, default=3.0)
    parser.add_argument("--expect-edge", type=float, default=0.05)
    parser.add_argument("--min-coverage", type=float, default=DEFAULTS["min_coverage"])
    parser.add_argument("--output", default=None)
    parser.add_argument("--save-cloud-dir", default="outputs/vision/clouds",
                        help="directory for compressed raw camera clouds + synchronized transforms; "
                             "set to an empty string only when archival is intentionally disabled")
    args = parser.parse_args()
    if args.captures < 2:
        parser.error("--captures must be at least 2 for a stability check")

    with open(absolute(ROOT, args.anchor), encoding="utf-8") as stream:
        anchor_doc = json.load(stream)
    # The autonomous first-pass locator writes an explicit estimated centre;
    # older geometry measurements use centre_base_m.  Both are fixed anchors
    # for a *local* boardless re-observation, never a board measurement.
    anchor = (anchor_doc.get("cube_center_base_m") or
              anchor_doc.get("center_base_m") or
              anchor_doc.get("estimated_cube_center_base_m") or
              # An auto pick/place plan stores the same physical point under
              # its task-specific name; accepting it lets the hover refresh
              # localise only the cube without re-reading the checkerboard.
              anchor_doc.get("gripper_center_pick_base_m"))
    if not anchor or len(anchor) != 3:
        raise RuntimeError("anchor has no verified center_base_m")
    with open(absolute(ROOT, args.calibration), encoding="utf-8") as stream:
        handeye = yaml.safe_load(stream)
    if not handeye.get("valid") or handeye.get("transform_direction") != "tool0_from_camera":
        raise RuntimeError("calibration must be a valid tool0_from_camera transform")
    tool_from_camera = np.eye(4)
    tool_from_camera[:3, :3] = np.asarray(handeye["rotation_matrix"], dtype=float)
    tool_from_camera[:3, 3] = np.asarray(handeye["translation_m"], dtype=float)

    import rclpy
    reader = RealtimeReader("192.168.1.3", port=args.port)
    rclpy.init()
    node = BoardCloudCapture()  # Name is historical; only capture_cloud is called.
    observations = []
    capture_stamp = datetime.now().astimezone().strftime("%Y%m%d-%H%M%S")
    try:
        for index in range(args.captures):
            print("点云观测 %d/%d（不采图、不看标定板）..." % (index + 1, args.captures), flush=True)
            observations.append(one_observation(node, reader, tool_from_camera,
                                                np.asarray(anchor, dtype=float), args,
                                                index, capture_stamp))
    finally:
        reader.close()
        node.destroy_node()
        rclpy.shutdown()

    centers = np.asarray([row["center_base_m"] for row in observations
                          if row.get("status") == "measured"], dtype=float)
    result = {
        "schema_version": 1,
        "measured_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "mode": "boardless_local_cube_tracking",
        "runtime_board_usage": "none: cloud only; no image capture or checkerboard detection",
        "motion_authorization": "none: observation and validation only; no robot command was sent",
        "anchor_center_base_m": anchor,
        "calibration": args.calibration,
        "observations": observations,
        "required_captures": args.captures,
        "measured_captures": int(len(centers)),
        "max_center_spread_mm": args.max_center_spread_mm,
        "status": "rejected",
    }
    if len(centers) == args.captures:
        mean = centers.mean(axis=0)
        distances = np.linalg.norm(centers - mean, axis=1) * 1000.0
        result["center_base_m"] = mean.tolist()
        result["center_spread_mm"] = {"max_from_mean": float(distances.max()),
                                      "per_capture": distances.tolist()}
        if distances.max() <= args.max_center_spread_mm:
            result["status"] = "tracked_stable"
        else:
            result["reason"] = "cube centre is not stable across captures"
    else:
        result["reason"] = "one or more cloud observations failed geometric gates"

    stamp = datetime.now().astimezone().strftime("%Y%m%d-%H%M%S")
    output = absolute(ROOT, args.output or "outputs/vision/cube-track-%s.json" % stamp)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output + ".tmp", "w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False, default=_jsonable)
        stream.write("\n")
    os.replace(output + ".tmp", output)
    print(json.dumps({key: result.get(key) for key in ("status", "reason", "center_base_m",
                                                        "center_spread_mm", "measured_captures")},
                     ensure_ascii=False, indent=2))
    print("OUTPUT:", output)
    return 0 if result["status"] == "tracked_stable" else 3


if __name__ == "__main__":
    sys.exit(main())
