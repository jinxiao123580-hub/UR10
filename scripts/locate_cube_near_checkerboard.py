#!/usr/bin/env python3
"""Locate a cube beside the fixed checkerboard without moving the robot."""
import argparse
import json
import os
import time

import cv2
import numpy as np
import rclpy
import yaml

from check_handeye_checkerboard import camera_matrices, detect, image_to_bgr
from collect_handeye_sample import URSampler, robot_summary
from solve_handeye_checkerboard import rt
from validate_checkerboard_pointcloud import BoardCloudCapture, organized_xyz


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--output", default="outputs/vision/cube-candidate-20260917.json")
    parser.add_argument("--calibration", default="config/handeye_eye_in_hand_20260917.yaml")
    args = parser.parse_args()
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    calibration_path = os.path.join(root, args.calibration)
    with open(calibration_path, encoding="utf-8") as stream:
        calibration = yaml.safe_load(stream)

    sampler = URSampler("192.168.1.3")
    rclpy.init()
    node = BoardCloudCapture()
    try:
        sampler.start()
        sampler.wait_ready()
        sampler.begin_window()
        image_msg, info_msg = node.capture(args.timeout)
        cloud_msg = node.capture_cloud(args.timeout)
    finally:
        sampler.stop()
        node.destroy_node()
        rclpy.shutdown()
    motion = robot_summary(sampler.rows)
    if motion["max_position_motion_mm"] > 0.5 or motion["max_rotation_motion_deg"] > 0.2:
        raise RuntimeError("robot moved during acquisition")

    image = image_to_bgr(image_msg)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    pattern = (9, 6)
    found, corners = detect(gray, pattern)
    if not found:
        raise RuntimeError("checkerboard not detected")
    k, distortion = camera_matrices(info_msg)
    objects = np.zeros((54, 3), dtype=np.float64)
    objects[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2) * 0.006
    ok, rvec, tvec = cv2.solvePnP(objects, corners, k, distortion,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise RuntimeError("solvePnP failed")
    camera_from_target = rt(rvec.reshape(3), tvec.reshape(3))
    target_from_camera = np.linalg.inv(camera_from_target)
    xyz = organized_xyz(cloud_msg)
    flat = xyz.reshape(-1, 3)
    homogeneous = np.column_stack((flat, np.ones(len(flat))))
    target = (target_from_camera @ homogeneous.T).T[:, :3].reshape(xyz.shape)
    finite = np.all(np.isfinite(target), axis=2)
    # Search up to 20 cm around the small board for a 2-8 cm tall object.
    nearby = (finite & (target[:, :, 0] > -0.20) & (target[:, :, 0] < 0.25) &
              (target[:, :, 1] > -0.20) & (target[:, :, 1] < 0.22))
    candidates = []
    for sign in (1.0, -1.0):
        height = sign * target[:, :, 2]
        mask = nearby & (height > 0.02) & (height < 0.08)
        count, labels = cv2.connectedComponents(mask.astype(np.uint8), connectivity=8)
        for label in range(1, count):
            pixels = labels == label
            if np.count_nonzero(pixels) < 80:
                continue
            points = target[pixels]
            low = np.percentile(points, 5, axis=0)
            high = np.percentile(points, 95, axis=0)
            extent = high - low
            center_target = np.median(points, axis=0)
            height_m = float(np.median(sign * points[:, 2]))
            # Prefer a compact component whose visible lateral span resembles 5 cm.
            lateral = sorted([float(extent[0]), float(extent[1])])
            score = abs(lateral[1] - 0.05) + abs(height_m - 0.05)
            candidates.append({"sign": sign, "pixels": int(len(points)),
                               "score": score, "center_target_m": center_target.tolist(),
                               "extent_p5_p95_m": extent.tolist(),
                               "height_m": height_m})
    if not candidates:
        raise RuntimeError("no 2-8 cm connected object found beside checkerboard")
    candidates.sort(key=lambda item: item["score"])
    best = candidates[0]
    measured_center = np.asarray(best["center_target_m"])
    extent = np.asarray(best["extent_p5_p95_m"])
    lateral_axis = int(np.argmin(extent[:2]))
    board_center = np.asarray([0.024, 0.015])
    outward = np.sign(measured_center[lateral_axis] - board_center[lateral_axis])
    inferred_center = measured_center.copy()
    inferred_center[lateral_axis] += outward * 0.025
    inferred_center[2] = best["sign"] * 0.025
    center_target = np.asarray([*inferred_center, 1.0])
    center_camera = camera_from_target @ center_target
    tcp = motion["base_to_tool0_tcp_mean"]
    base_from_tool = rt(tcp[3:], tcp[:3])
    tool_from_camera = np.eye(4)
    tool_from_camera[:3, :3] = np.asarray(calibration["rotation_matrix"])
    tool_from_camera[:3, 3] = calibration["translation_m"]
    center_base = base_from_tool @ tool_from_camera @ center_camera
    result = {
        "captured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "status": "candidate_not_motion_authorized",
        "robot_motion": motion,
        "checkerboard_target_to_camera": camera_from_target.tolist(),
        "candidate_count": len(candidates),
        "candidates": candidates,
        "best": best,
        "inference": {
            "basis": "visible side plane plus known 0.05 m cube size",
            "side_normal_target_axis": "xy"[lateral_axis],
            "cube_center_target_m": inferred_center.tolist(),
        },
        "estimated_cube_center_camera_m": center_camera[:3].tolist(),
        "estimated_cube_center_base_m": center_base[:3].tolist(),
        "assumed_cube_size_m": [0.05, 0.05, 0.05],
        "note": "Candidate only; no robot command was sent and physical reach validation is pending.",
    }
    output = args.output if os.path.isabs(args.output) else os.path.join(root, args.output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    print(json.dumps({"candidate_count": len(candidates), "best": best,
                      "center_camera_m": result["estimated_cube_center_camera_m"],
                      "center_base_m": result["estimated_cube_center_base_m"],
                      "robot_motion_mm_deg": [motion["max_position_motion_mm"],
                                               motion["max_rotation_motion_deg"]]},
                     indent=2, ensure_ascii=False))
    print("OUTPUT:", output)


if __name__ == "__main__":
    main()
