#!/usr/bin/env python3
"""Read-only: locate a nominal 50 mm cube beside a fixed checkerboard.

The result contains both the cube centre and the checkerboard centre in base
coordinates.  It never opens the UR motion port.  The checkerboard must remain
fixed for the single capture; it is permitted to have been moved since an
earlier calibration session because its pose is measured anew here.
"""
import argparse
import json
import os
import time

import cv2
import numpy as np
import rclpy
import yaml

from check_handeye_checkerboard import camera_matrices, detect, image_to_bgr
from record_ur_trajectory import RealtimeReader
from solve_handeye_checkerboard import rt
from validate_checkerboard_pointcloud import BoardCloudCapture, organized_xyz


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--output", default="outputs/vision/cube-board-observation.json")
    parser.add_argument("--calibration", default="config/handeye_eye_in_hand_20260921.yaml")
    parser.add_argument("--port", type=int, default=30013,
                        help="read-only UR realtime state port")
    parser.add_argument("--board-center-x-m", type=float, default=0.024,
                        help="centre of the 9x6 inner-corner grid in board coordinates")
    parser.add_argument("--board-center-y-m", type=float, default=0.015)
    parser.add_argument("--image-attempts", type=int, default=5,
                        help="retry colour captures because one trigger can be blurred")
    parser.add_argument("--search-half-xy-m", type=float, default=0.50,
                        help="board-frame half-width for coarse 50 mm cube search")
    args = parser.parse_args()
    if not 0.20 <= args.search_half_xy_m <= 0.50:
        parser.error("--search-half-xy-m must be within 0.20..0.50 m")
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    calibration_path = os.path.join(root, args.calibration)
    with open(calibration_path, encoding="utf-8") as stream:
        calibration = yaml.safe_load(stream)

    reader = RealtimeReader("192.168.1.3", port=args.port)
    rclpy.init()
    node = BoardCloudCapture()
    try:
        _, before_tcp, _, _ = reader.read_extended()
        found, corners, image_msg, info_msg = False, None, None, None
        for image_attempt in range(1, args.image_attempts + 1):
            candidate_image, candidate_info = node.capture(args.timeout)
            candidate_gray = cv2.cvtColor(image_to_bgr(candidate_image), cv2.COLOR_BGR2GRAY)
            found, corners = detect(candidate_gray, (9, 6))
            if found:
                image_msg, info_msg = candidate_image, candidate_info
                break
            print("棋盘格第 %d/%d 帧未检测到，重试…" %
                  (image_attempt, args.image_attempts), flush=True)
        if not found:
            raise RuntimeError("checkerboard not detected after %d image attempts" % args.image_attempts)
        cloud_msg = node.capture_cloud(args.timeout)
        _, after_tcp, _, _ = reader.read_extended()
    finally:
        reader.close()
        node.destroy_node()
        rclpy.shutdown()
    motion_mm = float(np.linalg.norm(np.asarray(after_tcp[:3]) - np.asarray(before_tcp[:3])) * 1000.0)
    if motion_mm > 0.5:
        raise RuntimeError("robot moved during acquisition")
    tcp = (np.asarray(before_tcp, dtype=float) + np.asarray(after_tcp, dtype=float)) / 2.0
    # Never average equivalent +/-pi rotation-vector representations.
    tcp[3:] = after_tcp[3:]

    pattern = (9, 6)
    k, distortion = camera_matrices(info_msg)
    objects = np.zeros((54, 3), dtype=np.float64)
    objects[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2) * 0.006
    ok, rvec, tvec = cv2.solvePnP(objects, corners, k, distortion,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise RuntimeError("solvePnP failed")
    projected, _ = cv2.projectPoints(objects, rvec, tvec, k, distortion)
    residual = np.linalg.norm(projected.reshape(-1, 2) - corners.reshape(-1, 2), axis=1)
    reprojection_rms_px = float(np.sqrt(np.mean(residual ** 2)))
    reprojection_max_px = float(np.max(residual))
    if reprojection_rms_px > 1.5 or reprojection_max_px > 4.0:
        raise RuntimeError("checkerboard PnP reprojection %.2f RMS / %.2f max px is unreliable" %
                           (reprojection_rms_px, reprojection_max_px))
    print("棋盘格已由 OpenCV 检出：54/54 角点，PnP 重投影 RMS %.3f px" %
          reprojection_rms_px, flush=True)
    camera_from_target = rt(rvec.reshape(3), tvec.reshape(3))
    target_from_camera = np.linalg.inv(camera_from_target)
    xyz = organized_xyz(cloud_msg)
    flat = xyz.reshape(-1, 3)
    homogeneous = np.column_stack((flat, np.ones(len(flat))))
    target = (target_from_camera @ homogeneous.T).T[:, :3].reshape(xyz.shape)
    finite = np.all(np.isfinite(target), axis=2)
    # The cube can be 35-40 cm from the board in the current workcell.
    # Keep the search bounded and require a 2-8 cm raised component below.
    half = args.search_half_xy_m
    nearby = (finite & (np.abs(target[:, :, 0]) < half) &
              (np.abs(target[:, :, 1]) < half))
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
            top_face_like = bool(extent[2] < 0.012 and
                                 all(0.035 <= span <= 0.070 for span in extent[:2]))
            candidates.append({"sign": sign, "pixels": int(len(points)),
                               "score": score, "center_target_m": center_target.tolist(),
                               "extent_p5_p95_m": extent.tolist(),
                               "surface_type": "top_face" if top_face_like else "side_or_partial",
                               "height_m": height_m})
    if not candidates:
        raise RuntimeError("棋盘格已检出，但棋盘附近点云中没有 2–8 cm 的连通物体；"
                           "确认物块完整入镜、靠近棋盘，并检查深度点云")
    candidates.sort(key=lambda item: item["score"])
    best = candidates[0]
    measured_center = np.asarray(best["center_target_m"])
    extent = np.asarray(best["extent_p5_p95_m"])
    lateral_axis = int(np.argmin(extent[:2]))
    board_center = np.asarray([args.board_center_x_m, args.board_center_y_m])
    inferred_center = measured_center.copy()
    if best["surface_type"] != "top_face":
        # Side-only first looks are provisional; hover re-observation must
        # replace this 25 mm nominal half-edge inference before descent.
        outward = np.sign(measured_center[lateral_axis] - board_center[lateral_axis])
        inferred_center[lateral_axis] += outward * 0.025
    inferred_center[2] = best["sign"] * 0.025
    center_target = np.asarray([*inferred_center, 1.0])
    center_camera = camera_from_target @ center_target
    base_from_tool = rt(tcp[3:], tcp[:3])
    tool_from_camera = np.eye(4)
    tool_from_camera[:3, :3] = np.asarray(calibration["rotation_matrix"])
    tool_from_camera[:3, 3] = calibration["translation_m"]
    center_base = base_from_tool @ tool_from_camera @ center_camera
    base_from_target = base_from_tool @ tool_from_camera @ camera_from_target
    board_center_target = np.array([board_center[0], board_center[1], 0.0, 1.0])
    board_center_base = base_from_target @ board_center_target
    # ``sign`` is the side of the board on which the observed cube sits, so it
    # selects the usable board normal for a later top-down placement.
    board_normal_base = best["sign"] * base_from_target[:3, 2]
    result = {
        "captured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "status": "candidate_not_motion_authorized",
        "motion_authorization": "none: observation only; no robot command was sent",
        "robot_motion_during_capture_mm": motion_mm,
        "checkerboard_image_attempt": image_attempt,
        "checkerboard_reprojection_rms_px": reprojection_rms_px,
        "checkerboard_reprojection_max_px": reprojection_max_px,
        "checkerboard_target_to_camera": camera_from_target.tolist(),
        "candidate_count": len(candidates),
        "candidates": candidates,
        "best": best,
        "inference": {
            "basis": ("measured top-face XY plus known 0.05 m height"
                      if best["surface_type"] == "top_face" else
                      "visible side plane plus known 0.05 m cube size"),
            "side_normal_target_axis": (None if best["surface_type"] == "top_face"
                                        else "xy"[lateral_axis]),
            "cube_center_target_m": inferred_center.tolist(),
        },
        "search_half_xy_m": half,
        "estimated_cube_center_camera_m": center_camera[:3].tolist(),
        "estimated_cube_center_base_m": center_base[:3].tolist(),
        "board_center_base_m": board_center_base[:3].tolist(),
        "board_normal_base": board_normal_base.tolist(),
        "board_x_axis_base": base_from_target[:3, 0].tolist(),
        "board_center_definition": "centre of 9x6 inner-corner grid; configurable CLI x/y",
        "assumed_cube_size_m": [0.05, 0.05, 0.05],
        "note": "Candidate only; no robot command was sent and physical reach validation is pending.",
    }
    output = args.output if os.path.isabs(args.output) else os.path.join(root, args.output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    print(json.dumps({"candidate_count": len(candidates), "best": best,
                      "cube_center_base_m": result["estimated_cube_center_base_m"],
                      "board_center_base_m": result["board_center_base_m"],
                      "robot_motion_mm": motion_mm},
                     indent=2, ensure_ascii=False))
    print("OUTPUT:", output)


if __name__ == "__main__":
    main()
