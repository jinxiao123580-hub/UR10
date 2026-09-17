#!/usr/bin/env python3
"""Append one synchronized, stationary eye-in-hand calibration sample.

The script triggers one Mech-Eye 2D capture while continuously reading the
UR controller's actual state on port 30003.  It never sends a robot command.
The sample is accepted only when the full checkerboard is detected and the
robot remains stationary throughout the camera acquisition.
"""
import argparse
import datetime as dt
import json
import os
import threading
import time

import cv2
import numpy as np
import rclpy

from check_handeye_checkerboard import (
    CheckerboardCapture, camera_matrices, detect, image_to_bgr)
from record_ur_trajectory import RealtimeReader


def atomic_json(path, value):
    temporary = path + ".tmp"
    with open(temporary, "w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    os.replace(temporary, path)


def rotation_distance_deg(a, b):
    ra, _ = cv2.Rodrigues(np.asarray(a, dtype=np.float64))
    rb, _ = cv2.Rodrigues(np.asarray(b, dtype=np.float64))
    relative = ra.T @ rb
    cosine = np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


class URSampler:
    def __init__(self, host):
        self.reader = RealtimeReader(host)
        self.rows = []
        self.stop_event = threading.Event()
        self.error = None
        self.thread = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        try:
            while not self.stop_event.is_set():
                q, tcp, velocity = self.reader.read()
                self.rows.append({
                    "monotonic_s": time.monotonic(),
                    "q_rad": list(q),
                    "tcp_pose": list(tcp),
                    "tcp_velocity": list(velocity),
                })
        except Exception as exc:  # retained unless caused by normal shutdown
            if not self.stop_event.is_set():
                self.error = repr(exc)

    def start(self):
        self.thread.start()

    def wait_ready(self, minimum_frames=20, timeout=5.0):
        deadline = time.monotonic() + timeout
        while len(self.rows) < minimum_frames and time.monotonic() < deadline:
            time.sleep(0.01)
        if len(self.rows) < minimum_frames:
            raise RuntimeError("UR state stream not ready: %d/%d frames" %
                               (len(self.rows), minimum_frames))

    def begin_window(self):
        self.rows.clear()

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=2.0)
        self.reader.close()


def robot_summary(rows):
    if len(rows) < 2:
        raise RuntimeError("too few UR state frames during camera capture: %d" % len(rows))
    positions = np.asarray([row["tcp_pose"][:3] for row in rows])
    reference_position = positions[0]
    max_position_mm = float(np.max(np.linalg.norm(
        positions - reference_position, axis=1)) * 1000.0)
    reference_rotation = rows[0]["tcp_pose"][3:]
    max_rotation_deg = max(rotation_distance_deg(
        reference_rotation, row["tcp_pose"][3:]) for row in rows)
    q = np.asarray([row["q_rad"] for row in rows])
    tcp = np.asarray([row["tcp_pose"] for row in rows])
    return {
        "frame_count": len(rows),
        "duration_s": rows[-1]["monotonic_s"] - rows[0]["monotonic_s"],
        "max_position_motion_mm": max_position_mm,
        "max_rotation_motion_deg": max_rotation_deg,
        "q_actual_mean_rad": np.mean(q, axis=0).tolist(),
        "base_to_tool0_tcp_mean": np.mean(tcp, axis=0).tolist(),
        "first_tcp_pose": rows[0]["tcp_pose"],
        "last_tcp_pose": rows[-1]["tcp_pose"],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.1.3")
    parser.add_argument("--squares-x", type=int, default=10)
    parser.add_argument("--squares-y", type=int, default=7)
    parser.add_argument("--square-size-m", type=float, default=0.006)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--max-motion-mm", type=float, default=0.5)
    parser.add_argument("--max-motion-deg", type=float, default=0.2)
    parser.add_argument("--min-robot-frames", type=int, default=20)
    parser.add_argument("--dataset", default="outputs/handeye/manual-eye-in-hand.json")
    args = parser.parse_args()
    pattern = (args.squares_x - 1, args.squares_y - 1)
    if min(pattern) < 2 or args.square_size_m <= 0:
        parser.error("invalid checkerboard dimensions")

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    dataset_path = os.path.abspath(os.path.join(root, args.dataset)) \
        if not os.path.isabs(args.dataset) else args.dataset
    os.makedirs(os.path.dirname(dataset_path), exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    sample_dir = os.path.join(os.path.dirname(dataset_path), "samples", stamp)
    os.makedirs(sample_dir, exist_ok=False)

    sampler = URSampler(args.host)
    rclpy.init()
    node = CheckerboardCapture()
    try:
        sampler.start()
        sampler.wait_ready()
        sampler.begin_window()
        image_msg, info_msg = node.capture(args.timeout)
    finally:
        sampler.stop()
        node.destroy_node()
        rclpy.shutdown()

    summary = robot_summary(sampler.rows)
    bgr = image_to_bgr(image_msg)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    k, d = camera_matrices(info_msg)
    found, corners = detect(gray, pattern)
    image_path = os.path.join(sample_dir, "image.png")
    corners_path = os.path.join(sample_dir, "corners.png")
    cv2.imwrite(image_path, bgr)
    marked = bgr.copy()
    result = {
        "sample_id": stamp,
        "captured_at": dt.datetime.now(dt.timezone.utc).astimezone().isoformat(),
        "accepted": False,
        "setup": "eye_in_hand",
        "robot_transform": "base_to_tool0",
        "camera_transform": "target_to_camera",
        "board_squares": [args.squares_x, args.squares_y],
        "inner_corners": list(pattern),
        "square_size_m": args.square_size_m,
        "detected": found,
        "detected_corner_count": 0 if corners is None else int(len(corners)),
        "robot": summary,
        "motion_limits": {"position_mm": args.max_motion_mm,
                          "rotation_deg": args.max_motion_deg},
        "camera_info": {"width": info_msg.width, "height": info_msg.height,
                        "distortion_model": info_msg.distortion_model,
                        "k": k.reshape(-1).tolist(), "d": d.reshape(-1).tolist(),
                        "r": list(info_msg.r), "p": list(info_msg.p)},
        "files": {"image": image_path, "corners": corners_path},
    }
    rejection = []
    if sampler.error:
        rejection.append("UR sampler error: " + sampler.error)
    if summary["frame_count"] < args.min_robot_frames:
        rejection.append("too few UR state frames: %d < %d" %
                         (summary["frame_count"], args.min_robot_frames))
    if not found:
        rejection.append("checkerboard not detected")
    if summary["max_position_motion_mm"] > args.max_motion_mm:
        rejection.append("robot translation exceeded limit")
    if summary["max_rotation_motion_deg"] > args.max_motion_deg:
        rejection.append("robot rotation exceeded limit")

    if found:
        cv2.drawChessboardCorners(marked, pattern, corners, True)
        obj = np.zeros((pattern[0] * pattern[1], 3), dtype=np.float64)
        obj[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2)
        obj[:, :2] *= args.square_size_m
        ok, rvec, tvec = cv2.solvePnP(
            obj, corners, k, d, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            rejection.append("solvePnP failed")
        else:
            projected, _ = cv2.projectPoints(obj, rvec, tvec, k, d)
            residual = np.linalg.norm(projected.reshape(-1, 2) -
                                      corners.reshape(-1, 2), axis=1)
            result["target_to_camera"] = {
                "rvec_rad": rvec.reshape(-1).tolist(),
                "translation_m": tvec.reshape(-1).tolist(),
                "distance_m": float(np.linalg.norm(tvec)),
                "reprojection_rms_px": float(np.sqrt(np.mean(residual ** 2))),
                "reprojection_max_px": float(np.max(residual)),
            }
            result["corners_px"] = corners.reshape(-1, 2).tolist()
    cv2.imwrite(corners_path, marked)
    result["rejection_reasons"] = rejection
    result["accepted"] = not rejection
    atomic_json(os.path.join(sample_dir, "sample.json"), result)

    if os.path.exists(dataset_path):
        with open(dataset_path, encoding="utf-8") as stream:
            dataset = json.load(stream)
    else:
        dataset = {
            "schema_version": 1,
            "setup": "eye_in_hand",
            "board_squares": [args.squares_x, args.squares_y],
            "inner_corners": list(pattern),
            "square_size_m": args.square_size_m,
            "samples": [],
        }
    expected = (dataset.get("board_squares"), dataset.get("square_size_m"))
    actual = ([args.squares_x, args.squares_y], args.square_size_m)
    if expected != actual:
        raise RuntimeError("dataset board geometry mismatch: %r != %r" %
                           (expected, actual))
    dataset["samples"].append(result)
    dataset["accepted_count"] = sum(s.get("accepted", False)
                                    for s in dataset["samples"])
    dataset["rejected_count"] = len(dataset["samples"]) - dataset["accepted_count"]
    atomic_json(dataset_path, dataset)

    print("样本 %s：%s" % (stamp, "接受" if result["accepted"] else "拒绝"))
    print("棋盘格：%d/%d；重投影 RMS：%s px" % (
        result["detected_corner_count"], pattern[0] * pattern[1],
        "%.4f" % result["target_to_camera"]["reprojection_rms_px"]
        if "target_to_camera" in result else "N/A"))
    print("采集期间运动：%.4f mm / %.4f deg；UR 帧数：%d" % (
        summary["max_position_motion_mm"], summary["max_rotation_motion_deg"],
        summary["frame_count"]))
    if rejection:
        print("拒绝原因：" + "；".join(rejection))
    print("数据集：%s（有效 %d，拒绝 %d）" % (
        dataset_path, dataset["accepted_count"], dataset["rejected_count"]))
    raise SystemExit(0 if result["accepted"] else 2)


if __name__ == "__main__":
    main()
