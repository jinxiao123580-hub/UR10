#!/usr/bin/env python3
"""Capture one Mech-Eye image and validate a hand-eye checkerboard.

Read-only with respect to the robot.  The script triggers only the camera's
2D capture service, detects checkerboard inner corners, refines them, solves a
target-to-camera PnP pose from CameraInfo, and saves auditable image/JSON files.
"""
import argparse
import json
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from mecheye_ros_interface.srv import CaptureColorImage


class CheckerboardCapture(Node):
    def __init__(self):
        super().__init__("check_handeye_checkerboard")
        self.image = None
        self.info = None
        self.create_subscription(
            Image, "/mechmind/color_image", self._on_image, 10)
        self.create_subscription(
            CameraInfo, "/mechmind/camera_info", self._on_info, 10)
        self.client = self.create_client(CaptureColorImage, "/capture_color_image")

    def _on_image(self, msg):
        self.image = msg

    def _on_info(self, msg):
        self.info = msg

    def capture(self, timeout):
        if not self.client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("/capture_color_image service unavailable")
        self.image = None
        self.info = None
        future = self.client.call_async(CaptureColorImage.Request())
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done() and self.image is not None and self.info is not None:
                response = future.result()
                if response is None:
                    raise RuntimeError("capture service returned no response")
                if response.error_code != 0:
                    raise RuntimeError("capture failed: %d %s" %
                                       (response.error_code,
                                        response.error_description))
                return self.image, self.info
        raise TimeoutError("image/CameraInfo timeout after %.1fs" % timeout)


def image_to_bgr(msg):
    encoding = msg.encoding.lower()
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    rows = raw.reshape(msg.height, msg.step)
    if encoding in ("bgr8", "rgb8"):
        image = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3).copy()
        return image if encoding == "bgr8" else cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding in ("mono8", "8uc1"):
        gray = rows[:, :msg.width].reshape(msg.height, msg.width).copy()
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    raise RuntimeError("unsupported image encoding: %s" % msg.encoding)


def camera_matrices(info):
    k = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
    d = np.asarray(info.d, dtype=np.float64).reshape(-1, 1)
    if info.width <= 0 or info.height <= 0 or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise RuntimeError("CameraInfo contains invalid dimensions or focal length")
    return k, d


def detect(gray, pattern):
    flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    found, corners = cv2.findChessboardCornersSB(gray, pattern, flags=flags)
    if not found:
        # ``ACCURACY`` can be counterproductive on a small board observed at a
        # steep angle (the current cell setup is such a case).  SB without it
        # still returns the complete grid; PnP reprojection error remains the
        # downstream quality gate, so this does not turn a partial grid into an
        # accepted hand-eye sample.
        relaxed = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE
        found, corners = cv2.findChessboardCornersSB(gray, pattern, flags=relaxed)
    if not found:
        # The classic detector is a useful fallback for small or mildly blurred boards.
        classic_flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCorners(gray, pattern, flags=classic_flags)
        if found:
            criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
                        50, 1e-4)
            corners = cv2.cornerSubPix(gray, corners, (7, 7), (-1, -1), criteria)
    if not found:
        # Last OpenCV-only fallback for the installed cell: the white board
        # carrier is clear but the checker grid is seen obliquely beside dark
        # table slats.  Rectify the largest bright carrier rectangle, run the
        # normal SB detector on that fronto-parallel view, then map corners
        # back to the original image for PnP.  This is deliberately a fallback
        # only; ordinary full-frame SB remains the preferred measurement.
        _level, bright = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY)
        contours, _hierarchy = cv2.findContours(bright, cv2.RETR_EXTERNAL,
                                                  cv2.CHAIN_APPROX_SIMPLE)
        candidates = [c for c in contours if cv2.contourArea(c) >= 2000.0]
        if candidates:
            carrier = max(candidates, key=cv2.contourArea)
            box = cv2.boxPoints(cv2.minAreaRect(carrier)).astype(np.float32)
            sums = box.sum(axis=1)
            differences = np.diff(box, axis=1).reshape(-1)
            source = np.asarray([box[np.argmin(sums)], box[np.argmin(differences)],
                                 box[np.argmax(sums)], box[np.argmax(differences)]],
                                dtype=np.float32)
            destination = np.asarray([[0, 0], [799, 0], [799, 499], [0, 499]],
                                     dtype=np.float32)
            homography = cv2.getPerspectiveTransform(source, destination)
            rectified = cv2.warpPerspective(gray, homography, (800, 500))
            relaxed = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE
            found, rectified_corners = cv2.findChessboardCornersSB(
                rectified, pattern, flags=relaxed)
            if found:
                inverse = cv2.getPerspectiveTransform(destination, source)
                corners = cv2.perspectiveTransform(rectified_corners, inverse)
    if not found:
        # Table slats can dominate a full-frame search.  Search compact bright
        # carrier candidates independently at 2x scale.  The detected points
        # are mapped exactly back to original pixels, so downstream PnP and
        # reprojection gates are unchanged.
        _level, bright = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
        contours, _hierarchy = cv2.findContours(bright, cv2.RETR_LIST,
                                                  cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            (cx, cy), (width, height), _angle = cv2.minAreaRect(contour)
            ratio = max(width, height) / max(min(width, height), 1.0)
            if area >= 1500.0 and 1.0 <= ratio <= 3.5:
                candidates.append((area, cx, cy, width, height))
        for _area, cx, cy, width, height in sorted(candidates, reverse=True)[:6]:
            side = max(width, height) * 1.35
            x0 = max(0, int(cx - side / 2)); x1 = min(gray.shape[1], int(cx + side / 2))
            y0 = max(0, int(cy - side / 2)); y1 = min(gray.shape[0], int(cy + side / 2))
            if x1 - x0 < 40 or y1 - y0 < 40:
                continue
            crop = cv2.resize(gray[y0:y1, x0:x1], None, fx=2.0, fy=2.0,
                              interpolation=cv2.INTER_CUBIC)
            found, local = cv2.findChessboardCornersSB(crop, pattern,
                flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE)
            if found:
                corners = local / 2.0 + np.array([[[x0, y0]]], dtype=np.float32)
                break
    return bool(found), corners


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--squares-x", type=int, default=10,
                        help="number of physical squares horizontally")
    parser.add_argument("--squares-y", type=int, default=7,
                        help="number of physical squares vertically")
    parser.add_argument("--square-size-m", type=float, default=0.006)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--output-dir")
    args = parser.parse_args()
    if args.squares_x < 3 or args.squares_y < 3 or args.square_size_m <= 0:
        parser.error("board must have >=3 squares per side and positive square size")
    pattern = (args.squares_x - 1, args.squares_y - 1)

    rclpy.init()
    node = CheckerboardCapture()
    try:
        image_msg, info_msg = node.capture(args.timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    bgr = image_to_bgr(image_msg)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    k, d = camera_matrices(info_msg)
    found, corners = detect(gray, pattern)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    output_dir = os.path.abspath(args.output_dir or
                                 os.path.join(root, "outputs", "handeye", "checkerboard-%s" % stamp))
    os.makedirs(output_dir, exist_ok=True)
    original_path = os.path.join(output_dir, "image.png")
    marked_path = os.path.join(output_dir, "corners.png")
    json_path = os.path.join(output_dir, "result.json")
    cv2.imwrite(original_path, bgr)
    marked = bgr.copy()
    result = {
        "captured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "setup": "eye_in_hand",
        "board_squares": [args.squares_x, args.squares_y],
        "inner_corners": list(pattern),
        "expected_corner_count": pattern[0] * pattern[1],
        "square_size_m": args.square_size_m,
        "detected": found,
        "detected_corner_count": 0 if corners is None else int(len(corners)),
        "image": {"width": image_msg.width, "height": image_msg.height,
                  "encoding": image_msg.encoding, "frame_id": image_msg.header.frame_id},
        "camera_info": {"width": info_msg.width, "height": info_msg.height,
                        "frame_id": info_msg.header.frame_id,
                        "distortion_model": info_msg.distortion_model,
                        "k": k.reshape(-1).tolist(), "d": d.reshape(-1).tolist(),
                        "r": list(info_msg.r), "p": list(info_msg.p)},
        "files": {"image": original_path, "corners": marked_path},
    }
    if found:
        cv2.drawChessboardCorners(marked, pattern, corners, found)
        obj = np.zeros((pattern[0] * pattern[1], 3), dtype=np.float64)
        obj[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2)
        obj[:, :2] *= args.square_size_m
        ok, rvec, tvec = cv2.solvePnP(obj, corners, k, d,
                                      flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            raise RuntimeError("checkerboard detected but solvePnP failed")
        projected, _ = cv2.projectPoints(obj, rvec, tvec, k, d)
        residual = np.linalg.norm(projected.reshape(-1, 2) - corners.reshape(-1, 2), axis=1)
        result["target_to_camera"] = {
            "rvec_rad": rvec.reshape(-1).tolist(),
            "translation_m": tvec.reshape(-1).tolist(),
            "distance_m": float(np.linalg.norm(tvec)),
            "reprojection_rms_px": float(np.sqrt(np.mean(residual ** 2))),
            "reprojection_max_px": float(np.max(residual)),
        }
        result["corners_px"] = corners.reshape(-1, 2).tolist()
    cv2.imwrite(marked_path, marked)
    with open(json_path, "w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    print(json.dumps(result, indent=2, ensure_ascii=False))
    print("OUTPUT:", output_dir)
    if not found:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
