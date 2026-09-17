#!/usr/bin/env python3
"""Compare checkerboard PnP corner geometry with the organized Mech-Eye cloud."""
import argparse
import json
import os
import time

import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2, PointField
from mecheye_ros_interface.srv import CapturePointCloud

from check_handeye_checkerboard import (
    CheckerboardCapture, camera_matrices, detect, image_to_bgr)


class BoardCloudCapture(CheckerboardCapture):
    def __init__(self):
        super().__init__()
        self.cloud = None
        self.create_subscription(PointCloud2, "/mechmind/point_cloud",
                                 self._on_cloud, 10)
        self.cloud_client = self.create_client(CapturePointCloud,
                                               "/capture_point_cloud")

    def _on_cloud(self, message):
        self.cloud = message

    def capture_cloud(self, timeout):
        if not self.cloud_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("/capture_point_cloud service unavailable")
        self.cloud = None
        future = self.cloud_client.call_async(CapturePointCloud.Request())
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done() and self.cloud is not None:
                response = future.result()
                if response is None or response.error_code != 0:
                    raise RuntimeError("point-cloud capture failed")
                return self.cloud
        raise TimeoutError("point-cloud capture timeout")


def organized_xyz(message):
    fields = {field.name: field for field in message.fields}
    if any(name not in fields for name in "xyz"):
        raise RuntimeError("point cloud lacks xyz")
    if any(fields[name].datatype != PointField.FLOAT32 for name in "xyz"):
        raise RuntimeError("point cloud xyz must be FLOAT32")
    endian = ">" if message.is_bigendian else "<"
    axes = []
    for name in "xyz":
        axes.append(np.ndarray(
            (message.height, message.width), dtype=endian + "f4",
            buffer=message.data, offset=fields[name].offset,
            strides=(message.row_step, message.point_step)).astype(np.float64))
    return np.stack(axes, axis=2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--squares-x", type=int, default=10)
    parser.add_argument("--squares-y", type=int, default=7)
    parser.add_argument("--square-size-m", type=float, default=0.006)
    parser.add_argument("--radius-px", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--output",
                        default="outputs/handeye/pointcloud-check-20260917.json")
    args = parser.parse_args()
    pattern = (args.squares_x - 1, args.squares_y - 1)
    rclpy.init()
    node = BoardCloudCapture()
    try:
        image_msg, info_msg = node.capture(args.timeout)
        cloud_msg = node.capture_cloud(args.timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    image = image_to_bgr(image_msg)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = detect(gray, pattern)
    if not found:
        raise RuntimeError("checkerboard not detected")
    if (cloud_msg.width, cloud_msg.height) != (image_msg.width, image_msg.height):
        raise RuntimeError("image/cloud resolution mismatch")
    k, distortion = camera_matrices(info_msg)
    objects = np.zeros((pattern[0] * pattern[1], 3), dtype=np.float64)
    objects[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2)
    objects[:, :2] *= args.square_size_m
    ok, rvec, tvec = cv2.solvePnP(objects, corners, k, distortion,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise RuntimeError("solvePnP failed")
    rotation, _ = cv2.Rodrigues(rvec)
    expected = (rotation @ objects.T).T + tvec.reshape(1, 3)
    xyz = organized_xyz(cloud_msg)
    measured = []
    errors = []
    valid_indices = []
    radius = args.radius_px
    for index, pixel in enumerate(corners.reshape(-1, 2)):
        u, v = [int(round(value)) for value in pixel]
        patch = xyz[max(0, v-radius):v+radius+1,
                    max(0, u-radius):u+radius+1].reshape(-1, 3)
        patch = patch[np.all(np.isfinite(patch), axis=1)]
        if len(patch) < 3:
            measured.append(None)
            continue
        point = np.median(patch, axis=0)
        error = float(np.linalg.norm(point - expected[index]) * 1000.0)
        measured.append(point.tolist())
        errors.append(error)
        valid_indices.append(index)
    if not errors:
        raise RuntimeError("no finite point-cloud neighborhoods at corners")
    errors = np.asarray(errors)
    result = {
        "captured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "image_frame": image_msg.header.frame_id,
        "cloud_frame": cloud_msg.header.frame_id,
        "resolution": [image_msg.width, image_msg.height],
        "corner_count": len(corners),
        "finite_corner_neighborhoods": len(errors),
        "radius_px": radius,
        "pnp_distance_m": float(np.linalg.norm(tvec)),
        "corner_xyz_error_mm": {
            "rms": float(np.sqrt(np.mean(errors ** 2))),
            "median": float(np.median(errors)),
            "p95": float(np.percentile(errors, 95)),
            "max": float(np.max(errors)),
        },
        "valid_corner_indices": valid_indices,
        "expected_camera_xyz_m": expected.tolist(),
        "measured_cloud_xyz_m": measured,
        "interpretation": "Direct pixel comparison assumes depthToTexture=I,0 as reported by this camera.",
    }
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    output = args.output if os.path.isabs(args.output) else os.path.join(root, args.output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    temporary = output + ".tmp"
    with open(temporary, "w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    os.replace(temporary, output)
    print(json.dumps({key: result[key] for key in (
        "image_frame", "cloud_frame", "resolution", "corner_count",
        "finite_corner_neighborhoods", "pnp_distance_m",
        "corner_xyz_error_mm")}, indent=2, ensure_ascii=False))
    print("OUTPUT:", output)


if __name__ == "__main__":
    main()
