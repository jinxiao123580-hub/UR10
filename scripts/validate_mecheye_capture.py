#!/usr/bin/env python3
"""Trigger Mech-Eye color/cloud captures and print compact data-quality evidence."""
import argparse
import json
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from mecheye_ros_interface.srv import (
    CaptureColorImage, CapturePointCloud, CaptureTexturedPointCloud)


class CaptureValidator(Node):
    def __init__(self):
        super().__init__("validate_mecheye_capture")
        self.messages = {}
        self.create_subscription(
            Image, "/mechmind/color_image",
            lambda msg: self.messages.__setitem__("image", msg), 10)
        self.create_subscription(
            PointCloud2, "/mechmind/point_cloud",
            lambda msg: self.messages.__setitem__("cloud", msg), 10)
        self.create_subscription(
            PointCloud2, "/mechmind/textured_point_cloud",
            lambda msg: self.messages.__setitem__("textured_cloud", msg), 10)
        self.color_client = self.create_client(CaptureColorImage, "/capture_color_image")
        self.cloud_client = self.create_client(CapturePointCloud, "/capture_point_cloud")
        self.textured_client = self.create_client(
            CaptureTexturedPointCloud, "/capture_textured_point_cloud")

    def capture(self, key, client, request_type, timeout):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("service unavailable for %s" % key)
        self.messages.pop(key, None)
        future = client.call_async(request_type.Request())
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done() and key in self.messages:
                response = future.result()
                if response is None:
                    raise RuntimeError("%s service returned no response" % key)
                if response.error_code != 0:
                    raise RuntimeError("%s capture failed: %d %s" %
                                       (key, response.error_code,
                                        response.error_description))
                return self.messages[key], response
        raise TimeoutError("%s capture/message timeout after %.1fs" % (key, timeout))


def image_stats(msg):
    expected = msg.height * msg.step
    raw = np.frombuffer(msg.data, dtype=np.uint8, count=min(len(msg.data), expected))
    result = {
        "frame_id": msg.header.frame_id,
        "encoding": msg.encoding,
        "width": msg.width,
        "height": msg.height,
        "step": msg.step,
        "bytes": len(msg.data),
        "is_bigendian": bool(msg.is_bigendian),
    }
    if msg.encoding.lower() in ("bgr8", "rgb8") and msg.step >= msg.width * 3:
        pixels = raw.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(-1, 3)
        means = pixels.mean(axis=0)
        stds = pixels.std(axis=0)
        result["channel_mean"] = [round(float(x), 3) for x in means]
        result["channel_std"] = [round(float(x), 3) for x in stds]
        result["mean_channel_spread"] = round(float(np.ptp(means)), 3)
        result["channels_identical_fraction"] = round(
            float(np.mean((pixels[:, 0] == pixels[:, 1]) &
                          (pixels[:, 1] == pixels[:, 2]))), 6)
    return result


def cloud_stats(msg):
    fields = {field.name: field for field in msg.fields}
    missing = [name for name in "xyz" if name not in fields]
    if missing:
        raise RuntimeError("point cloud missing fields: " + ", ".join(missing))
    if any(fields[name].datatype != PointField.FLOAT32 for name in "xyz"):
        raise RuntimeError("only FLOAT32 xyz fields are supported by this validator")
    endian = ">" if msg.is_bigendian else "<"
    count = msg.width * msg.height
    axes = []
    for name in "xyz":
        field = fields[name]
        axes.append(np.ndarray((count,), dtype=endian + "f4", buffer=msg.data,
                               offset=field.offset,
                               strides=(msg.point_step,)).astype(np.float64))
    xyz = np.column_stack(axes)
    finite = np.all(np.isfinite(xyz), axis=1)
    valid = xyz[finite]
    result = {
        "frame_id": msg.header.frame_id,
        "width": msg.width,
        "height": msg.height,
        "point_step": msg.point_step,
        "row_step": msg.row_step,
        "is_dense": bool(msg.is_dense),
        "fields": [{"name": f.name, "offset": f.offset, "datatype": f.datatype,
                    "count": f.count} for f in msg.fields],
        "points": count,
        "finite_points": int(finite.sum()),
        "finite_fraction": round(float(finite.mean()), 6),
    }
    if len(valid):
        result["xyz_min_m"] = [round(float(x), 6) for x in valid.min(axis=0)]
        result["xyz_max_m"] = [round(float(x), 6) for x in valid.max(axis=0)]
        result["xyz_median_m"] = [round(float(x), 6) for x in np.median(valid, axis=0)]
    rgb = fields.get("rgb")
    if rgb is not None and rgb.offset + 3 <= msg.point_step:
        packed = np.ndarray((count, 3), dtype=np.uint8, buffer=msg.data,
                            offset=rgb.offset, strides=(msg.point_step, 1))
        colors = packed[finite]
        if len(colors):
            result["bgr_mean"] = [round(float(x), 3) for x in colors.mean(axis=0)]
            result["bgr_identical_fraction"] = round(float(np.mean(
                (colors[:, 0] == colors[:, 1]) &
                (colors[:, 1] == colors[:, 2]))), 6)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--output")
    args = parser.parse_args()
    rclpy.init()
    node = CaptureValidator()
    try:
        image, _ = node.capture("image", node.color_client, CaptureColorImage,
                                args.timeout)
        cloud, _ = node.capture("cloud", node.cloud_client, CapturePointCloud,
                                args.timeout)
        textured, _ = node.capture("textured_cloud", node.textured_client,
                                   CaptureTexturedPointCloud, args.timeout)
        result = {"captured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                  "image": image_stats(image), "point_cloud": cloud_stats(cloud),
                  "textured_point_cloud": cloud_stats(textured)}
    finally:
        node.destroy_node()
        rclpy.shutdown()
    output = args.output
    if output is None:
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        output = os.path.join(root, "outputs", "camera", "capture-validation-%s.json" %
                              time.strftime("%Y%m%d-%H%M%S"))
    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    with open(output, "w") as stream:
        json.dump(result, stream, indent=2, ensure_ascii=True)
        stream.write("\n")
    print(json.dumps(result, indent=2, ensure_ascii=True))
    print("OUTPUT:", output)


if __name__ == "__main__":
    main()
