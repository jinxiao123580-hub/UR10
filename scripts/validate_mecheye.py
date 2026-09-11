#!/usr/bin/env python3
"""Trigger one Mech-Eye color frame and point cloud and print objective stats."""
import argparse
import time

import numpy as np


def wait_future(node, future, timeout):
    import rclpy
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline and not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
    if not future.done():
        raise TimeoutError("service timeout")
    return future.result()


def wait_message(node, holder, timeout):
    import rclpy
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline and not holder:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not holder:
        raise TimeoutError("topic timeout")
    return holder[-1]


def color_stats(msg):
    if msg.encoding not in ("bgr8", "rgb8"):
        return {"encoding": msg.encoding, "error": "not a 3-channel color encoding"}
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    image = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    unequal = np.any(image != image[:, :, :1], axis=2)
    labels = "BGR" if msg.encoding == "bgr8" else "RGB"
    return {
        "encoding": msg.encoding,
        "size": [msg.width, msg.height],
        "step": msg.step,
        "channel_order": labels,
        "channel_mean": image.mean(axis=(0, 1)).tolist(),
        "channel_min": image.min(axis=(0, 1)).tolist(),
        "channel_max": image.max(axis=(0, 1)).tolist(),
        "unequal_pixel_ratio": float(unequal.mean()),
    }


def point_cloud_stats(msg):
    fields = {field.name: field for field in msg.fields}
    missing = [name for name in ("x", "y", "z") if name not in fields]
    if missing:
        return {"error": "missing fields: " + ",".join(missing)}
    endian = ">f4" if msg.is_bigendian else "<f4"
    arrays = {}
    for name in ("x", "y", "z"):
        arrays[name] = np.ndarray(
            (msg.height, msg.width), dtype=endian, buffer=msg.data,
            offset=fields[name].offset, strides=(msg.row_step, msg.point_step))
    valid = np.isfinite(arrays["x"]) & np.isfinite(arrays["y"]) & np.isfinite(arrays["z"])
    count = int(valid.sum())
    result = {
        "size": [msg.width, msg.height],
        "frame_id": msg.header.frame_id,
        "point_step": msg.point_step,
        "row_step": msg.row_step,
        "fields": {name: fields[name].offset for name in fields},
        "valid_points": count,
        "invalid_points": int(valid.size - count),
    }
    if count:
        for name in ("x", "y", "z"):
            values = arrays[name][valid].astype(float)
            result[name + "_range_m"] = [float(values.min()), float(values.max())]
        result["z_span_m"] = result["z_range_m"][1] - result["z_range_m"][0]
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args()

    import rclpy
    from sensor_msgs.msg import Image, PointCloud2
    from mecheye_ros_interface.srv import CaptureColorImage, CapturePointCloud

    rclpy.init()
    node = rclpy.create_node("validate_mecheye")
    color_messages = []
    cloud_messages = []
    node.create_subscription(Image, "/mechmind/color_image",
                             lambda msg: color_messages.append(msg), 1)
    node.create_subscription(PointCloud2, "/mechmind/point_cloud",
                             lambda msg: cloud_messages.append(msg), 1)
    color_client = node.create_client(CaptureColorImage, "/capture_color_image")
    cloud_client = node.create_client(CapturePointCloud, "/capture_point_cloud")
    try:
        for name, client in (("color", color_client), ("point_cloud", cloud_client)):
            if not client.wait_for_service(timeout_sec=args.timeout):
                raise RuntimeError(name + " service unavailable")
        color_response = wait_future(
            node, color_client.call_async(CaptureColorImage.Request()), args.timeout)
        color = wait_message(node, color_messages, args.timeout)
        cloud_response = wait_future(
            node, cloud_client.call_async(CapturePointCloud.Request()), args.timeout)
        cloud = wait_message(node, cloud_messages, args.timeout)
        print("color_service", color_response.error_code, color_response.error_description)
        print("color", color_stats(color))
        print("point_cloud_service", cloud_response.error_code,
              cloud_response.error_description)
        print("point_cloud", point_cloud_stats(cloud))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
