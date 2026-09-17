#!/usr/bin/env python3
"""Publish the validated eye-in-hand TF chain without commanding the robot.

base->tool0 is read from the UR controller's actual TCP pose on port 30003.
tool0->color_map is loaded from the hand-eye YAML.  The remaining Mech-Eye
frames are identity aliases only when the device reports depthToTexture=I,0.
"""
import argparse
import math
import os

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import yaml

from record_ur_trajectory import RealtimeReader


def quaternion_from_matrix(matrix):
    # Stable branch-based conversion, returned as ROS x,y,z,w.
    m = np.asarray(matrix, dtype=np.float64)
    trace = np.trace(m)
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2.0
        return [(m[2, 1] - m[1, 2]) / s,
                (m[0, 2] - m[2, 0]) / s,
                (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    index = int(np.argmax(np.diag(m)))
    if index == 0:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        return [0.25 * s, (m[0, 1] + m[1, 0]) / s,
                (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    if index == 1:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        return [(m[0, 1] + m[1, 0]) / s, 0.25 * s,
                (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
    return [(m[0, 2] + m[2, 0]) / s,
            (m[1, 2] + m[2, 1]) / s, 0.25 * s,
            (m[1, 0] - m[0, 1]) / s]


def make_transform(node, parent, child, translation, rotation):
    message = TransformStamped()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = parent
    message.child_frame_id = child
    message.transform.translation.x = float(translation[0])
    message.transform.translation.y = float(translation[1])
    message.transform.translation.z = float(translation[2])
    quaternion = quaternion_from_matrix(rotation)
    message.transform.rotation.x = quaternion[0]
    message.transform.rotation.y = quaternion[1]
    message.transform.rotation.z = quaternion[2]
    message.transform.rotation.w = quaternion[3]
    return message


class HandEyeTF(Node):
    def __init__(self, args, calibration):
        super().__init__("ur10_handeye_tf")
        self.reader = RealtimeReader(args.host)
        self.dynamic = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        rotation = np.asarray(calibration["rotation_matrix"], dtype=np.float64)
        translation = calibration["translation_m"]
        camera_frame = calibration["child_frame"]
        messages = [make_transform(
            self, calibration["parent_frame"], camera_frame,
            translation, rotation)]
        if args.identity_camera_frames:
            for child in ("mechmind_camera/depth_map",
                          "mechmind_camera/point_cloud",
                          "mechmind_camera/textured_point_cloud"):
                messages.append(make_transform(
                    self, camera_frame, child, [0, 0, 0], np.eye(3)))
        self.static.sendTransform(messages)
        self.timer = self.create_timer(0.008, self.publish_robot)
        self.published = 0

    def publish_robot(self):
        if not rclpy.ok():
            return
        _, tcp, _ = self.reader.read()
        rotation, _ = cv2.Rodrigues(np.asarray(tcp[3:], dtype=np.float64))
        message = make_transform(self, "base", "tool0", tcp[:3], rotation)
        try:
            self.dynamic.sendTransform(message)
        except RCLError:
            if rclpy.ok():
                raise
        self.published += 1

    def close(self):
        self.reader.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.1.3")
    parser.add_argument("--calibration",
                        default="config/handeye_eye_in_hand_20260917.yaml")
    parser.add_argument("--identity-camera-frames", action="store_true",
                        help="publish color_map->depth/point-cloud identity aliases")
    args = parser.parse_args()
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = args.calibration if os.path.isabs(args.calibration) else os.path.join(root, args.calibration)
    with open(path, encoding="utf-8") as stream:
        calibration = yaml.safe_load(stream)
    if not calibration.get("valid"):
        raise RuntimeError("refusing to publish calibration not marked valid")
    if calibration.get("transform_direction") != "tool0_from_camera":
        raise RuntimeError("unexpected transform direction")
    rclpy.init()
    node = HandEyeTF(args, calibration)
    node.get_logger().info(
        "只读 TF 已启动：base->tool0->%s；不发送机器人命令" % calibration["child_frame"])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
