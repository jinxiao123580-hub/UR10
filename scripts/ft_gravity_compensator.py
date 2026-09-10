#!/usr/bin/env python3
"""Publish external wrench after software gravity and sensor-bias compensation."""
import argparse
import math
import os
import threading
import time

import numpy as np
import yaml

from calibrate_ft_gravity import GRAVITY, rotvec_to_matrix
from ur_arm import read_packet


def load_calibration(path):
    with open(os.path.expanduser(path), encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not document.get("valid"):
        raise ValueError("标定文件 valid 不是 true")
    c = document["calibration"]
    return {
        "mass": float(c["mass_kg"]),
        "com": np.asarray(c["com_sensor_m"], dtype=float),
        "force_bias": np.asarray(c["force_bias_n"], dtype=float),
        "torque_bias": np.asarray(c["torque_bias_nm"], dtype=float),
        "rotation_sensor_tool": np.asarray(
            c["rotation_sensor_from_tool"], dtype=float),
    }


class GravityCompensator:
    def __init__(self, args):
        import rclpy
        from geometry_msgs.msg import WrenchStamped
        from rclpy.node import Node

        self._rclpy = rclpy
        self._msg_type = WrenchStamped
        self.node = Node("ft_gravity_compensator")
        self.cal = load_calibration(args.calibration)
        self.robot_ip = args.robot_ip
        self.pose_period = 1.0 / args.pose_hz
        self._rotation_base_tool = None
        self._pose_lock = threading.Lock()
        self._stop = threading.Event()
        self._pose_thread = threading.Thread(target=self._poll_pose, daemon=True)
        self.pub = self.node.create_publisher(WrenchStamped, args.output_topic, 10)
        self.sub = self.node.create_subscription(
            WrenchStamped, args.input_topic, self._on_wrench, 10)
        self._pose_thread.start()
        self.node.get_logger().info(
            "重力补偿已加载: mass=%.4fkg, %s -> %s" %
            (self.cal["mass"], args.input_topic, args.output_topic))

    def _poll_pose(self):
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                pose = read_packet(host=self.robot_ip, timeout=1.0)
                if pose:
                    rotation = rotvec_to_matrix(pose[3:])
                    with self._pose_lock:
                        self._rotation_base_tool = rotation
            except OSError as exc:
                self.node.get_logger().warning("读取 UR TCP 失败: %s" % exc)
            delay = self.pose_period - (time.monotonic() - started)
            self._stop.wait(max(0.0, delay))

    def _on_wrench(self, msg):
        with self._pose_lock:
            rotation_base_tool = self._rotation_base_tool
        if rotation_base_tool is None:
            return
        c = self.cal
        gravity_base = np.array([0.0, 0.0, -GRAVITY])
        gravity_tool = rotation_base_tool.T @ gravity_base
        gravity_force = c["mass"] * c["rotation_sensor_tool"] @ gravity_tool
        gravity_torque = np.cross(c["com"], gravity_force)
        measured = np.array([
            msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
            msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z,
        ])
        predicted = np.r_[gravity_force + c["force_bias"],
                          gravity_torque + c["torque_bias"]]
        external = measured - predicted
        out = self._msg_type()
        out.header = msg.header
        out.header.frame_id = "ft_sensor"
        out.wrench.force.x, out.wrench.force.y, out.wrench.force.z = external[:3]
        out.wrench.torque.x, out.wrench.torque.y, out.wrench.torque.z = external[3:]
        self.pub.publish(out)

    def run(self):
        try:
            self._rclpy.spin(self.node)
        finally:
            self._stop.set()
            self._pose_thread.join(timeout=2.0)
            self.node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration", default="config/ft_gravity_calibration.yaml")
    parser.add_argument("--robot-ip", default=os.environ.get("UR_IP", "192.168.1.3"))
    parser.add_argument("--pose-hz", type=float, default=10.0)
    parser.add_argument("--input-topic", default="/ft_sensor/wrench")
    parser.add_argument("--output-topic", default="/ft_sensor/wrench_compensated")
    args = parser.parse_args()
    if not math.isfinite(args.pose_hz) or args.pose_hz <= 0:
        parser.error("--pose-hz 必须大于 0")
    import rclpy
    rclpy.init()
    try:
        GravityCompensator(args).run()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
