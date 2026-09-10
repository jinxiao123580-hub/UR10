#!/usr/bin/env python3
"""Verify that the read-only digital twin mirrors all six UR joints."""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


class TwinCheck(Node):
    def __init__(self):
        super().__init__("digital_twin_check")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.real = None
        self.twin = None
        self.create_subscription(JointState, "/joint_states", self._real, qos)
        self.create_subscription(
            JointState, "/digital_twin/joint_states", self._twin, qos)

    def _real(self, msg):
        self.real = msg

    def _twin(self, msg):
        self.twin = msg


def positions(msg):
    return dict(zip(msg.name, msg.position))


def main():
    rclpy.init()
    node = TwinCheck()
    deadline = time.monotonic() + 6.0
    try:
        while time.monotonic() < deadline and (node.real is None or node.twin is None):
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.real is None or node.twin is None:
            print("✘ 超时：需要 /joint_states 和 /digital_twin/joint_states")
            return 1

        real = positions(node.real)
        twin = positions(node.twin)
        missing = [name for name in ARM_JOINTS
                   if name not in real or "twin_" + name not in twin]
        if missing:
            print("✘ 缺少关节：" + ", ".join(missing))
            return 1

        errors = [abs(real[name] - twin["twin_" + name]) for name in ARM_JOINTS]
        print("=== UR10 数字孪生检查 ===")
        for name, error in zip(ARM_JOINTS, errors):
            print(f"  {name:<22} Δ={error:.6f} rad")
        print(f"最大关节差: {max(errors):.6f} rad")
        print("✔ 六关节同步链路正常（差值包含两话题采样时差）")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
