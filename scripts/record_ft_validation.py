#!/usr/bin/env python3
"""Record one static raw/compensated wrench validation pose to YAML."""
import argparse
from datetime import datetime, timezone
import os
import sys
import time

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from calibrate_ft_gravity import rotation_error_deg, rotvec_to_matrix  # noqa: E402
from ur_arm import read_packet  # noqa: E402


def wrench_array(msg):
    return np.array([
        msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
        msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z,
    ])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--min-messages", type=int, default=300)
    parser.add_argument("--output", default="config/ft_gravity_validation.yaml")
    args = parser.parse_args()

    import rclpy
    from geometry_msgs.msg import WrenchStamped

    pose_before = read_packet()
    if not pose_before:
        raise RuntimeError("cannot read UR TCP before sampling")
    rclpy.init()
    node = rclpy.create_node("record_ft_validation")
    raw, compensated, software_bias = [], [], []
    node.create_subscription(WrenchStamped, "/ft_sensor/wrench_raw",
                             lambda msg: raw.append(wrench_array(msg)), 50)
    node.create_subscription(WrenchStamped, "/ft_sensor/wrench_compensated",
                             lambda msg: compensated.append(wrench_array(msg)), 50)
    node.create_subscription(WrenchStamped, "/ft_sensor/software_bias",
                             lambda msg: software_bias.append(wrench_array(msg)), 10)
    try:
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    pose_after = read_packet()
    if not pose_after:
        raise RuntimeError("cannot read UR TCP after sampling")
    if len(raw) < args.min_messages or len(compensated) < args.min_messages:
        raise RuntimeError("insufficient messages: raw=%d compensated=%d" %
                           (len(raw), len(compensated)))
    position_motion = np.linalg.norm(np.asarray(pose_after[:3]) - pose_before[:3])
    angle_motion = rotation_error_deg(rotvec_to_matrix(pose_before[3:]),
                                      rotvec_to_matrix(pose_after[3:]))
    if position_motion > 0.001 or angle_motion > 0.2:
        raise RuntimeError("robot moved during sample: %.2fmm %.3fdeg" %
                           (position_motion * 1000.0, angle_motion))

    output = os.path.abspath(os.path.expanduser(args.output))
    document = {"schema_version": 1, "samples": []}
    if os.path.exists(output):
        with open(output, encoding="utf-8") as stream:
            document = yaml.safe_load(stream) or document
    raw_array = np.asarray(raw)
    comp_array = np.asarray(compensated)
    sample = {
        "recorded_at": datetime.now(timezone.utc).isoformat(),
        "tcp_pose": [float(value) for value in pose_after],
        "motion_during_sample": {
            "position_m": float(position_motion),
            "angle_deg": float(angle_motion),
        },
        "raw": {
            "count": len(raw),
            "mean": raw_array.mean(axis=0).tolist(),
            "std": raw_array.std(axis=0).tolist(),
        },
        "compensated": {
            "count": len(compensated),
            "mean": comp_array.mean(axis=0).tolist(),
            "std": comp_array.std(axis=0).tolist(),
        },
        "software_bias": software_bias[-1].tolist() if software_bias else None,
    }
    document.setdefault("samples", []).append(sample)
    document["calibration"] = "config/ft_gravity_calibration.yaml"
    temp = output + ".tmp"
    with open(temp, "w", encoding="utf-8") as stream:
        yaml.safe_dump(document, stream, sort_keys=False, allow_unicode=True)
    os.replace(temp, output)
    print("saved", output, "sample", len(document["samples"]))
    print("raw_mean", np.round(sample["raw"]["mean"], 5))
    print("compensated_mean", np.round(sample["compensated"]["mean"], 5))
    print("counts", len(raw), len(compensated), "motion", position_motion,
          angle_motion)
    print("software_bias", sample["software_bias"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
