#!/usr/bin/env python3
"""Record a no-motion ATI raw-wrench baseline for power-cycle comparison."""
import argparse
import json
import os
import socket
import struct
import threading
import time

import numpy as np


def read_tcp_pose(host):
    with socket.create_connection((host, 30003), timeout=3.0) as sock:
        sock.settimeout(3.0)
        buffer = bytearray()
        while len(buffer) < 4:
            buffer.extend(sock.recv(4096))
        size = struct.unpack_from(">I", buffer)[0]
        while len(buffer) < size:
            buffer.extend(sock.recv(4096))
        return list(struct.unpack_from(">6d", buffer, 444))


class Collector:
    def __init__(self, topic):
        import rclpy
        from geometry_msgs.msg import WrenchStamped
        from rclpy.node import Node
        self.rows = []
        self.lock = threading.Lock()
        self.node = Node("record_ft_static_baseline")

        def callback(msg):
            w = msg.wrench
            row = [w.force.x, w.force.y, w.force.z,
                   w.torque.x, w.torque.y, w.torque.z]
            with self.lock:
                self.rows.append((time.time(), row))

        self.node.create_subscription(WrenchStamped, topic, callback, 200)
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.thread.start()

    def close(self):
        import rclpy
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.thread.join(timeout=2.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.1.3")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--topic", default="/ft_sensor/wrench_raw")
    parser.add_argument("--output")
    parser.add_argument("--compare", help="compare this baseline with a prior JSON")
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("duration must be positive")

    import rclpy
    rclpy.init()
    collector = Collector(args.topic)
    started = time.time()
    pose_start = read_tcp_pose(args.host)
    time.sleep(args.duration)
    pose_end = read_tcp_pose(args.host)
    with collector.lock:
        rows = np.asarray([row for stamp, row in collector.rows if stamp >= started])
    collector.close()
    if len(rows) < max(20, int(args.duration * 50)):
        raise RuntimeError("only %d wrench samples collected" % len(rows))

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    output = args.output or os.path.join(
        root, "outputs", "ft_baseline", "baseline-%s.json" %
        time.strftime("%Y%m%d-%H%M%S"))
    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    document = {
        "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "duration_s": args.duration,
        "topic": args.topic,
        "sample_count": int(len(rows)),
        "tcp_pose_start": pose_start,
        "tcp_pose_end": pose_end,
        "wrench_mean": rows.mean(axis=0).tolist(),
        "wrench_std": rows.std(axis=0, ddof=1).tolist(),
        "wrench_min": rows.min(axis=0).tolist(),
        "wrench_max": rows.max(axis=0).tolist(),
    }
    if args.compare:
        with open(args.compare) as stream:
            previous = json.load(stream)
        old_pose = np.asarray(previous["tcp_pose_end"], dtype=float)
        new_pose = np.asarray(pose_start, dtype=float)
        from calibrate_ft_gravity import rotvec_to_matrix
        relative = rotvec_to_matrix(old_pose[3:]).T @ rotvec_to_matrix(new_pose[3:])
        angle_deg = float(np.degrees(np.arccos(np.clip(
            (np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))))
        delta_wrench = rows.mean(axis=0) - np.asarray(previous["wrench_mean"])
        document["comparison"] = {
            "previous": args.compare,
            "position_delta_m": float(np.linalg.norm(new_pose[:3] - old_pose[:3])),
            "orientation_delta_deg": angle_deg,
            "wrench_delta": delta_wrench.tolist(),
            "comparable": bool(np.linalg.norm(new_pose[:3] - old_pose[:3]) <= 0.005
                                and angle_deg <= 2.0),
        }
    with open(output, "w") as stream:
        json.dump(document, stream, indent=2)
        stream.write("\n")
    print(json.dumps(document, indent=2))
    print("OUTPUT:", output)


if __name__ == "__main__":
    main()
