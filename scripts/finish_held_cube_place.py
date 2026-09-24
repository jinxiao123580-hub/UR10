#!/usr/bin/env python3
"""Finish a placement when a verified cube is already held in the gripper."""
import argparse
import json
import math
import os
import socket
import sys
import threading
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from record_ur_trajectory import RealtimeReader
from rq_gripper import RobotiqGripper

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class ContactDetected(RuntimeError):
    pass


class RawForceZ:
    """Raw ATI Fz only; no tare or unvalidated gravity compensation."""
    def __init__(self, topic):
        import rclpy
        from geometry_msgs.msg import WrenchStamped
        from rclpy.node import Node
        rclpy.init()
        self.rclpy = rclpy
        self.lock = threading.Lock()
        self.values = []
        self.latest = None
        self.node = Node("guarded_held_cube_place")
        def callback(msg):
            with self.lock:
                self.latest = float(msg.wrench.force.z)
                self.values.append(self.latest)
                self.values = self.values[-1000:]
        self.node.create_subscription(WrenchStamped, topic, callback, 200)
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.thread.start()

    def baseline(self, seconds=1.0):
        time.sleep(seconds)
        with self.lock:
            values = list(self.values)
        if len(values) < 80:
            raise RuntimeError("insufficient raw Fz samples: %d" % len(values))
        return float(np.median(values))

    def delta(self, baseline):
        with self.lock:
            return None if self.latest is None else abs(self.latest - baseline)

    def close(self):
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()
        self.thread.join(timeout=2)


def rotation_error(actual, target):
    left, _ = cv2.Rodrigues(np.asarray(actual[3:], dtype=float))
    right, _ = cv2.Rodrigues(np.asarray(target[3:], dtype=float))
    return float(np.linalg.norm(cv2.Rodrigues(left.T @ right)[0]))


def stopj():
    with socket.create_connection(("192.168.1.3", 30002), timeout=3) as sock:
        sock.sendall(b"stopj(2.0)\n")


def move(reader, start, target, speed, accel, label, force=None, baseline=None, spike_n=None,
         position_tolerance=0.0015):
    print(label, flush=True)
    script = ("def finish_held_cube_place():\n  movel(p[%s], a=%.6f, v=%.6f)\n  sleep(0.4)\nend\n"
              "finish_held_cube_place()\n") % (", ".join("%.9f" % x for x in target), accel, speed)
    with socket.create_connection(("192.168.1.3", 30002), timeout=3) as sock:
        sock.sendall(script.encode("ascii"))
    expected = max(math.dist(start[:3], target[:3]) / speed,
                   rotation_error(start, target) / speed)
    deadline = time.monotonic() + max(15.0, expected * 2.5 + 8.0)
    latest = start
    departed = False
    while time.monotonic() < deadline:
        _q, latest, velocity = reader.read()
        if force is not None and force.delta(baseline) >= spike_n:
            stopj()
            raise ContactDetected("raw Fz transient %.2f N >= %.2f N" %
                                  (force.delta(baseline), spike_n))
        departed = departed or math.dist(latest[:3], start[:3]) > position_tolerance
        if (departed and math.dist(latest[:3], target[:3]) <= position_tolerance and
                rotation_error(latest, target) <= 0.02 and
                np.linalg.norm(velocity[:3]) <= 0.003):
            return latest
    stopj()
    raise RuntimeError("placement move did not settle; stopj sent")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", required=True)
    parser.add_argument("--height", type=float, default=0.08,
                        help="same safe height used for lift and placement approach")
    parser.add_argument("--speed", type=float, default=0.015)
    parser.add_argument("--acceleration", type=float, default=0.04)
    parser.add_argument("--step-mm", type=float, default=2.0)
    parser.add_argument("--max-descent-mm", type=float, default=90.0)
    parser.add_argument("--fz-spike-n", type=float, default=8.0)
    parser.add_argument("--force-topic", default="/ft_sensor/wrench_raw")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if not args.execute:
        raise SystemExit("requires --execute")
    path = args.plan if os.path.isabs(args.plan) else os.path.join(ROOT, args.plan)
    with open(path, encoding="utf-8") as stream:
        place = [float(x) for x in json.load(stream)["place"]]
    if not 1.0 <= args.step_mm <= 3.0 or not 10.0 <= args.max_descent_mm <= 100.0:
        parser.error("step must be 1..3 mm; max descent must be 10..100 mm")
    reader = RealtimeReader("192.168.1.3", 30013)
    gripper = RobotiqGripper()
    force = None
    try:
        _q, current, _velocity = reader.read(max_wait_seconds=5)
        gripper.connect()
        status = gripper.status()
        if status.get("OBJ") != 2:
            raise RuntimeError("refuse placement: gripper OBJ=%s, cube is not confirmed held" % status.get("OBJ"))
        up = list(place); up[2] += args.height
        current = move(reader, current, up, args.speed, args.acceleration,
                       "⑥ 到放置区上方（与抬起同高度）")
        force = RawForceZ(args.force_topic)
        baseline = force.baseline()
        print("⑦ 原始 Fz 基线 %.2f N；每次下探 %.1f mm，突变 %.1f N 停止" %
              (baseline, args.step_mm, args.fz_spike_n), flush=True)
        contacted = False
        for index in range(int(args.max_descent_mm / args.step_mm)):
            target = list(current); target[2] -= args.step_mm / 1000.0
            try:
                current = move(reader, current, target, args.speed, args.acceleration,
                               "  下探 %d" % (index + 1), force, baseline, args.fz_spike_n,
                               position_tolerance=0.0005)
            except ContactDetected as exc:
                print("⑧ 接触确认：%s；停止后张开夹爪" % exc, flush=True)
                gripper.open()
                contacted = True
                break
        if not contacted:
            raise RuntimeError("max guarded descent reached without raw-Fz contact; cube remains clamped")
        _q, current, _velocity = reader.read(max_wait_seconds=5)
        retreat = list(current); retreat[2] += args.height
        move(reader, current, retreat, args.speed, args.acceleration, "⑨ 抬起离开")
        print("✔ 放置收尾完成", flush=True)
    finally:
        gripper.close_socket()
        reader.close()
        if force:
            force.close()


if __name__ == "__main__":
    main()
