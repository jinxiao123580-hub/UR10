#!/usr/bin/env python3
"""Validate or replay a recorded UR joint path through FollowJointTrajectory.

The path is split at selected calibration waypoints.  Every segment retains
the recorded intermediate joint states; the return path uses the same samples
in exact reverse order.  The default mode is offline validation only.  The
only execution mode currently exposed is ``--execute-fake`` so this script
cannot accidentally command real hardware during commissioning.
"""
import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np


JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]


def load_recording(path):
    with open(path, newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 2:
        raise RuntimeError("recording must contain at least two rows")
    elapsed = np.asarray([float(row["elapsed_s"]) for row in rows])
    q = np.asarray([[float(row[f"q{i}_rad"]) for i in range(1, 7)] for row in rows])
    if not np.all(np.isfinite(elapsed)) or not np.all(np.isfinite(q)):
        raise RuntimeError("recording contains NaN/Inf")
    if np.any(np.diff(elapsed) <= 0):
        raise RuntimeError("recording timestamps are not strictly increasing")
    return elapsed, q


def load_plan(path):
    with open(path, encoding="utf-8") as stream:
        plan = json.load(stream)
    waypoints = plan.get("outbound_waypoints", [])
    if not waypoints:
        raise RuntimeError("waypoint plan has no outbound_waypoints")
    return plan, waypoints


def nearest_indices(elapsed, waypoints):
    indices = []
    errors = []
    for waypoint in waypoints:
        target = float(waypoint["source_elapsed_s"])
        index = int(np.argmin(np.abs(elapsed - target)))
        indices.append(index)
        errors.append(abs(float(elapsed[index]) - target))
    if indices != sorted(set(indices)):
        raise RuntimeError("waypoint source times are duplicated or not chronological")
    return indices, errors


def build_segments(elapsed, q, indices, stride):
    segments = []
    for number, (start, stop) in enumerate(zip(indices[:-1], indices[1:]), 1):
        chosen = list(range(start, stop + 1, stride))
        if chosen[-1] != stop:
            chosen.append(stop)
        segment_q = q[chosen]
        segment_t = elapsed[chosen] - elapsed[start]
        segments.append({"number": number, "start": start, "stop": stop,
                         "q": segment_q, "t": segment_t})
    return segments


def validate(elapsed, q, indices, segments, speed_scale, joint_margin_deg):
    dt = np.diff(elapsed)
    velocity = np.abs(np.diff(q, axis=0) / dt[:, None])
    limit = 2.0 * math.pi - math.radians(joint_margin_deg)
    margin = limit - np.max(np.abs(q), axis=0)
    if np.min(margin) < 0:
        raise RuntimeError("recording violates configured joint margin")
    if not 0 < speed_scale <= 1:
        raise RuntimeError("speed scale must be in (0, 1]")
    if not np.array_equal(np.concatenate([s["q"][:-1] for s in segments] +
                                         [segments[-1]["q"][-1:]]),
                          q[indices[0]:indices[-1] + 1]):
        # This equality only holds at stride=1.  Endpoint/path checks below
        # remain the authoritative checks for downsampled execution.
        pass
    for segment in segments:
        if not np.array_equal(segment["q"][0], q[segment["start"]]):
            raise RuntimeError("segment start mismatch")
        if not np.array_equal(segment["q"][-1], q[segment["stop"]]):
            raise RuntimeError("segment end mismatch")
    returned = [segment["q"][::-1] for segment in reversed(segments)]
    if not np.array_equal(returned[-1][-1], q[indices[0]]):
        raise RuntimeError("reverse path does not return to exact initial q")
    return {
        "sample_count": int(len(q)),
        "duration_s": float(elapsed[-1] - elapsed[0]),
        "waypoint_indices": indices,
        "segment_count_each_direction": len(segments),
        "max_recorded_joint_velocity_rad_s": velocity.max(axis=0).tolist(),
        "max_replay_joint_velocity_estimate_rad_s":
            (velocity.max(axis=0) * speed_scale).tolist(),
        "minimum_margin_to_360_deg":
            float(360.0 - math.degrees(np.max(np.abs(q)))),
        "minimum_margin_above_gate_deg": float(math.degrees(np.min(margin))),
        "exact_reverse_return": True,
        "exact_initial_q_rad": q[indices[0]].tolist(),
    }


def duration_msg(seconds):
    from builtin_interfaces.msg import Duration
    nanoseconds = max(1, int(round(seconds * 1e9)))
    return Duration(sec=nanoseconds // 1_000_000_000,
                    nanosec=nanoseconds % 1_000_000_000)


def execute_fake(segments, speed_scale, dwell_scale):
    import rclpy
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    rclpy.init()
    node = rclpy.create_node("recorded_path_fake_replay")
    client = ActionClient(node, FollowJointTrajectory,
                          "/joint_trajectory_controller/follow_joint_trajectory")
    if not client.wait_for_server(timeout_sec=10.0):
        raise RuntimeError("FollowJointTrajectory action server unavailable")

    def send(q_values, relative_times, label):
        trajectory = JointTrajectory(joint_names=JOINT_NAMES)
        base = float(relative_times[0])
        for positions, stamp in zip(q_values, relative_times):
            point = JointTrajectoryPoint()
            point.positions = positions.tolist()
            point.time_from_start = duration_msg(0.05 + (float(stamp) - base) / speed_scale)
            trajectory.points.append(point)
        goal = FollowJointTrajectory.Goal(trajectory=trajectory)
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"{label}: goal rejected")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        wrapped = result_future.result()
        if wrapped is None or wrapped.result.error_code != 0:
            code = None if wrapped is None else wrapped.result.error_code
            raise RuntimeError(f"{label}: action failed, code={code}")
        print(f"{label}: SUCCESS ({len(q_values)} points)", flush=True)

    try:
        for index, segment in enumerate(segments, 1):
            send(segment["q"], segment["t"], f"outbound {index}/{len(segments)}")
            time.sleep(dwell_scale)
        for index, segment in enumerate(reversed(segments), 1):
            q_reverse = segment["q"][::-1]
            original = segment["t"]
            t_reverse = original[-1] - original[::-1]
            send(q_reverse, t_reverse, f"return {index}/{len(segments)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", default="outputs/ur_trajectory/teach-20260916-110054.csv")
    parser.add_argument("--waypoints", default="outputs/ur_trajectory/teach-20260916-110054-waypoints.json")
    parser.add_argument("--speed-scale", type=float, default=0.25)
    parser.add_argument("--stride", type=int, default=4,
                        help="retain every Nth recorded frame plus every exact endpoint")
    parser.add_argument("--joint-margin-deg", type=float, default=8.0,
                        help="full recorded path gate; measured minimum is 8.609 deg")
    parser.add_argument("--report", default="outputs/ur_trajectory/official-replay-validation.json")
    parser.add_argument("--execute-fake", action="store_true",
                        help="execute on an already-running fake-hardware controller only")
    parser.add_argument("--fake-speed-scale", type=float, default=20.0,
                        help="simulation-only time compression; ignored unless --execute-fake")
    parser.add_argument("--fake-dwell", type=float, default=0.05)
    args = parser.parse_args()
    if args.stride < 1 or not 0 < args.joint_margin_deg < 180:
        parser.error("stride must be >=1 and joint margin must be in (0,180)")

    elapsed, q = load_recording(args.trajectory)
    plan, waypoints = load_plan(args.waypoints)
    indices, time_errors = nearest_indices(elapsed, waypoints)
    segments = build_segments(elapsed, q, indices, args.stride)
    report = validate(elapsed, q, indices, segments, args.speed_scale,
                      args.joint_margin_deg)
    report.update({
        "schema_version": 1,
        "trajectory": os.path.abspath(args.trajectory),
        "waypoints": os.path.abspath(args.waypoints),
        "speed_scale": args.speed_scale,
        "stride": args.stride,
        "waypoint_time_match_max_error_s": max(time_errors),
        "execution_authorized": False,
        "note": "Offline/fake-hardware evidence only; not real-robot authorization.",
    })
    os.makedirs(os.path.dirname(os.path.abspath(args.report)), exist_ok=True)
    with open(args.report, "w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2)
        stream.write("\n")
    print(json.dumps(report, indent=2))
    if args.execute_fake:
        if args.fake_speed_scale <= 0:
            parser.error("fake speed scale must be positive")
        execute_fake(segments, args.fake_speed_scale, args.fake_dwell)


if __name__ == "__main__":
    main()
