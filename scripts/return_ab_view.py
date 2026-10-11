#!/usr/bin/env python3
"""Guarded low-speed reverse of a checked, manually recorded A->B joint path.

Default is dry-run. This script cannot see people, cables, or external obstacles;
the operator must verify the scene is unchanged and remain at the emergency stop.
"""

import argparse
import csv
import json
import math
from pathlib import Path
import socket
import time

import numpy as np

from read_ur_state import dashboard
from record_ur_trajectory import RealtimeReader


ROOT = Path(__file__).resolve().parents[1]


def recorded_bounds(path):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    xyz = np.asarray([[float(row[key]) for key in
                       ("tcp_x_m", "tcp_y_m", "tcp_z_m")] for row in rows])
    if len(xyz) < 100 or not np.all(np.isfinite(xyz)):
        raise RuntimeError("recorded TCP path invalid")
    return xyz.min(axis=0) - 0.015, xyz.max(axis=0) + 0.015


def send(host, program):
    with socket.create_connection((host, 30002), timeout=3.0) as sock:
        sock.sendall(program.encode("ascii"))


def stop(host):
    try:
        send(host, "stopj(2.0)\n")
    except OSError as error:
        print("URGENT: automatic stopj failed; use physical E-stop:", error, flush=True)


def controller_gate():
    mode = dashboard("robotmode")
    safety = dashboard("safetystatus")
    program = dashboard("programState")
    if (mode != "Robotmode: RUNNING" or safety != "Safetystatus: NORMAL" or
            not (program or "").startswith("STOPPED")):
        raise RuntimeError(f"UR not RUNNING/NORMAL/STOPPED: {mode} / {safety} / {program}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gate", required=True)
    parser.add_argument("--trajectory", required=True)
    parser.add_argument("--a-state", required=True)
    parser.add_argument("--host", default="192.168.1.3")
    parser.add_argument("--execute", action="store_true",
                        help="send low-speed movej to real UR after all gates")
    parser.add_argument("--speed", type=float, default=0.06,
                        help="joint speed rad/s, maximum 0.08")
    parser.add_argument("--acceleration", type=float, default=0.06,
                        help="joint acceleration rad/s², maximum 0.10")
    args = parser.parse_args()
    if not (0.01 <= args.speed <= 0.08 and 0.01 <= args.acceleration <= 0.10):
        parser.error("speed/acceleration outside low-speed gate")
    gate = json.loads(Path(args.gate).read_text(encoding="utf-8"))
    if not gate.get("passed") or gate.get("reasons") or gate.get("selected_waypoints", 0) < 2:
        raise RuntimeError("offline gate did not pass")
    waypoints = np.asarray(gate["waypoint_q_a_to_b_rad"], dtype=float)
    if waypoints.shape != (gate["selected_waypoints"], 6) or not np.all(np.isfinite(waypoints)):
        raise RuntimeError("invalid waypoint array")
    a = np.asarray(json.loads(Path(args.a_state).read_text())["last"]["q_rad"])
    if np.max(np.abs(waypoints[0] - a)) > math.radians(0.25):
        raise RuntimeError("gate target no longer matches saved A")
    lower, upper = recorded_bounds(Path(args.trajectory))
    reader = RealtimeReader(args.host, port=30013)
    try:
        q, tcp, velocity = reader.read(max_wait_seconds=5)
        q = np.asarray(q)
        tcp = np.asarray(tcp)
        if np.max(np.abs(q - waypoints[-1])) > math.radians(0.3):
            raise RuntimeError("live robot not at recorded B; no motion")
        if np.linalg.norm(velocity[:3]) > 0.003:
            raise RuntimeError("robot not stationary")
        if np.any(tcp[:3] < lower) or np.any(tcp[:3] > upper):
            raise RuntimeError("live TCP outside recorded path corridor")
        controller_gate()
        targets = waypoints[-2::-1]
        print("reverse recorded path:", len(targets), "low-speed segments",
              "minimum modeled clearance mm:",
              round(gate["worst"]["min_self_clearance_mm"], 1), flush=True)
        if not args.execute:
            print("DRY-RUN: no motion sent", flush=True)
            return 0
        print("REAL ROBOT: remain at E-stop; returning B -> A", flush=True)
        for number, target in enumerate(targets, 1):
            q_start = q.copy()
            if np.max(np.abs(q_start - waypoints[-number])) > math.radians(0.5):
                raise RuntimeError("unexpected start of segment; no next motion")
            controller_gate()
            program = ("def return_ab_segment():\n"
                       "  movej([%s], a=%.6f, v=%.6f)\n"
                       "end\nreturn_ab_segment()\n" %
                       (", ".join("%.9f" % value for value in target),
                        args.acceleration, args.speed))
            send(args.host, program)
            print(f"segment {number}/{len(targets)} sent", flush=True)
            deadline = time.monotonic() + max(
                15.0, 4 * float(np.max(np.abs(target - q_start))) / args.speed + 10.0)
            settled = 0
            while time.monotonic() < deadline:
                try:
                    q_now, tcp_now, velocity_now = reader.read(max_wait_seconds=3)
                except TimeoutError:
                    stop(args.host)
                    raise RuntimeError("lost UR state during motion; stopj sent")
                q = np.asarray(q_now)
                tcp = np.asarray(tcp_now)
                delta = target - q_start
                fraction = np.clip(float((q - q_start) @ delta / max(delta @ delta, 1e-12)),
                                   0.0, 1.0)
                tube_error = np.max(np.abs(q - (q_start + fraction * delta)))
                if (tube_error > math.radians(2.0) or
                        np.any(tcp[:3] < lower) or np.any(tcp[:3] > upper)):
                    stop(args.host)
                    raise RuntimeError("joint path/TCP corridor deviation; stopj sent")
                if (np.max(np.abs(q - target)) < math.radians(0.25) and
                        np.linalg.norm(velocity_now[:3]) < 0.003):
                    settled += 1
                    if settled >= 8:
                        break
                else:
                    settled = 0
            else:
                stop(args.host)
                raise RuntimeError("segment timeout; stopj sent")
            print(f"segment {number}/{len(targets)} settled", flush=True)
        if np.max(np.abs(q - a)) > math.radians(0.3):
            raise RuntimeError("return ended outside A joint tolerance")
        print("RETURN A VERIFIED: max joint error deg",
              round(float(np.degrees(np.max(np.abs(q - a)))), 4), flush=True)
        return 0
    finally:
        reader.close()


if __name__ == "__main__":
    raise SystemExit(main())
