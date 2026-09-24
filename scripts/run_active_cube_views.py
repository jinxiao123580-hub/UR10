#!/usr/bin/env python3
"""Execute a pre-gated low-speed active cube-view scan and archive clouds.

No board or RGB image is used after arriving at the hover.  Each view is
captured by the existing boardless point-cloud tracker; rejected geometry is
still retained as raw cloud evidence for later multi-view fitting.
"""
import argparse
import json
import math
import os
import socket
import subprocess
import sys
import time

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HOST, COMMAND_PORT = "192.168.1.3", 30002
sys.path.insert(0, os.path.join(ROOT, "scripts"))
from record_ur_trajectory import RealtimeReader


def orientation_error(a, b):
    ra, _ = cv2.Rodrigues(np.asarray(a[3:], dtype=float))
    rb, _ = cv2.Rodrigues(np.asarray(b[3:], dtype=float))
    return float(np.linalg.norm(cv2.Rodrigues(ra.T @ rb)[0]))


def at_target(actual, target):
    return (math.dist(actual[:3], target[:3]) <= 0.002 and
            orientation_error(actual, target) <= 0.02)


def move(reader, target, speed, acceleration):
    _q, start, _v = reader.read()
    program = ("def active_cube_view_move():\n"
               "  movel(p[%s], a=%.6f, v=%.6f)\n"
               "  sleep(0.4)\nend\nactive_cube_view_move()\n") % (
                   ", ".join("%.9f" % value for value in target), acceleration, speed)
    with socket.create_connection((HOST, COMMAND_PORT), timeout=3.0) as sock:
        sock.sendall(program.encode("ascii"))
    # For a pure roll the TCP translation is nearly zero.  ``movel`` still
    # needs time to traverse its rotational part; using only position here
    # previously timed out after 10 s while URScript was still moving.
    rotation = orientation_error(start, target)
    expected = max(math.dist(start[:3], target[:3]) / speed,
                   rotation / speed)
    deadline = time.monotonic() + max(15.0, expected * 2.5 + 8.0)
    latest = start
    while time.monotonic() < deadline:
        _q, latest, velocity = reader.read()
        if (at_target(latest, target) and
                np.linalg.norm(velocity[:3]) <= 0.003):
            return latest
    raise RuntimeError("view move did not settle; position error %.1f mm" %
                       (math.dist(latest[:3], target[:3]) * 1000.0))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", required=True, help="active_cube_view_plan JSON")
    parser.add_argument("--anchor", required=True, help="initial auto pick/place plan")
    parser.add_argument("--gate", default="outputs/vision/active-cube-view-gate.json")
    parser.add_argument("--output-dir", default="outputs/vision/active-cube-views")
    parser.add_argument("--speed", type=float, default=0.01)
    parser.add_argument("--acceleration", type=float, default=0.025)
    parser.add_argument("--resume", action="store_true",
                        help="continue at the currently reached view; never rotate back to an archived view")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if not args.execute and not args.dry_run:
        raise SystemExit("real active camera motion requires --execute")
    if not 0.005 <= args.speed <= 0.02 or not 0.005 <= args.acceleration <= 0.05:
        raise SystemExit("active scan only permits speed 0.005..0.02 and acceleration 0.005..0.05")
    plan_path = args.plan if os.path.isabs(args.plan) else os.path.join(ROOT, args.plan)
    with open(plan_path, encoding="utf-8") as stream:
        plan = json.load(stream)
    if plan.get("kind") != "active_cube_view_plan":
        raise SystemExit("not an active_cube_view_plan")
    output_dir = args.output_dir if os.path.isabs(args.output_dir) else os.path.join(ROOT, args.output_dir)
    os.makedirs(output_dir, exist_ok=True)
    reader = RealtimeReader(HOST, port=30013)
    try:
        _q, live, _v = reader.read(max_wait_seconds=5.0)
    finally:
        reader.close()
    matches = [view["slot"] for view in plan["views"]
               if at_target(live, view["target_tcp_pose_m_rad"])]
    start_slot = matches[0] if args.resume and matches else 1
    if args.dry_run:
        print("DRY-RUN: live TCP is at active-view slots", matches or "none",
              "; resume will start at slot", start_slot)
        return
    gate_path = args.gate if os.path.isabs(args.gate) else os.path.join(ROOT, args.gate)
    gate_command = [sys.executable, os.path.join(ROOT, "scripts", "gate_active_cube_views.py"),
                    "--plan", plan_path, "--output", gate_path]
    if subprocess.run(gate_command, cwd=ROOT).returncode != 0:
        raise SystemExit("live gate failed; no active-view motion was sent")
    with open(gate_path, encoding="utf-8") as stream:
        if not json.load(stream).get("passed"):
            raise SystemExit("gate report not passed")
    manifest_path = os.path.join(output_dir, "manifest.json")
    reports = []
    # Preserve already archived earlier views on an interrupted scan.  They
    # are raw evidence, so resuming must not silently discard them.
    if args.resume:
        for view in plan["views"]:
            if view["slot"] >= start_slot:
                continue
            report = os.path.join(output_dir, "view-%02d.json" % view["slot"])
            if not os.path.exists(report):
                raise SystemExit("cannot resume: archived report missing for view %d" % view["slot"])
            reports.append({"slot": view["slot"], "roll_deg": view["tool_roll_deg"],
                            "settled_tcp_m_rad": None, "report": report,
                            "tracker_returncode": None, "resumed_archived": True})

    def write_manifest(completed):
        manifest = {"schema_version": 1, "kind": "active_cube_view_capture_manifest",
                    "motion_sent": True, "complete": completed,
                    "plan": os.path.relpath(plan_path, ROOT),
                    "gate": os.path.relpath(gate_path, ROOT), "views": reports,
                    "runtime_board_usage": "none after hover: cloud only"}
        with open(manifest_path + ".tmp", "w", encoding="utf-8") as stream:
            json.dump(manifest, stream, ensure_ascii=False, indent=2); stream.write("\n")
        os.replace(manifest_path + ".tmp", manifest_path)

    reader = RealtimeReader(HOST, port=30013)
    try:
        for view in plan["views"]:
            if view["slot"] < start_slot:
                continue
            print("active view %d/%d: roll %+g deg" %
                  (view["slot"], len(plan["views"]), view["tool_roll_deg"]), flush=True)
            _q, before, _v = reader.read(max_wait_seconds=5.0)
            if at_target(before, view["target_tcp_pose_m_rad"]):
                after = before
                print("  已在该视角：补采点云，不重复转动", flush=True)
            else:
                after = move(reader, view["target_tcp_pose_m_rad"], args.speed, args.acceleration)
            report = os.path.join(output_dir, "view-%02d.json" % view["slot"])
            command = [sys.executable, os.path.join(ROOT, "scripts", "track_cube_without_checkerboard.py"),
                       "--anchor", args.anchor, "--captures", "3", "--min-coverage", "0.45",
                       "--output", report]
            completed = subprocess.run(command, cwd=ROOT)
            reports.append({"slot": view["slot"], "roll_deg": view["tool_roll_deg"],
                            "settled_tcp_m_rad": list(after), "report": report,
                            "tracker_returncode": completed.returncode})
            write_manifest(False)
    finally:
        reader.close()
    write_manifest(True)
    print("active views archived:", manifest_path)


if __name__ == "__main__":
    main()
