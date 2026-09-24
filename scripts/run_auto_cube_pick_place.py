#!/usr/bin/env python3
"""Replay an inspected auto-cube plan through the guarded position executor.

Without ``--execute`` this is a dry run.  It is intentionally a separate
command from recognition and planning: a live point cloud cannot be treated as
permission to move a real UR10.
"""
import argparse
import json
import os
import subprocess
import sys


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", required=True)
    parser.add_argument("--execute", action="store_true",
                        help="explicitly authorise the real-robot sequence")
    parser.add_argument("--active-view-plan", default=None,
                        help="optional precomputed active camera-view plan for failed hover tracking")
    parser.add_argument("--skip-reobserve-at-hover", action="store_true",
                        help="use an already verified refreshed plan; do not run camera tracking again")
    parser.add_argument("--fixed-height-place", action="store_true",
                        help="legacy only: use fixed Cartesian placement instead of raw-Fz contact placement")
    args = parser.parse_args()
    path = args.plan if os.path.isabs(args.plan) else os.path.join(ROOT, args.plan)
    with open(path, encoding="utf-8") as stream:
        plan = json.load(stream)
    if plan.get("kind") != "auto_cube_pick_place_plan" or plan.get("motion_sent"):
        raise SystemExit("not an unexecuted auto-cube pick/place plan")
    limits = plan.get("execution_limits", {})
    command = [sys.executable, os.path.join(ROOT, "scripts", "position_pick_place.py"),
               "--poses", path, "--height", str(plan["height"]),
               "--speed", str(limits.get("speed_m_s", 0.015)),
               "--acceleration", str(limits.get("acceleration_m_s2", 0.04)),
               "--position-tolerance", str(limits.get("position_tolerance_m", 0.002))]
    if not args.skip_reobserve_at_hover:
        command += ["--reobserve-at-hover",
                    "--refresh-observation", "outputs/vision/cube-track-refresh.json",
                    "--refresh-poses", "outputs/vision/auto-cube-pick-place-plan-refresh.json"]
    if args.active_view_plan:
        command += ["--active-view-plan", args.active_view_plan]
    if not args.fixed_height_place:
        command.append("--hold-after-pick")
    if args.execute:
        command.append("--execute")
        print("真机执行：请保持急停可用并全程监看。", flush=True)
    else:
        command.append("--dry-run")
        print("DRY-RUN：未授权运动。确认生成点位和 RViz 后，才可加 --execute。", flush=True)
    result = subprocess.run(command, cwd=ROOT).returncode
    if result != 0 or args.fixed_height_place or not args.execute:
        return result
    # The position executor deliberately stopped with the verified cube held.
    # For a fresh automatic run the re-observation output is the authoritative
    # pick plan; for an explicitly skipped re-observation use the supplied plan.
    place_plan = ("outputs/vision/auto-cube-pick-place-plan-refresh.json"
                  if not args.skip_reobserve_at_hover else path)
    finish = [sys.executable, os.path.join(ROOT, "scripts", "finish_held_cube_place.py"),
              "--plan", place_plan, "--execute"]
    print("抓取已验证，进入原始 Fz 接触式放置。", flush=True)
    return subprocess.run(finish, cwd=ROOT).returncode


if __name__ == "__main__":
    raise SystemExit(main())
