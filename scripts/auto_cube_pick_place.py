#!/usr/bin/env python3
"""One-command 50 mm cube pick/place pipeline.

Stages are deliberately explicit in the saved artefacts:
  1. read-only OpenCV + point-cloud detection beside the checkerboard;
  2. board-aligned top-down plan generation;
  3. dry-run by default, or guarded real execution with ``--execute``;
  4. during real execution, a second observation/replan at the pick hover.

No robot motion port is opened unless ``--execute`` is supplied.
"""
import argparse
import os
import subprocess
import sys


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def run(command, env):
    print("\n$ " + " ".join(command), flush=True)
    subprocess.run(command, cwd=ROOT, env=env, check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true",
                        help="explicitly authorise controller-native real motion")
    parser.add_argument("--observation",
                        default="outputs/vision/cube-board-observation.json")
    parser.add_argument("--plan", default="outputs/vision/auto-cube-pick-place-plan.json")
    parser.add_argument("--active-view-plan", default="outputs/vision/active-cube-view-plan.json")
    parser.add_argument("--board-center-x-m", type=float, default=0.024)
    parser.add_argument("--board-center-y-m", type=float, default=0.015)
    parser.add_argument("--no-publish-scene", action="store_true",
                        help="skip the read-only MoveIt PlanningScene publication")
    args = parser.parse_args()
    env = os.environ.copy()
    env.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")
    python = sys.executable
    run([python, "scripts/locate_cube_near_checkerboard.py", "--output", args.observation,
         "--board-center-x-m", str(args.board_center_x_m),
         "--board-center-y-m", str(args.board_center_y_m)], env)
    run([python, "scripts/plan_cube_pick_place.py", "--observation", args.observation,
         "--output", args.plan], env)
    run([python, "scripts/plan_active_cube_views.py", "--plan", args.plan,
         "--output", args.active_view_plan], env)
    if not args.no_publish_scene:
        run([python, "scripts/publish_auto_pick_scene.py", "--observation", args.observation,
             "--seconds", "1"], env)
    command = [python, "scripts/run_auto_cube_pick_place.py", "--plan", args.plan,
               "--active-view-plan", args.active_view_plan]
    if args.execute:
        command.append("--execute")
    run(command, env)


if __name__ == "__main__":
    raise SystemExit(main())
