#!/usr/bin/env python3
"""Offline numeric gate for a scripted Cartesian move plan (no motion is sent).

This is the "离线数值门禁" layer that must pass before any real robot motion.
It answers the questions that can be answered without touching the robot:

* does the nominal URDF FK still agree with the controller's TCP at the current
  pose (if not, every IK target built on it is suspect)?
* is every planned pose reachable inside the joint limits with margin?
* does the straight-line ``movel`` path between waypoints stay away from
  singularities, and does the flange stay above a clearance floor?

``movel`` semantics matter here: it is a straight line in Cartesian space, so a
target that is reachable on its own can still be unreachable *along the way*.
Each segment is therefore densely sampled, IK is warm-started from the previous
sample, and the frame Jacobian's smallest singular value is tracked as the
singularity proxy.

What this gate deliberately does NOT do: prove collision-freedom.  There is no
obstacle model in this repository, so the plan compensates structurally instead -
lift straight up to a clearance height, translate there, then descend - and the
clearance height must be chosen above every object on the table.  That argument
is physical, not numerical, and it needs human eyes on the workspace.

Read-only: it connects to 30003 to read state and never opens 30002.
"""
import argparse
import datetime as dt
import json
import os
import socket
import sys

import numpy as np
import pinocchio as pin

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ur_pose_ik import LIMIT_MARGIN, LOWER, UPPER, UR10IK  # noqa: E402

# The controller's base frame and the URDF root differ by a 180 deg yaw at the
# same origin (UR official "Robot Frames").  test_ik_move.py validated FK against
# the controller through exactly this mapping.
BASE_FROM_URDF_ROOT = pin.SE3(pin.exp3(np.array([0.0, 0.0, np.pi])), np.zeros(3))

FLANGE_Z_FLOOR_M = 0.25     # ur_relative_movel.py refuses tool0 Z below this
FK_POSITION_GATE_M = 0.010  # test_ik_move.py refuses IK above this disagreement
FK_ROTATION_GATE_RAD = 0.020
MIN_SINGULAR_GATE_FRACTION = 0.15   # vs the current pose's smallest singular value
SINGULARITY_FLOOR = 0.02            # absolute floor for the smallest singular value


def read_state(host, port=30013):
    """Read one (q, tcp) pair from a read-only UR realtime port."""
    from record_ur_trajectory import RealtimeReader
    reader = RealtimeReader(host, port)
    try:
        q, tcp, _ = reader.read()
    finally:
        reader.close()
    return np.asarray(q, dtype=float), np.asarray(tcp, dtype=float)


def robot_pose(values):
    return pin.SE3(pin.exp3(np.asarray(values[3:], dtype=float)),
                   np.asarray(values[:3], dtype=float))


def pose_from_tcp_target(target):
    return pin.SE3(pin.exp3(np.asarray(target[3:], dtype=float)),
                   np.asarray(target[:3], dtype=float))


def tcp_values(pose):
    return list(pose.translation) + list(pin.log3(pose.rotation))


def interpolate(start, end, count):
    """Straight-line position and slerp orientation, as movel implies."""
    steps = []
    delta = pin.log6(start.inverse() * end).vector
    for index in range(count + 1):
        fraction = index / float(count)
        steps.append(start * pin.exp6(delta * fraction))
    return steps


def jacobian_singular_values(ik, q):
    jacobian = pin.computeFrameJacobian(ik.model, ik.data, np.asarray(q),
                                        ik.frame_id, pin.LOCAL)
    singular = np.linalg.svd(jacobian, compute_uv=False)
    return float(singular[-1]), float(singular[0])


def check_segment(ik, start_pose, end_pose, start_q, samples,
                  self_collision=None, min_clearance_m=None, joint6_range_rad=None):
    """Densely sample one straight-line segment and gate every sample.

    The camera-versus-arm clearance is checked at every sample, not just at the
    endpoints: on 2026-09-18 the camera contacted the forearm partway through a
    reorientation whose endpoints were both acceptable.
    """
    poses = interpolate(start_pose, end_pose, samples)
    q = np.asarray(start_q, dtype=float)
    rows = []
    worst = {"ik_position_m": 0.0, "ik_rotation_rad": 0.0,
             "joint_margin_rad": float("inf"), "min_singular": float("inf"),
             "max_joint_step_rad": 0.0, "min_flange_z_m": float("inf"),
             "min_camera_clearance_m": float("inf"),
             "worst_camera_pair": None}
    reasons = []
    for index, pose in enumerate(poses):
        target_root = BASE_FROM_URDF_ROOT.inverse() * pose
        solved, position_error, rotation_error, jump, failure, _ = \
            ik.solve_checked(target_root, q)
        margin = float(min(np.min(solved - LOWER), np.min(UPPER - solved)))
        singular_min, singular_max = jacobian_singular_values(ik, solved)
        step = float(np.max(np.abs(solved - q))) if index else 0.0
        flange_z = float(solved.size and ik.pose(solved).translation[2])
        flange_z_base = float((BASE_FROM_URDF_ROOT * ik.pose(solved)).translation[2])
        worst["ik_position_m"] = max(worst["ik_position_m"], position_error)
        worst["ik_rotation_rad"] = max(worst["ik_rotation_rad"], rotation_error)
        worst["joint_margin_rad"] = min(worst["joint_margin_rad"], margin)
        worst["min_singular"] = min(worst["min_singular"], singular_min)
        worst["max_joint_step_rad"] = max(worst["max_joint_step_rad"], step)
        worst["min_flange_z_m"] = min(worst["min_flange_z_m"], flange_z_base)
        if failure:
            reasons.append("sample %d rejected: %s" % (index, ",".join(failure)))
        if step > 0.6:
            reasons.append("sample %d joint step %.3f rad" % (index, step))
        if flange_z_base < FLANGE_Z_FLOOR_M:
            reasons.append("sample %d flange z %.4f m below floor" %
                           (index, flange_z_base))
        if joint6_range_rad is not None:
            low, high = joint6_range_rad
            if not low <= solved[5] <= high:
                reasons.append("sample %d J6 %.1f deg outside [%.1f, %.1f] deg" %
                               (index, np.degrees(solved[5]), np.degrees(low),
                                np.degrees(high)))
        if self_collision is not None:
            clearance, pair = self_collision.min_clearance(solved)
            if clearance < worst["min_camera_clearance_m"]:
                worst["min_camera_clearance_m"] = clearance
                worst["worst_camera_pair"] = pair
            if min_clearance_m is not None and clearance < min_clearance_m:
                reasons.append("sample %d camera clearance %.1f mm < %.1f mm (%s)" %
                               (index, clearance * 1000.0, min_clearance_m * 1000.0,
                                pair))
        rows.append({"index": index, "q": solved.tolist(),
                     "camera_clearance_m": (None if self_collision is None
                                            else clearance),
                     "position_error_m": position_error,
                     "rotation_error_rad": rotation_error,
                     "joint_margin_rad": margin,
                     "min_singular": singular_min,
                     "condition_number": (singular_max / singular_min
                                          if singular_min > 0 else None),
                     "joint6_deg": float(np.degrees(solved[5])),
                     "flange_z_base_m": flange_z_base})
        q = solved
    return {"worst": worst, "reasons": reasons, "samples": rows,
            "end_q": q.tolist()}


def build_waypoints(current, target, clearance_z, order):
    """Waypoints at or above the clearance height, in the requested order.

    Order matters: reorienting in place and translating both have to respect the
    wrist joint limits, and which one is feasible depends on where the arm
    already is.  Order A reorients first (shorter tool sweep), order B
    translates first (keeps the wrist closer to its current configuration).
    """
    lifted = pin.SE3(current.rotation.copy(),
                     np.array([current.translation[0], current.translation[1],
                               clearance_z]))
    moved = pin.SE3(target.rotation.copy(),
                    np.array([target.translation[0], target.translation[1],
                              clearance_z]))
    reoriented = pin.SE3(target.rotation.copy(),
                         np.array([current.translation[0], current.translation[1],
                                   clearance_z]))
    if order == "reorient_first":
        return [("lift", current, lifted), ("reorient", lifted, reoriented),
                ("translate", reoriented, moved), ("descend", moved, target)]
    if order == "translate_first":
        return [("lift", current, lifted),
                ("translate", lifted,
                 pin.SE3(current.rotation.copy(),
                         np.array([target.translation[0], target.translation[1],
                                   clearance_z]))),
                ("reorient", pin.SE3(current.rotation.copy(),
                                     np.array([target.translation[0],
                                               target.translation[1],
                                               clearance_z])), moved),
                ("descend", moved, target)]
    if order == "reorient_at_start":
        reoriented_start = pin.SE3(target.rotation.copy(), current.translation.copy())
        lifted_target = pin.SE3(target.rotation.copy(), np.array([
            current.translation[0], current.translation[1], clearance_z]))
        moved_target = pin.SE3(target.rotation.copy(), np.array([
            target.translation[0], target.translation[1], clearance_z]))
        return [("reorient_start", current, reoriented_start),
                ("lift", reoriented_start, lifted_target),
                ("translate", lifted_target, moved_target),
                ("descend", moved_target, target)]
    raise ValueError("unknown order: %s" % order)


MOVE_ORDERS = ("reorient_first", "translate_first", "reorient_at_start")

# Roll offsets tried when a slot fails: the roll is a free DOF, so these are
# equivalent shots with a different wrist twist.
RESCUE_ROLL_OFFSETS_DEG = (15.0, -15.0, 30.0, -30.0, 45.0, -45.0, 60.0, -60.0,
                           90.0, -90.0, 120.0, -120.0, 150.0, -150.0, 180.0)


def print_row(slot, name, order, worst, passed, reasons):
    clearance = worst.get("min_camera_clearance_m")
    clearance_text = ("     -" if clearance is None or clearance == float("inf")
                      else "%6.1f" % (clearance * 1000.0))
    print("%-4d %-10s %-15s %8.4f %9.4f %9.4f %9.4f %8.4f %smm %s" %
          (slot, name, order, worst["ik_position_m"] * 1000.0,
           worst["ik_rotation_rad"] * 1000.0, worst["joint_margin_rad"],
           worst["min_singular"], worst["min_flange_z_m"], clearance_text,
           "PASS" if passed else "FAIL " + ";".join(reasons[:2])))


def roll_about_optical_axis(target_matrix, tool0_from_camera, delta_deg):
    """Rotate a target TCP pose (4x4) about the camera's own optical axis.

    The roll is a free DOF: it does not change which part of the board the camera
    sees, only how the wrist is twisted.  Keeping ``base_from_camera`` fixed and
    rotating it about its own z axis gives an equivalent shot with a different
    wrist configuration.
    """
    base_from_camera = np.asarray(target_matrix) @ np.linalg.inv(tool0_from_camera)
    delta = np.radians(delta_deg)
    twist = np.eye(4)
    twist[:3, :3] = np.asarray([[np.cos(delta), -np.sin(delta), 0.0],
                                [np.sin(delta), np.cos(delta), 0.0],
                                [0.0, 0.0, 1.0]])
    return base_from_camera @ twist @ tool0_from_camera


def to_se3(matrix):
    return pin.SE3(np.asarray(matrix)[:3, :3], np.asarray(matrix)[:3, 3])


def to_matrix(pose):
    matrix = np.eye(4)
    matrix[:3, :3] = pose.rotation
    matrix[:3, 3] = pose.translation
    return matrix


def try_slot(ik, start_pose, start_q, target, clearance_z, samples, orders,
             baseline_min_singular, self_collision=None, min_clearance_m=None,
             joint6_range_rad=None):
    """Try the waypoint orders for one slot; return the first that passes."""
    for order in orders:
        trial_q = np.asarray(start_q, dtype=float)
        trial_pose = start_pose
        records = []
        ok = True
        for name, begin, end in build_waypoints(start_pose, target, clearance_z,
                                                order):
            result = check_segment(ik, begin, end, trial_q, samples,
                                   self_collision, min_clearance_m, joint6_range_rad)
            worst = result["worst"]
            singular_ok = (worst["min_singular"] >= SINGULARITY_FLOOR and
                           worst["min_singular"] >=
                           MIN_SINGULAR_GATE_FRACTION * baseline_min_singular)
            if not singular_ok and not result["reasons"]:
                result["reasons"].append(
                    "singularity margin: min %.4f < floor %.4f or < %.2f x "
                    "baseline %.4f" % (worst["min_singular"], SINGULARITY_FLOOR,
                                       MIN_SINGULAR_GATE_FRACTION,
                                       baseline_min_singular))
            records.append({"segment": name, "order": order,
                            "start_tcp_m_rad": tcp_values(begin),
                            "end_tcp_m_rad": tcp_values(end),
                            # Preserve the warm-started IK branch actually gated.
                            # Downstream collision checks must never re-solve an
                            # endpoint from an unrelated zero-joint seed.
                            "end_q_rad": result["end_q"],
                            "worst": worst,
                            "singularity_gate_passed": bool(singular_ok),
                            "passed": bool(not result["reasons"] and singular_ok),
                            "reasons": result["reasons"]})
            if not records[-1]["passed"]:
                ok = False
                break
            trial_q = np.asarray(result["end_q"], dtype=float)
            trial_pose = end
        if ok:
            return True, order, trial_q, trial_pose, records
    return False, None, np.asarray(start_q, dtype=float), start_pose, records


def order_slots_nearest(ik, entries, seed_q):
    """Greedy nearest-neighbour ordering in joint space.

    Whether a slot is reachable depends on where the previous slot left the wrist
    (measured: slot 17 passes in isolation but fails right after slot 16), so the
    visiting order is part of the plan, not a detail.
    """
    remaining = list(entries)
    ordered = []
    current = np.asarray(seed_q, dtype=float)
    while remaining:
        best = None
        for entry in remaining:
            target = BASE_FROM_URDF_ROOT.inverse() * \
                pose_from_tcp_target(entry["target_tcp_pose_m_rad"])
            solved, _pe, _re, _result = ik.solve(target, current)
            cost = float(np.max(np.abs(solved - current)))
            if best is None or cost < best[0]:
                best = (cost, entry, solved)
        cost, entry, solved = best
        value = dict(entry)
        value["ik_chain_step_rad"] = cost
        ordered.append(value)
        remaining = [item for item in remaining if item["slot"] != entry["slot"]]
        current = solved
    return ordered


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--plan", default="outputs/handeye/pose-plan-20260918.json")
    parser.add_argument("--slots", default=None,
                        help="comma-separated slot numbers to check (default: all)")
    parser.add_argument("--host", default="192.168.1.3")
    parser.add_argument("--port", type=int, default=30013,
                        help="UR read-only realtime port (30013 is verified on this robot)")
    parser.add_argument("--clearance-z", type=float, default=0.55,
                        help="travelling height in the base frame; must exceed every "
                             "object on the table (the board is a few mm, the cube 50 mm)")
    parser.add_argument("--samples-per-segment", type=int, default=60)
    parser.add_argument("--transit-home", action="store_true",
                        help="between slots, return to the starting x,y at the "
                             "clearance height. Makes each slot's reorientation "
                             "start from the same configuration, so a slot no longer "
                             "depends on where the previous slot happened to leave "
                             "the wrist")
    parser.add_argument("--rescue-clearance", type=float, nargs="*",
                        default=[0.50, 0.60, 0.65],
                        help="extra travelling heights tried when a slot fails; the "
                             "height is free as long as it clears every table object")
    parser.add_argument("--retry-deferred", action="store_true", default=True,
                        help="push a failing slot to the end of the queue and retry "
                             "it from a different predecessor (reachability depends on "
                             "where the previous slot left the wrist)")
    parser.add_argument("--max-deferral-passes", type=int, default=3)
    parser.add_argument("--order", choices=("nearest", "listed"), default="nearest",
                        help="visit order: 'nearest' greedily picks the next slot with "
                             "the smallest joint-space step, which avoids the bad "
                             "transitions that make an otherwise reachable slot fail")
    parser.add_argument("--path-orders", nargs="+", choices=MOVE_ORDERS,
                        default=list(MOVE_ORDERS),
                        help="waypoint-order candidates to try for each target")
    parser.add_argument("--joint6-range-deg", type=float, nargs=2, metavar=("MIN", "MAX"),
                        default=None,
                        help="require every dense path sample to keep J6 in this degree range")
    parser.add_argument("--self-collision-margin-mm", type=float, default=40.0,
                        help="gate the camera-vs-arm clearance at EVERY path sample; "
                             "0 disables it")
    parser.add_argument("--rescue-rolls", action="store_true",
                        help="when a slot fails, rotate it about the camera's optical "
                             "axis (the roll is a free DOF that does not change which "
                             "part of the board is seen) and retry, searching for an "
                             "orientation whose whole waypoint chain passes. Needs "
                             "--calibration for the camera-to-tool0 transform")
    parser.add_argument("--calibration",
                        default="config/handeye_eye_in_hand_20260917.yaml",
                        help="guidance tool0_from_camera, used only for --rescue-rolls")
    parser.add_argument("--assume-q", default=None,
                        help="6 joint angles (rad) to use instead of reading 30003. For "
                             "offline gate runs while the robot is unreachable (for "
                             "example when the wired NIC has lost carrier and traffic "
                             "falls back to a VPN tunnel). The result is then valid for "
                             "the ASSUMED state only and is labelled as such")
    parser.add_argument("--assume-tcp", default=None,
                        help="matching 6 TCP values (m, rad) for --assume-q")
    parser.add_argument("--output", default=None)
    args = parser.parse_args()
    if args.joint6_range_deg and args.joint6_range_deg[0] >= args.joint6_range_deg[1]:
        parser.error("--joint6-range-deg requires MIN < MAX")
    joint6_range_rad = (None if args.joint6_range_deg is None else
                        tuple(np.radians(args.joint6_range_deg)))

    plan_path = args.plan if os.path.isabs(args.plan) else os.path.join(root, args.plan)
    with open(plan_path, encoding="utf-8") as stream:
        plan = json.load(stream)
    entries = plan["plan"]
    if args.slots:
        wanted = {int(x) for x in args.slots.split(",") if x.strip()}
        entries = [e for e in entries if e["slot"] in wanted]
    entries = sorted(entries, key=lambda e: e["slot"])

    target_height = min(e["target_tcp_pose_m_rad"][2] for e in entries)
    if args.clearance_z <= target_height:
        parser.error("clearance-z %.3f must exceed the highest target z %.3f" %
                     (args.clearance_z, target_height))

    assumed_state = False
    if args.assume_q:
        q_now = np.asarray([float(x) for x in args.assume_q.split(",")], dtype=float)
        if q_now.size != 6:
            parser.error("--assume-q needs 6 comma-separated values (rad)")
        if args.assume_tcp:
            tcp_now = np.asarray([float(x) for x in args.assume_tcp.split(",")],
                                 dtype=float)
            if tcp_now.size != 6:
                parser.error("--assume-tcp needs 6 comma-separated values")
        else:
            tcp_now = None
        assumed_state = True
    else:
        q_now, tcp_now = read_state(args.host, args.port)
    ik = UR10IK()
    if tcp_now is None:
        tcp_now = np.asarray(tcp_values(BASE_FROM_URDF_ROOT * ik.pose(q_now)))
    fk_base = BASE_FROM_URDF_ROOT * ik.pose(q_now)
    measured = robot_pose(tcp_now)
    agreement = pin.log6(fk_base.inverse() * measured).vector
    fk_position = float(np.linalg.norm(agreement[:3]))
    fk_rotation = float(np.linalg.norm(agreement[3:]))

    document = {
        "schema_version": 1,
        "created_at": dt.datetime.now(dt.timezone.utc).astimezone().isoformat(),
        "kind": "scripted_move_offline_gate",
        "motion_sent": False,
        "plan": os.path.relpath(plan_path, root),
        "slots_checked": [e["slot"] for e in entries],
        "clearance_z_m": args.clearance_z,
        "model": {
            "urdf": os.path.expanduser("~/ur_learn/generated/ur10.urdf"),
            "note": "nominal URDF kinematics; the controller uses factory "
                    "calibration, so a small FK disagreement is expected and is "
                    "gated below",
            "fk_position_agreement_m": fk_position,
            "fk_rotation_agreement_rad": fk_rotation,
            "fk_position_gate_m": FK_POSITION_GATE_M,
            "fk_rotation_gate_rad": FK_ROTATION_GATE_RAD,
            "fk_gate_passed": bool(fk_position <= FK_POSITION_GATE_M and
                                   fk_rotation <= FK_ROTATION_GATE_RAD),
        },
        "current": {"q_rad": list(q_now), "tcp_m_rad": list(tcp_now),
                    "source": ("assumed via --assume-q (robot unreachable)"
                               if assumed_state else "read from %d" % args.port)},
        "segments": [],
    }
    print("当前 TCP:", [round(x, 4) for x in tcp_now[:3]],
          "| 姿态(rx,ry,rz rad):", [round(x, 4) for x in tcp_now[3:]])
    print("FK 一致度: %.6f m / %.6f rad (门禁 %.3f m / %.3f rad) -> %s" %
          (fk_position, fk_rotation, FK_POSITION_GATE_M, FK_ROTATION_GATE_RAD,
           "PASS" if document["model"]["fk_gate_passed"] else "FAIL"))
    if not document["model"]["fk_gate_passed"]:
        print("停止：FK 与控制器不一致，任何基于它的 IK 目标都不可信。")
        return 2

    if args.order == "nearest":
        entries = order_slots_nearest(ik, entries, q_now)
        document["visit_order"] = [e["slot"] for e in entries]
        document["visit_order_policy"] = "greedy nearest neighbour in joint space"
        print("访问顺序（关节空间最近邻）:", [e["slot"] for e in entries])
    else:
        document["visit_order"] = [e["slot"] for e in entries]
        document["visit_order_policy"] = "as listed in the plan"

    baseline_min_singular = jacobian_singular_values(ik, q_now)[0]
    document["baseline_min_singular"] = baseline_min_singular
    print("当前位姿最小奇异值 %.4f（作为奇异度基线）" % baseline_min_singular)
    print()

    tool0_from_camera = None
    if args.rescue_rolls or args.self_collision_margin_mm > 0:
        import yaml
        with open(os.path.join(root, args.calibration), encoding="utf-8") as stream:
            calibration = yaml.safe_load(stream)
        tool0_from_camera = np.eye(4)
        tool0_from_camera[:3, :3] = np.asarray(calibration["rotation_matrix"],
                                               dtype=float)
        tool0_from_camera[:3, 3] = np.asarray(calibration["translation_m"],
                                              dtype=float)

    self_collision = None
    min_clearance_m = None
    if args.self_collision_margin_mm > 0:
        from self_collision import SelfCollisionModel
        self_collision = SelfCollisionModel(tool0_from_camera)
        min_clearance_m = args.self_collision_margin_mm / 1000.0
        print("自碰撞门禁已启用：相机 vs 机械臂网格，门限 %.0f mm" %
              args.self_collision_margin_mm)

    def annotate(records, slot, extra=None):
        for record in records:
            record["slot"] = slot
            record["singularity_floor"] = SINGULARITY_FLOOR
            record["singularity_fraction_of_baseline"] = (
                record["worst"]["min_singular"] / baseline_min_singular
                if baseline_min_singular else None)
            if extra:
                record.update(extra)
        return records

    def attempt_slot(entry, seed_q_in, seed_pose_in):
        """Try one slot: clearance heights first, then roll offsets.

        Returns (ok, order, end_q, end_pose, records, clearance_z, roll_offset).
        """
        target = pose_from_tcp_target(entry["target_tcp_pose_m_rad"])
        heights = [args.clearance_z] + [
            value for value in args.rescue_clearance
            if abs(value - args.clearance_z) > 1e-9]
        last_records = []
        for clearance in heights:
            ok, order, end_q, end_pose, records = try_slot(
                ik, seed_pose_in, seed_q_in, target, clearance,
                args.samples_per_segment, args.path_orders, baseline_min_singular,
                self_collision, min_clearance_m, joint6_range_rad)
            last_records = records
            if ok:
                return (True, order, end_q, end_pose,
                        annotate(records, entry["slot"], {"clearance_z_m": clearance}),
                        clearance, None)
        if tool0_from_camera is not None:
            target_matrix = to_matrix(target)
            for offset in RESCUE_ROLL_OFFSETS_DEG:
                candidate = to_se3(roll_about_optical_axis(
                    target_matrix, tool0_from_camera, offset))
                ok, order, end_q, end_pose, records = try_slot(
                    ik, seed_pose_in, seed_q_in, candidate, args.clearance_z,
                    args.samples_per_segment, args.path_orders, baseline_min_singular,
                    self_collision, min_clearance_m, joint6_range_rad)
                if ok:
                    return (True, order, end_q, end_pose,
                            annotate(records, entry["slot"],
                                     {"roll_rescue_offset_deg": offset,
                                      "clearance_z_m": args.clearance_z}),
                            args.clearance_z, offset)
        return (False, None, seed_q_in, seed_pose_in,
                annotate(last_records, entry["slot"]), None, None)

    seed_q = np.asarray(q_now, dtype=float)
    seed_pose = measured.copy()
    overall_ok = True
    print("%-4s %-10s %-15s %8s %9s %9s %9s %8s %s" %
          ("slot", "段", "段序", "IKpos_mm", "rot_mrad", "关节余量", "最小奇异",
           "法兰z", "结论"))

    def handle_success(entry, order, end_q, end_pose, records, clearance, offset):
        nonlocal seed_q, seed_pose
        document["segments"].extend(records)
        document["segments"].append({
            "slot": entry["slot"], "segment": "slot_summary", "feasible": True,
            "order": order, "clearance_z_m": clearance,
            "roll_rescue_offset_deg": offset})
        seed_q, seed_pose = end_q, end_pose
        if offset is not None:
            print("%-4d %-10s %-15s %8s %9s %9s %9s %8s %s" %
                  (entry["slot"], "RESCUE", "roll %+g deg" % offset, "-", "-", "-",
                   "-", "-", "PASS 绕光轴转 %+g° 后整链通过" % offset))
        elif abs(clearance - args.clearance_z) > 1e-9:
            print("%-4d %-10s %-15s %8s %9s %9s %9s %8s %s" %
                  (entry["slot"], "RESCUE", "h=%.2f m" % clearance, "-", "-", "-",
                   "-", "-", "PASS 巡航高度改为 %.2f m 后整链通过" % clearance))
        print("%-4d %-10s %-15s %8s %9s %9s %9s %8s %s" %
              (entry["slot"], "SUMMARY", order, "-", "-", "-", "-", "-", "PASS"))
        if not args.transit_home:
            return
        home_pose = pin.SE3(seed_pose.rotation.copy(),
                            np.array([measured.translation[0],
                                      measured.translation[1], clearance]))
        result = check_segment(ik, seed_pose, home_pose, seed_q,
                               args.samples_per_segment)
        worst = result["worst"]
        singular_ok = (worst["min_singular"] >= SINGULARITY_FLOOR and
                       worst["min_singular"] >=
                       MIN_SINGULAR_GATE_FRACTION * baseline_min_singular)
        transit_ok = not result["reasons"] and singular_ok
        document["segments"].append({
            "slot": entry["slot"], "segment": "transit_home",
            "start_tcp_m_rad": tcp_values(seed_pose),
            "end_tcp_m_rad": tcp_values(home_pose), "worst": worst,
            "singularity_gate_passed": bool(singular_ok),
            "passed": bool(transit_ok), "reasons": result["reasons"]})
        print_row(entry["slot"], "transit", "home", worst, transit_ok,
                  result["reasons"])
        if transit_ok:
            seed_q = np.asarray(result["end_q"], dtype=float)
            seed_pose = home_pose

    pending = list(entries)
    deferred = []
    while pending:
        entry = pending.pop(0)
        ok, order, end_q, end_pose, records, clearance, offset = attempt_slot(
            entry, seed_q, seed_pose)
        if ok:
            for record in records:
                print_row(entry["slot"], record["segment"], record["order"],
                          record["worst"], record["passed"], record["reasons"])
            handle_success(entry, order, end_q, end_pose, records, clearance, offset)
            continue
        if args.retry_deferred:
            deferred.append((entry, records, order, end_q, end_pose, clearance, offset))
            print("%-4d %-10s %-15s %8s %9s %9s %9s %8s %s" %
                  (entry["slot"], "DEFER", "retry later", "-", "-", "-", "-", "-",
                   "本轮不可达，排到队尾重试（前驱不同可能就通了）"))
            continue
        for record in records:
            print_row(entry["slot"], record["segment"], record["order"],
                      record["worst"], record["passed"], record["reasons"])
        overall_ok = False
        document["segments"].extend(records)
        document["segments"].append({
            "slot": entry["slot"], "segment": "slot_summary", "feasible": False,
            "orders_tried": list(args.path_orders),
            "clearance_heights_tried": [args.clearance_z] + list(args.rescue_clearance),
            "roll_offsets_tried": (list(RESCUE_ROLL_OFFSETS_DEG)
                                   if tool0_from_camera is not None else []),
            "note": "no clearance height, waypoint order or roll offset satisfied "
                    "the gates from this chain position"})
        print("%-4d %-10s %-15s %8s %9s %9s %9s %8s %s" %
              (entry["slot"], "SUMMARY", "-", "-", "-", "-", "-", "-",
               "FAIL 该位姿不可达（高度、段序、滚转备选都不过）"))

    passes = 0
    while deferred and passes < args.max_deferral_passes:
        passes += 1
        print()
        print("--- 队尾重试第 %d 轮，剩余 %d 个位姿：%s ---" %
              (passes, len(deferred), [item[0]["slot"] for item in deferred]))
        still = []
        for item in deferred:
            entry, records, order, end_q, end_pose, clearance, offset = item
            ok, order, end_q, end_pose, records, clearance, offset = attempt_slot(
                entry, seed_q, seed_pose)
            if ok:
                for record in records:
                    print_row(entry["slot"], record["segment"], record["order"],
                              record["worst"], record["passed"], record["reasons"])
                handle_success(entry, order, end_q, end_pose, records, clearance,
                               offset)
            else:
                still.append((entry, records, order, end_q, end_pose, clearance,
                              offset))
        if len(still) == len(deferred):
            break
        deferred = still
    for entry, records, order, end_q, end_pose, clearance, offset in deferred:
        overall_ok = False
        for record in records:
            print_row(entry["slot"], record["segment"], record["order"],
                      record["worst"], record["passed"], record["reasons"])
        document["segments"].extend(records)
        document["segments"].append({
            "slot": entry["slot"], "segment": "slot_summary", "feasible": False,
            "orders_tried": list(args.path_orders),
            "clearance_heights_tried": [args.clearance_z] + list(args.rescue_clearance),
            "roll_offsets_tried": (list(RESCUE_ROLL_OFFSETS_DEG)
                                   if tool0_from_camera is not None else []),
            "note": "unreachable after deferral retries as well"})
        print("%-4d %-10s %-15s %8s %9s %9s %9s %8s %s" %
              (entry["slot"], "SUMMARY", "-", "-", "-", "-", "-", "-",
               "FAIL 重试后仍不可达"))

    document["passed"] = bool(overall_ok)
    document["interpretation"] = (
        "Numerical feasibility only: reachability, joint limits, singularity "
        "proximity and flange clearance along densely sampled straight-line "
        "movel segments. Collision-freedom is NOT proven - there is no obstacle "
        "model. The lift/reorient/translate/descend structure plus a clearance "
        "height above every table object is the substitute, and it still needs "
        "human eyes on the workspace before execution.")
    output = args.output or os.path.join(root, "outputs", "vision",
                                         "scripted-move-offline-gate.json")
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "w", encoding="utf-8") as stream:
        json.dump(document, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    print()
    print("离线数值门禁: %s" % ("PASS" if overall_ok else "FAIL"))
    print("注意：本门禁不证明无碰撞（仓库无场景模型）；避障依靠"
          "抬升-平移-下降结构与高于台面所有物体的巡航高度，仍需人眼确认。")
    print("OUTPUT:", output)
    return 0 if overall_ok else 3


if __name__ == "__main__":
    sys.exit(main())
