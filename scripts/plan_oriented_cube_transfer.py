#!/usr/bin/env python3
"""Read-only cube-oriented grasp and board-oriented transfer candidates.

The cube has four equivalent top-down jaw alignments.  A full top-face point
cloud, not a partial side fit, must establish its edge direction.  With a
measured starting joint/TCP state, each candidate is checked at intermediate
Cartesian poses for IK continuity, singularity and joint-limit margin.  This
script never sends motion and is not an execution permit.
"""
import argparse
import json
import math
import os
import sys

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def cube_yaw_from_track(track, max_spread_deg=3.0):
    if track.get("status") != "tracked_stable":
        raise ValueError("cube yaw requires all full top-face observations; partial fit is insufficient")
    observations = track.get("observations", [])
    multiview = track.get("mode") == "boardless_multi_view_yaw_consensus"
    if len(observations) < (4 if multiview else 3) or len(observations) != track.get("measured_captures"):
        raise ValueError("cube yaw requires at least three measured captures")
    angles = []
    high_coverage = 0
    by_view = {}
    centres = []
    camera_origins = {}
    for capture_index, row in enumerate(observations, 1):
        geometry = row.get("geometry") or {}
        face = geometry.get("top_face") or {}
        footprint = face.get("footprint") or {}
        gates = geometry.get("gates") or {}
        coverage = float(footprint.get("coverage", 0))
        if (row.get("status") != "measured" or
                not all(gates.get(key) for key in ("top_face_parallel", "coverage", "square")) or
                coverage < (0.55 if multiview else 0.85) or
                geometry.get("up_axis_target") != [0.0, 0.0, 1.0]):
            raise ValueError("capture %d: cube top-face coverage %.1f%%; at least %d%% and a reliable +Z square are required for grasp orientation" %
                             (capture_index, coverage * 100.0, 55 if multiview else 85))
        edges = footprint.get("edges_m") or []
        if len(edges) != 2 or any(abs(float(edge) - 0.05) > 0.004 for edge in edges):
            raise ValueError("cube top-face edges disagree with the measured 50 mm size")
        angle = float(footprint["angle_deg"])
        if not math.isfinite(angle):
            raise ValueError("non-finite cube edge angle")
        angles.append(angle)
        if multiview:
            high_coverage += coverage >= 0.85
            slot = row.get("view_slot")
            if slot is None:
                raise ValueError("multi-view yaw evidence lacks a view slot")
            by_view[slot] = by_view.get(slot, 0) + 1
            center = np.asarray(row.get("center_base_m"), dtype=float)
            camera = np.asarray(row.get("camera_origin_base_m"), dtype=float)
            if (center.shape != (3,) or camera.shape != (3,) or
                    not np.all(np.isfinite(center)) or not np.all(np.isfinite(camera))):
                raise ValueError("multi-view yaw evidence lacks finite centres or camera origins")
            centres.append(center)
            camera_origins.setdefault(slot, camera)
    if multiview:
        valid_views = [slot for slot, count in by_view.items() if count >= 2]
        if high_coverage < 1 or len(valid_views) < 2:
            raise ValueError("multi-view yaw requires one >=85%% top face and two captures in each of two views")
        if np.linalg.norm(camera_origins[valid_views[0]] - camera_origins[valid_views[1]]) < 0.05:
            raise ValueError("multi-view cameras are less than 50 mm apart")
        mean_center = np.mean(centres, axis=0)
        anchor = np.asarray(track.get("anchor_center_base_m"), dtype=float)
        if anchor.shape != (3,) or not np.all(np.isfinite(anchor)):
            raise ValueError("multi-view yaw lacks a verified single-view centre")
        if (max(np.linalg.norm(center[:2] - mean_center[:2]) for center in centres) > 0.003 or
                max(abs(center[2] - mean_center[2]) for center in centres) > 0.004 or
                np.linalg.norm(mean_center[:2] - anchor[:2]) > 0.003 or
                abs(mean_center[2] - anchor[2]) > 0.005):
            raise ValueError("multi-view cube centres disagree beyond 3 mm XY / 4-5 mm Z")
    # The physical square is unchanged by 90-degree rotations.  Circular mean
    # on the fourfold quotient avoids a false 45-degree result near 0/90 deg.
    phases = np.exp(4j * np.deg2rad(angles))
    yaw = math.degrees(np.angle(np.mean(phases))) / 4.0
    spread = max(abs(((angle - yaw + 45.0) % 90.0) - 45.0) for angle in angles)
    if spread > max_spread_deg:
        raise ValueError("cube yaw spread %.2f deg exceeds %.2f deg" %
                         (spread, max_spread_deg))
    return yaw, spread, angles


def pose_from_rotation(rotation, grasp_center, tool_offset):
    rvec, _ = cv2.Rodrigues(rotation)
    return np.r_[grasp_center - rotation @ tool_offset, rvec.reshape(3)].tolist()


def candidate_poses(plan, yaw_deg):
    pick_center = np.asarray(plan["gripper_center_pick_base_m"], dtype=float)
    place = np.asarray(plan["place"], dtype=float)
    place_rotation, _ = cv2.Rodrigues(place[3:])
    previous_pick = np.asarray(plan["pick"], dtype=float)
    previous_rotation, _ = cv2.Rodrigues(previous_pick[3:])
    offset = previous_rotation.T @ (pick_center - previous_pick[:3])
    down = -np.asarray(plan["board_normal_base"], dtype=float)
    down /= np.linalg.norm(down)
    if down[2] > -math.cos(math.radians(12.0)):
        raise ValueError("board normal is not near base +Z")
    candidates = []
    for quarter in range(4):
        theta = math.radians(yaw_deg + quarter * 90.0)
        x = np.array([math.cos(theta), math.sin(theta), 0.0])
        x -= down * np.dot(x, down)
        x /= np.linalg.norm(x)
        rotation = np.column_stack((x, np.cross(down, x), down))
        pick = pose_from_rotation(rotation, pick_center, offset)
        candidates.append({"quarter_turn": quarter,
                           "cube_aligned_yaw_deg": yaw_deg + quarter * 90.0,
                           "pick": pick, "place": place.tolist(),
                           "gripper_center_pick_base_m": pick_center.tolist(),
                           "tool0_to_gripper_center_m": offset.tolist(),
                           "relative_pick_to_place_rotation_deg": float(np.degrees(
                               np.linalg.norm(cv2.Rodrigues(rotation.T @ place_rotation)[0])))})
    return candidates


def gate_candidate(candidate, start_q, start_tcp, height_m, travel_height_m=None,
                   via_x_offset_m=0.0, samples=12,
                   min_margin_deg=5.0, collision_model=None,
                   collision_margin_m=0.020):
    import pinocchio as pin
    from check_move_plan import BASE_FROM_URDF_ROOT, jacobian_singular_values
    from ur_pose_ik import LOWER, UPPER, UR10IK

    ik = UR10IK()
    q = np.asarray(start_q, dtype=float)
    measured = pin.SE3(pin.exp3(np.asarray(start_tcp[3:])), np.asarray(start_tcp[:3]))
    nominal = BASE_FROM_URDF_ROOT * ik.pose(q)
    fk_error = pin.log6(nominal.inverse() * measured).vector
    if np.linalg.norm(fk_error[:3]) > 0.010 or np.linalg.norm(fk_error[3:]) > 0.020:
        return {"passed": False, "reason": "controller TCP and nominal FK disagree",
                "fk_error_m_rad": fk_error.tolist()}
    pick = np.asarray(candidate["pick"], dtype=float)
    place = np.asarray(candidate["place"], dtype=float)
    travel_height_m = height_m if travel_height_m is None else float(travel_height_m)
    if not height_m <= travel_height_m <= 0.25:
        raise ValueError("travel height must be between grasp hover height and 250 mm")
    pick_up = pick.copy(); pick_up[2] += height_m
    pick_travel = pick.copy(); pick_travel[2] += travel_height_m
    place_travel = place.copy(); place_travel[2] += travel_height_m
    place_up = place.copy(); place_up[2] += height_m
    # Rotate only after lift, about the held cube's grasp centre rather than
    # about tool0.  The measured grasp-centre offset is ~22 cm, so a bare TCP
    # rotation would otherwise sweep the cube through a large unintended arc.
    pick_rotation = pin.exp3(pick[3:])
    place_rotation = pin.exp3(place[3:])
    offset = np.asarray(candidate["tool0_to_gripper_center_m"], dtype=float)
    grasp_center_up = np.asarray(candidate["gripper_center_pick_base_m"], dtype=float) + \
        np.array([0.0, 0.0, travel_height_m])
    rotation_delta = pin.log3(pick_rotation.T @ place_rotation)
    rotation_steps = max(1, int(math.ceil(np.linalg.norm(rotation_delta) / math.radians(5.0))))
    waypoints = [("pick_hover", pick_up), ("pick_descent", pick),
                 ("lift_with_cube", pick_up)]
    if travel_height_m > height_m + 1e-9:
        waypoints.append(("lift_to_transfer_height", pick_travel))
    for index in range(1, rotation_steps + 1):
        rotation = pick_rotation @ pin.exp3(rotation_delta * index / rotation_steps)
        rvec = pin.log3(rotation)
        tool = grasp_center_up - rotation @ offset
        waypoints.append(("rotate_above_cube_%02d" % index, np.r_[tool, rvec]))
    if abs(via_x_offset_m) > 1e-9:
        # Rectangular detour in the horizontal clearance plane.  This is only
        # a self-collision candidate; the workcell/fixture is not modelled.
        via_near_cube = np.asarray(waypoints[-1][1]).copy()
        via_near_cube[0] += via_x_offset_m
        via_near_board = place_travel.copy()
        via_near_board[0] += via_x_offset_m
        waypoints.extend((("detour_leave_cube", via_near_cube),
                          ("detour_cross", via_near_board)))
    waypoints.append(("transfer_above_board", place_travel))
    if travel_height_m > height_m + 1e-9:
        waypoints.append(("descend_to_place_hover", place_up))
    start = measured
    smallest_margin = float("inf")
    smallest_singular = float("inf")
    largest_step = 0.0
    least_clearance = float("inf")
    samples_checked = 0
    validated_waypoints = []
    for name, values in waypoints:
        target = pin.SE3(pin.exp3(values[3:]), values[:3])
        twist = pin.log6(start.inverse() * target).vector
        for index in range(1, samples + 1):
            intermediate = start * pin.exp6(twist * (index / samples))
            solved, pe, re, step, failures, _ = ik.solve_checked(
                BASE_FROM_URDF_ROOT.inverse() * intermediate, q)
            margin = float(min(np.min(solved - LOWER), np.min(UPPER - solved)))
            singular = jacobian_singular_values(ik, solved)[0]
            smallest_margin = min(smallest_margin, margin)
            smallest_singular = min(smallest_singular, singular)
            largest_step = max(largest_step, step)
            samples_checked += 1
            if failures or margin < math.radians(min_margin_deg) or singular < 0.02:
                return {"passed": False, "reason": "%s sample %d: %s; margin %.1f deg; singular %.4f" %
                        (name, index, ",".join(failures), math.degrees(margin), singular),
                        "samples_checked": samples_checked}
            if collision_model is not None:
                clearance, pair = collision_model.min_clearance(solved)
                least_clearance = min(least_clearance, clearance)
                if clearance < collision_margin_m:
                    return {"passed": False, "reason": "%s sample %d: %s clearance %.1f mm < %.1f mm" %
                            (name, index, pair, clearance * 1000, collision_margin_m * 1000),
                            "samples_checked": samples_checked}
            q = solved
        validated_waypoints.append({"name": name, "tcp_m_rad": values.tolist(),
                                    "end_q_rad": q.tolist()})
        start = target
    return {"passed": True, "min_joint_margin_deg": math.degrees(smallest_margin),
            "min_singular": smallest_singular, "max_ik_step_deg": math.degrees(largest_step),
            "min_self_clearance_mm": None if collision_model is None else least_clearance * 1000,
            "travel_height_m": travel_height_m,
            "via_x_offset_m": via_x_offset_m,
            "samples_checked": samples_checked,
            "waypoints": validated_waypoints}


def gate_linear_segment(start_q, start_tcp, target_tcp, collision_model,
                        samples=8, min_margin_deg=5.0, collision_margin_m=0.020):
    """Fail-closed IK/collision check for one short live Cartesian segment."""
    import pinocchio as pin
    from check_move_plan import BASE_FROM_URDF_ROOT, jacobian_singular_values
    from ur_pose_ik import LOWER, UPPER, UR10IK

    ik = UR10IK()
    q = np.asarray(start_q, dtype=float)
    start = pin.SE3(pin.exp3(np.asarray(start_tcp[3:])), np.asarray(start_tcp[:3]))
    end = pin.SE3(pin.exp3(np.asarray(target_tcp[3:])), np.asarray(target_tcp[:3]))
    error = pin.log6((BASE_FROM_URDF_ROOT * ik.pose(q)).inverse() * start).vector
    if np.linalg.norm(error[:3]) > 0.010 or np.linalg.norm(error[3:]) > 0.020:
        raise ValueError("controller TCP/FK mismatch before contact descent")
    twist = pin.log6(start.inverse() * end).vector
    for index in range(1, samples + 1):
        pose = start * pin.exp6(twist * (index / samples))
        solved, _pe, _re, _step, reasons, _ = ik.solve_checked(
            BASE_FROM_URDF_ROOT.inverse() * pose, q)
        margin = float(min(np.min(solved - LOWER), np.min(UPPER - solved)))
        singular = jacobian_singular_values(ik, solved)[0]
        if reasons or margin < math.radians(min_margin_deg) or singular < 0.02:
            raise ValueError("contact segment sample %d failed IK/joint gate: %s" %
                             (index, reasons))
        clearance, pair = collision_model.min_clearance(solved)
        if clearance < collision_margin_m:
            raise ValueError("contact segment sample %d %s clearance %.1f mm < %.1f mm" %
                             (index, pair, clearance * 1000, collision_margin_m * 1000))
        q = solved
    return q.tolist()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", required=True)
    parser.add_argument("--track", required=True)
    parser.add_argument("--start-q", help="six measured joint angles in radians, comma-separated")
    parser.add_argument("--start-tcp", help="six matching measured base TCP values, comma-separated")
    parser.add_argument("--live-state", action="store_true",
                        help="read q/TCP from UR 30013; never opens a motion port")
    parser.add_argument("--self-collision-margin-mm", type=float, default=20.0,
                        help="camera/arm mesh clearance gate; 0 disables it")
    parser.add_argument("--travel-height-mm", type=float, nargs="+",
                        default=[80, 100, 120, 140, 160, 180, 200],
                        help="safe transfer heights above grasp/place, searched without motion")
    parser.add_argument("--via-x-offset-mm", type=float, nargs="+",
                        default=[0, -50, 50, -100, 100],
                        help="horizontal x detour offsets; offline self-collision search only")
    parser.add_argument("--dense-samples-per-segment", type=int, default=36,
                        help="repeat the selected path with denser IK/collision samples")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    def load(path):
        with open(path if os.path.isabs(path) else os.path.join(ROOT, path), encoding="utf-8") as stream:
            return json.load(stream)
    plan, track = load(args.plan), load(args.track)
    if plan.get("kind") != "auto_cube_pick_place_plan":
        raise SystemExit("expected auto cube pick/place plan")
    try:
        yaw, spread, angles = cube_yaw_from_track(track)
    except ValueError as exc:
        raise SystemExit("cube orientation rejected: %s; no motion sent by this planner" % exc)
    candidates = candidate_poses(plan, yaw)
    if args.live_state and (args.start_q or args.start_tcp):
        parser.error("--live-state cannot be combined with an explicit start state")
    if bool(args.start_q) != bool(args.start_tcp):
        parser.error("--start-q and --start-tcp must be provided together")
    if args.live_state:
        from check_move_plan import read_state
        q, tcp = read_state("192.168.1.3", 30013)
        state_source = "live UR 30013 read-only"
    elif args.start_q:
        q = np.asarray([float(x) for x in args.start_q.split(",")])
        tcp = np.asarray([float(x) for x in args.start_tcp.split(",")])
        state_source = "explicit measured values"
    else:
        q = tcp = None
        state_source = "not supplied: orientation candidates only"
    if q is not None:
        if q.shape != (6,) or tcp.shape != (6,) or not np.all(np.isfinite(np.r_[q, tcp])):
            parser.error("start state must contain two finite six-element vectors")
        if not 0.0 <= args.self_collision_margin_mm <= 100.0:
            parser.error("self-collision margin must be 0..100 mm")
        collision_model = None
        if args.self_collision_margin_mm:
            from self_collision import SelfCollisionModel
            collision_model = SelfCollisionModel()
        base_height = float(plan["height"])
        travel_heights = sorted(set(max(base_height, value / 1000.0)
                                    for value in args.travel_height_mm))
        if not travel_heights or travel_heights[-1] > 0.25 or min(args.travel_height_mm) < 0:
            parser.error("travel heights must be nonnegative and at most 250 mm")
        if any(abs(value) > 150 for value in args.via_x_offset_mm):
            parser.error("detour x offset must stay within 150 mm")
        if not 24 <= args.dense_samples_per_segment <= 100:
            parser.error("dense sample count must be 24..100")
        viable_routes = []
        for candidate in candidates:
            print("离线路径检查：物块等价朝向 %d/4（%d 个高度 × %d 个绕行选项）" %
                  (candidate["quarter_turn"] + 1, len(travel_heights), len(args.via_x_offset_mm)),
                  flush=True)
            trials = []
            for travel_height in travel_heights:
                for offset_mm in args.via_x_offset_mm:
                    gate = gate_candidate(candidate, q, tcp, base_height,
                                          travel_height_m=travel_height,
                                          via_x_offset_m=offset_mm / 1000.0,
                                          collision_model=collision_model,
                                          collision_margin_m=args.self_collision_margin_mm / 1000.0)
                    trials.append({"travel_height_m": travel_height,
                                   "via_x_offset_m": offset_mm / 1000.0,
                                   "gate": gate})
            candidate["route_trials"] = trials
            passed = [trial for trial in trials if trial["gate"]["passed"]]
            candidate["ik_path_gate"] = passed[0]["gate"] if passed else trials[0]["gate"]
            viable_routes.extend((candidate, trial) for trial in passed)
            print("  该朝向通过初筛 %d 条路径" % len(passed), flush=True)
        # The current state's margin is common to all branches.  Round tiny
        # optimizer noise away before preferring the smoother, shorter route.
        viable_routes.sort(key=lambda item: (
            round(item[1]["gate"]["min_joint_margin_deg"], 1),
            round(item[1]["gate"]["min_self_clearance_mm"] or 0, 1),
            round(item[1]["gate"]["min_singular"], 3),
            -item[1]["gate"]["max_ik_step_deg"],
            -abs(item[1]["via_x_offset_m"]),
            -item[1]["travel_height_m"]), reverse=True)
        selected = selected_route = None
        for candidate, trial in viable_routes:
            dense = gate_candidate(
                candidate, q, tcp, base_height,
                travel_height_m=trial["travel_height_m"],
                via_x_offset_m=trial["via_x_offset_m"],
                samples=args.dense_samples_per_segment,
                collision_model=collision_model,
                collision_margin_m=args.self_collision_margin_mm / 1000.0)
            trial["dense_gate"] = dense
            if dense["passed"]:
                selected = candidate
                selected_route = dense
                candidate["ik_path_gate"] = dense
                break
    else:
        selected = selected_route = None
    result = {"schema_version": 1, "kind": "cube_oriented_transfer_draft",
              "motion_sent": False, "motion_authorization": "none: offline IK is not a collision or execution permit",
              "cube_yaw_modulo_90_deg": yaw, "cube_yaw_spread_deg": spread,
              "measured_yaws_deg": angles, "candidates": candidates,
              "self_collision_margin_mm": args.self_collision_margin_mm,
              "travel_height_candidates_mm": args.travel_height_mm,
              "via_x_offset_candidates_mm": args.via_x_offset_mm,
              "start_state": {"source": state_source,
                              "q_rad": None if q is None else q.tolist(),
                              "tcp_m_rad": None if tcp is None else tcp.tolist()},
              "selected_quarter_turn": None if selected is None else selected["quarter_turn"],
              "selected_route": selected_route,
              "dense_samples_per_segment": args.dense_samples_per_segment,
              "pick": None if selected is None else selected["pick"],
              "place": None if selected is None else selected["place"],
              "note": "UR10 has no continuous null space for a fully fixed 6D TCP pose; four cube-symmetric orientations are ranked instead."}
    output = args.output if os.path.isabs(args.output) else os.path.join(ROOT, args.output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "w", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print("cube yaw modulo 90: %.2f deg (spread %.2f deg)" % (yaw, spread))
    print("offline-gated draft candidate:", result["selected_quarter_turn"])
    print("READ-ONLY draft:", output)
    return 0 if selected is not None or q is None else 2


if __name__ == "__main__":
    raise SystemExit(main())
