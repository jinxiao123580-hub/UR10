#!/usr/bin/env python3
"""Guide the teach-pendant jogging for an eye-in-hand checkerboard sample set.

For a fixed board the relative camera pose is fully described by four numbers:
distance, tilt from the board normal, azimuth around that normal, and roll about
the optical axis.  This tool does three things with them:

* ``--plan``    emit the agreed shot list as target TCP poses, so the operator can
                jog until the PolyScope pose readout matches a row
* ``--coverage`` report which slots a collected dataset already fills
* ``--live``    read the current 30003 pose once and print the achieved relative
                pose plus the delta to the nearest still-empty slot

The board pose in the base frame is estimated from one reference sample combined
with a ``tool0_from_camera`` YAML.  That estimate is **guidance only** - it
inherits the reference calibration's error (a few millimetres and about a degree)
and must never be quoted as a measurement.  Everything that ends up in the final
calibration comes from the samples' own FK and PnP values.

Read-only with respect to the robot: no motion command is ever sent.
"""
import argparse
import datetime as dt
import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from collect_handeye_sample import URSampler  # noqa: E402

CAMERA_BOX_CM = (20.0, 20.0)
BRACKET_WIDTH_MM = 50.0
from solve_handeye_checkerboard import rt  # noqa: E402

SHELLS = [
    {"name": "A1", "distance_m": 0.35, "tilt_deg": 15.0, "roll_deg": 25.0},
    {"name": "A2", "distance_m": 0.45, "tilt_deg": 30.0, "roll_deg": -25.0},
    {"name": "A3", "distance_m": 0.58, "tilt_deg": 50.0, "roll_deg": None},
]
AZIMUTHS_DEG = [0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0]
ROLL_ALTERNATION_DEG = [45.0, -45.0]
REPEAT_SLOTS = [("A1", 0.0), ("A2", 180.0)]


def target_from_camera_from_spherical(distance, tilt_deg, azimuth_deg, roll_deg):
    """``target_from_camera`` (maps camera coords -> target coords) from the four
    shot-list parameters.

    The camera origin sits at ``-distance * u`` in the target frame, where ``u``
    points from the camera towards the board; the optical axis is ``u``.  Note
    the sign: OpenCV's target frame comes out of ``solvePnP`` with **+z pointing
    away from the camera** (out of the back of the board, because the object
    points are ordered like the detected image corners), so the camera is on the
    ``-z`` side and looks along ``+z``.  Getting this side wrong puts the whole
    shot list underneath the table.

    ``transform[:3, 3]`` is therefore the **camera origin in target coords** -
    which is exactly what ``target_from_camera`` means, and the opposite of what
    ``camera_from_target`` (``solvePnP``'s tvec = target origin in camera coords)
    carries.  Callers holding a pipeline ``camera_from_target`` must invert first.
    """
    tilt = np.radians(tilt_deg)
    azimuth = np.radians(azimuth_deg)
    direction = np.asarray([np.sin(tilt) * np.cos(azimuth),
                            np.sin(tilt) * np.sin(azimuth),
                            np.cos(tilt)])
    position = -distance * direction
    optical_axis = direction
    helper = np.asarray([0.0, 0.0, 1.0])
    if abs(float(optical_axis @ helper)) > 0.999:
        helper = np.asarray([1.0, 0.0, 0.0])
    x_axis = np.cross(helper, optical_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(optical_axis, x_axis)
    rotation = np.column_stack((x_axis, y_axis, optical_axis))
    roll = np.radians(roll_deg)
    about_axis = np.asarray([[np.cos(roll), -np.sin(roll), 0.0],
                             [np.sin(roll), np.cos(roll), 0.0],
                             [0.0, 0.0, 1.0]])
    transform = np.eye(4)
    transform[:3, :3] = rotation @ about_axis
    transform[:3, 3] = position
    return transform


def spherical_from_target_from_camera(transform):
    """Inverse of target_from_camera_from_spherical (round-trip checked offline)."""
    position = transform[:3, 3]
    distance = float(np.linalg.norm(position))
    direction = -position / distance
    tilt = float(np.degrees(np.arccos(np.clip(direction[2], -1.0, 1.0))))
    azimuth = float(np.degrees(np.arctan2(direction[1], direction[0])) % 360.0)
    no_roll = target_from_camera_from_spherical(distance, tilt, azimuth, 0.0)
    residual = no_roll[:3, :3].T @ transform[:3, :3]
    roll = float(np.degrees(np.arctan2(residual[1, 0], residual[0, 0])))
    return {"distance_m": distance, "tilt_deg": tilt, "azimuth_deg": azimuth,
            "roll_deg": roll}


def apply_shell_overrides(overrides):
    """``NAME:distance_m:tilt_deg[:azimuth_offset_deg]`` per shell, for testing
    alternative shell geometry when a pose turns out to be unreachable.  The
    azimuth *offset* rotates the whole 8-azimuth set, which preserves even
    coverage while moving every pose off a bad configuration."""
    shells = [dict(shell) for shell in SHELLS]
    by_name = {shell["name"]: shell for shell in shells}
    for token in overrides or []:
        parts = token.split(":")
        if len(parts) < 3:
            raise ValueError("bad --shell-override %r (want NAME:dist:tilt[:azoff])"
                             % token)
        name = parts[0]
        if name not in by_name:
            raise ValueError("unknown shell %r" % name)
        by_name[name]["distance_m"] = float(parts[1])
        by_name[name]["tilt_deg"] = float(parts[2])
        if len(parts) > 3:
            by_name[name]["azimuth_offset_deg"] = float(parts[3])
    return shells


def build_plan(shells=None, azimuth_span_deg=360.0):
    """8 azimuths evenly spread over ``azimuth_span_deg``.

    A full 360 deg orbit is what winds the wrist (and therefore the camera cable)
    by most of a turn; the hand-eye solve only needs the directions to be spread,
    not to close a full circle, so a half-orbit is a legitimate and much gentler
    design.
    """
    shells = shells or SHELLS
    span = float(azimuth_span_deg)
    azimuths = [index * span / len(AZIMUTHS_DEG) for index in range(len(AZIMUTHS_DEG))]
    slots = []
    index = 0
    for shell in shells:
        offset = float(shell.get("azimuth_offset_deg", 0.0))
        for azimuth_index, azimuth in enumerate(azimuths):
            azimuth = (azimuth + offset) % 360.0
            if shell["roll_deg"] is None:
                roll = ROLL_ALTERNATION_DEG[azimuth_index % 2]
            else:
                roll = shell["roll_deg"]
            index += 1
            slots.append({
                "slot": index, "shell": shell["name"], "role": "primary",
                "distance_m": shell["distance_m"], "tilt_deg": shell["tilt_deg"],
                "azimuth_deg": azimuth, "roll_deg": roll,
            })
    for shell_name, azimuth in REPEAT_SLOTS:
        shell = next(value for value in shells if value["name"] == shell_name)
        azimuth = (azimuth + float(shell.get("azimuth_offset_deg", 0.0))) % 360.0
        index += 1
        slots.append({
            "slot": index, "shell": shell_name, "role": "repeatability",
            "distance_m": shell["distance_m"], "tilt_deg": shell["tilt_deg"],
            "azimuth_deg": azimuth,
            "roll_deg": shell["roll_deg"] if shell["roll_deg"] is not None else 45.0,
        })
    return slots


ROLL_CANDIDATES_DEG = list(np.arange(0.0, 360.0, 2.0).tolist())


def ik_feasibility(base_from_tool0, seed_q, ik=None, self_collision=None,
                   min_clearance_m=None):
    """Can the controller reach this TCP pose from ``seed_q``?

    Returns ``(feasible, joint_margin_rad, reasons, solved_q, clearance_m)``;
    ``joint_margin_rad``/``solved_q``/``clearance_m`` are ``None`` when the
    corresponding checker was not supplied, and callers must unpack all five.
    This uses the nominal
    URDF IK (``ur_pose_ik.UR10IK``), the same model and base/URDF yaw mapping that
    ``solve_checked``'s ``joint_discontinuity`` reason is deliberately dropped: it
    measures the joint change from the seed, which matters for a joint-space jump
    but not here.  ``movel`` follows a Cartesian line and the controller does its
    own IK; whether the *chain* between consecutive samples is reachable is a path
    property, gated separately by ``check_move_plan.py``.  Keeping it here rejected
    every roll for a pose that the path gate proves is reachable.
    """
    if ik is None or seed_q is None:
        # Keep the five-value contract: the caller unpacks five, and returning a
        # short tuple here crashed every advisor run that had no IK checker.
        return True, None, [], None, None
    import pinocchio as pin
    from check_move_plan import BASE_FROM_URDF_ROOT
    from ur_pose_ik import LIMIT_MARGIN, LOWER, UPPER
    base_pose = pin.SE3(np.asarray(base_from_tool0[:3, :3], dtype=np.float64),
                        np.asarray(base_from_tool0[:3, 3], dtype=np.float64))
    target_root = BASE_FROM_URDF_ROOT.inverse() * base_pose
    solved, position_error, rotation_error, _jump, reasons, _ = \
        ik.solve_checked(target_root, seed_q)
    reasons = [value for value in reasons if value != "joint_discontinuity"]
    margin = float(min(np.min(solved - LOWER), np.min(UPPER - solved)))
    if margin < LIMIT_MARGIN:
        reasons = list(reasons) + ["joint_margin_below_2deg"]
    clearance = None
    if self_collision is not None:
        clearance, pair = self_collision.min_clearance(solved)
        if min_clearance_m is not None and clearance < min_clearance_m:
            reasons = list(reasons) + [
                "camera_clearance_%.0fmm_lt_%.0fmm(%s)" % (
                    clearance * 1000.0, min_clearance_m * 1000.0, pair)]
    return (not reasons), margin, reasons, solved, clearance


def choose_roll(distance, tilt_deg, azimuth_deg, base_from_target,
                tool0_from_camera, preferred_roll_deg, candidates=None,
                ik=None, seed_q=None, flange_floor_m=None,
                wrist_reference_q=None, self_collision=None,
                min_clearance_m=None):
    """Pick the free roll that keeps the flange highest above the board.

    The roll about the optical axis does not change which part of the board is
    seen, but it does change where ``tool0`` sits relative to the camera, so it
    directly controls the flange clearance over the table.  ``preferred_roll_deg``
    is the design value from the shot list and is used to break ties.

    With ``ik`` and ``seed_q`` supplied the roll is additionally filtered by
    controller reachability: clearance alone is not enough, because the roll that
    keeps the flange highest can drive the wrist into a joint limit at some
    azimuths (measured: shot-list slot 16 was unreachable that way).  Feasible
    rolls are ranked by flange height first, then by joint-limit margin.
    """
    candidates = candidates or ROLL_CANDIDATES_DEG
    seeds = list(seed_q) if isinstance(seed_q, (list, tuple)) else [seed_q]
    scored = []
    rejected = []
    for roll in candidates:
        target_from_camera = target_from_camera_from_spherical(
            distance, tilt_deg, azimuth_deg, roll)
        base_from_camera = base_from_target @ target_from_camera
        base_from_tool0 = base_from_camera @ np.linalg.inv(tool0_from_camera)
        height = float(base_from_tool0[2, 3] - base_from_target[2, 3])
        if flange_floor_m is not None and base_from_tool0[2, 3] < flange_floor_m:
            rejected.append((roll, "flange_below_floor"))
            continue
        feasible = False
        margin = None
        last_reasons = []
        solved = None
        clearance = None
        for seed in seeds:
            feasible, margin, last_reasons, solved, clearance = ik_feasibility(
                base_from_tool0, seed, ik, self_collision, min_clearance_m)
            if feasible:
                break
        if not feasible:
            rejected.append((roll, "ik:" + ",".join(last_reasons or ["unknown"])))
            continue
        roll_error = abs(((roll - preferred_roll_deg + 180.0) % 360.0) - 180.0) / 90.0
        if wrist_reference_q is not None and solved is not None:
            # Cable strain is the demonstrated failure mode: a constant roll forces
            # the wrist to wind up as the arm orbits the board (measured: -221 deg
            # net J6 winding and a J6 travel of 2171 deg, driving J6 into its limit).
            # Rank by wrist continuity first, clearance second.
            delta = np.asarray(solved, dtype=float) - np.asarray(wrist_reference_q,
                                                                dtype=float)
            wrist_cost = abs(float(delta[5]))
            scored.append((wrist_cost, -height, -(margin or 0.0), roll_error, roll,
                           height, margin, solved, clearance))
        else:
            scored.append((-height, -(margin if margin is not None else 0.0),
                           roll_error, 0.0, roll, height, margin, solved, clearance))
    if not scored:
        return None
    scored.sort(key=lambda item: item[:5])
    _, _, _, _, roll, height, margin, solved, clearance = scored[0]
    return {"roll_deg": roll, "flange_height_m": height,
            "joint_margin_rad": margin, "candidate_clearance_m": clearance,
            "candidates_scored": len(scored),
            "candidates_rejected": len(rejected),
            "q_rad": (None if solved is None else np.asarray(solved).tolist()),
            "rejection_reasons": sorted({reason for _, reason in rejected})}


def plan_with_tcp(slots, base_from_target, tool0_from_camera, optimize_roll=True,
                  min_tcp_z_m=None, ik=None, seed_q=None, roll_step_deg=5.0,
                  roll_objective="clearance", self_collision=None,
                  min_clearance_m=None, skip_infeasible=False):
    board_origin = base_from_target[:3, 3]
    board_normal = base_from_target[:3, :3] @ np.asarray([0.0, 0.0, 1.0])
    planned = []
    infeasible = []
    work = list(slots)
    if roll_objective == "wrist":
        # Orbit the board in a stable order so the running configuration is a
        # meaningful reference for wrist continuity.
        work.sort(key=lambda item: (item["shell"], item["azimuth_deg"]))
    # Absolute anchor, not a running one: "minimise the step from the previous
    # slot" is order-dependent, and the roll optimisation runs in a different
    # order from the gate's execution order, so the benefit cancels out (measured:
    # net J6 winding stayed at +222 deg).  Anchoring every slot to the *starting*
    # J6 is order-independent and directly bounds the winding.
    wrist_reference = (np.asarray(seed_q, dtype=float)
                       if (roll_objective == "wrist" and seed_q is not None
                           and not isinstance(seed_q, list)) else None)
    for slot in work:
        roll = slot["roll_deg"]
        chosen_roll = roll
        selection = None
        if optimize_roll:
            selection = choose_roll(
                slot["distance_m"], slot["tilt_deg"], slot["azimuth_deg"],
                base_from_target, tool0_from_camera, roll,
                candidates=(list(np.arange(0.0, 360.0, roll_step_deg).tolist())
                            if roll_step_deg else None),
                ik=ik, seed_q=seed_q, flange_floor_m=min_tcp_z_m,
                wrist_reference_q=wrist_reference, self_collision=self_collision,
                min_clearance_m=min_clearance_m)

            if selection is None:
                message = ("no roll satisfies the clearance and IK gates for slot %s "
                           "(distance %.2f m, tilt %.1f deg, azimuth %.1f deg)" %
                           (slot["slot"], slot["distance_m"], slot["tilt_deg"],
                            slot["azimuth_deg"]))
                if not skip_infeasible:
                    raise RuntimeError(message)
                print("  跳过不可行位姿： " + message)
                infeasible.append({"slot": slot["slot"], "shell": slot["shell"],
                                   "distance_m": slot["distance_m"],
                                   "tilt_deg": slot["tilt_deg"],
                                   "azimuth_deg": slot["azimuth_deg"],
                                   "reason": message})
                continue
            chosen_roll = selection["roll_deg"]
        target_from_camera = target_from_camera_from_spherical(
            slot["distance_m"], slot["tilt_deg"], slot["azimuth_deg"], chosen_roll)
        base_from_camera = base_from_target @ target_from_camera
        base_from_tool0 = base_from_camera @ np.linalg.inv(tool0_from_camera)
        rvec, _ = cv2.Rodrigues(base_from_tool0[:3, :3])
        entry = dict(slot)
        entry["design_roll_deg"] = roll
        entry["roll_deg"] = chosen_roll
        entry["target_tcp_pose_m_rad"] = (base_from_tool0[:3, 3].tolist() +
                                          rvec.reshape(3).tolist())
        entry["target_tcp_pose_m_deg"] = (base_from_tool0[:3, 3].tolist() +
                                          [float(np.degrees(x))
                                           for x in rvec.reshape(3)])
        entry["camera_origin_base_m"] = base_from_camera[:3, 3].tolist()
        entry["camera_height_above_board_m"] = float(
            base_from_camera[2, 3] - board_origin[2])
        entry["flange_height_above_board_m"] = float(
            base_from_tool0[2, 3] - board_origin[2])
        entry["viewing_side_check"] = float(
            -((base_from_camera[:3, 3] - board_origin) @ board_normal) /
            slot["distance_m"])
        if selection is not None:
            entry["roll_selection"] = {
                "scored": selection["candidates_scored"],
                "rejected": selection["candidates_rejected"],
                "rejection_reasons": selection["rejection_reasons"],
                "joint_margin_rad": selection["joint_margin_rad"],
                "camera_clearance_m": selection.get("candidate_clearance_m"),
                "q_rad": selection.get("q_rad"),
            }
        if min_tcp_z_m is not None:
            entry["below_clearance_floor"] = bool(base_from_tool0[2, 3] < min_tcp_z_m)
        planned.append(entry)
    return planned, infeasible


def sample_pose(sample):
    tcp = np.asarray(sample["robot"]["base_to_tool0_tcp_mean"], dtype=np.float64)
    base_from_tool0 = rt(tcp[3:], tcp[:3])
    target = sample["target_to_camera"]
    camera_from_target = rt(np.asarray(target["rvec_rad"], dtype=np.float64),
                            np.asarray(target["translation_m"], dtype=np.float64))
    return base_from_tool0, camera_from_target


def load_reference(path, root):
    """Return (base_from_target_estimate, reference_sample_id, source)."""
    full = path if os.path.isabs(path) else os.path.join(root, path)
    with open(full, encoding="utf-8") as stream:
        document = json.load(stream)
    if "samples" in document:
        samples = [s for s in document["samples"] if s.get("accepted")]
    else:
        samples = [document] if document.get("accepted") else []
    if not samples:
        raise RuntimeError("no accepted sample found in %s" % full)
    return samples[0], os.path.relpath(full, root)


def estimate_base_from_target(sample, tool0_from_camera):
    base_from_tool0, camera_from_target = sample_pose(sample)
    return base_from_tool0 @ tool0_from_camera @ camera_from_target


def match_slots(entries, slots, tolerance_deg=25.0, tolerance_m=0.12):
    """Assign observed entries to the nearest unfilled slot."""
    remaining = list(slots)
    assignments = []
    for entry in entries:
        best = None
        for slot in remaining:
            distance_error = abs(entry["distance_m"] - slot["distance_m"])
            tilt_error = abs(entry["tilt_deg"] - slot["tilt_deg"])
            azimuth_error = min(abs(entry["azimuth_deg"] - slot["azimuth_deg"]),
                                360.0 - abs(entry["azimuth_deg"] -
                                            slot["azimuth_deg"]))
            score = (distance_error / tolerance_m +
                     tilt_error / tolerance_deg +
                     azimuth_error / tolerance_deg)
            if best is None or score < best[0]:
                best = (score, slot, distance_error, tilt_error, azimuth_error)
        score, slot, distance_error, tilt_error, azimuth_error = best
        filled = (distance_error <= tolerance_m and tilt_error <= tolerance_deg
                  and azimuth_error <= tolerance_deg)
        assignments.append({
            "sample_id": entry["sample_id"], "slot": slot["slot"] if filled else None,
            "nearest_slot": slot["slot"], "filled": filled,
            "distance_error_m": distance_error, "tilt_error_deg": tilt_error,
            "azimuth_error_deg": azimuth_error,
            "observed": {key: entry[key] for key in
                         ("distance_m", "tilt_deg", "azimuth_deg", "roll_deg")},
        })
        if filled:
            remaining.remove(slot)
    return assignments, remaining


def coverage(dataset_path, base_from_target, slots, root):
    full = dataset_path if os.path.isabs(dataset_path) else os.path.join(root, dataset_path)
    with open(full, encoding="utf-8") as stream:
        document = json.load(stream)
    entries = []
    for sample in document["samples"]:
        if not sample.get("accepted"):
            continue
        _, camera_from_target = sample_pose(sample)
        base_from_camera = base_from_target @ camera_from_target
        camera_from_target_relative = np.linalg.inv(base_from_target) @ base_from_camera
        values = spherical_from_target_from_camera(
            np.linalg.inv(camera_from_target_relative))
        values["sample_id"] = sample["sample_id"]
        entries.append(values)
    assignments, remaining = match_slots(entries, slots)
    return {"dataset": os.path.relpath(full, root), "accepted": len(entries),
            "assignments": assignments,
            "empty_slots": [slot["slot"] for slot in remaining],
            "next_slot": (remaining[0] if remaining else None)}


def self_test():
    """Round-trip the spherical<->transform mapping and the coverage matcher."""
    checks = []

    def record(name, passed, detail):
        checks.append({"name": name, "passed": bool(passed), "detail": detail})

    worst = 0.0
    for distance in (0.30, 0.42, 0.61):
        for tilt in (5.0, 20.0, 45.0, 70.0):
            for azimuth in (0.0, 37.0, 180.0, 300.0):
                for roll in (-45.0, 0.0, 33.0):
                    transform = target_from_camera_from_spherical(
                        distance, tilt, azimuth, roll)
                    back = spherical_from_target_from_camera(transform)
                    worst = max(worst, abs(back["distance_m"] - distance) * 1000.0,
                                abs(back["tilt_deg"] - tilt) * 60.0,
                                abs(back["roll_deg"] - roll) * 60.0)
    record("spherical_roundtrip_under_1e-6", worst < 1e-6,
           {"worst_combined_error": worst})

    # The round trip above only proves self-consistency; it passes even when the
    # translation semantics are reversed.  These two checks pin the *convention*:
    # a real solvePnP ``camera_from_target`` puts the target origin in front of the
    # camera (+z) at the working distance, and the coverage/live path must recover
    # the planned parameters from exactly that form.
    board_in_front = True
    worst_recovery = 0.0
    for distance in (0.31, 0.47, 0.62):
        for tilt in (8.0, 25.0, 50.0):
            for azimuth in (0.0, 120.0, 250.0):
                for roll in (-40.0, 15.0):
                    target_from_camera = target_from_camera_from_spherical(
                        distance, tilt, azimuth, roll)
                    camera_from_target = np.linalg.inv(target_from_camera)
                    translation = camera_from_target[:3, 3]
                    if translation[2] <= 0.0 or abs(
                            float(np.linalg.norm(translation)) - distance) > 1e-9:
                        board_in_front = False
                    recovered = spherical_from_target_from_camera(
                        np.linalg.inv(camera_from_target))
                    worst_recovery = max(
                        worst_recovery,
                        abs(recovered["distance_m"] - distance) * 1000.0,
                        abs(recovered["tilt_deg"] - tilt) * 60.0,
                        abs(((recovered["azimuth_deg"] - azimuth + 180.0) % 360.0) -
                            180.0) * 60.0,
                        abs(recovered["roll_deg"] - roll) * 60.0)
    record("solvepnp_camera_from_target_puts_board_in_front", board_in_front,
           {"translation_z_positive_and_norm_equals_distance": board_in_front})
    record("pipeline_convention_recovery_under_1e-6", worst_recovery < 1e-6,
           {"worst_combined_error": worst_recovery})

    plan = build_plan()
    record("plan_slot_count_is_26", len(plan) == 26,
           {"slots": len(plan), "shells": sorted({s["shell"] for s in plan})})
    grid = {}
    for slot in plan:
        if slot["role"] != "primary":
            continue
        grid.setdefault(slot["shell"], set()).add(slot["azimuth_deg"])
    record("eight_azimuths_per_primary_shell",
           all(len(found) == 8 for found in grid.values()),
           {key: len(found) for key, found in grid.items()})
    tilts = {slot["shell"]: slot["tilt_deg"] for slot in plan}
    record("tilt_increases_across_shells",
           tilts["A1"] < tilts["A2"] < tilts["A3"], tilts)

    # matcher: a dataset built exactly on the plan must fill every slot
    rng = np.random.default_rng(7)
    tool0_from_camera = rt(np.asarray([0.1, -0.8, 0.5]),
                           np.asarray([0.04, 0.25, 0.2]))
    base_from_target = rt(np.asarray([0.2, 0.05, -0.4]),
                          np.asarray([0.62, -0.12, 0.2]))
    entries = []
    for slot in plan:
        values = dict(slot)
        values["sample_id"] = "plan-%02d" % slot["slot"]
        values["distance_m"] += float(rng.normal(0.0, 0.01))
        values["tilt_deg"] += float(rng.normal(0.0, 1.5))
        values["azimuth_deg"] = (values["azimuth_deg"] +
                                 float(rng.normal(0.0, 2.0))) % 360.0
        entries.append(values)
    assignments, remaining = match_slots(entries, plan)
    filled = sum(1 for value in assignments if value["filled"])
    record("matcher_fills_plan_from_its_own_slots", filled == len(plan) and not remaining,
           {"filled": filled, "slots": len(plan), "remaining": len(remaining)})

    report = {
        "schema_version": 1,
        "created_at": dt.datetime.now(dt.timezone.utc).astimezone().isoformat(),
        "kind": "pose_advisor_offline_self_test",
        "checks": checks,
        "passed": all(check["passed"] for check in checks),
        "passed_count": sum(check["passed"] for check in checks),
        "total_count": len(checks),
    }
    for check in checks:
        print("%-46s %s" % (check["name"], "PASS" if check["passed"] else "FAIL"))
    print("OFFLINE GATE: %s (%d/%d)" % ("PASS" if report["passed"] else "FAIL",
                                        report["passed_count"], report["total_count"]))
    return report


def print_table(planned, min_tcp_z_m=None):
    print("%-4s %-5s %6s %6s %8s %6s  %-34s %9s %9s %5s" %
          ("slot", "shell", "dist", "tilt", "azimuth", "roll",
           "target TCP x y z rx ry rz (deg)", "cam_z", "tcp_z", "risk"))
    for entry in planned:
        pose = entry["target_tcp_pose_m_deg"]
        risk = "LOW" if entry.get("below_clearance_floor") else ""
        print("%-4d %-5s %6.2f %6.1f %8.1f %6.1f  %7.4f %7.4f %7.4f %7.1f %7.1f %7.1f %9.4f %9.4f %5s" %
              (entry["slot"], entry["shell"], entry["distance_m"], entry["tilt_deg"],
               entry["azimuth_deg"], entry["roll_deg"],
               pose[0], pose[1], pose[2], pose[3], pose[4], pose[5],
               entry["camera_origin_base_m"][2], pose[2], risk))


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--reference", default=None,
                        help="accepted sample.json or dataset used to place the board "
                             "in the base frame (guidance only)")
    parser.add_argument("--calibration",
                        default="config/handeye_eye_in_hand_20260917.yaml",
                        help="guidance-only tool0_from_camera YAML")
    parser.add_argument("--coverage", default=None,
                        help="dataset to report achieved coverage for")
    parser.add_argument("--live", action="store_true",
                        help="read the current 30003 pose once and report the delta "
                             "to the nearest empty slot")
    parser.add_argument("--host", default="192.168.1.3")
    parser.add_argument("--min-tcp-z", type=float, default=0.15,
                        help="flag slots whose flange height falls below this value "
                             "(metres above the board plane); guidance only")
    parser.add_argument("--design-roll", action="store_true",
                        help="keep the shot list's design roll instead of choosing the "
                             "roll that maximises flange clearance. Measured offline "
                             "(check_handeye_plan_observability.py): design roll drops "
                             "the lowest flange to 0.079 m, below the 0.202 m that the "
                             "2026-09-17 set actually used, and buys only a modest "
                             "rotation-axis isotropy gain (0.107 vs 0.032)")
    parser.add_argument("--azimuth-span-deg", type=float, default=360.0,
                        help="total azimuth spread of the 8 primary slots (360 = full "
                             "orbit, 180 = half orbit); smaller spans wind the wrist "
                             "less, which is what protects the camera cable")
    parser.add_argument("--shell-override", action="append", default=None,
                        metavar="NAME:DIST:TILT[:AZOFF]",
                        help="override a shell's distance/tilt, and optionally rotate "
                             "its 8-azimuth set; repeatable")
    parser.add_argument("--roll-step-deg", type=float, default=5.0,
                        help="roll candidate spacing; 5 deg keeps the IK-filtered "
                             "scan to a few thousand solves instead of tens of "
                             "thousands (each solve is a bounded least-squares run)")
    parser.add_argument("--skip-infeasible", action="store_true",
                        help="record slots with no feasible roll instead of aborting, "
                             "so alternative shell geometry can be searched")
    parser.add_argument("--self-collision-margin-mm", type=float, default=40.0,
                        help="hard gate: minimum clearance between the camera "
                             "assembly (20x20 cm housing on a 30 cm mount) and the "
                             "robot's own collision meshes. On 2026-09-18 the camera "
                             "physically hit the forearm because the gate had no "
                             "self-collision model at all")
    parser.add_argument("--roll-objective", choices=("clearance", "wrist"),
                        default="clearance",
                        help="'clearance' maximises flange height (can wind the wrist "
                             "up as the arm orbits); 'wrist' keeps J6 continuous, which "
                             "is what protects the camera cable")
    parser.add_argument("--ik-check", action="store_true",
                        help="filter candidate rolls by controller reachability "
                             "(joint-limit margin) in addition to flange clearance; "
                             "needs a seed pose, read live from 30003 unless "
                             "--ik-seed-q is given")
    parser.add_argument("--ik-seed-q", default=None,
                        help="comma-separated 6 joint angles (rad) used as the IK seed")
    parser.add_argument("--output", default=None)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    if args.self_test:
        report = self_test()
        return 0 if report["passed"] else 3
    if not args.reference:
        parser.error("--reference is required (or use --self-test)")

    import yaml
    with open(os.path.join(root, args.calibration), encoding="utf-8") as stream:
        calibration = yaml.safe_load(stream)
    tool0_from_camera = np.eye(4)
    tool0_from_camera[:3, :3] = np.asarray(calibration["rotation_matrix"],
                                           dtype=np.float64)
    tool0_from_camera[:3, 3] = np.asarray(calibration["translation_m"],
                                          dtype=np.float64)

    sample, source = load_reference(args.reference, root)
    base_from_target = estimate_base_from_target(sample, tool0_from_camera)
    shells = apply_shell_overrides(args.shell_override)
    if args.shell_override:
        print("壳参数覆写：%s" % [(s["name"], s["distance_m"], s["tilt_deg"],
                                 s.get("azimuth_offset_deg", 0.0)) for s in shells])
    slots = build_plan(shells, args.azimuth_span_deg)

    ik = None
    seed_q = None
    self_collision = None
    if args.ik_check:
        if args.ik_seed_q:
            seed_q = np.asarray([float(x) for x in args.ik_seed_q.split(",")],
                                dtype=np.float64)
            if seed_q.size != 6:
                parser.error("--ik-seed-q needs 6 comma-separated joint angles (rad)")
            seed_source = "explicit --ik-seed-q"
        else:
            sampler = URSampler(args.host)
            sampler.start()
            try:
                sampler.wait_ready()
                seed_q = np.asarray(sampler.rows[-1]["q_rad"], dtype=np.float64)
            finally:
                sampler.stop()
            seed_source = "live 30003 q_actual at %s" % args.host
        from ur_pose_ik import UR10IK
        ik = UR10IK()
        from self_collision import SelfCollisionModel
        self_collision = SelfCollisionModel(tool0_from_camera)
        print("自碰撞模型已加载：相机 %.0fx%.0f cm 外壳 + %.0f mm 支架，门限 %.0f mm" % (
            CAMERA_BOX_CM[0], CAMERA_BOX_CM[1], BRACKET_WIDTH_MM,
            args.self_collision_margin_mm))
        print("IK 感知滚转选取已启用；种子 = %s" % seed_source)
        print("种子 q(rad): %s" % np.round(seed_q, 4).tolist())

    planned, infeasible = plan_with_tcp(slots, base_from_target, tool0_from_camera,
                            optimize_roll=not args.design_roll,
                            min_tcp_z_m=args.min_tcp_z, ik=ik, seed_q=seed_q,
                            roll_step_deg=args.roll_step_deg,
                            roll_objective=args.roll_objective,
                            self_collision=self_collision,
                            min_clearance_m=args.self_collision_margin_mm / 1000.0,
                            skip_infeasible=args.skip_infeasible)

    document = {
        "schema_version": 1,
        "created_at": dt.datetime.now(dt.timezone.utc).astimezone().isoformat(),
        "kind": "pose_advisor_plan",
        "infeasible_slots": infeasible,
        "self_collision_margin_mm": args.self_collision_margin_mm,
        "flange_floor_m": args.min_tcp_z,
        "guidance_only": True,
        "warning": "base_from_target is estimated from one reference sample plus the "
                   "guidance YAML; it inherits that calibration's error and must not "
                   "be quoted as a measurement",
        "reference_sample": sample["sample_id"],
        "reference_source": source,
        "guidance_calibration": args.calibration,
        "base_from_target_estimate": base_from_target.tolist(),
        "ik_filtered": bool(args.ik_check),
        "ik_seed_source": seed_source if args.ik_check else None,
        "ik_seed_q_rad": seed_q.tolist() if seed_q is not None else None,
        "flange_floor_m": args.min_tcp_z,
        "plan": planned,
    }

    print("引导基准（仅导航，非测量）：参考样本 %s + %s" % (sample["sample_id"],
                                                            args.calibration))
    print("棋盘格原点估计（base）: [%.4f, %.4f, %.4f] m" %
          tuple(base_from_target[:3, 3]))
    print()
    print_table(planned, args.min_tcp_z)
    low = [entry["slot"] for entry in planned if entry.get("below_clearance_floor")]
    if low:
        print()
        print("⚠ 低于法兰高度门限 %.2f m 的槽位（点动前先目视确认夹爪不会碰台面）：%s" %
              (args.min_tcp_z, low))
    spread = sorted({round(entry["roll_deg"], 1) for entry in planned})
    print("实际使用的滚转集合（自由度用于抬高法兰）：%s" % spread)

    if args.coverage:
        result = coverage(args.coverage, base_from_target, slots, root)
        document["coverage"] = result
        print()
        print("覆盖率报告：%s（接受 %d）" % (result["dataset"], result["accepted"]))
        for assignment in result["assignments"]:
            print("  %-28s -> slot %-4s %s  (Δd %.3f m / Δtilt %.1f° / Δazim %.1f°)" %
                  (assignment["sample_id"], assignment["nearest_slot"],
                   "filled" if assignment["filled"] else "no match",
                   assignment["distance_error_m"], assignment["tilt_error_deg"],
                   assignment["azimuth_error_deg"]))
        print("  空槽位：", result["empty_slots"])
        if result["next_slot"]:
            next_slot = result["next_slot"]
            print("  下一个建议位姿：slot %d  dist %.2f m  tilt %.1f°  azimuth %.1f°  roll %.1f°" %
                  (next_slot["slot"], next_slot["distance_m"], next_slot["tilt_deg"],
                   next_slot["azimuth_deg"], next_slot["roll_deg"]))

    if args.live:
        sampler = URSampler(args.host)
        sampler.start()
        try:
            sampler.wait_ready()
            tcp = np.asarray(sampler.rows[-1]["tcp_pose"], dtype=np.float64)
        finally:
            sampler.stop()
        base_from_tool0 = rt(tcp[3:], tcp[:3])
        camera_from_target = np.linalg.inv(tool0_from_camera) @ \
            np.linalg.inv(base_from_tool0) @ base_from_target
        live = spherical_from_target_from_camera(
            np.linalg.inv(camera_from_target))
        live["tcp_pose_m_rad"] = tcp.tolist()
        live["tcp_pose_m_deg"] = tcp[:3].tolist() + \
            [float(np.degrees(x)) for x in tcp[3:]]
        document["live"] = live
        print()
        print("当前相对位姿：dist %.3f m  tilt %.1f°  azimuth %.1f°  roll %.1f°" %
              (live["distance_m"], live["tilt_deg"], live["azimuth_deg"],
               live["roll_deg"]))
        empty = [slot for slot in slots
                 if slot["slot"] in (document.get("coverage", {}).get("empty_slots")
                                     or [slot["slot"] for slot in slots])]
        if empty:
            def cost(slot):
                azimuth_error = min(
                    abs(live["azimuth_deg"] - slot["azimuth_deg"]),
                    360.0 - abs(live["azimuth_deg"] - slot["azimuth_deg"]))
                return (abs(live["distance_m"] - slot["distance_m"]) * 5.0 +
                        abs(live["tilt_deg"] - slot["tilt_deg"]) / 10.0 +
                        azimuth_error / 20.0)
            target = min(empty, key=cost)
            print("最省力的空槽位：slot %d（dist %.2f / tilt %.1f° / azimuth %.1f° / roll %.1f°）" %
                  (target["slot"], target["distance_m"], target["tilt_deg"],
                   target["azimuth_deg"], target["roll_deg"]))
            entry = next(value for value in planned if value["slot"] == target["slot"])
            delta = np.asarray(entry["target_tcp_pose_m_rad"][:3]) - tcp[:3]
            print("  需要平移 Δ = [%+.4f, %+.4f, %+.4f] m" % tuple(delta))

    if args.output:
        path = args.output if os.path.isabs(args.output) else os.path.join(root, args.output)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as stream:
            json.dump(document, stream, indent=2, ensure_ascii=False)
            stream.write("\n")
        print("OUTPUT:", path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
