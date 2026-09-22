#!/usr/bin/env python3
"""Generate a non-executable left-fingertip hover plan above a measured cube.

This is intentionally not a grasp planner: a single pivot-calibrated fingertip
has a position but no independently measured jaw centre, opposing fingertip,
or closing direction.  The resulting plan is only suitable for offline IK and
visual alignment checks.
"""
import argparse
import json
import os

import numpy as np
import yaml


def rotation_from_rotvec(rotvec):
    vector = np.asarray(rotvec, dtype=float)
    angle = np.linalg.norm(vector)
    if angle < 1e-12:
        return np.eye(3)
    axis = vector / angle
    cross = np.array([[0., -axis[2], axis[1]], [axis[2], 0., -axis[0]],
                      [-axis[1], axis[0], 0.]])
    return np.eye(3) + np.sin(angle) * cross + (1. - np.cos(angle)) * cross @ cross


def transform(rotation, translation):
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def rotvec_from_rotation(rotation):
    """Principal rotation vector for a proper 3x3 rotation matrix."""
    cosine = np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)
    angle = float(np.arccos(cosine))
    if angle < 1e-10:
        return np.zeros(3)
    if np.pi - angle < 1e-6:
        # Near pi the usual antisymmetric expression is numerically zero.
        # Recover the eigen-axis from the diagonal so a strictly vertical-down
        # pose serialises correctly instead of silently becoming identity.
        diagonal = np.maximum((np.diag(rotation) + 1.0) / 2.0, 0.0)
        axis = np.sqrt(diagonal)
        pivot = int(np.argmax(axis))
        if axis[pivot] > 1e-8:
            if pivot == 0:
                axis[1] = rotation[0, 1] / (2.0 * axis[0])
                axis[2] = rotation[0, 2] / (2.0 * axis[0])
            elif pivot == 1:
                axis[0] = rotation[0, 1] / (2.0 * axis[1])
                axis[2] = rotation[1, 2] / (2.0 * axis[1])
            else:
                axis[0] = rotation[0, 2] / (2.0 * axis[2])
                axis[1] = rotation[1, 2] / (2.0 * axis[2])
        axis /= max(np.linalg.norm(axis), 1e-12)
        return axis * angle
    vector = np.array([rotation[2, 1] - rotation[1, 2],
                       rotation[0, 2] - rotation[2, 0],
                       rotation[1, 0] - rotation[0, 1]])
    return vector * angle / (2.0 * np.sin(angle))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cube", default="outputs/vision/cube-measurement-20260921-provisional-r2.json")
    parser.add_argument("--tcp", default="outputs/tcp_calibration/fingertip-20260921.json")
    parser.add_argument("--grasp-center", default=None,
                        help="manual position-only grasp-centre TCP YAML; when supplied, plan that frame")
    parser.add_argument("--calibration", default="config/handeye_eye_in_hand_20260921.yaml")
    parser.add_argument("--standoff-mm", type=float, default=100.0)
    parser.add_argument("--vertical-down", action="store_true",
                        help="align tool0 +Z with the downward table normal for top-down grasping")
    parser.add_argument("--roll-deg", type=float, default=0.0,
                        help="rotate the tool about its own +Z after choosing the approach direction")
    parser.add_argument("--camera-look-at-top", action="store_true",
                        help="choose the free tool roll that best aligns the camera optical "
                             "axis with the cube top; intended for a read-only observation pose")
    parser.add_argument("--camera-roll-step-deg", type=float, default=2.0,
                        help="roll sampling resolution used by --camera-look-at-top")
    parser.add_argument("--align-live-vertical", action="store_true",
                        help="for a tracked live pose, project tool +Z exactly vertical down while "
                             "preserving its roll seed; requires a fresh offline gate")
    parser.add_argument("--max-live-down-error-deg", type=float, default=5.0,
                        help="accepted live tool-axis deviation from vertical down; preserves the "
                             "operator-selected camera/gripper roll within this tolerance")
    parser.add_argument("--output", default="outputs/vision/fingertip-hover-plan-20260922.json")
    args = parser.parse_args()
    if not 30.0 <= args.standoff_mm <= 200.0:
        parser.error("--standoff-mm must be within 30..200")
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    def load(path):
        path = path if os.path.isabs(path) else os.path.join(root, path)
        with open(path, encoding="utf-8") as stream:
            return json.load(stream), path

    cube, cube_path = load(args.cube)
    tcp, tcp_path = load(args.tcp)
    calibration_path = args.calibration if os.path.isabs(args.calibration) else os.path.join(root, args.calibration)
    with open(calibration_path, encoding="utf-8") as stream:
        calibration = yaml.safe_load(stream)
    tracked = cube.get("status") == "tracked_stable"
    if not tracked and (cube.get("status") != "measured" or
                        not cube.get("size_prior", {}).get("matches_nominal_size")):
        raise SystemExit("cube measurement did not pass geometry gates")
    result = tcp.get("result") or {}
    if not result.get("passed_numeric_gate"):
        raise SystemExit("fingertip TCP did not pass its independent numeric gate")

    tool_from_camera = transform(np.asarray(calibration["rotation_matrix"], dtype=float),
                                 np.asarray(calibration["translation_m"], dtype=float))
    if tracked:
        # The boardless tracker operates in a local frame parallel to the base
        # table.  It independently gated every cloud's 50 mm footprint; use
        # the stable mean centre and median measured height, not the old board
        # observation.  Vertical-down remains a manual grasp requirement.
        valid = [row["geometry"] for row in cube.get("observations", [])
                 if row.get("status") == "measured" and
                 row.get("geometry", {}).get("size_prior", {}).get("matches_nominal_size")]
        if len(valid) != cube.get("required_captures"):
            raise SystemExit("tracked cube lacks all independently measured observations")
        up_base = np.array([0.0, 0.0, 1.0])
        center = np.asarray(cube["center_base_m"], dtype=float)
        height = float(np.median([row["measured_height_m"] for row in valid]))
        # Preserve the actually successful observation wrist orientation as
        # the roll seed.  A generic identity seed is not equivalent: on this
        # robot it selects a different J6 branch and can cross its operator
        # imposed range during the reorientation.
        observed_tcp = cube["observations"][0].get("tcp_base_tool0")
        if observed_tcp is None or len(observed_tcp) != 6:
            raise SystemExit("tracked cube lacks the synchronized tool0 pose")
        rotation = rotation_from_rotvec(observed_tcp[3:])
        orientation_source = "boardless live cube track; synchronized observation wrist roll"
    else:
        capture_pose = cube["robot_motion"]["base_to_tool0_tcp_mean"]
        base_from_tool = transform(rotation_from_rotvec(capture_pose[3:]), capture_pose[:3])
        base_from_target = base_from_tool @ tool_from_camera @ np.asarray(cube["camera_from_target"], dtype=float)
        up_base = base_from_target[:3, :3] @ np.asarray(cube["up_axis_target"], dtype=float)
        up_base /= np.linalg.norm(up_base)
        center = np.asarray(cube["center_base_m"], dtype=float)
        height = float(cube["measured_height_m"])
        rotation = base_from_tool[:3, :3]
        orientation_source = "cube capture pose (visual alignment only)"
    top_center = center + up_base * height / 2.0
    fingertip_hover = top_center + up_base * (args.standoff_mm / 1000.0)
    frame = "left_fingertip_tcp"
    tcp_source = os.path.relpath(tcp_path, root)
    kind = "left_fingertip_hover_preview"
    if args.grasp_center:
        grasp_path = (args.grasp_center if os.path.isabs(args.grasp_center)
                      else os.path.join(root, args.grasp_center))
        with open(grasp_path, encoding="utf-8") as stream:
            grasp = yaml.safe_load(stream) or {}
        if grasp.get("parent_frame") != "tool0" or grasp.get("orientation") != "inherits_tool0":
            raise SystemExit("manual grasp centre must be a tool0 position-only measurement")
        offset = np.asarray(grasp.get("translation_m"), dtype=float)
        frame, tcp_source, kind = (str(grasp.get("frame", "gripper_grasp_center")),
                                   os.path.relpath(grasp_path, root),
                                   "gripper_grasp_center_hover_preview")
    else:
        offset = np.asarray(result["tool0_to_fingertip_m"], dtype=float)
    if args.vertical_down and tracked and not args.align_live_vertical:
        # The observation just proved that this exact wrist roll sees the full
        # top face. Keep it for the live re-anchor rather than selecting a
        # mathematically equivalent 180-degree representation on a new J6
        # branch. It must nevertheless already satisfy the manual down rule.
        down_error_deg = float(np.degrees(np.arccos(np.clip(
            (-up_base) @ rotation[:, 2], -1.0, 1.0))))
        if not 0.0 < args.max_live_down_error_deg <= 15.0:
            raise SystemExit("--max-live-down-error-deg must be within (0, 15]")
        if down_error_deg > args.max_live_down_error_deg:
            raise SystemExit("live observation tool +Z is %.2f deg from vertical down" % down_error_deg)
        orientation_source += "; preserved live camera-view roll (down error %.3f deg)" % down_error_deg
    elif args.vertical_down:
        # The operator confirmed a vertical top-down grasp. tool0 +Z is the
        # physical outward gripper-centre chain, so it must point down.  Roll
        # about that axis is chosen near the observed wrist direction; it does
        # not alter the centre or vertical approach.
        z_axis = -up_base
        x_axis = rotation[:, 0] - z_axis * float(rotation[:, 0] @ z_axis)
        if np.linalg.norm(x_axis) < 1e-8:
            x_axis = np.array([1.0, 0.0, 0.0]) - z_axis * z_axis[0]
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        rotation = np.column_stack((x_axis, y_axis, z_axis))
        orientation_source = "manual: gripper vertical down; roll near cube-capture wrist"
    if abs(args.roll_deg) > 1e-12:
        angle = np.radians(args.roll_deg)
        local_roll = np.array([[np.cos(angle), -np.sin(angle), 0.0],
                               [np.sin(angle), np.cos(angle), 0.0],
                               [0.0, 0.0, 1.0]])
        rotation = rotation @ local_roll
        orientation_source += "; local roll %.1f deg" % args.roll_deg
    view = None
    if args.camera_look_at_top:
        if not args.vertical_down:
            raise SystemExit("--camera-look-at-top requires --vertical-down")
        if args.camera_roll_step_deg <= 0 or args.camera_roll_step_deg > 30:
            raise SystemExit("--camera-roll-step-deg must be within (0, 30]")
        base_rotation = rotation.copy()
        best = None
        # Camera +Z is its OpenCV optical axis.  The score is a geometric
        # visibility proxy only; point-cloud gates still decide whether the
        # cube was actually measured.
        for roll_deg in np.arange(-180.0, 180.0 + 1e-9, args.camera_roll_step_deg):
            angle = np.radians(roll_deg)
            local_roll = np.array([[np.cos(angle), -np.sin(angle), 0.0],
                                   [np.sin(angle), np.cos(angle), 0.0],
                                   [0.0, 0.0, 1.0]])
            candidate_rotation = base_rotation @ local_roll
            candidate_translation = fingertip_hover - candidate_rotation @ offset
            base_from_candidate = transform(candidate_rotation, candidate_translation)
            base_from_camera = base_from_candidate @ tool_from_camera
            ray = top_center - base_from_camera[:3, 3]
            ray_norm = float(np.linalg.norm(ray))
            optical = base_from_camera[:3, 2]
            cosine = float(optical @ ray / ray_norm) if ray_norm > 1e-9 else -1.0
            candidate = (cosine, roll_deg, candidate_rotation, candidate_translation,
                         base_from_camera, ray_norm)
            if best is None or candidate[0] > best[0]:
                best = candidate
        cosine, roll_deg, rotation, tool_translation, camera_pose, distance = best
        orientation_source += "; camera optical-axis look-at roll %.1f deg" % roll_deg
        view = {"target": "cube_top_center", "selected_roll_deg": float(roll_deg),
                "optical_axis_cosine": cosine, "camera_to_target_distance_m": distance,
                "camera_origin_base_m": camera_pose[:3, 3].tolist(),
                "interpretation": "candidate scoring only; live cloud gates remain required"}
    else:
        tool_translation = fingertip_hover - rotation @ offset
    target_pose = tool_translation.tolist() + rotvec_from_rotation(rotation).tolist()
    output = args.output if os.path.isabs(args.output) else os.path.join(root, args.output)
    document = {
        "schema_version": 1,
        "kind": kind,
        "motion_sent": False,
        "authorization": "none: offline visual/IK check only; not a grasp or controller TCP write",
        "limitations": ["opposing fingertip and jaw centre are not calibrated",
                        "fingertip orientation inherits tool0 and is not measured",
                        "preliminary CAD collision model is not real-motion proof"],
        "inputs": {"cube": os.path.relpath(cube_path, root),
                   "tcp": tcp_source,
                   "handeye": os.path.relpath(calibration_path, root)},
        "target_frame": frame,
        "orientation_source": orientation_source,
        "fingertip_hover_base_m": fingertip_hover.tolist(),
        "cube_top_center_base_m": top_center.tolist(),
        "approach_normal_base": up_base.tolist(),
        "camera_view_candidate": view,
        "standoff_mm": args.standoff_mm,
        "plan": [{"slot": 1, "role": frame + "_hover_only",
                  "target_tcp_pose_m_rad": target_pose}],
    }
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "w", encoding="utf-8") as stream:
        json.dump(document, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    print("hover plan:", output)
    print("target hover base (m):", np.round(fingertip_hover, 6).tolist())
    print("tool0 target (m,rad):", np.round(target_pose, 6).tolist())


if __name__ == "__main__":
    main()
