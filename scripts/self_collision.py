#!/usr/bin/env python3
"""Mesh-accurate self-collision check: camera assembly versus the robot's own arm.

The offline move gate originally covered reachability, joint limits, singularity
proximity and flange clearance but **not self-collision**, and on 2026-09-18 the 3D
camera physically contacted the robot arm during a reorientation.  A camera that
hangs ~0.33 m off tool0 swings a long lever, so the arm can fold onto its own
sensor.

A capsule skeleton was tried first and abandoned: it reported a constant overlap at
poses the robot demonstrably moved through, because a fat proxy for the gripper sat
permanently inside the forearm capsule.  This module instead loads the URDF's real
collision meshes (they ship with ``ur_description``) and adds the camera assembly as
boxes, then asks hppfcl for actual distances.

Geometry from the operator (2026-09-18): the camera is on a ~30 cm side mount and
the housing is about 20 cm x 20 cm.  ``tool0_from_camera`` puts the camera origin at
``[0.046, 0.252, 0.202]`` m in the tool0 frame, i.e. ~0.33 m out, which agrees.

The mesh URDF is written to ``outputs/vision/ur10_meshpath.urdf`` because pinocchio
cannot resolve ``package://ur_description`` on its own.
"""
import os

import numpy as np
import pinocchio as pin

URDF_SOURCE = os.path.expanduser("~/ur_learn/generated/ur10.urdf")
URDF_PATCHED = "/home/jx/UR10/outputs/vision/ur10_meshpath.urdf"
MESH_PREFIX = "/home/jx/ros2_ws/install/ur_description/share/ur_description/"

# Geometry now comes from config/robot_attachments.yaml via robot_attachments.py,
# so the collision gate and the RViz/MuJoCo picture are built from one source.  The
# constants below stay as a fallback for callers that pass nothing.
CAMERA_BOX = (0.20, 0.20, 0.15)
BRACKET_WIDTH = 0.05


def patched_urdf(path=URDF_PATCHED):
    if not os.path.exists(path) or \
            os.path.getmtime(path) < os.path.getmtime(URDF_SOURCE):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(URDF_SOURCE, encoding="utf-8") as stream:
            text = stream.read()
        text = text.replace("package://ur_description/", MESH_PREFIX)
        with open(path, "w", encoding="utf-8") as stream:
            stream.write(text)
    return path


def _box_between(start, end, width):
    """(shape, placement) for a box spanning start->end with the given width."""
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    delta = end - start
    length = float(np.linalg.norm(delta))
    if length < 1e-9:
        length, delta = 1e-6, np.asarray([0.0, 0.0, 1e-6])
    z_axis = delta / length
    helper = np.asarray([0.0, 0.0, 1.0])
    if abs(float(z_axis @ helper)) > 0.99:
        helper = np.asarray([1.0, 0.0, 0.0])
    x_axis = np.cross(helper, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    rotation = np.column_stack((x_axis, y_axis, z_axis))
    return (pin.hppfcl.Box(width, width, length),
            pin.SE3(rotation, (start + end) / 2.0))


class SelfCollisionModel:
    """Robot collision meshes plus the camera assembly, with distance queries."""

    def __init__(self, tool0_from_camera=None, camera_box=None,
                 bracket_width=None, geometry=None, attachments_path=None):
        if geometry is None:
            from robot_attachments import load as _load_attachments
            geometry = _load_attachments(attachments_path)
        if tool0_from_camera is not None:
            # Legacy callers pass the 4x4 tool0_from_camera directly.
            geometry = dict(geometry)
            geometry["tool0_from_camera"] = np.asarray(tool0_from_camera, dtype=float)
        self.geometry_config = geometry
        tool0_from_camera = np.asarray(geometry["tool0_from_camera"], dtype=float)
        camera_box = tuple(camera_box or geometry["camera_housing"]["size_m"])
        bracket_width = float(bracket_width or geometry["camera_bracket"]["width_m"])
        self.model = pin.buildModelFromUrdf(patched_urdf())
        self.geometry = pin.buildGeomFromUrdf(self.model, patched_urdf(),
                                              pin.GeometryType.COLLISION)
        self.model_data = self.model.createData()
        tool0_frame = self.model.getFrameId("tool0")
        parent_joint = self.model.frames[tool0_frame].parentJoint
        tool0_placement = self.model.frames[tool0_frame].placement
        camera_offset = np.asarray(tool0_from_camera, dtype=float)
        housing = pin.SE3(camera_offset[:3, :3].copy(), camera_offset[:3, 3].copy())
        camera_id = self.geometry.addGeometryObject(pin.GeometryObject(
            "camera_housing", parent_joint, tool0_placement * housing,
            pin.hppfcl.Box(*camera_box)))
        bracket_shape, bracket_placement = _box_between(
            np.zeros(3), camera_offset[:3, 3], bracket_width)
        bracket_id = self.geometry.addGeometryObject(pin.GeometryObject(
            "camera_bracket", parent_joint, tool0_placement * bracket_placement,
            bracket_shape))
        # The force sensor and the gripper are real bodies on the same flange and
        # can also strike the arm, so they belong in the gate rather than only in
        # the picture.
        added = [("camera_housing", camera_id), ("camera_bracket", bracket_id)]
        sensor = geometry.get("force_sensor") or {}
        if sensor.get("enabled"):
            half = sensor["height_m"] / 2.0
            centre = np.array([0.0, 0.0, sensor["offset_m"] + half])
            sensor_id = self.geometry.addGeometryObject(pin.GeometryObject(
                "force_sensor", parent_joint, tool0_placement * pin.SE3(
                    np.eye(3), centre),
                pin.hppfcl.Cylinder(sensor["diameter_m"] / 2.0, sensor["height_m"])))
            added.append(("force_sensor", sensor_id))
        gripper = geometry.get("gripper") or {}
        if gripper.get("enabled"):
            size = gripper["size_m"]
            centre = np.array([0.0, 0.0, gripper["offset_m"] + size[1] / 2.0])
            gripper_id = self.geometry.addGeometryObject(pin.GeometryObject(
                "gripper_body", parent_joint, tool0_placement * pin.SE3(
                    np.eye(3), centre),
                pin.hppfcl.Box(size[0], size[1], size[2])))
            added.append(("gripper_body", gripper_id))
        self.attachments = added
        self.camera_ids = tuple(value for _name, value in added)
        self.camera_names = tuple(name for name, _value in added)
        # Everything rigidly connected to the last two wrist links moves with the
        # camera, so only the earlier links can genuinely be struck by it.
        checkable = ("base_link", "shoulder_link", "upper_arm_link", "forearm_link",
                     "wrist_1_link")
        self.robot_ids = [index for index, obj in enumerate(self.geometry.geometryObjects)
                          if obj.name.startswith(checkable)]
        self.pairs = []
        for camera_id_value in self.camera_ids:
            for robot_id in self.robot_ids:
                self.geometry.addCollisionPair(pin.CollisionPair(camera_id_value,
                                                                robot_id))
        self.geometry_data = self.geometry.createData()
        self.pair_order = list(self.pairs)

    def min_clearance(self, q):
        """(clearance in metres, description).  Negative means the boxes overlap."""
        pin.forwardKinematics(self.model, self.model_data, np.asarray(q, dtype=float))
        pin.updateGeometryPlacements(self.model, self.model_data, self.geometry,
                                     self.geometry_data)
        minimum = float("inf")
        label = None
        for index, pair in enumerate(self.geometry.collisionPairs):
            result = pin.computeDistance(self.geometry, self.geometry_data, index)
            distance = float(result.min_distance)
            if distance < minimum:
                minimum = distance
                first = self.geometry.geometryObjects[pair.first].name
                second = self.geometry.geometryObjects[pair.second].name
                label = "%s vs %s" % (first, second)
        return minimum, label

    def closest_pair_points(self, q):
        """(clearance, label, point_on_camera, point_on_arm) for the tightest pair.

        The nearest points make the picture diagnostic: a number alone does not
        show *where* the two bodies are about to meet.
        """
        pin.forwardKinematics(self.model, self.model_data, np.asarray(q, dtype=float))
        pin.updateGeometryPlacements(self.model, self.model_data, self.geometry,
                                     self.geometry_data)
        minimum = float("inf")
        best = (None, None, None)
        for index, pair in enumerate(self.geometry.collisionPairs):
            result = pin.computeDistance(self.geometry, self.geometry_data, index)
            distance = float(result.min_distance)
            if distance < minimum:
                minimum = distance
                first = self.geometry.geometryObjects[pair.first].name
                second = self.geometry.geometryObjects[pair.second].name
                points = [np.asarray(result.getNearestPoint1(), dtype=float),
                          np.asarray(result.getNearestPoint2(), dtype=float)]
                camera_first = first.startswith("camera")
                best = ("%s vs %s" % (first, second),
                        points[0] if camera_first else points[1],
                        points[1] if camera_first else points[0])
        return minimum, best[0], best[1], best[2]


def main():
    import argparse
    import json
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import yaml
    from check_move_plan import BASE_FROM_URDF_ROOT, pose_from_tcp_target
    from ur_pose_ik import UR10IK

    parser = argparse.ArgumentParser(description="Report camera-vs-arm clearances")
    parser.add_argument("--gate", default="outputs/vision/gate-half.json")
    parser.add_argument("--calibration",
                        default="config/handeye_eye_in_hand_20260917.yaml")
    parser.add_argument("--attachments", default="config/robot_attachments.yaml",
                        help="attachment geometry YAML; use a preliminary CAD envelope "
                             "only for offline comparison, never as real-motion proof")
    parser.add_argument("--output",
                        default="outputs/vision/self-collision-report.json")
    parser.add_argument("--margin-mm", type=float, default=30.0)
    args = parser.parse_args()

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    with open(os.path.join(root, args.calibration), encoding="utf-8") as stream:
        calibration = yaml.safe_load(stream)
    tool0_from_camera = np.eye(4)
    tool0_from_camera[:3, :3] = np.asarray(calibration["rotation_matrix"])
    tool0_from_camera[:3, 3] = np.asarray(calibration["translation_m"])

    attachments_path = args.attachments if os.path.isabs(args.attachments) else \
        os.path.join(root, args.attachments)
    model = SelfCollisionModel(tool0_from_camera, attachments_path=attachments_path)
    ik = UR10IK()
    with open(os.path.join(root, args.gate), encoding="utf-8") as stream:
        gate = json.load(stream)
    segments = [value for value in gate["segments"]
                if value.get("segment") != "slot_summary" and "end_tcp_m_rad" in value]

    report = {"schema_version": 1, "margin_mm": args.margin_mm,
              "attachments": os.path.relpath(attachments_path, root),
              "kinematic_source": "gated warm-started endpoint q when available",
              "per_slot": {}}
    worst = (float("inf"), None)
    for value in segments:
        if "end_q_rad" in value:
            q = np.asarray(value["end_q_rad"], dtype=float)
        else:
            # Legacy gate files lack the branch selected during their IK gate.
            # Keep this fallback explicitly labelled; it is diagnostic only.
            q, _, _, _ = ik.solve(BASE_FROM_URDF_ROOT.inverse() *
                                  pose_from_tcp_target(value["end_tcp_m_rad"]),
                                  np.zeros(6))
            report["kinematic_source"] = "legacy endpoint re-solve from zero seed (diagnostic only)"
        clearance, pair = model.min_clearance(q)
        slot_worst = report["per_slot"].get(value["slot"])
        if slot_worst is None or clearance < slot_worst[0]:
            report["per_slot"][value["slot"]] = [clearance, pair]
        if clearance < worst[0]:
            worst = (clearance, (value["slot"], value["segment"], pair))
    report["worst"] = {"clearance_m": worst[0],
                       "where": worst[1],
                       "below_margin": bool(worst[0] * 1000.0 < args.margin_mm)}
    print("最差相机-机械臂间隙: %+.1f mm  @ slot %s %s (%s)" %
          (worst[0] * 1000.0, worst[1][0], worst[1][1], worst[1][2]))
    print("门限 %.0f mm -> %s" % (args.margin_mm,
                                  "FAIL" if report["worst"]["below_margin"] else "PASS"))
    print()
    print("%-6s %12s  %s" % ("slot", "间隙(mm)", "最接近"))
    for slot in sorted(report["per_slot"]):
        clearance, pair = report["per_slot"][slot]
        flag = "  <<< 低于门限" if clearance * 1000.0 < args.margin_mm else ""
        print("%-6s %+12.1f  %s%s" % (slot, clearance * 1000.0, pair, flag))
    output = os.path.join(root, args.output)
    with open(output, "w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    print("OUTPUT:", output)


if __name__ == "__main__":
    main()
