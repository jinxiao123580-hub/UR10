#!/usr/bin/env python3
"""Generate a URDF that includes the REAL end-effector stack, for RViz and MuJoCo.

The first version of this script only drew a thin bracket plus a box for the
camera, which looked nothing like the bench: the real flange carries a force
sensor, an adapter, a gripper, and only then the camera on its side mount.  All of
those now come from ``config/robot_attachments.yaml`` through
``scripts/robot_attachments.py``, so one measured file drives both the picture and
the self-collision gate.

Chain emitted (each element optional, configured in the YAML):

    tool0 -- force_sensor -- adapter -- gripper
          \\-- camera_bracket -- camera_housing

Mesh URIs differ by consumer:

* ``--uri package`` (default) keeps ``package://ur_description/...``, which is what
  RViz's resource_retriever requires.  Plain absolute paths make RViz report
  "Could not load mesh resource" for every link.
* ``--uri local`` (implied by ``--stl-only``) copies the meshes next to the URDF and
  uses bare filenames, because MuJoCo's URDF parser discards directories and cannot
  read Collada.

    python3 scripts/build_urdf_with_camera.py                 # RViz
    python3 scripts/build_urdf_with_camera.py --stl-only      # MuJoCo
"""
import argparse
import os
import re
import shutil

import numpy as np

from robot_attachments import load, summary
from self_collision import MESH_PREFIX, URDF_SOURCE

URDF_OUTPUT = {
    "package": "outputs/vision/ur10_with_camera.urdf",
    "local": "outputs/vision/ur10_with_camera_mujoco.urdf",
}


def box_urdf(name, size, origin_xyz, origin_rpy, colour):
    return (
        '  <link name="%s">\n'
        '    <visual>\n'
        '      <origin xyz="%s" rpy="%s"/>\n'
        '      <geometry><box size="%s"/></geometry>\n'
        '      <material name="%s_mat"><color rgba="%s"/></material>\n'
        '    </visual>\n'
        '    <collision>\n'
        '      <origin xyz="%s" rpy="%s"/>\n'
        '      <geometry><box size="%s"/></geometry>\n'
        '    </collision>\n'
        '  </link>\n' % (name, origin_xyz, origin_rpy, size, name, colour,
                         origin_xyz, origin_rpy, size))


def cylinder_urdf(name, radius, length, origin_xyz, origin_rpy, colour):
    geometry = '<cylinder radius="%.6f" length="%.6f"/>' % (radius, length)
    return (
        '  <link name="%s">\n'
        '    <visual>\n'
        '      <origin xyz="%s" rpy="%s"/>\n'
        '      <geometry>%s</geometry>\n'
        '      <material name="%s_mat"><color rgba="%s"/></material>\n'
        '    </visual>\n'
        '    <collision>\n'
        '      <origin xyz="%s" rpy="%s"/>\n'
        '      <geometry>%s</geometry>\n'
        '    </collision>\n'
        '  </link>\n' % (name, origin_xyz, origin_rpy, geometry, name, colour,
                         origin_xyz, origin_rpy, geometry))


def fixed_joint(name, parent, child, origin_xyz, origin_rpy):
    return ('  <joint name="%s" type="fixed">\n'
            '    <parent link="%s"/>\n'
            '    <child link="%s"/>\n'
            '    <origin xyz="%s" rpy="%s"/>\n'
            '  </joint>\n' % (name, parent, child, origin_xyz, origin_rpy))


def rpy_from_rotation(rotation):
    """URDF rpy (fixed-axis roll-pitch-yaw) from a rotation matrix."""
    rotation = np.asarray(rotation, dtype=float)
    sy = float(np.clip(-rotation[2, 0], -1.0, 1.0))
    pitch = np.arcsin(sy)
    if abs(sy) < 0.999999:
        roll = np.arctan2(rotation[2, 1], rotation[2, 2])
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = np.arctan2(-rotation[1, 2], rotation[1, 1])
        yaw = 0.0
    return np.array([roll, pitch, yaw])


def frame_for_segment(start, end):
    """(centre, rotation, length) of a box spanning start->end along its own z."""
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    delta = end - start
    length = float(np.linalg.norm(delta))
    z_axis = delta / length
    helper = np.array([0.0, 0.0, 1.0])
    if abs(float(z_axis @ helper)) > 0.99:
        helper = np.array([1.0, 0.0, 0.0])
    x_axis = np.cross(helper, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    return (start + end) / 2.0, np.column_stack((x_axis, y_axis, z_axis)), length


def triple(values):
    return "%.6f %.6f %.6f" % tuple(float(v) for v in values)


def build(geometry, uri_mode, stl_only, output_path):
    with open(URDF_SOURCE, encoding="utf-8") as stream:
        text = stream.read()
    if stl_only:
        uri_mode = "local"
        text = text.replace("/visual/", "/collision/").replace(".dae", ".stl")
    if uri_mode == "local":
        text = text.replace("package://ur_description/", MESH_PREFIX)

    parts = [text]
    tool0_from_camera = np.asarray(geometry["tool0_from_camera"], dtype=float)
    camera_offset = tool0_from_camera[:3, 3]
    tool_axis = np.array([0.0, 0.0, 1.0])

    chain_parent = "tool0"
    cursor = np.zeros(3)
    sensor = geometry["force_sensor"]
    if sensor["enabled"]:
        centre = cursor + tool_axis * (sensor["offset_m"] + sensor["height_m"] / 2.0)
        parts.append(cylinder_urdf("force_sensor", sensor["diameter_m"] / 2.0,
                                   sensor["height_m"], triple(centre), "0 0 0",
                                   "0.55 0.55 0.60 1"))
        parts.append(fixed_joint("force_sensor_joint", chain_parent, "force_sensor",
                                 triple(centre), "0 0 0"))
        cursor = cursor + tool_axis * (sensor["offset_m"] + sensor["height_m"])
        chain_parent = "force_sensor"

    adapter = geometry["adapter"]
    if adapter["enabled"]:
        size = adapter["size_m"]
        centre = cursor + tool_axis * (size[2] / 2.0)
        parts.append(box_urdf("adapter", triple(size), triple(centre), "0 0 0",
                              "0.45 0.45 0.50 1"))
        parts.append(fixed_joint("adapter_joint", chain_parent, "adapter",
                                 triple(centre), "0 0 0"))
        cursor = cursor + tool_axis * size[2]
        chain_parent = "adapter"

    gripper = geometry["gripper"]
    if gripper["enabled"]:
        size = gripper["size_m"]
        centre = cursor + tool_axis * (gripper["offset_m"] + size[1] / 2.0)
        parts.append(box_urdf("gripper", triple(size), triple(centre), "0 0 0",
                              "0.30 0.32 0.36 1"))
        parts.append(fixed_joint("gripper_joint", chain_parent, "gripper",
                                 triple(centre), "0 0 0"))

    bracket = geometry["camera_bracket"]
    centre, rotation, length = frame_for_segment(np.zeros(3), camera_offset)
    if bracket["length_m"] is not None:
        length = float(bracket["length_m"])
    parts.append(box_urdf("camera_bracket",
                          triple([bracket["width_m"], bracket["width_m"], length]),
                          triple(centre), triple(rpy_from_rotation(rotation)),
                          "0.85 0.40 0.25 1"))
    parts.append(fixed_joint("camera_bracket_joint", "tool0", "camera_bracket",
                             triple(centre), triple(rpy_from_rotation(rotation))))

    # The housing sits at the camera origin with the camera's own orientation.
    # Its origin is expressed relative to the bracket, which means composing
    # transforms: inv(T_bracket) * T_housing.  Subtracting the two rpy vectors is
    # not a rotation composition and silently misplaces the camera.
    bracket_transform = np.eye(4)
    bracket_transform[:3, :3] = rotation
    bracket_transform[:3, 3] = centre
    relative = np.linalg.inv(bracket_transform) @ tool0_from_camera
    parts.append(box_urdf("camera_housing",
                          triple(geometry["camera_housing"]["size_m"]),
                          triple(relative[:3, 3]),
                          triple(rpy_from_rotation(relative[:3, :3])),
                          "0.95 0.45 0.20 1"))
    parts.append(fixed_joint("camera_housing_joint", "camera_bracket",
                             "camera_housing", triple(relative[:3, 3]),
                             triple(rpy_from_rotation(relative[:3, :3]))))
    parts.append("</robot>\n")

    text = "".join(parts)
    if uri_mode == "local":
        # MuJoCo keeps only the basename of each asset and resolves it against the
        # model file's own directory, so the meshes must sit right beside it.
        directory = os.path.dirname(output_path)
        for source in sorted(set(re.findall(r'filename="([^"]+)"', text))):
            shutil.copyfile(source, os.path.join(directory,
                                                 os.path.basename(source)))
        text = re.sub(r'filename="[^"]*/([^/"]+)"',
                      lambda match: 'filename="%s"' % match.group(1), text)
    return text


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--attachments", default=None,
                        help="geometry config (default config/robot_attachments.yaml)")
    parser.add_argument("--uri", choices=("package", "local"), default="package")
    parser.add_argument("--stl-only", action="store_true",
                        help="collision STL meshes for visuals too, local URIs "
                             "(MuJoCo cannot read Collada)")
    parser.add_argument("--output", default=None)
    args = parser.parse_args()

    geometry = load(args.attachments, root=root)
    if args.stl_only:
        args.uri = "local"
    output = args.output or URDF_OUTPUT[args.uri]
    output = output if os.path.isabs(output) else os.path.join(root, output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    text = build(geometry, args.uri, args.stl_only, output)
    with open(output, "w", encoding="utf-8") as stream:
        stream.write(text)
    print(summary(geometry))
    print("  mesh URI 模式: %s" % args.uri)
    print("  %s" % os.path.relpath(output, root))
    print()
    print("改几何请编辑 config/robot_attachments.yaml，然后重跑本脚本。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
