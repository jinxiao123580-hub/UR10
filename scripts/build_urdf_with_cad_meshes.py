#!/usr/bin/env python3
"""Build a visual/collision URDF using the uploaded end-effector STL assets.

This preserves the CAD meshes as a single rigid assembly, rather than replacing
them with boxes.  The ATI high-Y mounting face is provisionally coincident with
tool0; the physical chain runs along CAD -Y, which maps to tool0 +Z.  The clocking
is an explicit configuration value and this generator is for offline visual/
collision iteration only.
"""
import argparse
import json
import os
import re
import shutil
import struct

import numpy as np

from self_collision import URDF_SOURCE
from robot_attachments import load


def binary_bounds(path):
    with open(path, "rb") as stream:
        stream.read(80)
        count = struct.unpack("<I", stream.read(4))[0]
        raw = stream.read(count * 50)
    points = (np.frombuffer(raw, dtype=np.uint8).reshape(count, 50)[:, 12:48]
              .copy().view("<f4").reshape(-1, 3))
    return points.min(axis=0), points.max(axis=0)


def rpy(rotation):
    pitch = np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0))
    if abs(rotation[2, 0]) < 0.999999:
        roll = np.arctan2(rotation[2, 1], rotation[2, 2])
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    else:
        roll, yaw = np.arctan2(-rotation[1, 2], rotation[1, 1]), 0.0
    return roll, pitch, yaw


def fmt(values):
    return " ".join("%.9f" % float(value) for value in values)


def mesh_link(name, uri, origin_xyz, origin_rpy, colour):
    geometry = '<mesh filename="%s" scale="0.001 0.001 0.001"/>' % uri
    return '''  <link name="{name}">
    <visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry>{g}</geometry><material name="{name}_mat"><color rgba="{c}"/></material></visual>
    <collision><origin xyz="0 0 0" rpy="0 0 0"/><geometry>{g}</geometry></collision>
  </link>
  <joint name="tool0_to_{name}" type="fixed">
    <parent link="tool0"/><child link="{name}"/>
    <origin xyz="{xyz}" rpy="{rpy}"/>
  </joint>
'''.format(name=name, g=geometry, c=colour, xyz=fmt(origin_xyz), rpy=fmt(origin_rpy))


def fingertip_tcp_link(offset_m):
    """A visible, non-collision marker for the position-only pivot TCP."""
    return '''  <link name="left_fingertip_tcp">
    <visual><geometry><sphere radius="0.008"/></geometry>
      <material name="fingertip_tcp_marker"><color rgba="0.10 0.95 0.20 1"/></material></visual>
  </link>
  <joint name="tool0_to_left_fingertip_tcp" type="fixed">
    <parent link="tool0"/><child link="left_fingertip_tcp"/>
    <origin xyz="{xyz}" rpy="0 0 0"/>
  </joint>
'''.format(xyz=fmt(offset_m))


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attachments", default="config/robot_attachments.preliminary-cad-20260921.yaml")
    parser.add_argument("--output", default="outputs/vision/ur10_preliminary_cad_meshes.urdf")
    parser.add_argument("--fingertip-calibration",
                        default="outputs/tcp_calibration/fingertip-20260921.json",
                        help="validated fixed-point TCP JSON; adds a visual-only TCP marker")
    args = parser.parse_args()
    geometry = load(args.attachments, root=root)
    tcp_path = args.fingertip_calibration
    if not os.path.isabs(tcp_path):
        tcp_path = os.path.join(root, tcp_path)
    with open(tcp_path, encoding="utf-8") as stream:
        tcp_document = json.load(stream)
    tcp_result = tcp_document.get("result") or {}
    if not tcp_result.get("passed_numeric_gate"):
        raise RuntimeError("fingertip TCP did not pass its independent numeric gate: %s" % tcp_path)
    tcp_offset = np.asarray(tcp_result["tool0_to_fingertip_m"], dtype=float)
    if tcp_offset.shape != (3,):
        raise RuntimeError("expected a 3D tool0_to_fingertip_m offset: %s" % tcp_path)
    config_path = geometry["config_path"]
    import yaml
    with open(config_path, encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    model_dir = config.get("cad_models_dir")
    model_dir = model_dir if os.path.isabs(model_dir) else os.path.join(root, model_dir)
    paths = sorted(path for path in os.listdir(model_dir) if path.endswith(".STL"))
    if len(paths) != 4:
        raise RuntimeError("expected exactly four end-effector STL files in %s" % model_dir)
    full = [os.path.join(model_dir, path) for path in paths]
    sensor = next(path for path in full if "六轴力" in os.path.basename(path))
    low, high = binary_bounds(sensor)
    # CAD is in mm.  Its high-Y sensor face bolts to the robot flange.  The
    # connector and gripper run from that face along -Y, so -Y maps to tool +Z.
    origin_mm = np.array([(low[0] + high[0]) / 2.0, high[1],
                          (low[2] + high[2]) / 2.0])
    cad_to_tool = np.array([[1.0, 0.0, 0.0],
                            [0.0, 0.0, 1.0],
                            [0.0, -1.0, 0.0]])
    angle = np.radians(float(config.get("tool_yaw_deg", 0.0)))
    yaw = np.array([[np.cos(angle), -np.sin(angle), 0.0],
                    [np.sin(angle), np.cos(angle), 0.0],
                    [0.0, 0.0, 1.0]])
    rotation = yaw @ cad_to_tool
    translation = -rotation @ origin_mm / 1000.0
    source = open(URDF_SOURCE, encoding="utf-8").read()
    # The base URDF is a complete document; append our links inside its robot
    # element rather than after its existing closing tag.
    source = re.sub(r"\s*</robot>\s*$", "\n", source)
    colours = {"六轴力": "0.10 0.35 0.75 1", "2F85": "0.18 0.20 0.23 1",
               "连接1": "0.60 0.60 0.60 1", "3D": "0.85 0.25 0.10 1"}
    output = args.output if os.path.isabs(args.output) else os.path.join(root, args.output)
    asset_dir = os.path.join(os.path.dirname(output), "cad_meshes")
    os.makedirs(asset_dir, exist_ok=True)
    parts = [source]
    for index, path in enumerate(full):
        filename = os.path.basename(path)
        token = next(key for key in colours if key in filename)
        # RViz/resource_retriever on this host cannot resolve the uploaded
        # non-ASCII filenames through file://.  Stage byte-identical assets at
        # stable ASCII paths beside the generated URDF.
        staged = os.path.join(asset_dir, "cad_attachment_%d.stl" % index)
        shutil.copyfile(path, staged)
        parts.append(mesh_link("cad_attachment_%d" % index, "file://" + staged,
                               translation, rpy(rotation), colours[token]))
    # The pivot calibration supplies position only.  Its frame axes deliberately
    # inherit tool0; do not treat them as a measured fingertip orientation.
    parts.append(fingertip_tcp_link(tcp_offset))
    parts.append("</robot>\n")
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "w", encoding="utf-8") as stream:
        stream.write("".join(parts))
    print("CAD mesh URDF:", output)
    print("CAD mounting origin (mm):", np.round(origin_mm, 3).tolist())
    print("tool_yaw_deg:", config.get("tool_yaw_deg", 0.0))
    print("left_fingertip_tcp visual marker (m):", np.round(tcp_offset, 6).tolist())


if __name__ == "__main__":
    main()
