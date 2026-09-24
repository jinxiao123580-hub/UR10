#!/usr/bin/env python3
"""Render the uploaded end-effector CAD meshes before integrating them.

This is a visual-only aid.  It does not modify a URDF, collision gate, or robot.
The exported STL coordinates are millimetres, with their assembly axis along
+Y.  The ATI mounting face is made the local origin, +Y is mapped to tool0 +Z,
and the four physically plausible flange clockings are rendered side by side.
"""
import argparse
import glob
import os
import struct

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np


def load_binary_stl(path):
    with open(path, "rb") as stream:
        stream.read(80)
        count = struct.unpack("<I", stream.read(4))[0]
        raw = stream.read(count * 50)
    return (np.frombuffer(raw, dtype=np.uint8).reshape(count, 50)[:, 12:48]
            .copy().view("<f4").reshape(-1, 3, 3).astype(float))


def yaw_matrix(degrees):
    angle = np.radians(degrees)
    return np.array([[np.cos(angle), -np.sin(angle), 0.0],
                     [np.sin(angle), np.cos(angle), 0.0],
                     [0.0, 0.0, 1.0]])


def localise(meshes, sensor_mesh):
    """CAD mm -> candidate tool0 m, anchored at the sensor mounting face."""
    points = sensor_mesh.reshape(-1, 3)
    low, high = points.min(axis=0), points.max(axis=0)
    # The CAD's sensor axis is +Y.  Its -Y face is the mounting face; its x/z
    # midpoint keeps the flange centre on the local z axis.
    origin = np.array([(low[0] + high[0]) / 2.0, low[1],
                       (low[2] + high[2]) / 2.0])
    # CAD x -> tool x; CAD y -> tool z; CAD z -> -tool y.  Right handed.
    cad_to_tool = np.array([[1.0, 0.0, 0.0],
                            [0.0, 0.0, -1.0],
                            [0.0, 1.0, 0.0]])
    return {name: (cad_to_tool @ ((triangles - origin) / 1000.0).reshape(-1, 3).T).T
            .reshape(triangles.shape)
            for name, triangles in meshes.items()}


def draw_axes(axis):
    colours = ("#d62728", "#2ca02c", "#1f77b4")
    for vector, colour, label in zip(np.eye(3), colours, ("tool x", "tool y", "tool z")):
        axis.quiver(0, 0, 0, *vector * 0.12, color=colour, arrow_length_ratio=0.12)
        axis.text(*(vector * 0.13), label, color=colour, fontsize=8)


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--models", default="ros2_ws/src/ur10_digital_twin/models/end_effector")
    parser.add_argument("--output", default="outputs/vision/end-effector-cad-clockings.png")
    args = parser.parse_args()
    models = args.models if os.path.isabs(args.models) else os.path.join(root, args.models)
    paths = sorted(glob.glob(os.path.join(models, "*.STL")))
    if len(paths) != 4:
        raise RuntimeError("expected four STL files under %s, found %d" % (models, len(paths)))
    meshes = {os.path.basename(path): load_binary_stl(path) for path in paths}
    sensor_name = next((name for name in meshes if "六轴力" in name), None)
    if sensor_name is None:
        raise RuntimeError("ATI sensor STL (六轴力) not found")
    meshes = localise(meshes, meshes[sensor_name])
    colours = {"2F85": "#455a64", "3D": "#f57c00", "六轴力": "#1565c0", "连接1": "#9e9e9e"}
    figure = plt.figure(figsize=(14, 12))
    for index, clocking in enumerate((0, 90, 180, 270), start=1):
        axis = figure.add_subplot(2, 2, index, projection="3d")
        rotation = yaw_matrix(clocking)
        for name, triangles in meshes.items():
            transformed = (rotation @ triangles.reshape(-1, 3).T).T.reshape(triangles.shape)
            colour = next(value for token, value in colours.items() if token in name)
            axis.add_collection3d(Poly3DCollection(transformed, facecolor=colour,
                                                    edgecolor="none", alpha=0.9))
        draw_axes(axis)
        axis.set_title("candidate tool_yaw = %d°" % clocking)
        axis.set_xlim(-0.23, 0.23)
        axis.set_ylim(-0.23, 0.23)
        axis.set_zlim(-0.12, 0.34)
        axis.set_box_aspect((1, 1, 1.25))
        axis.view_init(elev=23, azim=-58)
        axis.set_xlabel("tool0 x (m)")
        axis.set_ylabel("tool0 y (m)")
        axis.set_zlabel("tool0 z (m)")
    figure.suptitle("Uploaded end-effector CAD — visual-only clocking candidates\n"
                     "origin: ATI mounting face; CAD +Y → tool0 +Z; scale: mm → m",
                     fontsize=14)
    figure.tight_layout()
    output = args.output if os.path.isabs(args.output) else os.path.join(root, args.output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    figure.savefig(output, dpi=160)
    print(output)


if __name__ == "__main__":
    main()
