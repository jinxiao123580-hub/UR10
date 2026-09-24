#!/usr/bin/env python3
"""Render the robot plus the camera assembly at the controller's current state.

The point is to *see* what the offline gate reasons about, using exactly the same
geometry the collision check uses: the URDF's real collision meshes, plus the
camera housing and bracket boxes.  If the picture and the gate ever disagree, one
of them is wrong and that is worth knowing.

The camera mount matters a lot here: it hangs ~0.33 m off tool0 with a ~20 x 20 cm
housing, which is what let the arm fold onto its own sensor on 2026-09-18.

No extra packages: binary STL is parsed with numpy and drawn with matplotlib.

Usage
-----
    python3 scripts/visualize_urdf.py                      # current state -> PNG
    python3 scripts/visualize_urdf.py --q "q1,...,q6"      # explicit configuration
    python3 scripts/visualize_urdf.py --watch 2            # re-render every 2 s
    python3 scripts/visualize_urdf.py --plan outputs/handeye/plan-safe-A1.json
"""
import argparse
import datetime as dt
import json
import os
import struct
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import pinocchio as pin  # noqa: E402

from check_move_plan import BASE_FROM_URDF_ROOT  # noqa: E402
from self_collision import SelfCollisionModel  # noqa: E402

VIEWS = ((22, -60), (22, 30), (70, -90), (8, 0))
MESH_CACHE = {}


def load_binary_stl(path):
    """Return an (n, 3, 3) float array of triangles from a binary STL."""
    if path in MESH_CACHE:
        return MESH_CACHE[path]
    with open(path, "rb") as stream:
        stream.read(80)
        count = struct.unpack("<I", stream.read(4))[0]
        raw = np.frombuffer(stream.read(count * 50), dtype=np.uint8)
    triangles = raw.reshape(count, 50)[:, 12:48].copy().view("<f4")
    triangles = triangles.reshape(count, 3, 3).astype(np.float64)
    MESH_CACHE[path] = triangles
    return triangles


def box_triangles(center, rotation, size):
    """12 triangles of an axis-aligned box transformed by rotation and centre."""
    half = np.asarray(size, dtype=float) / 2.0
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1)
                        for sz in (-1, 1)], dtype=float) * half
    corners = (np.asarray(rotation, dtype=float) @ corners.T).T + np.asarray(center)
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1),
             (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    return [np.array([corners[a], corners[b], corners[c], corners[d]])
            for a, b, c, d in faces]


def camera_geometry(tool0_from_camera):
    """(housing_faces, bracket_faces, camera_origin_local, rotation)."""
    offset = np.asarray(tool0_from_camera, dtype=float)
    return offset


def render(model, q, tool0_from_camera, board_origin, plan_cameras, title, path,
           highlight=None, clearance=None, pair=None):
    pin.forwardKinematics(model.model, model.model_data, np.asarray(q, dtype=float))
    pin.updateGeometryPlacements(model.model, model.model_data, model.geometry,
                                 model.geometry_data)
    figure = plt.figure(figsize=(15, 11))
    figure.suptitle(title, fontsize=13)

    # Everything is drawn in the controller's `base` frame, because that is the
    # frame the operator reads on the pendant and the frame check_move_plan works
    # in.  The meshes come out of forward kinematics in the URDF root frame, which
    # differs from `base` by a 180 deg yaw, so every placement is mapped through
    # BASE_FROM_URDF_ROOT.  Mixing the two frames silently mirrors the whole scene.
    offset = np.asarray(tool0_from_camera, dtype=float)
    tool0_frame = model.model.getFrameId("tool0")
    pin.updateFramePlacements(model.model, model.model_data)
    tool0_placement = BASE_FROM_URDF_ROOT * model.model_data.oMf[tool0_frame]
    camera_world = tool0_placement * pin.SE3(offset[:3, :3], offset[:3, 3])
    camera_center = camera_world.translation
    camera_rotation = camera_world.rotation

    for view_index, (elevation, azimuth) in enumerate(VIEWS):
        axes = figure.add_subplot(2, 2, view_index + 1, projection="3d")
        for index, geometry in enumerate(model.geometry.geometryObjects):
            if geometry.name.startswith("camera"):
                continue
            placement = BASE_FROM_URDF_ROOT * model.geometry_data.oMg[index]
            triangles = load_binary_stl(geometry.meshPath)
            transformed = (placement.rotation @ triangles.reshape(-1, 3).T).T \
                .reshape(-1, 3, 3) + placement.translation
            axes.add_collection3d(Poly3DCollection(
                transformed, alpha=0.85, facecolor="#9fb8d0",
                edgecolor="#4a5f75", linewidths=0.1))
        # Camera housing and the bracket running back to tool0.
        for faces, colour in ((box_triangles(camera_center, camera_rotation,
                                             (0.20, 0.20, 0.15)), "#d95f3b"),):
            axes.add_collection3d(Poly3DCollection(
                faces, alpha=0.95, facecolor=colour, edgecolor="#7a2f18"))
        tool_origin = tool0_placement.translation
        axes.add_collection3d(Line3DCollection(
            [np.array([tool_origin, camera_center])], colors="#d95f3b",
            linewidths=2.5))
        # Table plane and the checkerboard.
        axes.add_collection3d(Poly3DCollection(
            [np.array([[-0.4, -0.6, -0.01], [1.2, -0.6, -0.01],
                       [1.2, 0.6, -0.01], [-0.4, 0.6, -0.01]])],
            alpha=0.18, facecolor="#8a8a8a"))
        board = np.asarray(board_origin, dtype=float)
        axes.add_collection3d(Poly3DCollection(
            [np.array([[board[0] - 0.03, board[1] - 0.021, board[2] + 0.002],
                       [board[0] + 0.03, board[1] - 0.021, board[2] + 0.002],
                       [board[0] + 0.03, board[1] + 0.021, board[2] + 0.002],
                       [board[0] - 0.03, board[1] + 0.021, board[2] + 0.002]])],
            alpha=0.95, facecolor="#f2e7c9", edgecolor="#666"))
        if highlight is not None:
            camera_point, arm_point = highlight
            axes.add_collection3d(Line3DCollection(
                [np.array([camera_point, arm_point])], colors="#c00000",
                linewidths=3.5))
            axes.scatter(*np.asarray([camera_point, arm_point]).T, s=45,
                         c="#c00000", depthshade=False)
        if plan_cameras is not None and len(plan_cameras):
            points = np.asarray(plan_cameras, dtype=float)
            axes.scatter(points[:, 0], points[:, 1], points[:, 2], s=14,
                         c="#2b7a3d", depthshade=False,
                         label="plan camera poses")
        axes.set_xlim(-0.4, 1.2)
        axes.set_ylim(-0.7, 0.7)
        axes.set_zlim(0.0, 1.3)
        axes.set_box_aspect((16, 14, 13))
        axes.view_init(elev=elevation, azim=azimuth)
        axes.set_xlabel("X (m)")
        axes.set_ylabel("Y (m)")
        axes.set_zlabel("Z (m)")
        if view_index == 0:
            axes.set_title("camera-arm clearance %+.1f mm\n(%s)  red = nearest points"
                           % (clearance * 1000.0, pair), fontsize=10)
        else:
            axes.set_title("elev %d / azim %d" % (elevation, azimuth), fontsize=10)
    figure.tight_layout()
    figure.savefig(path, dpi=110)
    plt.close(figure)
    return path


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--q", default=None, help="6 joint angles (rad); default: read "
                                                  "the controller's current state")
    parser.add_argument("--host", default="192.168.1.3")
    parser.add_argument("--calibration",
                        default="config/handeye_eye_in_hand_20260917.yaml")
    parser.add_argument("--plan", default=None,
                        help="also scatter the camera poses of this pose plan")
    parser.add_argument("--board-origin", default=None,
                        help="board origin (x y z, m); default: from the plan if given, "
                             "else 0.5227 -0.1143 -0.0107")
    parser.add_argument("--output", default="outputs/vision/urdf-view.png")
    parser.add_argument("--watch", type=float, default=None,
                        help="re-render every N seconds from the controller state")
    args = parser.parse_args()

    import yaml
    with open(os.path.join(root, args.calibration), encoding="utf-8") as stream:
        calibration = yaml.safe_load(stream)
    tool0_from_camera = np.eye(4)
    tool0_from_camera[:3, :3] = np.asarray(calibration["rotation_matrix"])
    tool0_from_camera[:3, 3] = np.asarray(calibration["translation_m"])

    plan_cameras = None
    board_origin = np.array([0.5227, -0.1143, -0.0107])
    if args.plan:
        with open(os.path.join(root, args.plan), encoding="utf-8") as stream:
            plan = json.load(stream)
        plan_cameras = [entry["camera_origin_base_m"] for entry in plan["plan"]]
        estimate = plan.get("base_from_target_estimate")
        if estimate is not None:
            board_origin = np.asarray(estimate)[:3, 3]
    if args.board_origin:
        board_origin = np.array([float(x) for x in args.board_origin.split()])

    model = SelfCollisionModel(tool0_from_camera)
    output = os.path.join(root, args.output)
    os.makedirs(os.path.dirname(output), exist_ok=True)

    def current_q():
        if args.q:
            return np.asarray([float(x) for x in args.q.split(",")], dtype=float)
        from record_ur_trajectory import RealtimeReader
        reader = RealtimeReader(args.host)
        try:
            q, tcp, _velocity = reader.read()
        finally:
            reader.close()
        return np.asarray(q, dtype=float), np.asarray(tcp, dtype=float)

    if args.watch:
        print("每 %.1f s 重渲染一次 -> %s（Ctrl-C 退出）" % (args.watch, output))
        while True:
            q, tcp = current_q()
            clearance, pair, camera_point, arm_point = model.closest_pair_points(q)
            camera_point = BASE_FROM_URDF_ROOT.act(np.asarray(camera_point, dtype=float))
            arm_point = BASE_FROM_URDF_ROOT.act(np.asarray(arm_point, dtype=float))
            render(model, q, tool0_from_camera, board_origin, plan_cameras,
                   "UR10 + camera | %s | TCP %s | clearance %+.1f mm (%s)" %
                   (dt.datetime.now().strftime("%H:%M:%S"),
                    np.round(tcp[:3], 3).tolist(), clearance * 1000.0, pair),
                   output, highlight=(camera_point, arm_point),
                   clearance=clearance, pair=pair)
            print("  %s  clearance %+7.1f mm  %s" %
                  (dt.datetime.now().strftime("%H:%M:%S"), clearance * 1000.0, pair),
                  flush=True)
            time.sleep(args.watch)

    result = current_q()
    if args.q:
        q, tcp = result, None
    else:
        q, tcp = result
    clearance, pair, camera_point, arm_point = model.closest_pair_points(q)
    # closest_pair_points works in the URDF root frame; map to base for display.
    camera_point = BASE_FROM_URDF_ROOT.act(np.asarray(camera_point, dtype=float))
    arm_point = BASE_FROM_URDF_ROOT.act(np.asarray(arm_point, dtype=float))
    title = "UR10 + camera assembly | q = %s" % np.round(q, 3).tolist()
    if tcp is not None:
        title = ("UR10 + camera assembly | TCP %s\ncamera-arm clearance %+.1f mm "
                 "(%s)" % (np.round(tcp[:3], 4).tolist(), clearance * 1000.0, pair))
    render(model, q, tool0_from_camera, board_origin, plan_cameras, title, output,
           highlight=(camera_point, arm_point), clearance=clearance, pair=pair)
    print("最近点 相机侧 %s / 机械臂侧 %s" %
          (np.round(camera_point, 4).tolist(), np.round(arm_point, 4).tolist()))
    print("关节角 q(rad): %s" % np.round(q, 4).tolist())
    if tcp is not None:
        print("TCP(m, rad) : %s" % np.round(tcp, 4).tolist())
    print("相机-机械臂最小间隙: %+.1f mm  (%s)" % (clearance * 1000.0, pair))
    print("画面: %s" % output)


if __name__ == "__main__":
    main()
