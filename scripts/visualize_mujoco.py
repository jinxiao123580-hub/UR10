#!/usr/bin/env python3
"""Visualise (and cross-check) the robot + camera assembly in MuJoCo.

Two jobs:

* ``--watch``  open the MuJoCo viewer and follow the controller's live ``q_actual``
  from 30003.  Nothing is commanded; this is a viewer.
* ``--check``  independently re-test the camera-versus-arm clearance that
  ``check_move_plan.py`` gates with hppfcl.  MuJoCo 2.3.7 has no ``mj_geomDistance``,
  but setting ``geom_margin`` makes MuJoCo generate a contact whenever the gap is
  under that margin, and ``contact.dist`` then reports the gap itself - so a
  generous margin turns the contact engine into a distance query.  Two independent
  collision engines agreeing is much stronger evidence than one.

The model comes from ``build_urdf_with_camera.py``, which bakes the same camera
geometry the gate uses into a URDF.
"""
import argparse
import datetime as dt
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mujoco  # noqa: E402
import mujoco.viewer  # noqa: E402  (imported here, not in a function: a local
# ``import mujoco.viewer`` inside main() would make the whole name local and
# shadow the module-level binding used for mujoco.__version__)

ARM_JOINTS = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")


class MujocoScene:
    def __init__(self, urdf):
        self.model = mujoco.MjModel.from_xml_path(urdf)
        # MuJoCo skips collisions between bodies linked in the same kinematic chain
        # unless this is switched off, and the whole robot plus the camera is one
        # chain - without it no camera-vs-arm pair is ever tested and every pose
        # looks collision-free.
        self.model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_FILTERPARENT
        self.data = mujoco.MjData(self.model)
        self.qpos_address = []
        for name in ARM_JOINTS:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                raise RuntimeError("joint not found in MJCF: %s" % name)
            self.qpos_address.append(int(self.model.jnt_qposadr[joint_id]))
        # Camera geoms are the two boxes attached to the last wrist link.
        self.camera_geoms = []
        for index in range(self.model.ngeom):
            if self.model.geom_type[index] == mujoco.mjtGeom.mjGEOM_BOX and \
                    self.model.geom_bodyid[index] == \
                    self.model.geom_bodyid[self.model.ngeom - 1]:
                self.camera_geoms.append(index)
        self.arm_geoms = [index for index in range(self.model.ngeom)
                          if index not in self.camera_geoms]

    def set_q(self, q):
        for address, value in zip(self.qpos_address, q):
            self.data.qpos[address] = float(value)
        mujoco.mj_forward(self.model, self.data)

    def set_margin(self, margin):
        for index in self.camera_geoms:
            self.model.geom_margin[index] = margin
        mujoco.mj_forward(self.model, self.data)

    def closest_camera_gap(self):
        """(gap in metres, camera geom id, other geom id) using margins as a probe."""
        best = (None, None, None)
        for contact_index in range(self.data.ncon):
            contact = self.data.contact[contact_index]
            first, second = int(contact.geom1), int(contact.geom2)
            if first not in self.camera_geoms and second not in self.camera_geoms:
                continue
            distance = float(contact.dist)
            if best[0] is None or distance < best[0]:
                best = (distance, first, second)
        return best

    def name(self, geom_id):
        if geom_id is None:
            return "-"
        body = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY,
                                 self.model.geom_bodyid[geom_id]) or "?"
        kind = "camera" if geom_id in self.camera_geoms else "arm"
        return "%s(%s)" % (body, kind)


def read_q(host):
    from record_ur_trajectory import RealtimeReader
    reader = RealtimeReader(host)
    try:
        q, tcp, _velocity = reader.read()
    finally:
        reader.close()
    return np.asarray(q, dtype=float), np.asarray(tcp, dtype=float)


def pose_q(tcp_target):
    import pinocchio as pin
    from check_move_plan import BASE_FROM_URDF_ROOT
    from ur_pose_ik import UR10IK
    ik = UR10IK()
    target = pin.SE3(pin.exp3(np.asarray(tcp_target[3:], dtype=float)),
                     np.asarray(tcp_target[:3], dtype=float))
    q, _, _, _ = ik.solve(BASE_FROM_URDF_ROOT.inverse() * target, np.zeros(6))
    return q


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--urdf", default="outputs/vision/ur10_with_camera_mujoco.urdf")
    parser.add_argument("--host", default="192.168.1.3")
    parser.add_argument("--watch", action="store_true",
                        help="open the MuJoCo viewer and follow the live state")
    parser.add_argument("--hz", type=float, default=60.0)
    parser.add_argument("--check", default=None,
                        help="cross-check this pose plan's camera clearance")
    parser.add_argument("--margin", type=float, default=0.15,
                        help="geom margin used as the distance probe (m)")
    parser.add_argument("--threshold-mm", type=float, default=40.0)
    parser.add_argument("--output", default="outputs/vision/mujoco-clearance-check.json")
    parser.add_argument("--q", default=None, help="explicit 6 joint angles (rad)")
    args = parser.parse_args()

    urdf = args.urdf if os.path.isabs(args.urdf) else os.path.join(root, args.urdf)
    scene = MujocoScene(urdf)
    print("MuJoCo 模型：ngeom=%d（相机 %d 个）njnt=%d" %
          (scene.model.ngeom, len(scene.camera_geoms), scene.model.njnt))

    if args.check:
        plan_path = args.check if os.path.isabs(args.check) \
            else os.path.join(root, args.check)
        with open(plan_path, encoding="utf-8") as stream:
            plan = json.load(stream)
        scene.set_margin(args.margin)
        document = {"schema_version": 1, "plan": os.path.relpath(plan_path, root),
                    "margin_m": args.margin, "threshold_mm": args.threshold_mm,
                    "engine": "MuJoCo %s geom_margin probe" % mujoco.__version__,
                    "poses": []}
        print()
        print("%-6s %12s  %s" % ("slot", "间隙(mm)", "最近"))
        worst = (None, None)
        for entry in plan["plan"]:
            selection = entry.get("roll_selection") or {}
            if selection.get("q_rad") is not None:
                q = np.asarray(selection["q_rad"], dtype=float)
                source = "plan roll_selection.q_rad"
            else:
                q = pose_q(entry["target_tcp_pose_m_rad"])
                source = "re-solved IK (may be another branch)"
            scene.set_q(q)
            gap, first, second = scene.closest_camera_gap()
            record = {"slot": entry["slot"], "shell": entry["shell"],
                      "gap_m": gap, "camera_geom": scene.name(first),
                      "other_geom": scene.name(second), "q_source": source,
                      "below_threshold": (gap is None or
                                          gap * 1000.0 < args.threshold_mm)}
            document["poses"].append(record)
            if gap is not None and (worst[0] is None or gap < worst[0]):
                worst = (gap, entry["slot"])
            print("%-6s %12s  %s%s" % (
                entry["slot"],
                "-" if gap is None else "%+.1f" % (gap * 1000.0),
                "%s vs %s" % (scene.name(first), scene.name(second)),
                "   <<< 低于门限" if record["below_threshold"] else ""))
        document["worst"] = {"gap_m": worst[0], "slot": worst[1]}
        document["violations"] = [record["slot"] for record in document["poses"]
                                  if record["below_threshold"]]
        print()
        print("MuJoCo 判定：最差 %s mm @ slot %s；低于 %.0f mm 的位姿 %d 个 %s" % (
            "-" if worst[0] is None else "%.1f" % (worst[0] * 1000.0), worst[1],
            args.threshold_mm, len(document["violations"]),
            document["violations"]))
        output = args.output if os.path.isabs(args.output) \
            else os.path.join(root, args.output)
        os.makedirs(os.path.dirname(output), exist_ok=True)
        with open(output, "w", encoding="utf-8") as stream:
            json.dump(document, stream, indent=2, ensure_ascii=False)
            stream.write("\n")
        print("OUTPUT:", output)
        return 0

    if args.q:
        q = np.asarray([float(x) for x in args.q.split(",")], dtype=float)
        scene.set_q(q)
        gap, first, second = scene.closest_camera_gap()
        print("给定关节角下的相机-机械臂间隙：%s mm (%s vs %s)" %
              ("-" if gap is None else "%.1f" % (gap * 1000.0),
               scene.name(first), scene.name(second)))
        if not args.watch:
            return 0

    if not args.watch:
        q, tcp = read_q(args.host)
        scene.set_q(q)
        gap, first, second = scene.closest_camera_gap()
        print("当前 TCP %s" % np.round(tcp[:3], 4).tolist())
        print("相机-机械臂间隙：%s mm (%s vs %s)" %
              ("-" if gap is None else "%.1f" % (gap * 1000.0),
               scene.name(first), scene.name(second)))
        print("加 --watch 打开 MuJoCo 查看器实时跟随。")
        return 0

    scene.set_margin(args.margin)
    print("打开 MuJoCo 查看器，按 %.0f Hz 跟随 30003 的 q_actual（Ctrl-C 退出）" % args.hz)
    with mujoco.viewer.launch_passive(scene.model, scene.data) as viewer:
        while viewer.is_running():
            q, _tcp = read_q(args.host)
            scene.set_q(q)
            gap, first, second = scene.closest_camera_gap()
            viewer.sync()
            print("  %s  相机间隙 %s mm (%s vs %s)" % (
                dt.datetime.now().strftime("%H:%M:%S"),
                "-" if gap is None else "%+.1f" % (gap * 1000.0),
                scene.name(first), scene.name(second)), flush=True)
            time.sleep(1.0 / max(args.hz, 1.0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
