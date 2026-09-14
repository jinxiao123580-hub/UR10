#!/usr/bin/env python3
"""Experimental IK motion probe. Execution is disabled after a protective stop."""
import argparse
import csv
import datetime
import os
import socket
import struct
import threading
import time
import numpy as np
import pinocchio as pin

from ur_pose_ik import UR10IK
from test_speedl import monitor


def build_servoj(target, duration=1.0, period=0.008):
    values = ", ".join("%.12f" % value for value in target)
    return """def guarded_ik_move():
  q0 = get_actual_joint_positions()
  qt = [%s]
  elapsed = 0.0
  while (elapsed < %.6f):
    u = elapsed / %.6f
    s = 3.0*u*u - 2.0*u*u*u
    q = [q0[0]+s*(qt[0]-q0[0]), q0[1]+s*(qt[1]-q0[1]), q0[2]+s*(qt[2]-q0[2]), q0[3]+s*(qt[3]-q0[3]), q0[4]+s*(qt[4]-q0[4]), q0[5]+s*(qt[5]-q0[5])]
    servoj(q, t=%.6f, lookahead_time=0.1, gain=100)
    elapsed = elapsed + %.6f
  end
  servoj(qt, t=%.6f, lookahead_time=0.1, gain=100)
  stopj(1.0)
end
guarded_ik_move()
""" % (values, duration, duration, period, period, period)


def read_realtime(host, timeout=3.0):
    with socket.create_connection((host, 30003), timeout=2.0) as sock:
        sock.settimeout(0.5)
        buffer = bytearray(); deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                buffer.extend(sock.recv(65536))
            except socket.timeout:
                continue
            while len(buffer) >= 4:
                size = struct.unpack_from(">I", buffer, 0)[0]
                if size < 540 or size > 4096:
                    raise RuntimeError("unexpected 30003 frame size %d" % size)
                if len(buffer) < size:
                    break
                frame = bytes(buffer[:size]); del buffer[:size]
                return (np.array(struct.unpack_from(">6d", frame, 252)),
                        np.array(struct.unpack_from(">6d", frame, 444)))
    raise RuntimeError("30003 did not provide a complete frame")


def robot_pose(values):
    return pin.SE3(pin.exp3(values[3:]), values[:3])


BASE_FROM_URDF_ROOT = pin.SE3(pin.exp3(np.array([0.0, 0.0, np.pi])), np.zeros(3))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.1.3")
    ap.add_argument("--axis", type=int, choices=range(3), default=0)
    ap.add_argument("--distance", type=float, default=0.002)
    ap.add_argument("--execute", action="store_true")
    args = ap.parse_args()
    if args.execute:
        ap.error("真机 IK 执行已禁用：首次 2 mm 测试触发过保护性停止")
    if not 0 < abs(args.distance) <= 0.002:
        ap.error("distance magnitude must be in (0, 0.002] m")

    q, tcp = read_realtime(args.host)
    ik = UR10IK()
    fk_root = ik.pose(q)
    fk = BASE_FROM_URDF_ROOT * fk_root
    measured = robot_pose(tcp)
    agreement = pin.log6(fk.inverse() * measured).vector
    position_agreement = float(np.linalg.norm(agreement[:3]))
    rotation_agreement = float(np.linalg.norm(agreement[3:]))
    print("q_actual", np.round(q, 6))
    print("tcp_actual", np.round(tcp, 6))
    print("FK agreement: position=%.6fm rotation=%.6frad" %
          (position_agreement, rotation_agreement))
    if position_agreement > 0.01 or rotation_agreement > 0.02:
        raise RuntimeError("URDF FK and robot TCP disagree; refuse IK target")

    target_base = measured.copy()
    target_base.translation[args.axis] += args.distance
    target_root = BASE_FROM_URDF_ROOT.inverse() * target_base
    solved, pe, re, jump, reasons, result = ik.solve_checked(target_root, q)
    print("target_tcp", np.round(np.r_[target_base.translation,
                                       pin.log3(target_base.rotation)], 6))
    print("q_target", np.round(solved, 7))
    print("IK: position_error=%.9fm rotation_error=%.9frad max_joint_step=%.6frad nfev=%d" %
          (pe, re, jump, result.nfev))
    if reasons:
        raise RuntimeError("IK target rejected: " + ", ".join(reasons))
    if jump > 0.05:
        raise RuntimeError("2 mm target unexpectedly requires >0.05 rad joint step")
    print("DRY RUN PASS: target passed all gates")
    if not args.execute:
        print("No command was sent")
        return
    if input("确认空中 2 mm 路径无障碍、人在旁且手放急停？输入 YES: ") != "YES":
        print("已取消")
        return
    stop = threading.Event(); rows = []; errors = []
    thread = threading.Thread(target=monitor, args=(args.host, stop, rows, errors), daemon=True)
    thread.start(); deadline = time.monotonic() + 2.0
    while len(rows) < 20 and not errors and time.monotonic() < deadline:
        time.sleep(0.05)
    if errors or len(rows) < 20:
        stop.set(); thread.join(1.0)
        raise RuntimeError("30003 运动前门禁失败: %s frames=%d" % (errors, len(rows)))
    if np.max(np.abs(read_realtime(args.host)[0] - q)) > 0.01:
        stop.set(); thread.join(1.0)
        raise RuntimeError("确认期间关节位置变化，拒绝使用过期 IK")
    with socket.create_connection((args.host, 30002), timeout=2.0) as sock:
        sock.sendall(build_servoj(solved).encode("ascii"))
    time.sleep(2.0); stop.set(); thread.join(2.0)
    if errors or not rows:
        raise RuntimeError("30003 运动监测失败: %s" % errors)
    final_tcp = np.mean([row[1] for row in rows[-20:]], axis=0)
    final_error = float(np.linalg.norm(final_tcp[:3] - target_base.translation))
    actual_delta = float(final_tcp[args.axis] - tcp[args.axis])
    delta_xyz = final_tcp[:3] - tcp[:3]
    out_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                           "outputs", "ik")
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "ik-move-%s.csv" %
                        datetime.datetime.now().strftime("%Y%m%d-%H%M%S"))
    with open(path, "w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["time_s", "x", "y", "z", "rx", "ry", "rz",
                         "vx", "vy", "vz", "wx", "wy", "wz"])
        origin = rows[0][0]
        for timestamp, pose, velocity in rows:
            writer.writerow([timestamp-origin, *pose, *velocity])
    print("delta_xyz", np.round(delta_xyz, 7), "CSV", path)
    print("actual_delta=%.6fm target_position_error=%.6fm frames=%d" %
          (actual_delta, final_error, len(rows)))
    if abs(actual_delta - args.distance) > 0.0005 or final_error > 0.001:
        raise RuntimeError("真机 IK 位姿验收失败")
    print("PASS: guarded IK servoj move")


if __name__ == "__main__":
    main()
