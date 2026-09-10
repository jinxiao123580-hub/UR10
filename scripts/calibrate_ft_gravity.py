#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""主动多姿态标定 ATI 末端工具的质量、重心、安装旋转和六维零偏。"""
import argparse
from collections import deque
import math
import os
import sys
import threading
import time

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ur_arm import Arm  # noqa: E402

GRAVITY = 9.80665


def skew(v):
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=float)


def rotvec_to_matrix(v):
    v = np.asarray(v, dtype=float)
    angle = np.linalg.norm(v)
    if angle < 1e-12:
        return np.eye(3)
    k = v / angle
    kx = skew(k)
    return np.eye(3) + math.sin(angle) * kx + (1 - math.cos(angle)) * (kx @ kx)


def matrix_to_rotvec(r):
    angle = math.acos(float(np.clip((np.trace(r) - 1) / 2, -1, 1)))
    if angle < 1e-10:
        return np.zeros(3)
    if abs(math.pi - angle) < 1e-5:
        values, vectors = np.linalg.eig(r)
        axis = np.real(vectors[:, np.argmin(np.abs(values - 1))])
        axis /= np.linalg.norm(axis)
        return axis * angle
    axis = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0],
                     r[1, 0] - r[0, 1]]) / (2 * math.sin(angle))
    return axis * angle


def fit_gravity(samples):
    if len(samples) < 8:
        raise ValueError("至少需要 8 个有效姿态")
    gravity_base = np.array([0.0, 0.0, -GRAVITY])
    gravity_tool = np.array([s["rotation_base_tool"].T @ gravity_base
                             for s in samples])
    force = np.array([s["wrench"][:3] for s in samples])
    torque = np.array([s["wrench"][3:] for s in samples])

    # F_sensor = A * g_tool + b_f, where A = mass * R_sensor_tool.
    design = np.hstack([gravity_tool, np.ones((len(samples), 1))])
    coeff, _, rank, singular = np.linalg.lstsq(design, force, rcond=None)
    if rank < 4:
        raise ValueError("姿态覆盖不足，力模型不满秩")
    a = coeff[:3].T
    force_bias = coeff[3]
    u, _, vt = np.linalg.svd(a)
    correction = np.diag([1.0, 1.0, np.linalg.det(u @ vt)])
    rotation_sensor_tool = u @ correction @ vt
    mass = float(np.trace(rotation_sensor_tool.T @ a) / 3.0)
    if not 0.05 < mass < 10.0:
        raise ValueError("质量拟合值 %.3f kg 不可信，检查 TF/符号/外力" % mass)

    gravity_force = np.array([mass * rotation_sensor_tool @ g
                              for g in gravity_tool])
    force_pred = gravity_force + force_bias

    # T_sensor = r_com_sensor x F_gravity_sensor + b_t.
    torque_design = np.vstack([
        np.hstack([-skew(f), np.eye(3)]) for f in gravity_force
    ])
    torque_coeff, _, torque_rank, _ = np.linalg.lstsq(
        torque_design, torque.reshape(-1), rcond=None)
    if torque_rank < 6:
        raise ValueError("姿态覆盖不足，重心模型不满秩")
    com_sensor = torque_coeff[:3]
    torque_bias = torque_coeff[3:]
    torque_pred = np.array([np.cross(com_sensor, f) + torque_bias
                            for f in gravity_force])
    force_error = force - force_pred
    torque_error = torque - torque_pred
    spread = np.linalg.svd(gravity_tool - gravity_tool.mean(axis=0),
                           compute_uv=False)
    if spread[-1] < 0.25:
        raise ValueError("姿态覆盖过小，请增大 --angle-deg 或增加姿态")
    return {
        "mass_kg": mass,
        "com_sensor_m": com_sensor.tolist(),
        "force_bias_n": force_bias.tolist(),
        "torque_bias_nm": torque_bias.tolist(),
        "rotation_sensor_from_tool": rotation_sensor_tool.tolist(),
        "force_rms_n": float(np.sqrt(np.mean(force_error ** 2))),
        "force_max_n": float(np.max(np.abs(force_error))),
        "torque_rms_nm": float(np.sqrt(np.mean(torque_error ** 2))),
        "torque_max_nm": float(np.max(np.abs(torque_error))),
        "pose_spread_singular": spread.tolist(),
        "design_condition": float(singular[0] / singular[-1]),
        "sample_count": len(samples),
    }

class WrenchCollector:
    def __init__(self, topic):
        import rclpy
        from geometry_msgs.msg import WrenchStamped
        from rclpy.node import Node
        if not rclpy.ok():
            rclpy.init()
        self.node = Node("ft_gravity_calibrator")
        self.rows = deque(maxlen=10000)
        self.lock = threading.Lock()

        def callback(msg):
            w = msg.wrench
            row = [w.force.x, w.force.y, w.force.z,
                   w.torque.x, w.torque.y, w.torque.z]
            with self.lock:
                self.rows.append((time.monotonic(), row))

        self.sub = self.node.create_subscription(WrenchStamped, topic, callback, 200)
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.thread.start()

    def sample(self, duration, min_messages):
        start = time.monotonic()
        time.sleep(duration)
        with self.lock:
            block = np.array([row for stamp, row in self.rows if stamp >= start])
        if len(block) < min_messages:
            raise RuntimeError("采样 %.1fs 仅收到 %d 条数据" % (duration, len(block)))
        return np.mean(block, axis=0), np.std(block, axis=0), len(block)

    def close(self):
        import rclpy
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.thread.join(timeout=2)


def calibration_offsets(angle_deg):
    a = math.radians(angle_deg)
    b = math.radians(angle_deg * 0.7)
    return [(0, 0, 0), (a, 0, 0), (-a, 0, 0), (0, a, 0),
            (0, -a, 0), (b, b, 0), (b, -b, 0), (-b, b, 0),
            (-b, -b, 0), (a, 0, b), (-a, 0, -b), (0, a, -b)]


def make_targets(start_pose, angle_deg):
    start_rotation = rotvec_to_matrix(start_pose[3:])
    targets = []
    for rx, ry, rz in calibration_offsets(angle_deg):
        delta = rotvec_to_matrix([rx, ry, rz])
        target_rotation = start_rotation @ delta
        targets.append(list(start_pose[:3]) + matrix_to_rotvec(target_rotation).tolist())
    return targets


def rotation_error_deg(a, b):
    return math.degrees(math.acos(float(np.clip(
        (np.trace(a.T @ b) - 1) / 2, -1, 1))))


def move_and_wait(arm, target, acceleration, speed, settle, timeout=60.0):
    """移动并用旋转矩阵判断到位，避免轴角 pi 分支的等价表示误判。"""
    arm.movel(target, a=acceleration, v=speed, wait=False)
    target_rotation = rotvec_to_matrix(target[3:])
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        time.sleep(0.25)
        actual = arm.get_tcp_pose()
        if not actual:
            continue
        position_error = np.linalg.norm(np.asarray(actual[:3]) - target[:3])
        angle_error = rotation_error_deg(target_rotation,
                                         rotvec_to_matrix(actual[3:]))
        if position_error < 0.0015 and angle_error < 0.5:
            time.sleep(settle)
            return actual
    raise RuntimeError("运动 60 秒仍未到位")


def self_test():
    rng = np.random.default_rng(42)
    true_mass = 2.3
    true_com = np.array([0.02, -0.012, 0.09])
    true_bf = np.array([0.4, -0.6, 0.2])
    true_bt = np.array([0.02, -0.03, 0.01])
    rst = rotvec_to_matrix([0.2, -0.1, 0.12])
    rows = []
    for _ in range(30):
        rbt = rotvec_to_matrix(rng.normal(size=3))
        fg = true_mass * rst @ (rbt.T @ [0, 0, -GRAVITY])
        wrench = np.r_[fg + true_bf, np.cross(true_com, fg) + true_bt]
        wrench += rng.normal(0, [0.01] * 3 + [0.001] * 3)
        rows.append({"rotation_base_tool": rbt, "wrench": wrench})
    result = fit_gravity(rows)
    assert abs(result["mass_kg"] - true_mass) < 0.01
    assert np.max(np.abs(np.array(result["com_sensor_m"]) - true_com)) < 0.001
    print("自测通过: mass=%.4fkg, CoM=%s" %
          (result["mass_kg"], np.round(result["com_sensor_m"], 5)))


def run(args):
    arm = Arm()
    start = arm.get_tcp_pose()
    if not start:
        raise RuntimeError("读不到 UR10 TCP 位姿")
    targets = make_targets(start, args.angle_deg)
    gravity_base = np.array([0.0, 0.0, -GRAVITY])
    preview_gravity = np.array([
        rotvec_to_matrix(target[3:]).T @ gravity_base for target in targets
    ])
    preview_condition = float(np.linalg.cond(
        np.hstack([preview_gravity, np.ones((len(targets), 1))])))
    print("起始 TCP: [%s]" % ", ".join("%.5f" % x for x in start))
    print("轨迹: 位置保持不变，%d 个姿态，工具坐标轴最大转动 %.1f deg"
          % (len(targets), args.angle_deg))
    print("预计设计矩阵条件数: %.1f（要求 <= %.1f）" %
          (preview_condition, args.max_condition))
    for i, pose in enumerate(targets, 1):
        print("  %02d rotvec=[% .4f % .4f % .4f]" % (i, *pose[3:]))
    if not args.execute:
        print("\n仅预览，未发送运动。确认后加 --execute。")
        return 0
    if preview_condition > args.max_condition:
        raise RuntimeError("姿态覆盖不足：条件数 %.1f > %.1f，请增大 --angle-deg"
                           % (preview_condition, args.max_condition))
    answer = input("确认末端周围无障碍、无外部接触，人在机械臂旁且手放急停？输入 YES: ")
    if answer != "YES":
        raise RuntimeError("用户未确认，已取消")

    arm._ensure_ros()
    if arm._node.count_subscribers("/ur_link/urscript") < 1:
        raise RuntimeError("/ur_link/urscript 没有订阅者，请先启动 ur_command_node")

    collector = WrenchCollector(args.topic)
    samples = []
    moved = False
    try:
        time.sleep(1.0)
        try:
            pre_mean, _, pre_count = collector.sample(0.5, max(20, args.min_messages // 2))
        except RuntimeError as exc:
            raise RuntimeError(
                "运动前力数据门禁失败（%s）；请先启动 ati_netft_node.py" % exc)
        print("力数据门禁通过: %d 点/0.5s, F=[% .3f % .3f % .3f]N" %
              (pre_count, *pre_mean[:3]))
        for i, target in enumerate(targets, 1):
            print("[%02d/%02d] 移动..." % (i, len(targets)))
            moved = True
            actual = move_and_wait(arm, target, args.acceleration, args.speed,
                                   args.settle)
            target_r = rotvec_to_matrix(target[3:])
            actual_r = rotvec_to_matrix(actual[3:])
            angle_error = rotation_error_deg(target_r, actual_r)
            position_error = np.linalg.norm(np.asarray(actual[:3]) - target[:3])
            if position_error > 0.003 or angle_error > 1.0:
                raise RuntimeError("姿态 %d 未到位: %.1fmm / %.2fdeg" %
                                   (i, position_error * 1000, angle_error))
            mean, std, count = collector.sample(args.duration, args.min_messages)
            samples.append({"rotation_base_tool": rotvec_to_matrix(actual[3:]),
                            "wrench": mean, "std": std, "count": count,
                            "tcp_pose": list(actual)})
            print("  %d 点, F=[% .3f % .3f % .3f]N, max_std=%.3fN"
                  % (count, *mean[:3], np.max(std[:3])))
        result = fit_gravity(samples)
    finally:
        if moved:
            print("返回起始姿态...")
            try:
                move_and_wait(arm, start, args.acceleration, args.speed, 0.4)
            except Exception as exc:
                print("⚠ 返回起始姿态失败: %s" % exc, file=sys.stderr)
        collector.close()

    output = os.path.abspath(os.path.expanduser(args.output))
    os.makedirs(os.path.dirname(output), exist_ok=True)
    failures = []
    if result["design_condition"] > args.max_condition:
        failures.append("design_condition %.1f > %.1f" %
                        (result["design_condition"], args.max_condition))
    if result["force_rms_n"] > args.max_force_rms:
        failures.append("force_rms %.3fN > %.3fN" %
                        (result["force_rms_n"], args.max_force_rms))
    if result["torque_rms_nm"] > args.max_torque_rms:
        failures.append("torque_rms %.4fNm > %.4fNm" %
                        (result["torque_rms_nm"], args.max_torque_rms))
    valid = not failures
    serializable_samples = [{
        "tcp_pose": [float(x) for x in sample["tcp_pose"]],
        "wrench_mean": [float(x) for x in sample["wrench"]],
        "wrench_std": [float(x) for x in sample["std"]],
        "message_count": int(sample["count"]),
    } for sample in samples]
    document = {"schema_version": 1, "valid": valid,
                "frames": {"base": "base", "tool": "tool0", "sensor": "ft_sensor"},
                "gravity_m_s2": GRAVITY, "calibration": result,
                "validation_failures": failures,
                "samples": serializable_samples,
                "note": "ATI hardware Bias must remain zero."}
    with open(output, "w", encoding="utf-8") as stream:
        yaml.safe_dump(document, stream, sort_keys=False, allow_unicode=True)
    print("拟合结果: mass=%.4fkg, CoM(sensor)=%s m" %
          (result["mass_kg"], np.round(result["com_sensor_m"], 6)))
    print("残差: force RMS=%.4fN max=%.4fN; torque RMS=%.5fNm max=%.5fNm" %
          (result["force_rms_n"], result["force_max_n"],
           result["torque_rms_nm"], result["torque_max_nm"]))
    print("输出: %s" % output)
    if not valid:
        raise RuntimeError("标定质量门禁失败: " + "; ".join(failures))
    print("✔ 标定质量门禁通过")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="确认后实际移动机械臂")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--topic", default="/ft_sensor/wrench")
    parser.add_argument("--angle-deg", type=float, default=40.0)
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--min-messages", type=int, default=100)
    parser.add_argument("--settle", type=float, default=1.0)
    parser.add_argument("--speed", type=float, default=0.05)
    parser.add_argument("--acceleration", type=float, default=0.2)
    parser.add_argument("--max-condition", type=float, default=120.0)
    parser.add_argument("--max-force-rms", type=float, default=1.0)
    parser.add_argument("--max-torque-rms", type=float, default=0.1)
    parser.add_argument("--output", default="config/ft_gravity_calibration.yaml")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return 0
    return run(args)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (KeyboardInterrupt, EOFError):
        print("\n已中止")
        sys.exit(130)
    except Exception as exc:
        print("✘ %s" % exc, file=sys.stderr)
        sys.exit(2)
