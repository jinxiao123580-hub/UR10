#!/usr/bin/env python3
"""Manual fixed-point calibration of one Robotiq fingertip TCP.

The operator uses freedrive/teach-pendant jogging to place ONE chosen
fingertip on the same rigid reference point.  This script only reads actual
UR 30003 state; it never commands the robot or gripper.
"""
import argparse
import json
import math
import os
import time

import numpy as np

from record_ur_trajectory import RealtimeReader


def rotation_matrix(rotvec):
    angle = np.linalg.norm(rotvec)
    if angle < 1e-12:
        return np.eye(3)
    axis = np.asarray(rotvec, dtype=float) / angle
    cross = np.array([[0., -axis[2], axis[1]], [axis[2], 0., -axis[0]],
                      [-axis[1], axis[0], 0.]])
    return np.eye(3) + math.sin(angle) * cross + (1. - math.cos(angle)) * cross @ cross


def rotation_distance_deg(a, b):
    relative = rotation_matrix(a).T @ rotation_matrix(b)
    cosine = np.clip((np.trace(relative) - 1.) / 2., -1., 1.)
    return float(np.degrees(np.arccos(cosine)))


def mean_rotation(rotvecs):
    matrices = np.asarray([rotation_matrix(value) for value in rotvecs])
    u, _, vt = np.linalg.svd(np.sum(matrices, axis=0))
    result = u @ vt
    if np.linalg.det(result) < 0:
        u[:, -1] *= -1
        result = u @ vt
    angle = math.acos(float(np.clip((np.trace(result) - 1.) / 2., -1., 1.)))
    if angle < 1e-12:
        return np.zeros(3)
    vector = np.array([result[2, 1] - result[1, 2], result[0, 2] - result[2, 0],
                       result[1, 0] - result[0, 1]])
    return vector * angle / (2. * math.sin(angle))


def capture_static(reader, duration):
    started = time.monotonic()
    rows = []
    while time.monotonic() - started < duration:
        q, tcp, _ = reader.read()
        rows.append((q, tcp))
    if len(rows) < 20:
        raise RuntimeError("UR state frames too few: %d" % len(rows))
    poses = np.asarray([tcp for _, tcp in rows])
    position = poses[:, :3]
    reference_position = np.mean(position, axis=0)
    reference_rotation = mean_rotation(poses[:, 3:])
    return {
        "frame_count": len(rows),
        "q_actual_mean_rad": np.mean([q for q, _ in rows], axis=0).tolist(),
        "tcp_pose": [*reference_position.tolist(), *reference_rotation.tolist()],
        "max_motion_mm": float(np.max(np.linalg.norm(position - reference_position, axis=1)) * 1000.),
        "max_motion_deg": max(rotation_distance_deg(reference_rotation, value)
                              for value in poses[:, 3:]),
    }


def solve(samples):
    matrices = []
    positions = []
    for sample in samples:
        pose = sample["tcp_pose"]
        matrices.append(rotation_matrix(pose[3:]))
        positions.append(np.asarray(pose[:3]))
    a, b = [], []
    for i in range(len(samples)):
        for j in range(i + 1, len(samples)):
            a.append(matrices[i] - matrices[j])
            b.append(positions[j] - positions[i])
    a, b = np.vstack(a), np.hstack(b)
    offset, _, rank, singular = np.linalg.lstsq(a, b, rcond=None)
    fixed_points = np.asarray([r @ offset + p for r, p in zip(matrices, positions)])
    fixed_mean = np.mean(fixed_points, axis=0)
    errors = np.linalg.norm(fixed_points - fixed_mean, axis=1) * 1000.
    return {"tool0_to_fingertip_m": offset.tolist(), "fixed_point_base_m": fixed_mean.tolist(),
            "rank": int(rank), "condition_number": float(singular[0] / singular[-1]),
            "per_sample_error_mm": errors.tolist(),
            "rms_error_mm": float(np.sqrt(np.mean(errors ** 2))),
            "max_error_mm": float(np.max(errors))}


def atomic_write(path, document):
    temporary = path + ".tmp"
    with open(temporary, "w", encoding="utf-8") as stream:
        json.dump(document, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    os.replace(temporary, path)


def evaluate(document, holdout):
    accepted = [sample for sample in document["samples"] if sample["accepted"]]
    if len(accepted) < holdout + 4:
        return None
    train, test = accepted[:-holdout], accepted[-holdout:]
    result = solve(train)
    offset = np.asarray(result["tool0_to_fingertip_m"])
    reference = np.asarray(result["fixed_point_base_m"])
    errors = []
    for sample in test:
        pose = sample["tcp_pose"]
        point = rotation_matrix(pose[3:]) @ offset + np.asarray(pose[:3])
        errors.append(float(np.linalg.norm(point - reference) * 1000.))
    result["training_sample_ids"] = [sample["index"] for sample in train]
    result["holdout_sample_ids"] = [sample["index"] for sample in test]
    result["holdout_rms_error_mm"] = float(np.sqrt(np.mean(np.square(errors))))
    result["holdout_max_error_mm"] = float(np.max(errors))
    result["passed_numeric_gate"] = bool(result["condition_number"] <= 120. and
                                         result["holdout_max_error_mm"] <= 2.0)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=12)
    parser.add_argument("--holdout", type=int, default=3)
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--max-motion-mm", type=float, default=0.5)
    parser.add_argument("--max-motion-deg", type=float, default=0.2)
    parser.add_argument("--tcp-name", default="left_fingertip_closed")
    parser.add_argument("--output")
    args = parser.parse_args()
    if args.samples < args.holdout + 4:
        parser.error("samples must be >= holdout + 4")
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    stamp = time.strftime("%Y%m%d-%H%M%S")
    path = os.path.abspath(args.output or os.path.join(
        root, "outputs", "tcp_calibration", "fingertip-%s.json" % stamp))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    document = {"schema_version": 1, "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                "method": "fixed-point pivot, least squares", "tcp_name": args.tcp_name,
                "gripper_state_requirement": "Keep the gripper closed and do not change jaws/fingertips.",
                "reference_requirement": "Same rigid physical point for every sample.",
                "static_limits": {"motion_mm": args.max_motion_mm, "motion_deg": args.max_motion_deg},
                "samples": []}
    print("固定尖点 TCP 标定（纯读取）。选择左指尖；夹爪必须保持闭合。")
    print("每次让同一指尖接触同一尖点，换明显不同姿态后按回车。最后 %d 个有效样本留作独立验证。" % args.holdout)
    reader = RealtimeReader("192.168.1.3")
    try:
        while sum(item["accepted"] for item in document["samples"]) < args.samples:
            wanted = sum(item["accepted"] for item in document["samples"]) + 1
            input("\n对准固定尖点并停稳后按回车，采集有效样本 %d/%d：" % (wanted, args.samples))
            sample = capture_static(reader, args.duration)
            sample["index"] = len(document["samples"]) + 1
            sample["accepted"] = (sample["max_motion_mm"] <= args.max_motion_mm and
                                  sample["max_motion_deg"] <= args.max_motion_deg)
            if not sample["accepted"]:
                sample["rejection_reason"] = "not stationary during capture"
            document["samples"].append(sample)
            atomic_write(path, document)
            print("%s：%d 帧，运动 %.3f mm / %.3f deg，已落盘 %s" % (
                "接受" if sample["accepted"] else "拒绝", sample["frame_count"],
                sample["max_motion_mm"], sample["max_motion_deg"], path))
    finally:
        reader.close()
    document["result"] = evaluate(document, args.holdout)
    atomic_write(path, document)
    print("\n结果：", json.dumps(document["result"], indent=2, ensure_ascii=False))
    print("提示：通过数值门禁也只授权该夹爪状态和该选定指尖；尚未自动写入控制器 TCP。")


if __name__ == "__main__":
    main()
