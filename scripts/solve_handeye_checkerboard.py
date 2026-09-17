#!/usr/bin/env python3
"""Solve and independently validate an eye-in-hand checkerboard calibration."""
import argparse
import datetime as dt
import json
import os

import cv2
import numpy as np


METHODS = {
    "tsai": cv2.CALIB_HAND_EYE_TSAI,
    "park": cv2.CALIB_HAND_EYE_PARK,
    "horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
    "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def rt(rvec, translation):
    rotation, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray(translation, dtype=np.float64)
    return transform


def rotation_angle_deg(rotation):
    cosine = np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def transform_distance(a, b):
    return (float(np.linalg.norm(a[:3, 3] - b[:3, 3]) * 1000.0),
            rotation_angle_deg(a[:3, :3].T @ b[:3, :3]))


def average_rotation(rotations):
    u, _, vt = np.linalg.svd(np.sum(rotations, axis=0))
    result = u @ vt
    if np.linalg.det(result) < 0:
        u[:, -1] *= -1
        result = u @ vt
    return result


def average_transform(transforms):
    result = np.eye(4)
    result[:3, :3] = average_rotation(np.asarray([x[:3, :3] for x in transforms]))
    result[:3, 3] = np.mean([x[:3, 3] for x in transforms], axis=0)
    return result


def metrics(transforms, reference):
    errors = [transform_distance(reference, value) for value in transforms]
    translation = np.asarray([value[0] for value in errors])
    rotation = np.asarray([value[1] for value in errors])
    return {
        "count": len(errors),
        "translation_mm": {
            "rms": float(np.sqrt(np.mean(translation ** 2))),
            "median": float(np.median(translation)),
            "max": float(np.max(translation)),
            "per_sample": translation.tolist(),
        },
        "rotation_deg": {
            "rms": float(np.sqrt(np.mean(rotation ** 2))),
            "median": float(np.median(rotation)),
            "max": float(np.max(rotation)),
            "per_sample": rotation.tolist(),
        },
    }


def sample_transforms(sample):
    tcp = sample["robot"]["base_to_tool0_tcp_mean"]
    base_to_gripper = rt(tcp[3:], tcp[:3])
    target = sample["target_to_camera"]
    camera_from_target = rt(target["rvec_rad"], target["translation_m"])
    return base_to_gripper, camera_from_target


def deduplicate(samples, translation_mm, rotation_deg):
    kept = []
    removed = []
    for sample in samples:
        pose, _ = sample_transforms(sample)
        matches = [(index, transform_distance(old_pose, pose))
                   for index, (old, old_pose) in enumerate(kept)
                   if transform_distance(old_pose, pose)[0] <= translation_mm
                   and transform_distance(old_pose, pose)[1] <= rotation_deg]
        if not matches:
            kept.append((sample, pose))
            continue
        index, distance = matches[0]
        old, old_pose = kept[index]
        old_rms = old["target_to_camera"]["reprojection_rms_px"]
        new_rms = sample["target_to_camera"]["reprojection_rms_px"]
        if new_rms < old_rms:
            kept[index] = (sample, pose)
            removed.append({"sample_id": old["sample_id"],
                            "kept_sample_id": sample["sample_id"],
                            "translation_mm": distance[0],
                            "rotation_deg": distance[1],
                            "reason": "duplicate pose; higher reprojection RMS"})
        else:
            removed.append({"sample_id": sample["sample_id"],
                            "kept_sample_id": old["sample_id"],
                            "translation_mm": distance[0],
                            "rotation_deg": distance[1],
                            "reason": "duplicate pose; higher reprojection RMS"})
    return [value[0] for value in kept], removed


def solve(method, samples):
    base_from_gripper = []
    camera_from_target = []
    for sample in samples:
        bg, ct = sample_transforms(sample)
        base_from_gripper.append(bg)
        camera_from_target.append(ct)
    rotation, translation = cv2.calibrateHandEye(
        [x[:3, :3] for x in base_from_gripper],
        [x[:3, 3] for x in base_from_gripper],
        [x[:3, :3] for x in camera_from_target],
        [x[:3, 3] for x in camera_from_target], method=method)
    gripper_from_camera = np.eye(4)
    gripper_from_camera[:3, :3] = rotation
    gripper_from_camera[:3, 3] = np.asarray(translation).reshape(3)
    return gripper_from_camera


def closure(samples, gripper_from_camera):
    values = []
    for sample in samples:
        base_from_gripper, camera_from_target = sample_transforms(sample)
        values.append(base_from_gripper @ gripper_from_camera @ camera_from_target)
    return values


def matrix_json(value):
    rvec, _ = cv2.Rodrigues(value[:3, :3])
    return {"matrix_4x4": value.tolist(),
            "translation_m": value[:3, 3].tolist(),
            "rvec_rad": rvec.reshape(3).tolist()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", default="outputs/handeye/manual-eye-in-hand.json")
    parser.add_argument("--holdout", type=int, default=5)
    parser.add_argument("--duplicate-mm", type=float, default=0.5)
    parser.add_argument("--duplicate-deg", type=float, default=0.2)
    parser.add_argument("--output", default="outputs/handeye/solution-candidate.json")
    args = parser.parse_args()
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    dataset_path = args.dataset if os.path.isabs(args.dataset) else os.path.join(root, args.dataset)
    output_path = args.output if os.path.isabs(args.output) else os.path.join(root, args.output)
    with open(dataset_path, encoding="utf-8") as stream:
        dataset = json.load(stream)
    accepted = [sample for sample in dataset["samples"] if sample.get("accepted")]
    unique, duplicates = deduplicate(accepted, args.duplicate_mm, args.duplicate_deg)
    if len(unique) < args.holdout + 3:
        raise RuntimeError("not enough unique samples after deduplication")
    train = unique[:-args.holdout]
    holdout = unique[-args.holdout:]
    document = {
        "schema_version": 1,
        "created_at": dt.datetime.now(dt.timezone.utc).astimezone().isoformat(),
        "status": "candidate_not_accepted",
        "dataset": dataset_path,
        "counts": {"accepted_raw": len(accepted), "unique": len(unique),
                   "training": len(train), "holdout": len(holdout),
                   "duplicates_removed": len(duplicates)},
        "deduplication": {"translation_mm": args.duplicate_mm,
                          "rotation_deg": args.duplicate_deg,
                          "removed": duplicates},
        "training_sample_ids": [x["sample_id"] for x in train],
        "holdout_sample_ids": [x["sample_id"] for x in holdout],
        "methods": {},
    }
    for name, method in METHODS.items():
        try:
            gripper_from_camera = solve(method, train)
            train_closure = closure(train, gripper_from_camera)
            target_reference = average_transform(train_closure)
            holdout_closure = closure(holdout, gripper_from_camera)
            document["methods"][name] = {
                "success": True,
                "tool0_from_camera": matrix_json(gripper_from_camera),
                "base_from_target_training_mean": matrix_json(target_reference),
                "training_closure": metrics(train_closure, target_reference),
                "holdout_closure": metrics(holdout_closure, target_reference),
            }
        except (cv2.error, ValueError, np.linalg.LinAlgError) as exc:
            document["methods"][name] = {"success": False, "error": str(exc)}
    successful = [(name, value) for name, value in document["methods"].items()
                  if value["success"]]
    if successful:
        successful.sort(key=lambda item: (
            item[1]["holdout_closure"]["translation_mm"]["rms"],
            item[1]["holdout_closure"]["rotation_deg"]["rms"]))
        document["best_by_holdout_translation_rms"] = successful[0][0]
        document["method_pairwise_difference"] = {}
        for left_index, (left_name, left) in enumerate(successful):
            left_tf = np.asarray(left["tool0_from_camera"]["matrix_4x4"])
            for right_name, right in successful[left_index + 1:]:
                right_tf = np.asarray(right["tool0_from_camera"]["matrix_4x4"])
                distance = transform_distance(left_tf, right_tf)
                document["method_pairwise_difference"][
                    "%s_vs_%s" % (left_name, right_name)] = {
                        "translation_mm": distance[0], "rotation_deg": distance[1]}
        if "park" in document["methods"] and "horaud" in document["methods"]:
            park = document["methods"]["park"]
            horaud = document["methods"]["horaud"]
            consensus = document["method_pairwise_difference"].get(
                "park_vs_horaud",
                document["method_pairwise_difference"].get("horaud_vs_park"))
            gate = (park["success"] and horaud["success"] and
                    park["holdout_closure"]["translation_mm"]["max"] <= 5.0 and
                    park["holdout_closure"]["rotation_deg"]["max"] <= 2.0 and
                    consensus["translation_mm"] <= 2.0 and
                    consensus["rotation_deg"] <= 0.5)
            document["acceptance_gate"] = {
                "criteria": {"park_holdout_max_translation_mm": 5.0,
                             "park_holdout_max_rotation_deg": 2.0,
                             "park_horaud_translation_difference_mm": 2.0,
                             "park_horaud_rotation_difference_deg": 0.5},
                "park_horaud_difference": consensus,
                "passed": bool(gate),
            }
            if gate:
                document["status"] = "passed_numeric_holdout_gate"
                document["recommended_method"] = "park"
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    temporary = output_path + ".tmp"
    with open(temporary, "w", encoding="utf-8") as stream:
        json.dump(document, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    os.replace(temporary, output_path)
    print("原始有效 %d，去重后 %d，训练 %d，留出 %d" %
          (len(accepted), len(unique), len(train), len(holdout)))
    for name, value in document["methods"].items():
        if not value["success"]:
            print("%-10s FAILED %s" % (name, value["error"]))
            continue
        test = value["holdout_closure"]
        print("%-10s holdout RMS %.3f mm / %.3f deg, max %.3f mm / %.3f deg" % (
            name, test["translation_mm"]["rms"], test["rotation_deg"]["rms"],
            test["translation_mm"]["max"], test["rotation_deg"]["max"]))
    if "acceptance_gate" in document:
        difference = document["acceptance_gate"]["park_horaud_difference"]
        print("Park/Horaud 差异 %.3f mm / %.3f deg；数值门禁：%s" % (
            difference["translation_mm"], difference["rotation_deg"],
            "通过" if document["acceptance_gate"]["passed"] else "未通过"))
    print("候选结果：", output_path)


if __name__ == "__main__":
    main()
