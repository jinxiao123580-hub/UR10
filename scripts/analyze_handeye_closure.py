#!/usr/bin/env python3
"""Independent acceptance analysis for an eye-in-hand checkerboard calibration.

This is deliberately separate from ``solve_handeye_checkerboard.py``: the solver
produces a candidate, this script judges one.  Given a frozen calibration YAML it
re-derives the board pose in the robot base frame for every accepted sample

    base_from_target = base_from_tool0 . tool0_from_camera . camera_from_target

and reports the fixed-board closure three ways that a solver residual does not
show directly: where the board *origin* lands, how the board *orientation*
scatters, and how far the *54 corner points* move in the base frame across
samples.  It also re-checks the two gates that live in the sample files (motion,
reprojection) and, when ``cloud.npz`` files exist, compares the PnP-expected
corner positions against the raw organized cloud - a check that is independent of
the hand-eye transform because it only uses the camera intrinsics and the depth
channel.

Read-only with respect to the robot; ``--self-test`` needs no hardware at all.
"""
import argparse
import datetime as dt
import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from solve_handeye_checkerboard import (  # noqa: E402
    average_transform, observability, rotation_angle_deg, rt, sample_transforms)

GATES = {
    "holdout_translation_rms_mm": 3.0,
    "holdout_translation_max_mm": 5.0,
    "holdout_rotation_rms_deg": 1.0,
    "holdout_rotation_max_deg": 2.0,
    "independent_translation_mm": 5.0,
    "independent_rotation_deg": 2.0,
    "cross_method_translation_mm": 1.0,
    "cross_method_rotation_deg": 0.5,
    "reprojection_rms_px": 0.5,
    "motion_position_mm": 0.5,
    "motion_rotation_deg": 0.2,
}


def load_calibration(path):
    if not path or not os.path.exists(path):
        return None, None
    with open(path, encoding="utf-8") as stream:
        import yaml
        document = yaml.safe_load(stream)
    transform = np.eye(4)
    transform[:3, :3] = np.asarray(document["rotation_matrix"], dtype=np.float64)
    transform[:3, 3] = np.asarray(document["translation_m"], dtype=np.float64)
    return transform, document


def corner_object_points(pattern, square_size):
    points = np.zeros((pattern[0] * pattern[1], 3), dtype=np.float64)
    points[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2)
    points[:, :2] *= square_size
    return points


def board_geometry(samples, tool0_from_camera, pattern, square_size):
    """Fixed-board closure: origin, orientation and per-corner world scatter."""
    transforms = []
    for sample in samples:
        base_from_tool0, camera_from_target = sample_transforms(sample)
        transforms.append(base_from_tool0 @ tool0_from_camera @ camera_from_target)
    if not transforms:
        return None
    reference = average_transform(transforms)
    origins = np.asarray([value[:3, 3] for value in transforms]) * 1000.0
    rotation_errors = np.asarray([rotation_angle_deg(
        reference[:3, :3].T @ value[:3, :3]) for value in transforms])
    normals = np.asarray([value[:3, :3] @ np.asarray([0.0, 0.0, 1.0])
                          for value in transforms])
    mean_normal = normals.mean(axis=0)
    mean_normal /= np.linalg.norm(mean_normal)
    normal_errors = np.degrees(np.arccos(np.clip(normals @ mean_normal, -1.0, 1.0)))

    points = corner_object_points(pattern, square_size)
    world = np.asarray([(value[:3, :3] @ points.T).T + value[:3, 3]
                        for value in transforms])
    centroid = world.mean(axis=0)
    residuals = np.linalg.norm(world - centroid[None, :, :], axis=2) * 1000.0
    per_corner_max = residuals.max(axis=0)

    origin_errors = np.linalg.norm(origins - origins.mean(axis=0), axis=1)
    return {
        "sample_count": len(transforms),
        "board_origin_mm": {
            "rms": float(np.sqrt(np.mean(origin_errors ** 2))),
            "max": float(np.max(origin_errors)),
            "per_sample": origin_errors.tolist(),
            "xy_span_mm": (origins[:, :2].max(axis=0) -
                           origins[:, :2].min(axis=0)).tolist(),
            "std_mm": origins.std(axis=0).tolist(),
        },
        "board_orientation_deg": {
            "rms": float(np.sqrt(np.mean(rotation_errors ** 2))),
            "max": float(np.max(rotation_errors)),
            "normal_rms": float(np.sqrt(np.mean(normal_errors ** 2))),
            "normal_max": float(np.max(normal_errors)),
            "per_sample": rotation_errors.tolist(),
        },
        "corner_world_scatter_mm": {
            "corner_count": int(len(per_corner_max)),
            "rms": float(np.sqrt(np.mean(residuals ** 2))),
            "p95": float(np.percentile(residuals, 95)),
            "max": float(np.max(residuals)),
            "per_corner_max": per_corner_max.tolist(),
        },
        "reference_base_from_target": {
            "matrix_4x4": reference.tolist(),
        },
    }


def gate_report(samples):
    """Re-check the per-sample gates stored by the collector."""
    accepted = [s for s in samples if s.get("accepted")]
    rejected = [s for s in samples if not s.get("accepted")]
    reprojection = [s["target_to_camera"]["reprojection_rms_px"]
                    for s in accepted if s.get("target_to_camera")]
    motion_mm = [s["robot"]["max_position_motion_mm"] for s in accepted]
    motion_deg = [s["robot"]["max_rotation_motion_deg"] for s in accepted]
    detections = [s.get("detected_corner_count", 0) for s in accepted]
    return {
        "accepted": len(accepted),
        "rejected": len(rejected),
        "rejected_ids": [s["sample_id"] for s in rejected],
        "reprojection_rms_px": {
            "count": len(reprojection),
            "max": float(np.max(reprojection)) if reprojection else None,
            "mean": float(np.mean(reprojection)) if reprojection else None,
            "over_gate": [s["sample_id"] for s in accepted
                          if s.get("target_to_camera") and
                          s["target_to_camera"]["reprojection_rms_px"] >
                          GATES["reprojection_rms_px"]],
        },
        "motion_position_mm_max": float(np.max(motion_mm)) if motion_mm else None,
        "motion_rotation_deg_max": float(np.max(motion_deg)) if motion_deg else None,
        "corner_count_min": int(np.min(detections)) if detections else None,
        "passed": bool(
            reprojection and np.max(reprojection) <= GATES["reprojection_rms_px"]
            and np.max(motion_mm) <= GATES["motion_position_mm"]
            and np.max(motion_deg) <= GATES["motion_rotation_deg"]
            and min(detections) == 54),
    }


def cloud_corner_check(path, radius_px=3):
    """Compare the PnP-expected corner xyz against the raw organized cloud.

    Uses only the intrinsics, the corner pixels and the depth channel, so it
    never touches the hand-eye transform: it answers "is the stored
    target->camera pose geometrically consistent with the raw cloud".
    """
    data = np.load(path)
    if "camera_xyz_m" not in data.files or "corners_px" not in data.files:
        return {"status": "not_checkable",
                "reason": "cloud.npz lacks camera_xyz_m or corners_px"}
    xyz = data["camera_xyz_m"].astype(np.float64)
    corners = data["corners_px"]
    k = data["k"]
    d = data["d"]
    camera_from_target = data["camera_from_target"]
    pattern = tuple(int(x) for x in data["inner_corners"]) \
        if "inner_corners" in data.files else (corners.shape[0] // 6, 6)
    square_size = float(data["square_size_m"]) if "square_size_m" in data.files else 0.006
    points = corner_object_points(pattern, square_size)
    expected = (camera_from_target[:3, :3] @ points.T).T + camera_from_target[:3, 3]

    errors = []
    missing = 0
    for index, pixel in enumerate(corners.reshape(-1, 2)):
        u, v = [int(round(value)) for value in pixel]
        patch = xyz[max(0, v - radius_px):v + radius_px + 1,
                    max(0, u - radius_px):u + radius_px + 1].reshape(-1, 3)
        patch = patch[np.all(np.isfinite(patch), axis=1)]
        if len(patch) < 3:
            missing += 1
            continue
        errors.append(float(np.linalg.norm(np.median(patch, axis=0) -
                                           expected[index]) * 1000.0))
    errors = np.asarray(errors)
    return {
        "status": "checked",
        "radius_px": radius_px,
        "corners": int(len(corners)),
        "missing_neighborhoods": int(missing),
        "corner_xyz_error_mm": {
            "count": int(len(errors)),
            "rms": float(np.sqrt(np.mean(errors ** 2))) if len(errors) else None,
            "median": float(np.median(errors)) if len(errors) else None,
            "p95": float(np.percentile(errors, 95)) if len(errors) else None,
            "max": float(np.max(errors)) if len(errors) else None,
        },
    }


# ---------------------------------------------------------------------------
# offline self-test: no camera, no robot
# ---------------------------------------------------------------------------
def synthetic_samples(rng, tool0_from_camera, base_from_target, count,
                      pattern=(9, 6), square_size=0.006, spread=(0.35, 0.55)):
    """Build samples that satisfy A X B = C exactly for a known X."""
    samples = []
    for index in range(count):
        distance = rng.uniform(*spread)
        azimuth = rng.uniform(-np.pi, np.pi)
        tilt = rng.uniform(0.0, 0.9)
        direction = np.asarray([np.sin(tilt) * np.cos(azimuth),
                                np.sin(tilt) * np.sin(azimuth),
                                np.cos(tilt)])
        camera_position = base_from_target[:3, 3] + distance * direction
        camera_z = -direction                       # look back at the board
        helper = np.asarray([0.0, 0.0, 1.0]) if abs(camera_z[2]) < 0.9 \
            else np.asarray([1.0, 0.0, 0.0])
        camera_x = np.cross(helper, camera_z)
        camera_x /= np.linalg.norm(camera_x)
        camera_y = np.cross(camera_z, camera_x)
        base_from_camera = np.eye(4)
        base_from_camera[:3, :3] = np.column_stack((camera_x, camera_y, camera_z))
        base_from_camera[:3, 3] = camera_position
        base_from_tool0 = base_from_camera @ np.linalg.inv(tool0_from_camera)
        camera_from_target = np.linalg.inv(base_from_camera) @ base_from_target
        tcp_rvec, _ = cv2.Rodrigues(base_from_tool0[:3, :3])
        target_rvec, _ = cv2.Rodrigues(camera_from_target[:3, :3])
        samples.append({
            "sample_id": "synthetic-%02d" % index,
            "accepted": True,
            "detected": True,
            "detected_corner_count": pattern[0] * pattern[1],
            "inner_corners": list(pattern),
            "square_size_m": square_size,
            "robot": {
                "max_position_motion_mm": 0.02,
                "max_rotation_motion_deg": 0.005,
                "base_to_tool0_tcp_mean": (base_from_tool0[:3, 3].tolist() +
                                           tcp_rvec.reshape(3).tolist()),
            },
            "target_to_camera": {
                "rvec_rad": target_rvec.reshape(3).tolist(),
                "translation_m": camera_from_target[:3, 3].tolist(),
                "reprojection_rms_px": 0.2,
            },
        })
    return samples


def synthetic_planar_cloud(k, camera_from_target, pattern, square_size,
                           resolution=(256, 256)):
    """A perfect pinhole depth rendering of the board plane, as an organized cloud.

    Every pixel is the exact intersection of its ray with the board plane, so the
    neighbourhood median at a corner must reproduce the PnP-expected point; any
    disagreement is a bug in the pixel<->corner correspondence, not noise.
    """
    width, height = resolution
    normal = camera_from_target[:3, 2]
    offset = float(normal @ camera_from_target[:3, 3])
    inverse_k = np.linalg.inv(k)
    grid_u, grid_v = np.meshgrid(np.arange(width), np.arange(height))
    rays = np.stack((grid_u.ravel(), grid_v.ravel(),
                     np.ones(width * height)), axis=1) @ inverse_k.T
    scale = offset / (rays @ normal)
    xyz = (rays * scale[:, None]).reshape(height, width, 3)
    xyz[scale.reshape(height, width) <= 0] = np.nan
    return xyz


def project_points(points, k, camera_from_target):
    camera = (camera_from_target[:3, :3] @ points.T).T + camera_from_target[:3, 3]
    pixels = (camera @ k.T)
    pixels = pixels[:, :2] / pixels[:, 2:3]
    return pixels


def self_test(output_path, root):
    """Offline numeric gate for this script and for the new cloud.npz schema."""
    checks = []

    def record(name, passed, detail):
        checks.append({"name": name, "passed": bool(passed), "detail": detail})

    rng = np.random.default_rng(20260918)
    pattern = (9, 6)
    square_size = 0.006
    k = np.asarray([[1743.688303805104, 0.0, 538.1910185232366],
                    [0.0, 1743.688303805104, 512.7044230258488],
                    [0.0, 0.0, 1.0]])
    d = np.zeros((5, 1))
    tool0_from_camera = rt(np.asarray([0.15, -0.9, 0.4]),
                           np.asarray([0.04, 0.05, 0.18]))
    base_from_target = rt(np.asarray([0.2, 0.1, -0.4]),
                          np.asarray([0.62, -0.12, 0.15]))

    samples = synthetic_samples(rng, tool0_from_camera, base_from_target, 14,
                                pattern=pattern, square_size=square_size)
    geometry = board_geometry(samples, tool0_from_camera, pattern, square_size)
    record("closure_origin_rms_under_1e-6_mm",
           geometry["board_origin_mm"]["rms"] < 1e-6,
           {"board_origin_mm_rms": geometry["board_origin_mm"]["rms"]})
    record("closure_corner_scatter_under_1e-6_mm",
           geometry["corner_world_scatter_mm"]["max"] < 1e-6,
           {"corner_world_scatter_mm_max": geometry["corner_world_scatter_mm"]["max"]})
    record("closure_orientation_under_1e-4_deg",
           geometry["board_orientation_deg"]["max"] < 1e-4,
           {"board_orientation_deg_max": geometry["board_orientation_deg"]["max"]})

    # a 2 mm error in the calibration must show up as ~2 mm of closure error
    broken = tool0_from_camera.copy()
    broken[0, 3] += 0.002
    broken_geometry = board_geometry(samples, broken, pattern, square_size)
    record("closure_detects_2mm_calibration_error",
           broken_geometry["board_origin_mm"]["rms"] > 1.0,
           {"board_origin_mm_rms": broken_geometry["board_origin_mm"]["rms"]})

    # the observability diagnostic must separate a spread design from a degenerate one
    spread = observability(samples)
    flat = []
    for index, sample in enumerate(samples):
        clone = json.loads(json.dumps(sample))
        tcp = clone["robot"]["base_to_tool0_tcp_mean"]
        tcp[3:] = [0.0, 0.0, 0.35 + 0.001 * index]      # pure translation, no rotation
        clone["target_to_camera"]["rvec_rad"] = [0.0, 0.0, 0.0]
        flat.append(clone)
    degenerate = observability(flat)
    spread_isotropy = (spread.get("relative_rotation_axis_isotropy") or {}).get(
        "min_over_max")
    record("observability_flags_spread_design",
           spread_isotropy is not None and spread_isotropy > 0.01,
           {"axis_isotropy_min_over_max": spread_isotropy})
    record("observability_flags_degenerate_design",
           degenerate.get("relative_rotation_deg", {}).get("max", 1.0) < 1e-6 or
           (degenerate.get("relative_rotation_axis_isotropy") or {}).get(
               "min_over_max", 1.0) < spread_isotropy,
           {"degenerate_axis_isotropy": (degenerate.get(
               "relative_rotation_axis_isotropy") or {}).get("min_over_max"),
            "degenerate_relative_rotation_max_deg": degenerate.get(
                "relative_rotation_deg", {}).get("max")})

    # cloud.npz round trip through the real payload builder
    from collect_handeye_sample import build_cloud_payload
    camera_from_target = rt(np.asarray([0.1, -0.05, 0.02]),
                            np.asarray([0.01, -0.02, 0.45]))
    points = corner_object_points(pattern, square_size)
    pixels = project_points(points, k, camera_from_target)
    cloud = synthetic_planar_cloud(k, camera_from_target, pattern, square_size,
                                  resolution=(1280, 1024))
    npz_path = os.path.join(root, "outputs", "handeye",
                            "selftest-cloud-roundtrip.npz")
    os.makedirs(os.path.dirname(npz_path), exist_ok=True)
    payload = build_cloud_payload(
        sample_id="selftest", camera_xyz=cloud, cloud_resolution=(1280, 1024),
        cloud_frame_id="camera", image_frame_id="camera", cloud_is_dense=False,
        k=k, d=d, base_from_tool0=np.eye(4),
        target_to_camera=(cv2.Rodrigues(camera_from_target[:3, :3])[0].reshape(3),
                          camera_from_target[:3, 3]),
        corners_px=pixels, inner_corners=pattern, square_size_m=square_size)
    np.savez_compressed(npz_path, **payload)
    cloud_result = cloud_corner_check(npz_path, radius_px=3)
    cloud_rms = (cloud_result.get("corner_xyz_error_mm") or {}).get("rms")
    record("cloud_npz_roundtrip_corner_rms_under_0.5mm",
           cloud_result["status"] == "checked" and cloud_rms is not None
           and cloud_rms < 0.5,
           {"status": cloud_result["status"], "corner_xyz_rms_mm": cloud_rms,
            "npz": os.path.relpath(npz_path, root)})
    record("cloud_npz_has_audit_keys",
           {"camera_xyz_m", "camera_from_target", "base_from_tool0", "k", "d",
            "corners_px", "inner_corners", "square_size_m"}.issubset(set(payload)),
           {"keys": sorted(payload)})

    report = {
        "schema_version": 1,
        "created_at": dt.datetime.now(dt.timezone.utc).astimezone().isoformat(),
        "kind": "offline_self_test",
        "checks": checks,
        "passed": all(check["passed"] for check in checks),
        "passed_count": sum(check["passed"] for check in checks),
        "total_count": len(checks),
    }
    with open(output_path, "w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    for check in checks:
        print("%-46s %s" % (check["name"], "PASS" if check["passed"] else "FAIL"))
    print("OFFLINE GATE: %s (%d/%d)" % ("PASS" if report["passed"] else "FAIL",
                                        report["passed_count"], report["total_count"]))
    print("OUTPUT:", output_path)
    return 0 if report["passed"] else 3


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", default=None,
                        help="dataset JSON written by collect_handeye_sample.py")
    parser.add_argument("--calibration", default=None,
                        help="frozen calibration YAML to judge (tool0_from_camera)")
    parser.add_argument("--radius-px", type=int, default=3)
    parser.add_argument("--no-cloud-check", action="store_true",
                        help="skip the raw-cloud cross-check even if cloud.npz exists")
    parser.add_argument("--output", default=None)
    parser.add_argument("--self-test", action="store_true",
                        help="run the offline numeric gate (no camera, no robot)")
    args = parser.parse_args()

    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    if args.self_test:
        output = args.output or os.path.join(
            root, "outputs", "handeye", "closure-offline-selftest-%s.json" % stamp)
        return self_test(output, root)

    if not args.dataset or not args.calibration:
        parser.error("--dataset and --calibration are required unless --self-test")
    dataset_path = args.dataset if os.path.isabs(args.dataset) \
        else os.path.join(root, args.dataset)
    calibration_path = args.calibration if os.path.isabs(args.calibration) \
        else os.path.join(root, args.calibration)
    with open(dataset_path, encoding="utf-8") as stream:
        dataset = json.load(stream)
    tool0_from_camera, calibration = load_calibration(calibration_path)
    if tool0_from_camera is None:
        raise RuntimeError("calibration file not found: %s" % calibration_path)
    pattern = tuple(dataset.get("inner_corners", (9, 6)))
    square_size = float(dataset.get("square_size_m", 0.006))

    accepted = [s for s in dataset["samples"] if s.get("accepted")]
    geometry = board_geometry(accepted, tool0_from_camera, pattern, square_size)
    gates = gate_report(dataset["samples"])

    cloud_results = {}
    if not args.no_cloud_check:
        sample_root = os.path.join(os.path.dirname(dataset_path), "samples")
        for sample in accepted:
            path = os.path.join(sample_root, sample["sample_id"], "cloud.npz")
            if os.path.exists(path):
                cloud_results[sample["sample_id"]] = cloud_corner_check(
                    path, args.radius_px)
    cloud_summary = None
    checked = [value for value in cloud_results.values()
               if value.get("status") == "checked"]
    if checked:
        rms = [value["corner_xyz_error_mm"]["rms"] for value in checked]
        worst = max(checked, key=lambda value: value["corner_xyz_error_mm"]["max"])
        cloud_summary = {
            "samples_checked": len(checked),
            "samples_without_cloud": len(accepted) - len(checked),
            "corner_xyz_rms_mm_mean": float(np.mean(rms)),
            "corner_xyz_rms_mm_max": float(np.max(rms)),
            "worst_sample": worst["corner_xyz_error_mm"],
            "over_2mm": [sample_id for sample_id, value in cloud_results.items()
                         if value.get("status") == "checked" and
                         value["corner_xyz_error_mm"]["max"] is not None and
                         value["corner_xyz_error_mm"]["max"] > 2.0],
        }

    document = {
        "schema_version": 1,
        "created_at": dt.datetime.now(dt.timezone.utc).astimezone().isoformat(),
        "kind": "independent_closure_analysis",
        "dataset": os.path.relpath(dataset_path, root),
        "calibration": os.path.relpath(calibration_path, root),
        "calibration_valid_flag": (calibration or {}).get("valid"),
        "calibration_method": (calibration or {}).get("method"),
        "board": {"inner_corners": list(pattern), "square_size_m": square_size},
        "sample_gates": gates,
        "board_geometry": geometry,
        "observability_of_accepted": observability(accepted),
        "cloud_corner_check": {
            "summary": cloud_summary,
            "per_sample": cloud_results,
            "interpretation": "PnP-expected corner xyz vs raw organized cloud; "
                              "independent of the hand-eye transform. Assumes "
                              "depthToTexture = identity, as reported by this camera.",
        },
        "gate_thresholds": GATES,
    }

    output = args.output or os.path.join(
        root, "outputs", "handeye", "closure-%s.json" % stamp)
    output = output if os.path.isabs(output) else os.path.join(root, output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "w", encoding="utf-8") as stream:
        json.dump(document, stream, indent=2, ensure_ascii=False)
        stream.write("\n")

    print(json.dumps({
        "samples": gates["accepted"],
        "sample_gates_passed": gates["passed"],
        "reprojection_rms_px_max": gates["reprojection_rms_px"]["max"],
        "board_origin_mm": geometry["board_origin_mm"]["rms"] if geometry else None,
        "board_origin_xy_span_mm": (geometry["board_origin_mm"]["xy_span_mm"]
                                    if geometry else None),
        "corner_world_scatter_mm": (geometry["corner_world_scatter_mm"]
                                    if geometry else None),
        "board_orientation_deg": (geometry["board_orientation_deg"]
                                  if geometry else None),
        "cloud_corner_check": cloud_summary,
    }, indent=2, ensure_ascii=False))
    print("OUTPUT:", output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
