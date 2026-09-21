#!/usr/bin/env python3
"""Measure a cube beside the checkerboard from its TOP FACE (P0-B fix).

Why this exists
---------------
The 2026-09-17 locator (``locate_cube_near_checkerboard.py``) inferred the cube
centre from a *single visible vertical side* plus the nominal 50 mm edge, and it
chose which way to offset with

    outward = sign(side_centre - board_centre)[lateral_axis]

which is geometrically meaningless.  For the archived capture the camera sat at
target x = +0.364 m looking in -x, so the visible face (x = +0.0528 m) could only
be the cube's +x face and the body had to be on the -x side.  The locator instead
offset by +25 mm, i.e. 50 mm the wrong way, and the sampled face was ~44 mm wide
by >=25 mm tall, not 50 mm - so the "known 50 mm edge" prior was never verified
either.

This tool replaces the inference with a measurement:

    table plane (ring around the object)  ->  standing surface
    object cluster (organised cloud)      ->  candidate points
    RANSAC top face parallel to the board ->  footprint + measured edges
    centre = footprint centre - (h/2) * up_normal

Hard gates (a failure is a rejection, never a silent centre):
  * enough cluster and top-face points
  * the top face is parallel to the board plane within ``--max-normal-tilt-deg``
  * footprint coverage >= ``--min-coverage`` and square within ``--max-aspect``
  * a reference plane exists around the object and is below the top face
The nominal size prior is checked separately and only reported: a measured edge
that disagrees with ``--expect-edge`` sets ``size_prior_mismatch`` instead of
quietly producing a centre.

The geometry core (``measure_cube_geometry``) is a pure function of a target-frame
point cloud, so it is exercised offline by
``scripts/test_measure_cube_geometry.py`` without any camera or robot.
"""
import argparse
import json
import os
import sys
from datetime import datetime

import cv2
import numpy as np

DEFAULTS = {
    "search_half_xy_m": 0.25,
    "object_min_height_m": 0.002,
    "object_max_height_m": 0.30,
    "plane_threshold_m": 0.002,
    "ransac_iterations": 400,
    "max_normal_tilt_deg": 12.0,
    "max_reference_tilt_deg": 20.0,
    "min_cluster_points": 200,
    "min_face_points": 200,
    "min_coverage": 0.85,
    "max_aspect": 1.10,
    "expect_edge_m": 0.05,
    "edge_tolerance_m": 0.0025,
    "cluster_cell_m": 0.003,
    "ring_margin_m": 0.005,
    "board_half_extent_m": [0.054, 0.036],  # 10x7 squares of 6 mm, origin at a corner
}


def rotation_from_rotvec(rotvec):
    theta = float(np.linalg.norm(rotvec))
    if theta < 1e-12:
        return np.eye(3)
    axis = np.asarray(rotvec, dtype=np.float64) / theta
    k = np.array([[0.0, -axis[2], axis[1]],
                  [axis[2], 0.0, -axis[0]],
                  [-axis[1], axis[0], 0.0]])
    return np.eye(3) + np.sin(theta) * k + (1.0 - np.cos(theta)) * (k @ k)


def fit_plane(points):
    """Total-least-squares plane through points: returns (normal, offset).

    ``full_matrices=False`` matters: with the default an N x 3 input allocates an
    N x N ``u`` matrix, which turns a 45 k point cloud into a multi-gigabyte SVD.
    """
    centroid = points.mean(axis=0)
    _, _, vh = np.linalg.svd(points - centroid, full_matrices=False)
    normal = vh[-1]
    return normal, float(normal @ centroid)


def ransac_plane(points, threshold, iterations, rng, normal_hint=None, max_tilt_deg=None,
                 max_search_points=4000):
    """Largest plane whose normal stays within ``max_tilt_deg`` of ``normal_hint``.

    The random search runs on at most ``max_search_points`` points (a subsample gives a
    statistically identical plane in a fraction of the time); the final inlier set and the
    refit always use every point handed in.
    """
    count = len(points)
    if count < 3:
        return None
    search = points
    if count > max_search_points:
        search = points[rng.choice(count, size=max_search_points, replace=False)]
    best = None
    cos_limit = None if max_tilt_deg is None else np.cos(np.radians(max_tilt_deg))
    for _ in range(iterations):
        idx = rng.choice(len(search), size=3, replace=False)
        normal, offset = fit_plane(search[idx])
        if normal @ normal_hint < 0:
            normal, offset = -normal, -offset
        if cos_limit is not None and normal @ normal_hint < cos_limit:
            continue
        distance = np.abs(search @ normal - offset)
        total = int((distance < threshold).sum())
        if best is None or total > best[0]:
            best = (total, normal, offset)
    if best is None or best[0] < 3:
        return None
    _, normal, offset = best
    distance = np.abs(points @ normal - offset)
    inliers = distance < threshold
    normal, offset = fit_plane(points[inliers])
    if normal @ normal_hint < 0:
        normal, offset = -normal, -offset
    distance = np.abs(points @ normal - offset)
    inliers = distance < threshold
    return {"normal": normal, "offset": float(offset), "inliers": inliers,
            "count": int(inliers.sum()), "search_points": int(len(search))}


def planar_extent(points, up, right_hint=(1.0, 0.0, 0.0)):
    """Project points onto the plane orthogonal to ``up``; return 2D coords and basis."""
    right = np.asarray(right_hint, dtype=np.float64)
    right = right - up * float(right @ up)
    if np.linalg.norm(right) < 1e-6:
        right = np.cross(up, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    forward = np.cross(up, right)
    return np.column_stack((points @ right, points @ forward)), right, forward


def denoise_planar_points(points_2d, cell=0.0015, min_neighbors=3):
    """Drop isolated in-plane spikes (structured-light edge/mixed pixels).

    A point survives only if its 3x3 cell neighbourhood (``cell`` metres) holds at least
    ``min_neighbors`` points, which removes lone spikes without eroding a real dense face.
    Median filtering is not enough here: a single spike far outside the true face is exactly
    what inflates the min-area rectangle and fakes a "50 mm" edge measurement.
    """
    if len(points_2d) == 0:
        return np.zeros(0, dtype=bool)
    origin = points_2d.min(axis=0) - cell
    grid = np.floor((points_2d - origin) / cell).astype(np.int64)
    shape = (int(grid[:, 0].max()) + 3, int(grid[:, 1].max()) + 3)
    counts = np.zeros(shape, dtype=np.float32)
    np.add.at(counts, (grid[:, 0], grid[:, 1]), 1.0)
    local = cv2.boxFilter(counts, ddepth=-1, ksize=(3, 3), normalize=False,
                          borderType=cv2.BORDER_CONSTANT)
    return local[grid[:, 0], grid[:, 1]] >= min_neighbors


def footprint_coverage(points_2d, rect_area):
    """Fraction of the min-area rectangle actually covered by measured points.

    Measured as the convex hull of the 2D inliers over the rectangle they span, so a
    half-visible or edge-on top face scores low and is rejected while a fully visible
    one scores ~1.  O(N log N) and never allocates a distance matrix.
    """
    if len(points_2d) < 3 or rect_area <= 0:
        return 0.0, 0.0
    hull = cv2.convexHull(points_2d.astype(np.float32))
    hull_area = abs(float(cv2.contourArea(hull)))
    pitch = float(np.sqrt(hull_area / len(points_2d))) if hull_area > 0 else 0.0
    return float(min(1.0, hull_area / rect_area)), pitch


def footprint_properties(points_2d):
    """Min-area rectangle plus a coverage estimate."""
    rect = cv2.minAreaRect(points_2d.astype(np.float32))
    (cx, cy), (w, h), angle = rect
    if w < h:
        w, h, angle = h, w, angle + 90.0
    area = float(w) * float(h)
    coverage, pitch = footprint_coverage(points_2d, area)
    return {"center_2d": [float(cx), float(cy)], "edges_m": [float(w), float(h)],
            "angle_deg": float(angle), "area_m2": area, "pitch_m": pitch,
            "coverage": coverage, "points": int(len(points_2d))}


def segment_object(points, up, heights, cfg):
    """Largest rasterised blob of points standing above the board surface."""
    mask_height = (heights > cfg["object_min_height_m"]) & (heights < cfg["object_max_height_m"])
    window = (np.abs(points[:, 0]) < cfg["search_half_xy_m"]) & \
             (np.abs(points[:, 1]) < cfg["search_half_xy_m"])
    keep = mask_height & window
    if int(keep.sum()) < cfg["min_cluster_points"]:
        return None, keep
    xy = points[keep][:, :2]
    cell = cfg["cluster_cell_m"]
    origin = xy.min(axis=0) - cell
    grid = np.floor((xy - origin) / cell).astype(np.int32)
    shape = (int(grid[:, 0].max()) + 3, int(grid[:, 1].max()) + 3)
    dense = np.zeros(shape, dtype=np.uint8)
    dense[grid[:, 0], grid[:, 1]] = 1
    count, labels = cv2.connectedComponents(dense, connectivity=8)
    if count < 2:
        return None, keep
    sizes = [(int((labels == i).sum()), i) for i in range(1, count)]
    sizes.sort(reverse=True)
    best_label = sizes[0][1]
    keep_indices = np.flatnonzero(keep)
    label_of_point = labels[grid[:, 0], grid[:, 1]]
    return {"indices": keep_indices[label_of_point == best_label],
            "blob_cells": sizes[0][0], "blobs": len(sizes)}, keep


def measure_cube_geometry(target_points, camera_origin_target, cfg=None, board_half_extent=None):
    """Pure geometry core: target-frame points in, measured cube centre out."""
    settings = dict(DEFAULTS)
    if cfg:
        settings.update(cfg)
    if board_half_extent is not None:
        settings["board_half_extent_m"] = list(board_half_extent)
    rng = np.random.default_rng(7)

    points = np.asarray(target_points, dtype=np.float64).reshape(-1, 3)
    points = points[np.all(np.isfinite(points), axis=1)]
    report = {"settings": {k: settings[k] for k in ("search_half_xy_m", "min_coverage",
                                                   "max_aspect", "expect_edge_m",
                                                   "edge_tolerance_m", "max_normal_tilt_deg")},
              "input_points": int(len(points)), "gates": {}, "reasons": [],
              "status": "rejected", "center_target_m": None, "center_base_m": None}

    def provisional_cluster(reason):
        """Diagnostic fallback: never infer an unseen cube centre from one side."""
        low, high = object_points.min(axis=0), object_points.max(axis=0)
        report["status"] = "provisional_partial"
        report["reasons"].append(reason)
        report["provisional_only"] = True
        report["motion_authorization"] = "none: visible cluster only; not a cube centre"
        report["visible_cluster_centroid_target_m"] = object_points.mean(axis=0).tolist()
        report["visible_cluster_bounds_target_m"] = {"min": low.tolist(), "max": high.tolist()}
        report["visible_cluster_span_m"] = (high - low).tolist()
        return report

    camera_origin = np.asarray(camera_origin_target, dtype=np.float64)
    # The board plane is z=0 in the target frame; the cube stands on that surface and
    # grows toward the camera, which is what makes the top face visible at all.
    if abs(camera_origin[2]) < 1e-6:
        report["reasons"].append("camera lies in the board plane; no viewing side")
        return report
    up = np.array([0.0, 0.0, 1.0]) * np.sign(camera_origin[2])
    heights = points @ up
    report["up_axis_target"] = up.tolist()
    report["camera_origin_target_m"] = camera_origin.tolist()

    cluster, _ = segment_object(points, up, heights, settings)
    if cluster is None:
        report["reasons"].append("no object cluster above the board surface")
        return report
    object_points = points[cluster["indices"]]
    object_heights = heights[cluster["indices"]]
    report["cluster_points"] = int(len(object_points))
    report["cluster_height_m"] = [float(object_heights.min()), float(object_heights.max())]
    report["gates"]["cluster_points"] = bool(len(object_points) >= settings["min_cluster_points"])

    # Reference (standing) surface: a ring around the object, outside the checkerboard.
    rect_lo = object_points[:, :2].min(axis=0) - settings["ring_margin_m"]
    rect_hi = object_points[:, :2].max(axis=0) + settings["ring_margin_m"]
    half = np.asarray(settings["board_half_extent_m"], dtype=np.float64)
    outside_object = ~((points[:, 0] > rect_lo[0]) & (points[:, 0] < rect_hi[0]) &
                       (points[:, 1] > rect_lo[1]) & (points[:, 1] < rect_hi[1]))
    outside_board = ~((np.abs(points[:, 0]) < half[0] + 0.005) &
                      (np.abs(points[:, 1]) < half[1] + 0.005))
    low = (heights < settings["object_min_height_m"]) & (heights > -0.05)
    ring = points[outside_object & outside_board & low]
    report["ring_points"] = int(len(ring))
    reference = None
    if len(ring) >= 100:
        plane = ransac_plane(ring, settings["plane_threshold_m"],
                             settings["ransac_iterations"], rng,
                             normal_hint=up, max_tilt_deg=settings["max_reference_tilt_deg"])
        if plane is not None:
            reference_h = float(np.median(heights[outside_object & outside_board & low]
                                          [plane["inliers"]]))
            reference = {"normal": plane["normal"], "offset": plane["offset"],
                         "height_m": reference_h, "inliers": plane["count"],
                         "tilt_deg": float(np.degrees(np.arccos(
                             np.clip(abs(plane["normal"] @ up), -1.0, 1.0))))}
    report["gates"]["reference_plane"] = reference is not None
    if reference is None:
        report["reasons"].append("no reference surface around the object")
        return report
    report["reference_plane"] = {k: v for k, v in reference.items() if k != "inliers"}
    report["reference_plane"]["inliers"] = int(reference["inliers"])

    top = ransac_plane(object_points, settings["plane_threshold_m"],
                       settings["ransac_iterations"], rng,
                       normal_hint=up, max_tilt_deg=settings["max_normal_tilt_deg"])
    if top is None:
        report["reasons"].append("no plane parallel to the board inside the cluster")
        if settings.get("allow_partial", False):
            return provisional_cluster("partial fallback: no top face; centre is deliberately withheld")
        return report
    top_points = object_points[top["inliers"]]
    top_heights = top_points @ up
    report["top_face"] = {"points": int(len(top_points)),
                          "height_m": float(np.median(top_heights)),
                          "tilt_deg": float(np.degrees(np.arccos(
                              np.clip(abs(top["normal"] @ up), -1.0, 1.0))))}
    report["gates"]["top_face_points"] = bool(len(top_points) >= settings["min_face_points"])
    report["gates"]["top_face_parallel"] = bool(
        report["top_face"]["tilt_deg"] <= settings["max_normal_tilt_deg"])

    height = report["top_face"]["height_m"] - reference["height_m"]
    report["measured_height_m"] = float(height)
    report["gates"]["height_positive"] = bool(0.010 < height < 0.20)
    if not all(report["gates"][k] for k in ("cluster_points", "reference_plane",
                                           "top_face_points", "top_face_parallel",
                                           "height_positive")):
        for key, ok in report["gates"].items():
            if not ok:
                report["reasons"].append("gate failed: %s" % key)
        if settings.get("allow_partial", False):
            return provisional_cluster("partial fallback: top-face quality gate failed; centre is deliberately withheld")
        return report

    points_2d, right, forward = planar_extent(top_points, up)
    dense = denoise_planar_points(points_2d)
    report["top_face"]["outliers_dropped"] = int(len(points_2d) - int(dense.sum()))
    footprint = footprint_properties(points_2d[dense] if dense.sum() >= 3 else points_2d)
    report["top_face"]["footprint"] = footprint
    report["gates"]["coverage"] = bool(footprint["coverage"] >= settings["min_coverage"])
    report["gates"]["square"] = bool(
        max(footprint["edges_m"]) / max(min(footprint["edges_m"]), 1e-9) <= settings["max_aspect"])

    expect = settings["expect_edge_m"]
    tol = settings["edge_tolerance_m"]
    report["size_prior"] = {
        "expected_edge_m": expect,
        "measured_edges_m": footprint["edges_m"],
        "deviation_m": [float(abs(edge - expect)) for edge in footprint["edges_m"]],
        "matches_nominal_size": bool(all(abs(edge - expect) <= tol for edge in footprint["edges_m"])),
    }
    report["height_vs_edge_mm"] = float(
        (report["measured_height_m"] - float(np.mean(footprint["edges_m"]))) * 1000.0)

    if not (report["gates"]["coverage"] and report["gates"]["square"]):
        for key in ("coverage", "square"):
            if not report["gates"][key]:
                report["reasons"].append("gate failed: %s" % key)
        if settings.get("allow_partial", False):
            return provisional_cluster("partial fallback: footprint incomplete; centre is deliberately withheld")
        return report

    center_2d = np.asarray(footprint["center_2d"], dtype=np.float64)
    center_on_top = center_2d[0] * right + center_2d[1] * forward + \
        report["top_face"]["height_m"] * up
    center = center_on_top - up * (height / 2.0)
    report["center_target_m"] = center.tolist()
    report["status"] = "measured" if report["size_prior"]["matches_nominal_size"] \
        else "measured_size_prior_mismatch"
    if report["status"] == "measured_size_prior_mismatch":
        report["reasons"].append(
            "measured edges %s m disagree with the nominal %.3f m prior"
            % ([round(e, 5) for e in footprint["edges_m"]], expect))
    return report


# --------------------------------------------------------------------------
# capture path (read-only: no robot command is ever sent)
# --------------------------------------------------------------------------
def capture_scene(args, root):
    from check_handeye_checkerboard import camera_matrices, detect, image_to_bgr
    from collect_handeye_sample import URSampler, robot_summary
    from solve_handeye_checkerboard import rt
    from validate_checkerboard_pointcloud import BoardCloudCapture, organized_xyz
    import yaml
    import rclpy

    with open(os.path.join(root, args.calibration), encoding="utf-8") as stream:
        calibration = yaml.safe_load(stream)

    sampler = URSampler("192.168.1.3")
    rclpy.init()
    node = BoardCloudCapture()
    try:
        sampler.start()
        sampler.wait_ready()
        sampler.begin_window()
        image_msg, info_msg = node.capture(args.timeout)
        cloud_msg = node.capture_cloud(args.timeout)
    finally:
        sampler.stop()
        node.destroy_node()
        rclpy.shutdown()
    motion = robot_summary(sampler.rows)
    if motion["max_position_motion_mm"] > 0.5 or motion["max_rotation_motion_deg"] > 0.2:
        raise RuntimeError("robot moved during acquisition")

    image = image_to_bgr(image_msg)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = detect(gray, (9, 6))
    if not found:
        raise RuntimeError("checkerboard not detected")
    camera, distortion = camera_matrices(info_msg)
    objects = np.zeros((54, 3), dtype=np.float64)
    objects[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2) * 0.006
    ok, rvec, tvec = cv2.solvePnP(objects, corners, camera, distortion,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise RuntimeError("solvePnP failed")
    camera_from_target = rt(rvec.reshape(3), tvec.reshape(3))
    target_from_camera = np.linalg.inv(camera_from_target)
    organized = organized_xyz(cloud_msg)
    flat = organized.reshape(-1, 3)
    homogeneous = np.column_stack((flat, np.ones(len(flat))))
    target = (target_from_camera @ homogeneous.T).T[:, :3]
    camera_origin_target = target_from_camera[:3, 3]

    tcp = motion["base_to_tool0_tcp_mean"]
    base_from_tool = rt(tcp[3:], tcp[:3])
    tool_from_camera = np.eye(4)
    tool_from_camera[:3, :3] = np.asarray(calibration["rotation_matrix"], dtype=np.float64)
    tool_from_camera[:3, 3] = np.asarray(calibration["translation_m"], dtype=np.float64)
    base_from_target = base_from_tool @ tool_from_camera @ camera_from_target
    return {"target_points": target, "camera_origin_target": camera_origin_target,
            "base_from_target": base_from_target, "robot_motion": motion,
            "camera_from_target": camera_from_target,
            "image_size": [image.shape[1], image.shape[0]]}


def _jsonable(value):
    """``json.dump`` fallback: numpy scalars/arrays are not JSON-native.

    ``measure_cube_geometry`` reports plane normals, centres and footprints as
    numpy values, so serializing the report without this raises
    ``TypeError: Object of type ndarray is not JSON serializable`` — and it does
    so **after** the capture succeeded, which throws away the whole measurement.
    """
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError("Object of type %s is not JSON serializable" % type(value).__name__)


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--capture", action="store_true",
                        help="capture image+cloud from the running camera (read-only)")
    parser.add_argument("--npz", default=None,
                        help="offline scene saved by --save-cloud (no camera needed)")
    parser.add_argument("--save-cloud", default=None,
                        help="save the target-frame cloud + transforms to this .npz")
    parser.add_argument("--calibration", default="config/handeye_eye_in_hand_20260917.yaml")
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--expect-edge", type=float, default=DEFAULTS["expect_edge_m"])
    parser.add_argument("--min-coverage", type=float, default=DEFAULTS["min_coverage"])
    parser.add_argument("--allow-partial", action="store_true",
                        help="emit a visible-cluster diagnostic when top-face gates reject; "
                             "never infers a cube centre and never authorizes motion")
    parser.add_argument("--output", default=None)
    args = parser.parse_args()

    if args.npz:
        data = np.load(os.path.join(root, args.npz) if not os.path.isabs(args.npz) else args.npz)
        scene = {"target_points": data["target_points"],
                 "camera_origin_target": data["camera_origin_target"],
                 "base_from_target": data["base_from_target"],
                 "robot_motion": json.loads(str(data["robot_motion"]))}
    elif args.capture:
        scene = capture_scene(args, root)
    else:
        parser.error("choose --capture or --npz")

    cfg = {"expect_edge_m": args.expect_edge, "min_coverage": args.min_coverage,
           "allow_partial": args.allow_partial}
    result = measure_cube_geometry(scene["target_points"], scene["camera_origin_target"], cfg)
    if result["center_target_m"] is not None:
        center = np.asarray([*result["center_target_m"], 1.0])
        base = scene["base_from_target"] @ center
        result["center_base_m"] = base[:3].tolist()
    elif result.get("visible_cluster_centroid_target_m") is not None:
        visible = np.asarray([*result["visible_cluster_centroid_target_m"], 1.0])
        result["visible_cluster_centroid_base_m"] = (scene["base_from_target"] @ visible)[:3].tolist()

    stamp = datetime.now().astimezone().strftime("%Y%m%d-%H%M%S")
    result.update({
        "schema_version": 1,
        "measured_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "source": "capture" if args.capture else args.npz,
        "calibration": args.calibration,
        "robot_motion": scene.get("robot_motion"),
        "camera_from_target": scene.get("camera_from_target"),
        "motion_authorization": "none: measurement only, no robot command was sent",
    })

    if args.save_cloud:
        path = args.save_cloud if os.path.isabs(args.save_cloud) else os.path.join(root, args.save_cloud)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        np.savez_compressed(path,
                            target_points=np.asarray(scene["target_points"], dtype=np.float64),
                            camera_origin_target=np.asarray(scene["camera_origin_target"]),
                            base_from_target=np.asarray(scene["base_from_target"]),
                            robot_motion=json.dumps(scene.get("robot_motion")))
        result["saved_cloud"] = os.path.relpath(path, root)

    output = args.output or "outputs/vision/cube-measurement-%s.json" % stamp
    output = output if os.path.isabs(output) else os.path.join(root, output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False, default=_jsonable)
        stream.write("\n")
    print(json.dumps({k: result.get(k) for k in ("status", "reasons", "gates", "top_face",
                                                  "measured_height_m", "size_prior",
                                                  "center_target_m", "center_base_m",
                                                  "visible_cluster_centroid_target_m",
                                                  "visible_cluster_centroid_base_m")}, indent=2,
                     ensure_ascii=False, default=_jsonable))
    print("OUTPUT:", output)
    return 0 if result["status"] == "measured" else 3


if __name__ == "__main__":
    sys.exit(main())
