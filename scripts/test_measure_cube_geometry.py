#!/usr/bin/env python3
"""Offline numerical gate for ``scripts/measure_cube_geometry.py`` (no camera, no robot).

It ray-casts a synthetic scene - a table, the checkerboard plate and a cube - through a
pinhole grid from a chosen camera pose, so the point set reproduces what the eye-in-hand
camera really produces: an organised grid with holes, self-occlusion, only the top face
plus 1-2 side faces visible, range noise and occasional spikes.  The measured centre is
then compared with the known synthetic centre.

Cases that MUST pass (centre recovered) and cases that MUST be rejected (the old
single-side locator would still have emitted a centre here) are both asserted, because a
geometry fix that cannot refuse a degenerate view is not a fix.

    python3 scripts/test_measure_cube_geometry.py
"""
import json
import os
import sys
from datetime import datetime

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from measure_cube_geometry import measure_cube_geometry  # noqa: E402

BOARD_HALF = (0.054, 0.036)


def camera_frame(position, look_at):
    forward = np.asarray(look_at, dtype=np.float64) - np.asarray(position, dtype=np.float64)
    forward /= np.linalg.norm(forward)
    world_up = np.array([0.0, 0.0, 1.0])
    right = np.cross(forward, world_up)
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    return forward, right, up


def ray_grid(position, look_at, fov_deg, width, height):
    forward, right, up = camera_frame(position, look_at)
    half = np.tan(np.radians(fov_deg) / 2.0)
    aspect = width / height
    xs = np.linspace(-half * aspect, half * aspect, width)
    ys = np.linspace(half, -half, height)
    gx, gy = np.meshgrid(xs, ys)
    directions = forward[None, :] + gx.reshape(-1, 1) * right[None, :] + \
        gy.reshape(-1, 1) * up[None, :]
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    origins = np.tile(np.asarray(position, dtype=np.float64), (len(directions), 1))
    return origins, directions, (height, width)


def ray_plane(origins, directions, plane_z, inside_mask):
    """Hit a horizontal surface at plane_z, restricted to ``inside_mask`` of the board rect."""
    t = np.full(len(origins), np.inf)
    dz = directions[:, 2]
    valid = np.abs(dz) > 1e-9
    t_candidate = np.full(len(origins), np.inf)
    t_candidate[valid] = (plane_z - origins[valid, 2]) / dz[valid]
    hit = origins + t_candidate[:, None] * directions
    on_surface = inside_mask(hit[:, 0], hit[:, 1])
    good = valid & on_surface & (t_candidate > 1e-6)
    t[good] = t_candidate[good]
    return t


def ray_box(origins, directions, center, edge, yaw_rad):
    """Slab test against a yaw-rotated cube; returns t and the hit face normal."""
    c, s = np.cos(-yaw_rad), np.sin(-yaw_rad)
    rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    local_o = (origins - center) @ rot
    local_d = directions @ rot
    half = edge / 2.0
    t_enter = np.full(len(origins), -np.inf)
    t_exit = np.full(len(origins), np.inf)
    axis = np.zeros(len(origins), dtype=np.int32)
    sign = np.ones(len(origins))
    for k in range(3):
        d = local_d[:, k]
        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = (-half - local_o[:, k]) / d
            t2 = (half - local_o[:, k]) / d
        lo = np.minimum(t1, t2)
        hi = np.maximum(t1, t2)
        update = lo > t_enter
        t_enter = np.where(update, lo, t_enter)
        axis = np.where(update, k, axis)
        sign = np.where(update, np.sign(d), sign)
        t_exit = np.minimum(t_exit, hi)
    hit = (t_enter > 1e-6) & (t_enter <= t_exit)
    t = np.where(hit, t_enter, np.inf)
    normals = np.zeros_like(origins)
    normals[np.arange(len(origins)), axis] = -np.sign(sign)  # outward face normal
    normals = normals @ rot.T
    return t, normals


def synth_scene(center_xy, edge, yaw_deg, camera, fov_deg=32.0, width=260, height=200,
                noise_m=0.0003, dropout=0.08, outliers=0.01, board_thickness=0.0, seed=3):
    """Ray-cast table + checkerboard plate + cube; return target-frame organised points."""
    rng = np.random.default_rng(seed)
    cube_center = np.array([center_xy[0], center_xy[1], -board_thickness + edge / 2.0])
    origins, directions, shape = ray_grid(camera, cube_center, fov_deg, width, height)

    def in_board(x, y):
        return (np.abs(x) < BOARD_HALF[0]) & (np.abs(y) < BOARD_HALF[1])

    table_mask = lambda x, y: ~in_board(x, y)          # noqa: E731
    board_mask = lambda x, y: in_board(x, y)            # noqa: E731
    t_board = ray_plane(origins, directions, 0.0, board_mask)
    t_table = ray_plane(origins, directions, -board_thickness, table_mask)
    t_cube, _ = ray_box(origins, directions, cube_center, edge, np.radians(yaw_deg))

    t = np.minimum(np.minimum(t_board, t_table), t_cube)
    points = origins + t[:, None] * directions
    valid = np.isfinite(t)

    points[valid] += rng.normal(0.0, noise_m, size=(int(valid.sum()), 3))
    keep = valid & (rng.random(len(t)) > dropout)
    if outliers > 0:
        spike = keep & (rng.random(len(t)) < outliers)
        points[spike] += rng.normal(0.0, 0.02, size=(int(spike.sum()), 3))
    points[~keep] = np.nan
    return points.reshape(*shape, 3), cube_center


def evaluate(name, cloud, camera, truth, expect_edge, expect_status, tolerance_m):
    result = measure_cube_geometry(cloud, camera, {"expect_edge_m": expect_edge})
    measured = result["center_target_m"]
    error = None
    if measured is not None:
        error = float(np.linalg.norm(np.asarray(measured) - truth))

    if expect_status == "rejected":
        passed = result["status"] == "rejected"
    else:
        passed = result["status"] == expect_status and error is not None and error <= tolerance_m
    return {
        "case": name,
        "expected_status": expect_status,
        "status": result["status"],
        "reasons": result["reasons"],
        "gates": result["gates"],
        "center_error_mm": None if error is None else error * 1000.0,
        "tolerance_mm": tolerance_m * 1000.0,
        "measured_height_mm": None if result.get("measured_height_m") is None
        else result["measured_height_m"] * 1000.0,
        "measured_edges_mm": None if "footprint" not in result.get("top_face", {})
        else [e * 1000.0 for e in result["top_face"]["footprint"]["edges_m"]],
        "coverage": None if "footprint" not in result.get("top_face", {})
        else result["top_face"]["footprint"]["coverage"],
        "height_vs_edge_mm": result.get("height_vs_edge_mm"),
        "truth_center_m": truth.tolist(),
        "measured_center_m": measured,
        "passed": bool(passed),
    }


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    cases = []

    # 1. well-posed oblique view of a nominal 50 mm cube
    cam = np.array([0.36, -0.14, 0.30])
    cloud, truth = synth_scene((0.06, -0.12), 0.05, 12.0, cam)
    cases.append(evaluate("oblique_top_visible_50mm", cloud, cam, truth, 0.05, "measured", 0.0015))

    # 2. same view, but the cube is really 45 mm: the centre must still be measured and the
    #    nominal size prior must be reported as mismatched rather than silently trusted
    cloud, truth = synth_scene((0.06, -0.12), 0.045, 12.0, cam)
    cases.append(evaluate("measured_45mm_size_prior_mismatch", cloud, cam, truth, 0.05,
                          "measured_size_prior_mismatch", 0.0015))

    # 3. grazing view: only a vertical side face is visible - the exact 2026-09-17 situation.
    #    The tool must refuse instead of inferring a centre from the side plus a size prior.
    graze = np.array([0.62, -0.12, 0.06])
    cloud, truth = synth_scene((0.06, -0.12), 0.05, 12.0, graze)
    cases.append(evaluate("grazing_view_must_reject", cloud, graze, truth, 0.05, "rejected", 0.0))

    # 4. noisy, sparse capture with spikes and 30% dropout
    cloud, truth = synth_scene((0.06, -0.12), 0.05, 12.0, cam, noise_m=0.0006,
                               dropout=0.30, outliers=0.02, seed=11)
    cases.append(evaluate("noisy_sparse_with_outliers", cloud, cam, truth, 0.05, "measured", 0.002))

    # 5. board as a 5 mm plate on the table: the standing surface is the table, not the board
    cloud, truth = synth_scene((0.06, -0.12), 0.05, 0.0, cam, board_thickness=0.005)
    cases.append(evaluate("board_plate_5mm_standing_surface", cloud, cam, truth, 0.05, "measured", 0.0015))

    # 6. cube rotated 20 deg about the vertical: edges must still be recovered
    cloud, truth = synth_scene((0.06, -0.12), 0.05, 20.0, cam)
    cases.append(evaluate("cube_yaw_20deg", cloud, cam, truth, 0.05, "measured", 0.002))

    report = {
        "schema_version": 1,
        "generated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "purpose": "offline numerical gate for the top-face cube measurement before any real capture",
        "scene_model": "pinhole ray cast of table + checkerboard plate + yaw-rotated cube, "
                       "organised grid with holes, self-occlusion, range noise and spikes",
        "cases": cases,
        "passed": all(case["passed"] for case in cases),
        "failed_cases": [case["case"] for case in cases if not case["passed"]],
    }
    output = os.path.join(root, "outputs/vision/measure-cube-offline-selftest-20260918.json")
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, ensure_ascii=False)
        stream.write("\n")

    for case in cases:
        print("%-40s %-32s %s" % (
            case["case"], case["status"],
            "PASS" if case["passed"] else "FAIL -> %s" % "; ".join(case["reasons"])))
        if case["center_error_mm"] is not None:
            print("      centre error %.3f mm (tol %.2f), height %.2f mm, edges %s, coverage %.3f"
                  % (case["center_error_mm"], case["tolerance_mm"], case["measured_height_mm"],
                     [round(e, 2) for e in case["measured_edges_mm"]], case["coverage"]))
    print("OFFLINE GATE:", "PASS" if report["passed"] else "FAIL")
    print("OUTPUT:", output)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
