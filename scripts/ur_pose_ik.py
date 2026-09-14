#!/usr/bin/env python3
"""UR10 pose IK: Pinocchio FK plus bounded trust-region optimization."""
import argparse
import datetime
import json
import os
import time
import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares

URDF = os.path.expanduser("~/ur_learn/generated/ur10.urdf")
LOWER = np.deg2rad([-360, -360, -180, -360, -360, -360])
UPPER = -LOWER
POSITION_TOLERANCE = 1e-4
ROTATION_TOLERANCE = 1e-3
LIMIT_MARGIN = np.deg2rad(2.0)
MAX_SEED_DISTANCE = np.deg2rad(45.0)


class UR10IK:
    def __init__(self, urdf=URDF, frame="tool0"):
        self.model = pin.buildModelFromUrdf(urdf)
        self.data = self.model.createData()
        self.frame_id = self.model.getFrameId(frame)
        if self.frame_id >= len(self.model.frames):
            raise ValueError("URDF frame not found: %s" % frame)

    def pose(self, q):
        pin.forwardKinematics(self.model, self.data, np.asarray(q))
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.frame_id].copy()

    def solve(self, target, seed):
        seed = np.clip(np.asarray(seed, dtype=float), LOWER, UPPER)

        def residual(q):
            error = pin.log6(self.pose(q).inverse() * target).vector
            return np.r_[error[:3], 0.35 * error[3:], 1e-4 * (q - seed)]

        result = least_squares(residual, seed, bounds=(LOWER, UPPER), method="trf",
                               xtol=1e-11, ftol=1e-11, gtol=1e-11, max_nfev=300)
        error = pin.log6(self.pose(result.x).inverse() * target).vector
        return result.x, np.linalg.norm(error[:3]), np.linalg.norm(error[3:]), result

    def solve_checked(self, target, seed):
        q, pos_error, rot_error, result = self.solve(target, seed)
        reasons = []
        if not result.success:
            reasons.append("optimizer_failed")
        if pos_error > POSITION_TOLERANCE:
            reasons.append("position_residual")
        if rot_error > ROTATION_TOLERANCE:
            reasons.append("rotation_residual")
        if np.any(q < LOWER + LIMIT_MARGIN) or np.any(q > UPPER - LIMIT_MARGIN):
            reasons.append("joint_limit_margin")
        seed_distance = float(np.max(np.abs(q - np.asarray(seed))))
        if seed_distance > MAX_SEED_DISTANCE:
            reasons.append("joint_discontinuity")
        return q, pos_error, rot_error, seed_distance, reasons, result


def batch_test(ik, count, random_seed):
    rng = np.random.default_rng(random_seed)
    rows = []
    for index in range(count):
        reference = rng.uniform(np.deg2rad([-150, -150, -150, -150, -150, -150]),
                                np.deg2rad([150, 150, 150, 150, 150, 150]))
        seed = reference + rng.normal(0.0, 0.08, 6)
        started = time.perf_counter()
        q, pe, re, jump, reasons, result = ik.solve_checked(ik.pose(reference), seed)
        rows.append({"case": index + 1, "accepted": not reasons,
                     "position_error_m": float(pe), "rotation_error_rad": float(re),
                     "max_seed_distance_rad": jump, "solve_ms": (time.perf_counter()-started)*1000,
                     "nfev": result.nfev, "reasons": reasons,
                     "reference_q": reference.tolist(), "solution_q": q.tolist()})
    unreachable = pin.SE3(np.eye(3), np.array([3.0, 0.0, 2.0]))
    _, pe, re, _, reasons, _ = ik.solve_checked(unreachable, np.zeros(6))
    unreachable_rejected = bool(reasons)
    boundary = {}
    for name, degrees in (
            ("safe_near_limit", [350, -350, 170, 350, -350, 350]),
            ("inside_limit_margin", [359, -359, 179, 359, -359, 359])):
        reference = np.deg2rad(degrees)
        _, bpe, bre, _, breasons, _ = ik.solve_checked(ik.pose(reference), reference)
        boundary[name] = {"accepted": not breasons, "reasons": breasons,
                          "position_error_m": float(bpe), "rotation_error_rad": float(bre)}
    return rows, {"position_error_m": float(pe), "rotation_error_rad": float(re),
                  "reasons": reasons, "rejected": unreachable_rejected}, boundary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default=URDF)
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--batch-test", type=int, metavar="N")
    ap.add_argument("--random-seed", type=int, default=20260914)
    args = ap.parse_args()
    if not args.self_test and not args.batch_test:
        ap.error("请使用 --self-test 或 --batch-test N")
    ik = UR10IK(args.urdf)
    if args.batch_test:
        rows, unreachable, boundary = batch_test(ik, args.batch_test, args.random_seed)
        accepted = sum(row["accepted"] for row in rows)
        report = {"generated_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  "solver": "Pinocchio FK + SciPy TRF bounded least_squares",
                  "random_seed": args.random_seed, "cases": rows,
                  "unreachable_case": unreachable, "boundary_cases": boundary}
        out_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                               "outputs", "ik")
        os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(out_dir, "ik-batch-%s.json" %
                            datetime.datetime.now().strftime("%Y%m%d-%H%M%S"))
        with open(path, "w") as stream:
            json.dump(report, stream, indent=2)
        times = [row["solve_ms"] for row in rows]
        print("accepted=%d/%d median_ms=%.3f max_ms=%.3f" %
              (accepted, len(rows), np.median(times), max(times)))
        print("unreachable_rejected=%s residual=%.4fm reasons=%s" %
              (unreachable["rejected"], unreachable["position_error_m"],
               unreachable["reasons"]))
        print("boundary", boundary)
        print("report", path)
        if (accepted != len(rows) or not unreachable["rejected"] or
                not boundary["safe_near_limit"]["accepted"] or
                boundary["inside_limit_margin"]["accepted"]):
            raise RuntimeError("IK batch test failed")
        print("PASS: IK batch acceptance gates")
        return
    reference = np.array([0.35, -1.15, 1.25, -1.65, -1.1, 0.4])
    started = time.perf_counter()
    q, pos_error, rot_error, result = ik.solve(ik.pose(reference), reference + 0.08)
    solve_ms = (time.perf_counter() - started) * 1000.0
    print("success=%s nfev=%d" % (result.success, result.nfev))
    print("q", np.round(q, 7))
    print("position_error_m=%.9g rotation_error_rad=%.9g" % (pos_error, rot_error))
    print("solve_ms=%.3f" % solve_ms)
    if not result.success or pos_error > 1e-5 or rot_error > 1e-4:
        raise RuntimeError("IK self-test failed")
    print("PASS: bounded trust-region IK")


if __name__ == "__main__":
    main()
