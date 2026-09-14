#!/usr/bin/env python3
"""UR10 pose IK: Pinocchio FK plus bounded trust-region optimization."""
import argparse
import os
import time
import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares

URDF = os.path.expanduser("~/ur_learn/generated/ur10.urdf")
LOWER = np.deg2rad([-360, -360, -180, -360, -360, -360])
UPPER = -LOWER


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default=URDF)
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if not args.self_test:
        ap.error("目前仅开放 --self-test；真机目标接口要等安全门禁完成")
    ik = UR10IK(args.urdf)
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
