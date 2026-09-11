#!/usr/bin/env python3
"""Compare force gravity model conventions on calibration and validation data."""
import argparse
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from calibrate_ft_gravity import GRAVITY, rotvec_to_matrix  # noqa: E402


def load_rows(path, validation=False):
    with open(path, encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    rows = []
    for row in document["samples"]:
        wrench = row["raw"]["mean"] if validation else row["wrench_mean"]
        rows.append((rotvec_to_matrix(row["tcp_pose"][3:]),
                     np.asarray(wrench[:3], dtype=float)))
    return rows


def gravity_vectors(rows, transpose):
    gravity = np.array([0.0, 0.0, -GRAVITY])
    return np.array([(rotation.T if transpose else rotation) @ gravity
                     for rotation, _ in rows])


def project_scaled_rotation(a):
    u, _, vt = np.linalg.svd(a)
    rotation = u @ np.diag([1.0, 1.0, np.linalg.det(u @ vt)]) @ vt
    mass = float(np.trace(rotation.T @ a) / 3.0)
    return mass, rotation


def rms(error):
    return float(np.sqrt(np.mean(np.asarray(error) ** 2)))


def evaluate(train, validation, transpose):
    g_train = gravity_vectors(train, transpose)
    f_train = np.array([force for _, force in train])
    design = np.hstack([g_train, np.ones((len(train), 1))])
    coeff = np.linalg.lstsq(design, f_train, rcond=None)[0]
    a = coeff[:3].T
    bias_linear = coeff[3]

    g_validation = gravity_vectors(validation, transpose)
    f_validation = np.array([force for _, force in validation])
    linear_train = design @ coeff
    linear_validation = np.hstack(
        [g_validation, np.ones((len(validation), 1))]) @ coeff

    mass, rotation = project_scaled_rotation(a)
    constrained_train = np.array([mass * rotation @ g for g in g_train])
    bias = np.mean(f_train - constrained_train, axis=0)
    constrained_train += bias
    constrained_validation = np.array(
        [mass * rotation @ g + bias for g in g_validation])
    validation_error = f_validation - constrained_validation
    singular = np.linalg.svd(a, compute_uv=False)
    return {
        "mass_kg": mass,
        "axis_scales_kg": singular.tolist(),
        "linear_train_rms_n": rms(f_train - linear_train),
        "linear_validation_rms_n": rms(f_validation - linear_validation),
        "linear_bias_n": bias_linear.tolist(),
        "constrained_train_rms_n": rms(f_train - constrained_train),
        "constrained_validation_rms_n": rms(validation_error),
        "constrained_validation_mean_n": validation_error.mean(axis=0).tolist(),
        "constrained_validation_rows_n": validation_error.tolist(),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration", default="config/ft_gravity_calibration.yaml")
    parser.add_argument("--validation", default="config/ft_gravity_validation.yaml")
    args = parser.parse_args()
    train = load_rows(args.calibration)
    validation = load_rows(args.validation, validation=True)
    for label, transpose in (("R_base_tool.T @ g", True),
                             ("R_base_tool @ g", False)):
        print("\n" + label)
        for key, value in evaluate(train, validation, transpose).items():
            print("  %s: %s" % (key, np.round(value, 6)))


if __name__ == "__main__":
    main()
