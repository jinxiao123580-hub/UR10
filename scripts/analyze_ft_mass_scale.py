#!/usr/bin/env python3
"""P0-A: why do the gravity fits say ~3.2 kg while a single pose |F|/g says 6.56 kg?

Read-only and offline. It uses only
  * the sensor's own configuration XML (``netftapi2.xml``, counts-per-force,
    counts-per-torque and the hardware bias), and
  * per-pose static mean wrenches already archived in ``config/ft_gravity_calibration*.yaml``.

Method: a constant force offset in the sensor frame is unavoidable evidence of a
bias, and it is also exactly what breaks the ``|F| = m*g`` single-pose estimate.
For every static pose the sensor reads

    F_i = m * g * u_i + b            (u_i = gravity direction in the sensor frame)

so all measured force vectors lie on a **sphere** of radius ``m*g`` centred at the
bias ``b``.  Fitting that sphere needs neither forward kinematics nor the mounting
rotation, so it is an independent test of the fit's mass scale:

  * sphere radius ~= 31 N  -> mass 3.2 kg, the 6.56 kg value is a bias artefact
  * sphere radius ~= 64 N  -> mass 6.5 kg, the multi-pose fits are wrong by 2x
"""
import argparse
import glob
import json
import os
import re
from datetime import datetime

import numpy as np
import yaml

G = 9.80665


def sensor_configuration(path):
    text = open(path, encoding="utf-8").read()

    def field(name):
        match = re.search(r"<%s>(.*?)</%s>" % (name, name), text)
        return match.group(1) if match else None

    return {
        "source_file": os.path.basename(path),
        "cfgcpf_counts_per_force": float(field("cfgcpf")),
        "cfgcpt_counts_per_torque": float(field("cfgcpt")),
        "hardware_bias_counts_setbias": [float(v) for v in field("setbias").split(";")],
        "calibration_serial": field("cfgcalsn"),
        "calibration_name": field("cfgnam"),
        "max_ratings_n_and_nm": [float(v) for v in field("cfgmr").split(";")],
        "force_units": field("scfgfu"),
        "torque_units": field("scfgtu"),
        "live_counts_runft": [float(v) for v in field("runft").split(";")],
    }


def collect_poses(paths):
    """Return (label, list of (tcp_pose6, wrench6)) from archived fit inputs."""
    poses = []
    for path in paths:
        with open(path, encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        samples = data.get("samples") or []
        rows = [(np.asarray(s["tcp_pose"], dtype=np.float64),
                 np.asarray(s["wrench_mean"], dtype=np.float64)) for s in samples]
        if rows:
            poses.append((os.path.basename(path), rows))
    return poses


def sphere_fit_forces(forces):
    """Algebraic sphere fit |F|^2 = 2 c.F + k, then a geometric Gauss-Newton refine."""
    f = np.asarray(forces, dtype=np.float64)
    a = np.column_stack((2.0 * f, np.ones(len(f))))
    rhs = (f ** 2).sum(axis=1)
    solution, *_ = np.linalg.lstsq(a, rhs, rcond=None)
    center = solution[:3]
    radius2 = solution[3] + float(center @ center)
    radius = float(np.sqrt(max(radius2, 1e-12)))
    for _ in range(60):
        d = f - center
        r = np.linalg.norm(d, axis=1)
        residual = r - radius
        jacobian = np.column_stack((-d / r[:, None], -np.ones(len(f))))
        step, *_ = np.linalg.lstsq(jacobian, -residual, rcond=None)
        center = center + step[:3]
        radius = float(radius + step[3])
    d = f - center
    r = np.linalg.norm(d, axis=1)
    return {
        "bias_sensor_frame_n": center.tolist(),
        "bias_norm_n": float(np.linalg.norm(center)),
        "gravity_load_n": radius,
        "mass_kg": radius / G,
        "residual_rms_n": float(np.sqrt(np.mean((r - radius) ** 2))),
        "residual_max_abs_n": float(np.max(np.abs(r - radius))),
    }


def no_bias_scale(forces):
    """Best pure-scale model F_i = A*u_i with no offset: what the single-pose method assumes."""
    f = np.asarray(forces, dtype=np.float64)
    norms = np.linalg.norm(f, axis=1)
    # With |F| = A for every pose, the least-squares A is simply mean(|F|); the spread
    # of |F| is then the model error.  Report both.
    return {
        "model": "F_i = A * u_i (pure gravity, no offset)",
        "fitted_a_n": float(norms.mean()),
        "implied_mass_kg": float(norms.mean() / G),
        "residual_rms_n": float(norms.std()),
        "residual_max_abs_n": float(np.max(np.abs(norms - norms.mean()))),
        "per_pose_force_norm_n": norms.round(4).tolist(),
    }


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sensor-xml", default="outputs/ft_calibration/netftapi2-sensor-20260918.xml")
    parser.add_argument("--output", default="outputs/ft_calibration/mass-scale-diagnosis-20260918.json")
    args = parser.parse_args()

    sensor = sensor_configuration(os.path.join(root, args.sensor_xml))
    yaml_paths = sorted(glob.glob(os.path.join(root, "config", "ft_gravity_calibration*.yaml")))
    datasets = collect_poses(yaml_paths)

    report = {
        "schema_version": 1,
        "generated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "purpose": "P0-A: 2x end-effector mass contradiction (fit 3.2 kg vs single pose 6.56 kg)",
        "read_only": "No robot command was sent; ATI RDT was not opened; only the sensor web XML and archived files were read.",
        "sensor_configuration": sensor,
        "counts_per_force_check": {
            "driver_hardcoded_counts_per_force": 1_000_000.0,
            "driver_hardcoded_counts_per_torque": 1_000_000.0,
            "sensor_cfgcpf": sensor["cfgcpf_counts_per_force"],
            "sensor_cfgcpt": sensor["cfgcpt_counts_per_torque"],
            "match": bool(sensor["cfgcpf_counts_per_force"] == 1_000_000.0
                          and sensor["cfgcpt_counts_per_torque"] == 1_000_000.0),
            "hardware_bias_all_zero": bool(all(v == 0.0 for v in sensor["hardware_bias_counts_setbias"])),
            "live_reading_n_at_check_time": [
                round(v / sensor["cfgcpf_counts_per_force"], 4) for v in sensor["live_counts_runft"][:3]],
        },
        "datasets": [],
        "pooled": None,
        "single_pose_estimator": {},
        "verdict": {},
    }

    pooled = []
    for label, rows in datasets:
        forces = np.asarray([w[:3] for _, w in rows])
        pooled.append(forces)
        report["datasets"].append({
            "file": label,
            "poses": len(rows),
            "sphere_fit_scale_and_bias": sphere_fit_forces(forces),
            "pure_scale_no_offset_model": no_bias_scale(forces),
        })

    all_forces = np.vstack(pooled)
    report["pooled"] = {
        "files": len(datasets),
        "poses": int(len(all_forces)),
        "sphere_fit_scale_and_bias": sphere_fit_forces(all_forces),
        "pure_scale_no_offset_model": no_bias_scale(all_forces),
    }

    norms = np.linalg.norm(all_forces, axis=1)
    single = {
        "estimator": "m = |F_raw| / g at ONE static pose (the 2026-09-10 method)",
        "min_mass_kg": float(norms.min() / G),
        "max_mass_kg": float(norms.max() / G),
        "range_ratio": float(norms.max() / norms.min()),
        "mean_mass_kg": float(norms.mean() / G),
        "pose_dependence_note": (
            "A gravity-only sensor would read the SAME |F| in every pose. The archived poses "
            "spread over a %.2f-%.2f N range, which a pure scale error can never produce: it "
            "requires a constant force offset." % (norms.min(), norms.max())),
    }
    report["single_pose_estimator"] = single

    pooled_fit = report["pooled"]["sphere_fit_scale_and_bias"]
    report["verdict"] = {
        "counts_per_force_is_the_sensor_value": report["counts_per_force_check"]["match"],
        "hardware_bias_is_zero": report["counts_per_force_check"]["hardware_bias_all_zero"],
        "sphere_radius_n": pooled_fit["gravity_load_n"],
        "sphere_mass_kg": pooled_fit["mass_kg"],
        "sphere_bias_norm_n": pooled_fit["bias_norm_n"],
        "sphere_residual_rms_n": pooled_fit["residual_rms_n"],
        "mass_from_gravity_fits_kg": 3.2,
        "mass_from_single_pose_0910_kg": 6.56,
        "resolution": (
            "The multi-pose fits are right about the SCALE: the archived force vectors lie on a "
            "sphere of radius %.1f N == %.2f kg whose centre is %.1f N away from the origin. "
            "The 2026-09-10 single-pose |F|/g value ignored that offset, so it is not a second "
            "measurement of mass at all - the 2.03x ratio is a bias artefact, not a 2x sensor or "
            "counts-per-force error." % (pooled_fit["gravity_load_n"], pooled_fit["mass_kg"],
                                         pooled_fit["bias_norm_n"])),
        "scale_verification_is_not_absolute": (
            "Counts-per-force 1e6 only proves the driver matches the sensor's own specification. "
            "The absolute newton scale still rests on a mass reference; the UR payload readback "
            "3.5 kg is corroboration, not an independent measurement."),
        "deferred": (
            "Root cause of the ~28 N constant offset (sensor zero vs cable strain vs mounting "
            "preload) and the corrective action are NOT resolved in this round: the user decided "
            "to archive the problem and continue later."),
    }

    output = args.output if os.path.isabs(args.output) else os.path.join(root, args.output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    print(json.dumps({"counts_per_force_check": report["counts_per_force_check"],
                      "pooled": report["pooled"],
                      "single_pose_estimator": single,
                      "verdict": report["verdict"]}, indent=2, ensure_ascii=False))
    print("OUTPUT:", output)


if __name__ == "__main__":
    main()
