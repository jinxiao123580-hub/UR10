#!/usr/bin/env python3
"""Single source of truth for the real end-effector geometry.

Everything that needs to know what is physically bolted to the flange loads this:
the URDF generator behind RViz/MuJoCo, and the self-collision gate.  Keeping one
loader means the picture and the gate can never drift apart.

Values come from ``config/robot_attachments.yaml`` (operator measurements, mm) and,
for the camera placement, fall back to the 2026-09-17 hand-eye calibration
``tool0_from_camera``.
"""
import os

import numpy as np

DEFAULTS = {
    "force_sensor": {"enabled": True, "diameter_mm": 80.0, "height_mm": 30.0,
                     "offset_mm": 0.0},
    "adapter": {"enabled": False, "size_mm": [60.0, 60.0, 10.0]},
    "gripper": {"enabled": True, "size_mm": [80.0, 150.0, 70.0], "offset_mm": 30.0},
    "camera_bracket": {"width_mm": 50.0, "length_mm": None},
    "camera_housing": {"size_mm": [200.0, 200.0, 150.0]},
    "handeye_calibration": "config/handeye_eye_in_hand_20260917.yaml",
    "camera_offset_tool0_m": None,
    "camera_rotation_source": "handeye",
    "self_collision_margin_mm": 40.0,
}


def _merge(base, override):
    result = dict(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge(result[key], value)
        else:
            result[key] = value
    return result


def load(config_path=None, calibration_path=None, root=None):
    """Return a dict of geometry in METRES, plus the tool0_from_camera matrix."""
    root = root or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    def resolve(path, default):
        if path is None:
            path = os.path.join(root, default)
        elif not os.path.isabs(path):
            path = os.path.join(root, path)
        return path

    import yaml
    config_path = resolve(config_path, "config/robot_attachments.yaml")
    document = dict(DEFAULTS)
    if os.path.exists(config_path):
        with open(config_path, encoding="utf-8") as stream:
            document = _merge(DEFAULTS, yaml.safe_load(stream) or {})
    # A configuration may pin the hand-eye result it was built around.  Explicit
    # caller input still wins, preserving the existing command-line behaviour.
    calibration_path = resolve(calibration_path, document["handeye_calibration"])

    tool0_from_camera = np.eye(4)
    if document.get("camera_rotation_source", "handeye") == "handeye":
        with open(calibration_path, encoding="utf-8") as stream:
            calibration = yaml.safe_load(stream)
        tool0_from_camera[:3, :3] = np.asarray(calibration["rotation_matrix"],
                                               dtype=float)
        tool0_from_camera[:3, 3] = np.asarray(calibration["translation_m"],
                                              dtype=float)
        camera_offset_source = "handeye calibration %s" % os.path.basename(
            calibration_path)
    else:
        camera_offset_source = "identity rotation (camera aligned with tool0)"
    override = document.get("camera_offset_tool0_m")
    if override is not None:
        tool0_from_camera[:3, 3] = np.asarray(override, dtype=float)
        camera_offset_source = ("measured camera_offset_tool0_m %s (overrides the "
                                "calibration)" % list(override))

    def millimetres(values):
        return [float(value) / 1000.0 for value in values]

    geometry = {
        "force_sensor": {
            "enabled": bool(document["force_sensor"]["enabled"]),
            "diameter_m": float(document["force_sensor"]["diameter_mm"]) / 1000.0,
            "height_m": float(document["force_sensor"]["height_mm"]) / 1000.0,
            "offset_m": float(document["force_sensor"]["offset_mm"]) / 1000.0,
        },
        "adapter": {
            "enabled": bool(document["adapter"]["enabled"]),
            "size_m": millimetres(document["adapter"]["size_mm"]),
        },
        "gripper": {
            "enabled": bool(document["gripper"]["enabled"]),
            "size_m": millimetres(document["gripper"]["size_mm"]),
            "offset_m": float(document["gripper"]["offset_mm"]) / 1000.0,
        },
        "camera_bracket": {
            "width_m": float(document["camera_bracket"]["width_mm"]) / 1000.0,
            "length_m": (None if document["camera_bracket"]["length_mm"] is None
                         else float(document["camera_bracket"]["length_mm"]) / 1000.0),
        },
        "camera_housing": {
            "size_m": millimetres(document["camera_housing"]["size_mm"]),
        },
        "tool0_from_camera": tool0_from_camera,
        "camera_offset_tool0_m": np.asarray(tool0_from_camera[:3, 3], dtype=float),
        "camera_offset_source": camera_offset_source,
        "self_collision_margin_mm": float(document.get("self_collision_margin_mm",
                                                       40.0)),
        "config_path": config_path,
    }
    if geometry["camera_bracket"]["length_m"] is None:
        geometry["camera_bracket"]["length_m"] = float(
            np.linalg.norm(geometry["camera_offset_tool0_m"]))
    return geometry


def summary(geometry):
    lines = [
        "末端附件几何（来源 %s）" % os.path.basename(geometry["config_path"]),
        "  相机中心相对 tool0: %s m  [%s]" % (
            np.round(geometry["camera_offset_tool0_m"], 4).tolist(),
            geometry["camera_offset_source"]),
        "  相机外壳: %s mm" % [round(x * 1000, 1)
                             for x in geometry["camera_housing"]["size_m"]],
        "  相机支架: %.0f x %.0f x %.0f mm" % (
            geometry["camera_bracket"]["width_m"] * 1000,
            geometry["camera_bracket"]["width_m"] * 1000,
            geometry["camera_bracket"]["length_m"] * 1000),
    ]
    if geometry["force_sensor"]["enabled"]:
        lines.append("  力传感器: 直径 %.0f mm x 高 %.0f mm" % (
            geometry["force_sensor"]["diameter_m"] * 1000,
            geometry["force_sensor"]["height_m"] * 1000))
    if geometry["adapter"]["enabled"]:
        lines.append("  转接件: %s mm" % [round(x * 1000, 1)
                                        for x in geometry["adapter"]["size_m"]])
    if geometry["gripper"]["enabled"]:
        lines.append("  夹爪: %s mm（起点 %.0f mm）" % (
            [round(x * 1000, 1) for x in geometry["gripper"]["size_m"]],
            geometry["gripper"]["offset_m"] * 1000))
    return "\n".join(lines)


if __name__ == "__main__":
    print(summary(load()))
