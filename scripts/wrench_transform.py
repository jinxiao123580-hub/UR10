#!/usr/bin/env python3
"""Pure spatial-wrench frame transform with a no-hardware self-test.

Convention: ``r_tool_sensor`` is the vector from the tool0 origin to the
ft_sensor origin, expressed in tool0.  ``rotation_tool_sensor`` maps a vector
expressed in ft_sensor into tool0.  The returned torque is about tool0.
"""
import numpy as np


def transform_wrench(wrench_sensor, rotation_tool_sensor, r_tool_sensor):
    """Express a wrench measured about ft_sensor origin about tool0 origin."""
    wrench_sensor = np.asarray(wrench_sensor, dtype=float)
    rotation_tool_sensor = np.asarray(rotation_tool_sensor, dtype=float)
    r_tool_sensor = np.asarray(r_tool_sensor, dtype=float)
    if wrench_sensor.shape != (6,):
        raise ValueError("wrench_sensor must contain [Fx,Fy,Fz,Tx,Ty,Tz]")
    if rotation_tool_sensor.shape != (3, 3):
        raise ValueError("rotation_tool_sensor must be 3x3")
    if r_tool_sensor.shape != (3,):
        raise ValueError("r_tool_sensor must have three components")
    force_tool = rotation_tool_sensor @ wrench_sensor[:3]
    torque_tool = (rotation_tool_sensor @ wrench_sensor[3:]
                   + np.cross(r_tool_sensor, force_tool))
    return np.r_[force_tool, torque_tool]


def self_test():
    identity = np.eye(3)
    wrench = np.array([0.0, 10.0, 0.0, 0.0, 0.0, 0.0])
    actual = transform_wrench(wrench, identity, [0.1, 0.0, 0.0])
    expected = np.array([0.0, 10.0, 0.0, 0.0, 0.0, 1.0])
    np.testing.assert_allclose(actual, expected, atol=1e-12)
    rotated = np.diag([-1.0, -1.0, 1.0])
    actual = transform_wrench([1, 2, 3, 4, 5, 6], rotated, [0, 0, 0])
    np.testing.assert_allclose(actual, [-1, -2, 3, -4, -5, 6], atol=1e-12)
    print("self-test passed")


if __name__ == "__main__":
    self_test()
