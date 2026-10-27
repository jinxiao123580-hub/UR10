"""Factory-calibrated kinematics for UR10 CB3 serial 2022300602.

The values are copied from the real-robot repository's verified
``config/ur10_factory_calibration.yaml``.  Transforms are expressed in the UR
controller ``base`` frame, which is the frame used by the controller TCP pose.
"""

import math
from typing import Sequence

import numpy as np


CALIBRATION_HASH = "calib_15120592593058779304"
ROBOT_MODEL = "UR10 CB3"
ROBOT_SERIAL = "2022300602"

# xyz + fixed-axis rpy for each URDF revolute joint, in chain order.
FACTORY_JOINT_ORIGINS = (
    ((0.0, 0.0, 0.12825181172186206),
     (0.0, 0.0, -5.6460204029449082e-05)),
    ((5.1805159516364943e-05, 0.0, 0.0),
     (1.5706719269899678, 0.0, 1.5727956372191948e-05)),
    ((-0.6123833190672483, 0.0, 0.0),
     (3.1382365514883417, 3.1415661039055771, -3.1415384532606816)),
    ((-0.57143067297075534, 0.0003930062332988286, 0.16358048424247151),
     (3.1391901330265317, -3.141519693345717, 3.1415835322576635)),
    ((2.4084619826573817e-05, -0.11568595442769607,
      5.0442556072512075e-05),
     (1.5703602967415915, 0.0, -3.664276842906769e-05)),
    ((2.9069810266962225e-05, 0.092166082125499024,
      4.9531942445487072e-05),
     (1.5713337472486533, math.pi, -3.1415839915094663)),
)

WRIST3_TO_FLANGE_RPY = (0.0, -math.pi / 2.0, -math.pi / 2.0)
FLANGE_TO_TOOL0_RPY = (math.pi / 2.0, 0.0, math.pi / 2.0)
# Historical optical calibration, retained as a record; not the new housing pose.
MECH_EYE_TRANSLATION_M = (0.04849423522761702, 0.25202240709668144,
                          0.19931610726118562)
MECH_EYE_RPY = (-1.1547020625425162, -0.00025322052810483777,
                -3.1353642944665703)

# SolidWorks assembly registration from the real-robot repository.  The CAD
# source is in millimetres, its -Y axis maps to tool0 +Z, and the confirmed
# flange clocking is 270 degrees.  Centres/half sizes below are exact AABBs of
# the four uploaded STL parts, retained as oriented boxes after registration.
CAD_ASSEMBLY_RPY = (-math.pi / 2.0, 0.0, -math.pi / 2.0)
CAD_ATTACHMENT_ENVELOPES = (
    ("force_sensor", (0.0, 0.0, 0.025000003),
     (0.06300002, 0.02500000, 0.06300000)),
    ("connector", (0.0, 0.0350592134, 0.0789999932),
     (0.09494080, 0.02900000, 0.08000000)),
    ("robotiq_2f85", (0.0, 0.0, 0.154467878),
     (0.03750002, 0.08310988, 0.06317389)),
    ("mecheye_camera", (0.0, 0.1550000190, 0.1055000027),
     (0.02500000, 0.042500003, 0.06500000)),
)


def _rot_x(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array(((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c)))


def _rot_y(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array(((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c)))


def _rot_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)))


def transform(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = _rot_z(rpy[2]) @ _rot_y(rpy[1]) @ _rot_x(rpy[0])
    result[:3, 3] = np.asarray(xyz, dtype=np.float64)
    return result


def forward_kinematics(joints: Sequence[float]) -> np.ndarray:
    """Return ``base -> tool0`` using this robot's factory calibration."""
    if len(joints) != 6:
        raise ValueError("UR10 forward kinematics requires six joints")
    current = np.eye(4, dtype=np.float64)
    for joint, (xyz, rpy) in zip(joints, FACTORY_JOINT_ORIGINS):
        current = current @ transform(xyz, rpy) @ transform(rpy=(0.0, 0.0, joint))
    current = current @ transform(rpy=WRIST3_TO_FLANGE_RPY)
    return current @ transform(rpy=FLANGE_TO_TOOL0_RPY)


def joint_origins(joints: Sequence[float]) -> list[np.ndarray]:
    """Return base and six moving-joint origins in controller ``base``."""
    if len(joints) != 6:
        raise ValueError("UR10 joint origins require six joints")
    current = np.eye(4, dtype=np.float64)
    points = [current[:3, 3].copy()]
    for joint, (xyz, rpy) in zip(joints, FACTORY_JOINT_ORIGINS):
        current = current @ transform(xyz, rpy) @ transform(rpy=(0.0, 0.0, joint))
        points.append(current[:3, 3].copy())
    return points


def attachment_envelopes(joints: Sequence[float], *, oriented=False):
    """Return conservative oriented-box envelopes in controller ``base``.

    Each item is ``(name, centre, axis-aligned half extents)`` after rotating
    the physical envelope into the base frame.  The brush is excluded because
    its tip is intentionally allowed to contact the writing surface.
    """
    tool = forward_kinematics(joints)
    result = []
    for name, centre_tool0, half_size in CAD_ATTACHMENT_ENVELOPES:
        local = transform(centre_tool0, CAD_ASSEMBLY_RPY)
        half_size = np.asarray(half_size, dtype=np.float64)
        pose = tool @ local
        centre = pose[:3, 3]
        axis_aligned_half_extent = np.abs(pose[:3, :3]) @ half_size
        result.append((name, centre, (pose[:3, :3], half_size) if oriented else axis_aligned_half_extent))
    return result


def attachment_intersects_obstacle(centre, orientation, half_size, bounds):
    """Exact OBB/vertical keep-out intersection via separating axes."""
    xmin, xmax, ymin, ymax, top = bounds
    centre = np.asarray(centre, float)
    rotation = np.asarray(orientation, float)
    half_size = np.asarray(half_size, float)
    extent = np.abs(rotation) @ half_size
    if centre[2] - extent[2] >= top:
        return False
    bottom = min(top - 1.0, centre[2] - extent[2] - 1.0)
    obstacle_centre = np.array(((xmin+xmax)/2, (ymin+ymax)/2, (bottom+top)/2))
    obstacle_half = np.array(((xmax-xmin)/2, (ymax-ymin)/2, (top-bottom)/2))
    axes = [*np.eye(3), *rotation.T]
    axes += [np.cross(left, right) for left in np.eye(3) for right in rotation.T]
    delta = centre - obstacle_centre
    for axis in axes:
        if np.linalg.norm(axis) < 1e-12:
            continue
        radius = np.abs(axis) @ obstacle_half + np.abs(axis @ rotation) @ half_size
        if abs(axis @ delta) > radius + 1e-12:
            return False
    return True
