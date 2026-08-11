"""Forward kinematics used to validate the OpenMANIPULATOR-X demo poses."""

import math
from typing import Tuple


def end_effector_position(
    joint1: float,
    joint2: float,
    joint3: float,
    joint4: float,
) -> Tuple[float, float, float]:
    """Return the URDF end-effector position in the base/world frame."""

    radial = (
        0.024 * math.cos(joint2)
        + 0.128 * math.sin(joint2)
        + 0.124 * math.cos(joint2 + joint3)
        + 0.126 * math.cos(joint2 + joint3 + joint4)
    )
    z = (
        0.0595
        - 0.024 * math.sin(joint2)
        + 0.128 * math.cos(joint2)
        - 0.124 * math.sin(joint2 + joint3)
        - 0.126 * math.sin(joint2 + joint3 + joint4)
    )
    return (
        0.012 + radial * math.cos(joint1),
        radial * math.sin(joint1),
        z,
    )


def camera_position(
    joint1: float,
    joint2: float,
    joint3: float,
    joint4: float,
    offset_x: float = 0.05,
    offset_z: float = 0.04,
) -> Tuple[float, float, float]:
    """Return the camera joint origin for the link5 mounting offset."""

    end_x, end_y, end_z = end_effector_position(joint1, joint2, joint3, joint4)
    radial = math.hypot(end_x - 0.012, end_y)
    wrist_angle = joint2 + joint3 + joint4
    radial += offset_x * math.cos(wrist_angle) + offset_z * math.sin(
        wrist_angle
    )
    z = end_z - offset_x * math.sin(wrist_angle) + offset_z * math.cos(
        wrist_angle
    )
    return (
        0.012 + radial * math.cos(joint1),
        radial * math.sin(joint1),
        z,
    )
