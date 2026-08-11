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
