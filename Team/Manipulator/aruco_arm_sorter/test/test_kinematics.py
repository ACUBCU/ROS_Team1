import math
from pathlib import Path

import pytest

from aruco_arm_sorter.kinematics import camera_position, end_effector_position
from aruco_arm_sorter.motion_config import load_motion_config


ROOT = Path(__file__).resolve().parents[1]
CONFIG = load_motion_config(ROOT / "config" / "motions.yaml")


def _step(sequence, name):
    return next(step for step in CONFIG.sequences[sequence] if step.name == name)


@pytest.mark.parametrize(
    "sequence,step_name,yaw",
    [
        ("marker_0_to_A", "pick_0_lower_and_attach", 0.45),
        ("marker_1_to_B", "pick_1_lower_and_attach", -0.45),
        ("marker_0_to_A", "place_A_lower", 1.20),
        ("marker_1_to_B", "place_B_lower", -1.20),
    ],
)
def test_lower_poses_match_box_or_destination(sequence, step_name, yaw):
    step = _step(sequence, step_name)
    x, y, z = end_effector_position(**step.positions)
    assert math.hypot(x - 0.012, y) == pytest.approx(0.260, abs=0.004)
    assert math.atan2(y, x - 0.012) == pytest.approx(yaw, abs=0.004)
    assert z == pytest.approx(0.0225, abs=0.003)


def test_approach_pose_is_above_pick_pose():
    approach = _step("marker_0_to_A", "pick_0_approach")
    lower = _step("marker_0_to_A", "pick_0_lower_and_attach")
    assert (
        end_effector_position(**approach.positions)[2]
        > end_effector_position(**lower.positions)[2] + 0.04
    )


def test_home_pose_is_above_boxes():
    _, _, z = end_effector_position(**CONFIG.initial_positions)
    assert z > 0.14


def test_observation_camera_is_centered_above_markers():
    x, y, z = camera_position(**CONFIG.observation_positions)
    assert x == pytest.approx(0.246, abs=0.004)
    assert y == pytest.approx(0.0, abs=0.002)
    assert z == pytest.approx(0.330, abs=0.006)
    assert sum(CONFIG.observation_positions.values()) == pytest.approx(0.0, abs=0.01)
