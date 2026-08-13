from pathlib import Path

from aruco_arm_sorter.motion_config import ARM_JOINTS, load_motion_config


ROOT = Path(__file__).resolve().parents[1]
CONFIG = load_motion_config(ROOT / "config" / "motions.yaml")


def test_official_openmanipulator_joint_names():
    assert ARM_JOINTS == ("joint1", "joint2", "joint3", "joint4")


def test_two_marker_ids_have_different_sequences():
    assert CONFIG.marker_sequences == {0: "marker_0_to_A", 1: "marker_1_to_B"}


def test_every_step_has_all_arm_joints():
    for steps in CONFIG.sequences.values():
        for step in steps:
            assert tuple(step.positions) == ARM_JOINTS


def test_uses_ros2_control_action_interfaces():
    assert CONFIG.arm_action == "/arm_controller/follow_joint_trajectory"
    assert CONFIG.gripper_action == "/gripper_controller/gripper_cmd"


def test_each_sequence_has_one_matching_attach_and_detach_event():
    events_0 = [step.after for step in CONFIG.sequences["marker_0_to_A"]]
    events_1 = [step.after for step in CONFIG.sequences["marker_1_to_B"]]
    assert events_0.count("attach_marker_0") == 1
    assert events_0.count("detach_marker_0") == 1
    assert events_1.count("attach_marker_1") == 1
    assert events_1.count("detach_marker_1") == 1
