import ast
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_python_files_parse():
    for path in list((ROOT / "aruco_arm_sorter").glob("*.py")) + list(
        (ROOT / "launch").glob("*.py")
    ):
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))


def test_xml_and_sdf_files_parse():
    paths = [ROOT / "package.xml"]
    paths.extend((ROOT / "worlds").glob("*.sdf"))
    paths.extend((ROOT / "models").rglob("*.sdf"))
    paths.extend((ROOT / "models").rglob("model.config"))
    for path in paths:
        ET.parse(path)


def test_yaml_files_parse():
    for path in (ROOT / "config").glob("*.yaml"):
        assert yaml.safe_load(path.read_text(encoding="utf-8")) is not None


def test_uses_openmanipulator_without_moveit():
    package_text = (ROOT / "package.xml").read_text(encoding="utf-8").lower()
    assert "open_manipulator_description" in package_text
    assert "open_manipulator_bringup" in package_text
    assert "gz_ros2_control" in package_text
    assert "position_controllers" in package_text
    assert "moveit" not in package_text


def test_simple_placeholder_arm_is_removed():
    assert not (ROOT / "models" / "simple_sorting_arm").exists()


def test_link5_camera_and_sensor_pipeline_are_configured():
    description_text = (
        ROOT / "aruco_arm_sorter" / "robot_description.py"
    ).read_text(
        encoding="utf-8"
    )
    world_text = (ROOT / "worlds" / "sorting_world.sdf").read_text(
        encoding="utf-8"
    )
    bridge = yaml.safe_load(
        (ROOT / "config" / "bridge.yaml").read_text(encoding="utf-8")
    )
    ros_topics = {entry["ros_topic_name"] for entry in bridge}

    assert 'parent", {"link": "link5"}' in description_text
    assert '"name": "gripper_camera"' in description_text
    assert '"0 1.57079632679 0"' in description_text
    assert "gz-sim-sensors-system" in world_text
    assert "/gripper_camera/image_raw" in ros_topics
    assert "/gripper_camera/camera_info" in ros_topics


def test_preflight_checks_active_prefix_and_installed_sensor_pipeline():
    preflight_text = (ROOT / "aruco_arm_sorter" / "preflight.py").read_text(
        encoding="utf-8"
    )
    assert "ARUCO_ARM_SORTER_EXPECTED_PREFIX" in preflight_text
    assert "ARUCO_ARM_SORTER_WORKSPACE_ROOT" in preflight_text
    assert "REQUIRED_EXECUTABLES" in preflight_text
    assert "AMENT_PREFIX_PATH" in preflight_text
    assert "gz::sim::systems::Sensors" in preflight_text
    assert "/gripper_camera/image_raw" in preflight_text
    assert "/gripper_camera/camera_info" in preflight_text


def test_aruco_detector_is_installed_and_launched():
    setup_text = (ROOT / "setup.py").read_text(encoding="utf-8")
    launch_text = (ROOT / "launch" / "aruco_arm_sorter.launch.py").read_text(
        encoding="utf-8"
    )
    assert "aruco_detector = aruco_arm_sorter.aruco_detector:main" in setup_text
    assert 'executable="aruco_detector"' in launch_text


def test_ros_team1_identity_and_versions_match():
    package_root = ET.parse(ROOT / "package.xml").getroot()
    setup_text = (ROOT / "setup.py").read_text(encoding="utf-8")
    identity = yaml.safe_load(
        (ROOT / "config" / "project.yaml").read_text(encoding="utf-8")
    )["project"]

    assert package_root.findtext("version") == "3.1.0"
    assert 'version="3.1.0"' in setup_text
    assert identity == {
        "repository": "ACUBCU/ROS_Team1",
        "package_path": "Team/Manipulator/aruco_arm_sorter",
        "layout_version": 1,
        "package_version": "3.1.0",
    }


def test_workspace_helper_is_repository_relative_and_guards_overlay():
    helper = ROOT.parent / "workspace.sh"
    helper_text = helper.read_text(encoding="utf-8")
    launch_text = (ROOT / "launch" / "aruco_arm_sorter.launch.py").read_text(
        encoding="utf-8"
    )

    assert helper.is_file()
    assert "BASH_SOURCE[0]" in helper_text
    assert 'PACKAGE_DIR="${SCRIPT_DIR}/${PACKAGE_NAME}"' in helper_text
    assert 'WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/../.."' in helper_text
    assert "--base-paths" in helper_text
    assert "ARUCO_ARM_SORTER_EXPECTED_PREFIX" in helper_text
    assert "aruco_detector" in helper_text
    assert "~/" + "Ros/" not in helper_text
    assert "setup_contains_conflicting_package" in helper_text
    assert "구버전 %s 포함 workspace 건너뜀" in helper_text
    assert "_verify_active_prefix" in launch_text


def test_bashrc_prefers_ros_team1_and_does_not_source_legacy_overlay():
    manipulator_root = ROOT.parent
    bashrc = manipulator_root / "shell" / "bashrc_ROS_Team1"
    bashrc_text = bashrc.read_text(encoding="utf-8")

    assert bashrc.is_file()
    assert 'export ROS_TEAM1_ROOT="${ROS_TEAM1_ROOT:-$HOME/ROS_Team1}"' in bashrc_text
    assert 'source "$ROS_TEAM1_ROOT/install/setup.bash"' in bashrc_text
    assert 'source "$HOME/Ros/move_arm/install/setup.bash"' not in bashrc_text
    assert 'source "$HOME/Ros/open_manipulator_ws/install/setup.bash"' not in bashrc_text
    assert 'ARUCO_DEPENDENCY_SETUP="$HOME/Ros/open_manipulator_ws/install/setup.bash"' in bashrc_text
    assert "GZ_SIM_RESOURCE_PATH=" not in bashrc_text
    assert "_ros_team1_remove_old_path_entries" in bashrc_text
    assert "AMENT_PREFIX_PATH" in bashrc_text
    assert "LD_LIBRARY_PATH" in bashrc_text
    assert "alias cb='ros_team1 build'" in bashrc_text
    assert (manipulator_root / "BASHRC_SETUP.md").is_file()
