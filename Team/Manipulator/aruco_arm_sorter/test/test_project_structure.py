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
