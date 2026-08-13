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
