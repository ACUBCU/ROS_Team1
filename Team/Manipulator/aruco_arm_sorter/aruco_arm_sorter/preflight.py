"""Preflight checks for the OpenMANIPULATOR-X Gazebo project."""

import importlib.util
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

from aruco_arm_sorter.motion_config import load_motion_config


def _check_ros_package(name: str) -> bool:
    result = subprocess.run(
        ["ros2", "pkg", "prefix", name],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return result.returncode == 0


def _check_active_prefix(share: Path, failures) -> None:
    """Show the selected overlay and optionally require an exact prefix."""

    prefix = share.parents[1]
    print(f"INFO 활성 aruco_arm_sorter prefix: {prefix}")
    expected_value = os.environ.get("ARUCO_ARM_SORTER_EXPECTED_PREFIX", "").strip()
    if not expected_value:
        return

    expected = Path(os.path.expandvars(os.path.expanduser(expected_value))).resolve()
    if prefix != expected:
        failures.append(
            "다른 작업공간의 aruco_arm_sorter가 선택됨: "
            f"현재={prefix}, 기대={expected}"
        )
        print(f"FAIL 활성 패키지 위치: 현재={prefix}, 기대={expected}")
    else:
        print(f"OK  활성 패키지 위치: {prefix}")


def _check_sensor_pipeline(paths, failures) -> None:
    """Validate the installed world and bridge, not only the source tree."""

    try:
        world_root = ET.parse(paths["world SDF"]).getroot()
    except (OSError, ET.ParseError):
        return

    sensor_plugins = world_root.findall(
        ".//world/plugin[@name='gz::sim::systems::Sensors']"
    )
    if not sensor_plugins:
        failures.append("world SDF에 Gazebo Sensors 시스템이 없음")
        print("FAIL Gazebo Sensors 시스템")
    else:
        print("OK  Gazebo Sensors 시스템")

    try:
        bridge_entries = yaml.safe_load(
            paths["bridge YAML"].read_text(encoding="utf-8")
        )
    except (OSError, yaml.YAMLError) as exc:
        failures.append(f"bridge YAML 읽기 실패: {exc}")
        print(f"FAIL 카메라 bridge 설정: {exc}")
        return

    if not isinstance(bridge_entries, list):
        failures.append("bridge YAML 최상위 항목이 목록이 아님")
        print("FAIL 카메라 bridge 설정")
        return

    mappings = {
        (entry.get("ros_topic_name"), entry.get("gz_topic_name"))
        for entry in bridge_entries
        if isinstance(entry, dict)
    }
    required = {
        ("/gripper_camera/image_raw", "/gripper_camera/image_raw"),
        ("/gripper_camera/camera_info", "/gripper_camera/camera_info"),
    }
    missing = required - mappings
    if missing:
        failures.append(f"카메라 bridge 매핑 누락: {sorted(missing)}")
        print(f"FAIL 카메라 bridge 매핑: {sorted(missing)}")
    else:
        print("OK  카메라 image/CameraInfo bridge 매핑")


def main() -> None:
    failures = []
    print("[OpenMANIPULATOR-X ArUco sorter 사전 점검]")

    for command in ("ros2", "gz"):
        if shutil.which(command):
            print(f"OK  실행 명령: {command}")
        else:
            failures.append(f"실행 명령을 찾지 못함: {command}")
            print(f"FAIL 실행 명령: {command}")

    for module_name in ("cv2", "cv_bridge", "numpy"):
        if importlib.util.find_spec(module_name) is None:
            failures.append(f"Python 모듈 없음: {module_name}")
            print(f"FAIL Python 모듈: {module_name}")
        else:
            print(f"OK  Python 모듈: {module_name}")

    try:
        import cv2
    except ImportError:
        pass
    else:
        if not hasattr(cv2, "aruco"):
            failures.append("OpenCV aruco 모듈 없음")
            print("FAIL OpenCV aruco 모듈")
        else:
            print("OK  OpenCV aruco 모듈")

    share = Path(get_package_share_directory("aruco_arm_sorter")).resolve()
    _check_active_prefix(share, failures)
    paths = {
        "동작 YAML": share / "config" / "motions.yaml",
        "bridge YAML": share / "config" / "bridge.yaml",
        "world SDF": share / "worlds" / "sorting_world.sdf",
        "ArUco 0 모델": share / "models" / "aruco_box_0" / "model.sdf",
        "ArUco 1 모델": share / "models" / "aruco_box_1" / "model.sdf",
        "ArUco 0 텍스처": (
            share / "models" / "aruco_box_0" / "materials" / "textures" / "aruco_0.png"
        ),
        "ArUco 1 텍스처": (
            share / "models" / "aruco_box_1" / "materials" / "textures" / "aruco_1.png"
        ),
    }

    try:
        config = load_motion_config(paths["동작 YAML"])
    except Exception as exc:
        failures.append(f"동작 YAML 오류: {exc}")
        print(f"FAIL 동작 YAML: {exc}")
    else:
        print(
            "OK  동작 YAML: "
            f"ID {sorted(config.marker_sequences)} / "
            f"joints {list(config.initial_positions)}"
        )

    for label, path in paths.items():
        if label == "동작 YAML":
            continue
        if not path.is_file():
            failures.append(f"파일 없음: {path}")
            print(f"FAIL {label}: {path}")
            continue
        if path.suffix in (".sdf", ".xml"):
            try:
                ET.parse(path)
            except ET.ParseError as exc:
                failures.append(f"{label} XML 오류: {exc}")
                print(f"FAIL {label}: {exc}")
                continue
        print(f"OK  {label}: {path}")

    _check_sensor_pipeline(paths, failures)

    if shutil.which("ros2"):
        packages = (
            "open_manipulator_description",
            "open_manipulator_bringup",
            "gz_ros2_control",
            "controller_manager",
            "joint_trajectory_controller",
            "ros_gz_sim",
            "ros_gz_bridge",
            "control_msgs",
            "cv_bridge",
            "sensor_msgs",
        )
        for package in packages:
            if _check_ros_package(package):
                print(f"OK  ROS 패키지: {package}")
            else:
                failures.append(f"ROS 패키지 없음: {package}")
                print(f"FAIL ROS 패키지: {package}")

    if failures:
        print("\n점검 실패:")
        for failure in failures:
            print(f"- {failure}")
        raise SystemExit(1)

    print("\n모든 사전 점검을 통과했습니다.")
    raise SystemExit(0)


if __name__ == "__main__":
    main()
