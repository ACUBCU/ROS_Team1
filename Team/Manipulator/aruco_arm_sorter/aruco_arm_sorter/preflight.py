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


PACKAGE_NAME = "aruco_arm_sorter"
REQUIRED_EXECUTABLES = ("arm_sequence_controller", "aruco_detector", "preflight")


def _check_ros_package(name: str) -> bool:
    result = subprocess.run(
        ["ros2", "pkg", "prefix", name],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return result.returncode == 0


def _expanded_path(value: str) -> Path:
    return Path(os.path.expandvars(os.path.expanduser(value))).resolve()


def _expected_prefix() -> Path | None:
    expected_value = os.environ.get("ARUCO_ARM_SORTER_EXPECTED_PREFIX", "").strip()
    if expected_value:
        return _expanded_path(expected_value)

    workspace_value = os.environ.get("ARUCO_ARM_SORTER_WORKSPACE_ROOT", "").strip()
    if workspace_value:
        return _expanded_path(workspace_value) / "install" / PACKAGE_NAME
    return None


def _check_active_prefix(share: Path, failures) -> Path:
    """Show the selected overlay and require the requested repository install."""

    prefix = share.parent.parent.resolve()
    print(f"INFO 활성 {PACKAGE_NAME} prefix: {prefix}")
    expected = _expected_prefix()
    if expected is None:
        print(
            "WARN 기대 prefix가 지정되지 않았습니다. "
            "Team/Manipulator/workspace.sh doctor 사용을 권장합니다."
        )
        return prefix

    expected = expected.resolve()
    if prefix != expected:
        failures.append(
            f"다른 작업공간의 {PACKAGE_NAME}가 선택됨: "
            f"현재={prefix}, 기대={expected}"
        )
        print(f"FAIL 활성 패키지 위치: 현재={prefix}, 기대={expected}")
    else:
        print(f"OK  활성 패키지 위치: {prefix}")
    return prefix


def _check_duplicate_prefixes(active_prefix: Path) -> None:
    matches = []
    for value in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if not value:
            continue
        prefix = _expanded_path(value)
        marker = (
            prefix
            / "share"
            / "ament_index"
            / "resource_index"
            / "packages"
            / PACKAGE_NAME
        )
        if marker.is_file() and prefix not in matches:
            matches.append(prefix)

    inactive = [path for path in matches if path != active_prefix]
    if inactive:
        print("WARN 동명 패키지가 다른 prefix에도 남아 있습니다:")
        for path in inactive:
            print(f"- {path}")
        print(
            "INFO 현재 활성 prefix가 위의 OK 위치와 같으면 "
            "실행에는 사용되지 않습니다."
        )


def _check_runtime_executables(prefix: Path, failures) -> None:
    executable_dir = prefix / "lib" / PACKAGE_NAME
    for executable in REQUIRED_EXECUTABLES:
        path = executable_dir / executable
        if not path.is_file() or not os.access(path, os.X_OK):
            failures.append(f"설치 실행 파일 없음: {path}")
            print(f"FAIL 설치 실행 파일: {executable}")
        else:
            print(f"OK  설치 실행 파일: {executable}")


def _check_project_identity(path: Path, failures) -> None:
    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
        project = raw["project"]
    except (AttributeError, OSError, TypeError, KeyError, yaml.YAMLError) as exc:
        failures.append(f"프로젝트 식별 파일 오류: {exc}")
        print(f"FAIL 프로젝트 식별 파일: {path}")
        return

    expected_values = {
        "repository": "ACUBCU/ROS_Team1",
        "package_path": "Team/Manipulator/aruco_arm_sorter",
        "layout_version": 1,
        "package_version": "3.1.0",
    }
    mismatches = {
        key: (project.get(key), expected)
        for key, expected in expected_values.items()
        if project.get(key) != expected
    }
    if mismatches:
        failures.append(f"프로젝트 식별 값 불일치: {mismatches}")
        print(f"FAIL 프로젝트 식별 값: {mismatches}")
    else:
        print("OK  ROS_Team1 프로젝트 식별 및 패키지 버전")


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

    share = Path(get_package_share_directory(PACKAGE_NAME)).absolute()
    active_prefix = _check_active_prefix(share, failures)
    _check_duplicate_prefixes(active_prefix)
    _check_runtime_executables(active_prefix, failures)
    paths = {
        "프로젝트 식별 YAML": share / "config" / "project.yaml",
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

    _check_project_identity(paths["프로젝트 식별 YAML"], failures)

    _check_sensor_pipeline(paths, failures)

    if shutil.which("ros2"):
        packages = (
            "open_manipulator_description",
            "open_manipulator_bringup",
            "gz_ros2_control",
            "controller_manager",
            "joint_trajectory_controller",
            "position_controllers",
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
