"""Preflight checks for the OpenMANIPULATOR-X Gazebo project."""

import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

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


def main() -> None:
    failures = []
    print("[OpenMANIPULATOR-X ArUco sorter 사전 점검]")

    for command in ("ros2", "gz"):
        if shutil.which(command):
            print(f"OK  실행 명령: {command}")
        else:
            failures.append(f"실행 명령을 찾지 못함: {command}")
            print(f"FAIL 실행 명령: {command}")

    share = Path(get_package_share_directory("aruco_arm_sorter"))
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
