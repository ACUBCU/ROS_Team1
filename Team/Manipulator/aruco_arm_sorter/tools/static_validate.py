#!/usr/bin/env python3
"""Run source-tree validation without importing ROS 2 runtime modules."""

import ast
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
MANIPULATOR_ROOT = ROOT.parent


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def validate_parsers() -> None:
    for path in list((ROOT / "aruco_arm_sorter").glob("*.py")) + list(
        (ROOT / "launch").glob("*.py")
    ):
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))

    xml_paths = [ROOT / "package.xml"]
    xml_paths.extend((ROOT / "worlds").glob("*.sdf"))
    xml_paths.extend((ROOT / "models").rglob("*.sdf"))
    xml_paths.extend((ROOT / "models").rglob("model.config"))
    for path in xml_paths:
        ET.parse(path)

    for path in (ROOT / "config").glob("*.yaml"):
        require(
            yaml.safe_load(path.read_text(encoding="utf-8")) is not None,
            str(path),
        )


def validate_layout_and_versions() -> None:
    helper = MANIPULATOR_ROOT / "workspace.sh"
    require(helper.is_file(), f"workspace helper 없음: {helper}")
    subprocess.run(["bash", "-n", str(helper)], check=True)

    bashrc = MANIPULATOR_ROOT / "shell" / "bashrc_ROS_Team1"
    require(bashrc.is_file(), f"프로젝트 bashrc 없음: {bashrc}")
    subprocess.run(["bash", "-n", str(bashrc)], check=True)
    bashrc_text = bashrc.read_text(encoding="utf-8")
    require(
        'source "$ROS_TEAM1_ROOT/install/setup.bash"' in bashrc_text,
        "ROS_Team1 install source 누락",
    )
    require(
        'source "$HOME/Ros/move_arm/install/setup.bash"' not in bashrc_text,
        "구버전 move_arm 자동 source가 남아 있음",
    )
    require(
        "_ros_team1_remove_old_path_entries" in bashrc_text,
        "상속된 구 workspace 경로 정리 누락",
    )
    require("GZ_SIM_RESOURCE_PATH=" not in bashrc_text, "전역 Gazebo 경로 중복")

    package_version = ET.parse(ROOT / "package.xml").getroot().findtext("version")
    setup_text = (ROOT / "setup.py").read_text(encoding="utf-8")
    identity = yaml.safe_load(
        (ROOT / "config" / "project.yaml").read_text(encoding="utf-8")
    )["project"]
    require(package_version == "3.1.0", "package.xml 버전 불일치")
    require('version="3.1.0"' in setup_text, "setup.py 버전 불일치")
    require(identity.get("package_version") == "3.1.0", "project.yaml 버전 불일치")
    require(
        identity.get("package_path") == "Team/Manipulator/aruco_arm_sorter",
        "project.yaml 패키지 경로 불일치",
    )

    launch_text = (ROOT / "launch" / "aruco_arm_sorter.launch.py").read_text(
        encoding="utf-8"
    )
    preflight_text = (ROOT / "aruco_arm_sorter" / "preflight.py").read_text(
        encoding="utf-8"
    )
    require('executable="aruco_detector"' in launch_text, "검출 노드 launch 누락")
    require("_verify_active_prefix" in launch_text, "launch prefix 보호 누락")
    for executable in ("arm_sequence_controller", "aruco_detector", "preflight"):
        require(
            executable in setup_text,
            f"setup.py 실행 파일 누락: {executable}",
        )
    require(
        "REQUIRED_EXECUTABLES" in preflight_text,
        "preflight 실행 파일 검사 누락",
    )

    forbidden = ("/home/" + "gagho0/",)
    text_paths = [helper, bashrc]
    text_paths.extend(
        path
        for path in ROOT.rglob("*")
        if path.suffix in {".py", ".md", ".yaml", ".xml"}
    )
    for path in text_paths:
        text = path.read_text(encoding="utf-8")
        for value in forbidden:
            require(
                value not in text,
                f"구 경로가 남아 있습니다: {path}: {value}",
            )


def validate_project_logic() -> None:
    sys.path.insert(0, str(ROOT))
    from aruco_arm_sorter.detection_gate import StableMarkerGate
    from aruco_arm_sorter.kinematics import camera_position, end_effector_position
    from aruco_arm_sorter.motion_config import ARM_JOINTS, load_motion_config
    from aruco_arm_sorter.robot_description import customize_robot_description

    config = load_motion_config(ROOT / "config" / "motions.yaml")
    require(ARM_JOINTS == ("joint1", "joint2", "joint3", "joint4"), "관절 이름")
    require(
        config.marker_sequences == {0: "marker_0_to_A", 1: "marker_1_to_B"},
        "ID 매핑",
    )
    for steps in config.sequences.values():
        require(
            steps[-1].positions == config.observation_positions,
            "관찰 자세 복귀",
        )

    camera = camera_position(**config.observation_positions)
    require(math.isclose(camera[0], 0.246, abs_tol=0.004), "카메라 x 위치")
    require(math.isclose(camera[1], 0.0, abs_tol=0.002), "카메라 y 위치")
    require(math.isclose(camera[2], 0.330, abs_tol=0.006), "카메라 z 위치")

    for sequence, step_name in (
        ("marker_0_to_A", "pick_0_lower_and_attach"),
        ("marker_1_to_B", "pick_1_lower_and_attach"),
    ):
        step = next(
            item for item in config.sequences[sequence] if item.name == step_name
        )
        _, _, z = end_effector_position(**step.positions)
        require(math.isclose(z, 0.0225, abs_tol=0.003), f"{step_name} 높이")

    gate = StableMarkerGate([0, 1], required_frames=3)
    gate.set_ready(True)
    require(gate.observe([0]) is None, "1프레임에서 조기 발행")
    require(gate.observe([0]) is None, "2프레임에서 조기 발행")
    require(gate.observe([0]) == 0, "3프레임 안정 검출 실패")

    description = customize_robot_description(
        '<robot name="test"><link name="link5"/></robot>'
    )
    robot = ET.fromstring(description)
    camera_joint = robot.find("./joint[@name='camera_joint']")
    require(camera_joint is not None, "camera_joint 누락")
    require(camera_joint.find("parent").get("link") == "link5", "카메라 parent")
    require(
        len(robot.findall("./gazebo/plugin[@name='gz::sim::systems::DetachableJoint']"))
        == 2,
        "DetachableJoint 개수",
    )


def main() -> None:
    validate_parsers()
    validate_layout_and_versions()
    validate_project_logic()
    print("정적 검증 통과: 구조, 경로, 설정, 카메라, 동작, ArUco gate")


if __name__ == "__main__":
    main()
