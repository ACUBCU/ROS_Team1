"""Launch the official OpenMANIPULATOR-X model and ArUco sorting demo."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

from aruco_arm_sorter.robot_description import customize_robot_description


def _expanded_path(value: str) -> Path:
    return Path(os.path.expandvars(os.path.expanduser(value))).resolve()


def _verify_active_prefix(package_share: str) -> Path:
    """Stop before Gazebo if a requested ROS_Team1 install is not active."""

    active = Path(package_share).absolute().parent.parent.resolve()
    expected_value = os.environ.get("ARUCO_ARM_SORTER_EXPECTED_PREFIX", "").strip()
    workspace_value = os.environ.get("ARUCO_ARM_SORTER_WORKSPACE_ROOT", "").strip()
    expected = None
    if expected_value:
        expected = _expanded_path(expected_value)
    elif workspace_value:
        expected = _expanded_path(workspace_value) / "install" / "aruco_arm_sorter"

    if expected is not None and active != expected.resolve():
        raise RuntimeError(
            "다른 작업공간의 aruco_arm_sorter가 선택됐습니다: "
            f"현재={active}, 기대={expected.resolve()}. "
            "Team/Manipulator/workspace.sh run으로 실행하세요."
        )
    return active


def generate_launch_description():
    package_share = get_package_share_directory("aruco_arm_sorter")
    active_prefix = _verify_active_prefix(package_share)
    description_share = get_package_share_directory("open_manipulator_description")
    bringup_share = get_package_share_directory("open_manipulator_bringup")
    ros_gz_share = get_package_share_directory("ros_gz_sim")

    world_path = os.path.join(package_share, "worlds", "sorting_world.sdf")
    model_path = os.path.join(package_share, "models")
    bridge_path = os.path.join(package_share, "config", "bridge.yaml")
    motion_path = os.path.join(package_share, "config", "motions.yaml")
    xacro_path = os.path.join(
        description_share,
        "urdf",
        "open_manipulator_x",
        "open_manipulator_x.urdf.xacro",
    )

    doc = xacro.process_file(xacro_path, mappings={"use_sim": "true"})
    robot_description = customize_robot_description(doc.toxml())

    resource_entries = [
        model_path,
        os.path.join(bringup_share, "worlds"),
        str(Path(description_share).parent.resolve()),
    ]
    previous = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if previous:
        resource_entries.append(previous)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r -v 4 {world_path}",
            "on_exit_shutdown": "True",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description,
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
            "-name",
            "open_manipulator_x",
            "-allow_renaming",
            "false",
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="aruco_arm_bridge",
        output="screen",
        parameters=[{"config_file": bridge_path}],
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )
    sequence_controller = Node(
        package="aruco_arm_sorter",
        executable="arm_sequence_controller",
        name="arm_sequence_controller",
        output="screen",
        parameters=[{"motion_file": motion_path, "use_sim_time": True}],
    )
    aruco_detector = Node(
        package="aruco_arm_sorter",
        executable="aruco_detector",
        name="aruco_detector",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "allowed_marker_ids": [0, 1],
                "required_consecutive_detections": 5,
            }
        ],
    )

    return LaunchDescription(
        [
            LogInfo(msg=f"aruco_arm_sorter 활성 prefix: {active_prefix}"),
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH", os.pathsep.join(resource_entries)
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=spawn_robot, on_exit=[joint_state_spawner])
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_spawner,
                    on_exit=[controller_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=controller_spawner,
                    on_exit=[sequence_controller, aruco_detector],
                )
            ),
            gazebo,
            bridge,
            robot_state_publisher,
            spawn_robot,
        ]
    )
