"""Launch the official OpenMANIPULATOR-X model and ArUco sorting demo."""

import os
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def _add_detachable_joints(robot_description: str) -> str:
    root = ET.fromstring(robot_description)
    gazebo = ET.SubElement(root, "gazebo")
    ET.SubElement(gazebo, "self_collide").text = "true"

    for index in (0, 1):
        plugin = ET.SubElement(
            gazebo,
            "plugin",
            {
                "filename": "gz-sim-detachable-joint-system",
                "name": "gz::sim::systems::DetachableJoint",
            },
        )
        values = {
            "parent_link": "link5",
            "child_model": f"marker{index}_box",
            "child_link": "box_link",
            "attach_topic": f"/arm/grasp/marker{index}/attach",
            "detach_topic": f"/arm/grasp/marker{index}/detach",
            "output_topic": f"/arm/grasp/marker{index}/state",
        }
        for tag, value in values.items():
            ET.SubElement(plugin, tag).text = value
    return ET.tostring(root, encoding="unicode")


def generate_launch_description():
    package_share = get_package_share_directory("aruco_arm_sorter")
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
    robot_description = _add_detachable_joints(doc.toxml())

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

    return LaunchDescription(
        [
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
                    on_exit=[sequence_controller],
                )
            ),
            gazebo,
            bridge,
            robot_state_publisher,
            spawn_robot,
        ]
    )
