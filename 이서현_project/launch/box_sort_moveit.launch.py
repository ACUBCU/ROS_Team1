# box_sort_moveit_project.py 를 MoveItPy와 함께 실행하기 위한 launch 파일입니다.
#
# 왜 launch 파일이 필요한가: MoveItPy는 "SRDF, kinematics.yaml, ompl 플래너 설정,
# 컨트롤러 설정" 같은 것들을 자기 혼자서는 못 찾고, ROS 파라미터로 직접 전달받아야 합니다.
# 그냥 `ros2 run redball_sort_project box_sort_moveit_project` 로 실행하면 이 파라미터들이
# 하나도 없어서 "Failed to load planning pipelines" 에러로 바로 죽습니다 (실제로 겪음).
# 이 launch 파일이 tf2_basic 패키지의 moveit_test.launch.py 와 똑같은 방식으로 그 설정들을
# 전부 읽어서 파라미터로 넘겨줍니다.
#
# 실행 방법:
#   (Gazebo가 이미 켜져 있어야 함 - open_manipulator_x_gazebo.launch.py)
#   ros2 launch redball_sort_project box_sort_moveit.launch.py use_sim_time:=true

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def generate_launch_description() -> LaunchDescription:
    moveit_config_share = Path(get_package_share_directory("open_manipulator_moveit_config"))
    description_share = Path(get_package_share_directory("open_manipulator_description"))
    moveit_utils_share = Path(get_package_share_directory("moveit_configs_utils"))

    robot_config_dir = moveit_config_share / "config" / "open_manipulator_x"
    urdf_xacro = (
        description_share / "urdf" / "open_manipulator_x" / "open_manipulator_x.urdf.xacro"
    )
    srdf_file = robot_config_dir / "open_manipulator_x.srdf"
    kinematics_file = robot_config_dir / "kinematics.yaml"
    joint_limits_file = robot_config_dir / "joint_limits.yaml"
    controllers_file = robot_config_dir / "moveit_controllers.yaml"
    ompl_file = moveit_config_share / "config" / "ompl_planning.yaml"
    ompl_defaults_file = moveit_utils_share / "default_configs" / "ompl_defaults.yaml"

    ompl_parameters = load_yaml(ompl_defaults_file)
    ompl_parameters.update(load_yaml(ompl_file))

    moveit_py_parameters = {
        "robot_description_semantic": srdf_file.read_text(encoding="utf-8"),
        "robot_description_kinematics": load_yaml(kinematics_file),
        "robot_description_planning": load_yaml(joint_limits_file),
        "planning_pipelines": {"pipeline_names": ["ompl"]},
        "ompl": ompl_parameters,
        "moveit_manage_controllers": True,
        "plan_request_params": {
            # RRTConnect는 확률적(무작위 샘플링) 플래너라 1번만 시도하면 가끔 (goal이 박스
            # 표면처럼 빡빡한 위치일 때) 실패합니다. 여러 번 재시도하도록 늘림.
            "planning_attempts": 10,
            "planning_pipeline": "ompl",
            "planner_id": "RRTConnectkConfigDefault",
            "max_velocity_scaling_factor": 0.2,
            "max_acceleration_scaling_factor": 0.2,
            "planning_time": 5.0,
        },
        "planning_scene_monitor_options": {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": "/joint_states",
            "wait_for_initial_state_timeout": 10.0,
        },
    }
    moveit_py_parameters.update(load_yaml(controllers_file))

    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", str(urdf_xacro), " use_sim:=", use_sim_time]),
            value_type=str,
        )
    }

    node_executable = LaunchConfiguration("node_executable")
    extra_args = LaunchConfiguration("extra_args")

    moveit_py_node = Node(
        package="redball_sort_project",
        executable=node_executable,
        name="box_sort_moveit_project",
        output="screen",
        arguments=[extra_args],
        parameters=[
            moveit_py_parameters,
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Gazebo의 /clock 사용 여부"
            ),
            DeclareLaunchArgument(
                "node_executable",
                default_value="box_sort_moveit_project",
                description="redball_sort_project setup.py의 console_scripts에 등록된 실행 파일 이름",
            ),
            DeclareLaunchArgument(
                "extra_args", default_value="",
                description="노드에 그대로 넘길 추가 인자 (예: --teach, --check-slots)",
            ),
            moveit_py_node,
        ]
    )
