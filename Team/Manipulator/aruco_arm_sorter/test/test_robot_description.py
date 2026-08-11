import xml.etree.ElementTree as ET

from aruco_arm_sorter.robot_description import customize_robot_description


MINIMAL_ROBOT = """
<robot name="test_open_manipulator_x">
  <link name="link5"/>
</robot>
"""


def _root(description=MINIMAL_ROBOT):
    return ET.fromstring(customize_robot_description(description))


def test_camera_is_fixed_to_link5_and_points_down():
    root = _root()
    assert root.find("./link[@name='camera_link']") is not None
    joint = root.find("./joint[@name='camera_joint']")
    assert joint is not None
    assert joint.get("type") == "fixed"
    assert joint.find("parent").get("link") == "link5"
    assert joint.find("child").get("link") == "camera_link"
    assert joint.find("origin").get("xyz") == "0.05 0 0.04"
    assert joint.find("origin").get("rpy") == "0 1.57079632679 0"


def test_camera_sensor_configuration():
    sensor = _root().find(
        "./gazebo[@reference='camera_link']/sensor[@name='gripper_camera']"
    )
    assert sensor is not None
    assert sensor.get("type") == "camera"
    assert sensor.findtext("topic") == "/gripper_camera/image_raw"
    assert sensor.findtext("camera/image/width") == "640"
    assert sensor.findtext("camera/image/height") == "480"
    assert sensor.findtext("camera/horizontal_fov") == "1.0471975512"


def test_detachable_joints_use_harmonic_child_link_parameter():
    root = _root()
    plugins = root.findall(
        "./gazebo/plugin[@name='gz::sim::systems::DetachableJoint']"
    )
    assert len(plugins) == 2
    assert {plugin.findtext("child_model") for plugin in plugins} == {
        "marker0_box",
        "marker1_box",
    }
    assert all(plugin.findtext("child_link") == "box_link" for plugin in plugins)
    assert all(plugin.find("child_model_link") is None for plugin in plugins)


def test_existing_camera_is_updated_without_duplication():
    first = customize_robot_description(MINIMAL_ROBOT)
    second = customize_robot_description(first)
    root = ET.fromstring(second)
    assert len(root.findall("./link[@name='camera_link']")) == 1
    assert len(root.findall("./joint[@name='camera_joint']")) == 1
    assert len(
        root.findall(
            "./gazebo[@reference='camera_link']/sensor[@name='gripper_camera']"
        )
    ) == 1
    assert len(
        root.findall("./gazebo/plugin[@name='gz::sim::systems::DetachableJoint']")
    ) == 2
