"""Inject the project camera and grasp systems into a URDF description."""

import xml.etree.ElementTree as ET


def _set_text(parent, tag: str, value: str):
    element = parent.find(tag)
    if element is None:
        element = ET.SubElement(parent, tag)
    element.text = value
    return element


def _add_link5_camera(root) -> None:
    camera_link = root.find("./link[@name='camera_link']")
    if camera_link is None:
        camera_link = ET.SubElement(root, "link", {"name": "camera_link"})
        inertial = ET.SubElement(camera_link, "inertial")
        ET.SubElement(inertial, "mass", {"value": "0.03"})
        ET.SubElement(
            inertial,
            "inertia",
            {
                "ixx": "4.0e-6",
                "ixy": "0",
                "ixz": "0",
                "iyy": "5.3e-6",
                "iyz": "0",
                "izz": "6.3e-6",
            },
        )
        visual = ET.SubElement(camera_link, "visual")
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(geometry, "box", {"size": "0.04 0.03 0.025"})
        material = ET.SubElement(visual, "material", {"name": "camera_black"})
        ET.SubElement(material, "color", {"rgba": "0.05 0.05 0.05 1"})

    camera_joint = None
    for joint in root.findall("./joint"):
        child = joint.find("child")
        if child is not None and child.get("link") == "camera_link":
            camera_joint = joint
            break
    if camera_joint is None:
        camera_joint = ET.SubElement(
            root, "joint", {"name": "camera_joint", "type": "fixed"}
        )
        ET.SubElement(camera_joint, "parent", {"link": "link5"})
        ET.SubElement(camera_joint, "child", {"link": "camera_link"})
        origin = ET.SubElement(camera_joint, "origin")
    else:
        camera_joint.set("type", "fixed")
        parent = camera_joint.find("parent")
        if parent is None:
            parent = ET.SubElement(camera_joint, "parent")
        parent.set("link", "link5")
        origin = camera_joint.find("origin")
        if origin is None:
            origin = ET.SubElement(camera_joint, "origin")
    origin.set("xyz", "0.05 0 0.04")
    # Gazebo camera optical axis is +X. Rotate it toward the table (-Z of link5).
    origin.set("rpy", "0 1.57079632679 0")

    camera_gazebo = None
    for gazebo in root.findall("./gazebo"):
        if gazebo.get("reference") == "camera_link":
            camera_gazebo = gazebo
            break
    if camera_gazebo is None:
        camera_gazebo = ET.SubElement(root, "gazebo", {"reference": "camera_link"})

    sensor = camera_gazebo.find("./sensor[@name='gripper_camera']")
    if sensor is None:
        sensor = ET.SubElement(
            camera_gazebo,
            "sensor",
            {"name": "gripper_camera", "type": "camera"},
        )
    sensor.set("type", "camera")
    for child in list(sensor):
        sensor.remove(child)
    _set_text(sensor, "pose", "0 0 0 0 0 0")
    _set_text(sensor, "always_on", "true")
    _set_text(sensor, "update_rate", "20")
    _set_text(sensor, "visualize", "true")
    _set_text(sensor, "topic", "/gripper_camera/image_raw")
    camera = ET.SubElement(sensor, "camera")
    _set_text(camera, "horizontal_fov", "1.0471975512")
    image = ET.SubElement(camera, "image")
    _set_text(image, "width", "640")
    _set_text(image, "height", "480")
    _set_text(image, "format", "R8G8B8")
    clip = ET.SubElement(camera, "clip")
    _set_text(clip, "near", "0.03")
    _set_text(clip, "far", "2.0")


def _model_gazebo(root):
    for gazebo in root.findall("./gazebo"):
        if gazebo.get("reference") is None:
            return gazebo
    return ET.SubElement(root, "gazebo")


def _add_detachable_joints(root) -> None:
    gazebo = _model_gazebo(root)
    _set_text(gazebo, "self_collide", "true")

    for index in (0, 1):
        child_model = f"marker{index}_box"
        plugin = None
        for candidate in gazebo.findall("plugin"):
            model_element = candidate.find("child_model")
            if (
                candidate.get("name") == "gz::sim::systems::DetachableJoint"
                and model_element is not None
                and model_element.text == child_model
            ):
                plugin = candidate
                break
        if plugin is None:
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
            "child_model": child_model,
            "child_link": "box_link",
            "attach_topic": f"/arm/grasp/marker{index}/attach",
            "detach_topic": f"/arm/grasp/marker{index}/detach",
            "output_topic": f"/arm/grasp/marker{index}/state",
        }
        for tag, value in values.items():
            _set_text(plugin, tag, value)


def customize_robot_description(robot_description: str) -> str:
    """Return URDF XML with the camera and both detachable joints added."""

    root = ET.fromstring(robot_description)
    _add_link5_camera(root)
    _add_detachable_joints(root)
    return ET.tostring(root, encoding="unicode")
