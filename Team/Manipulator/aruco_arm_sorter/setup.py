import os
from glob import glob

from setuptools import find_packages, setup

package_name = "aruco_arm_sorter"


def package_files(directory):
    data_files = []
    for path, _, filenames in os.walk(directory):
        files = [os.path.join(path, filename) for filename in filenames]
        if files:
            data_files.append((os.path.join("share", package_name, path), files))
    return data_files


setup(
    name=package_name,
    version="3.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            glob(os.path.join("launch", "*.launch.py")),
        ),
        (
            "share/" + package_name + "/config",
            glob(os.path.join("config", "*.yaml")),
        ),
        (
            "share/" + package_name + "/worlds",
            glob(os.path.join("worlds", "*.sdf")),
        ),
    ]
    + package_files("models"),
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="ROS Student",
    maintainer_email="student@example.com",
    description=(
        "Camera-based ArUco sorting for the official "
        "OpenMANIPULATOR-X Gazebo model."
    ),
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "arm_sequence_controller = aruco_arm_sorter.arm_sequence_controller:main",
            "aruco_detector = aruco_arm_sorter.aruco_detector:main",
            "preflight = aruco_arm_sorter.preflight:main",
        ],
    },
)
