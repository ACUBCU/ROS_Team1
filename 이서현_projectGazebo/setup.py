import os
from glob import glob

from setuptools import find_packages, setup

package_name = "redball_sort_project"


def package_files(directory):
    data_files = []
    for path, _directories, filenames in os.walk(directory):
        files = [os.path.join(path, filename) for filename in filenames]
        if not files:
            continue
        data_files.append((os.path.join("share", package_name, path), files))
    return data_files


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ]
    + package_files("models")
    + package_files("config")
    + package_files("launch"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lee",
    maintainer_email="britams505@gmail.com",
    description="빨간 공 추적/집기 + 아루코 박스 자리 이동 프로젝트 (이서현)",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "redball_tracking_project = redball_sort_project.redball_tracking_project:main",
            "box_sort_project = redball_sort_project.box_sort_project:main",
            "web_control = redball_sort_project.web_control:main",
            "web_control_moveit = redball_sort_project.web_control_moveit:main",
        ],
    },
)
