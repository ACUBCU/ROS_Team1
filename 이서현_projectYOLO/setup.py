import os
from glob import glob

from setuptools import find_packages, setup

package_name = "person_follow_project"


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
    + package_files("models_data"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lee",
    maintainer_email="britams505@gmail.com",
    description="YOLO26 사람 추적/인사 프로젝트 (이서현)",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "person_follow_project = person_follow_project.person_follow_project:main",
            "web_control_person = person_follow_project.web_control_person:main",
        ],
    },
)
