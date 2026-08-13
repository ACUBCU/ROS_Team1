from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'final_project'


def package_files(directory):
    data_files = []

    for path, directories, filenames in os.walk(directory):
        files = [os.path.join(path, filename) for filename in filenames]

        if not files:
            continue

        install_path = os.path.join(
            "share", 
            package_name,
            path,
        )

        data_files.append((install_path, files))

    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/" + package_name + "/launch", glob(os.path.join("launch", "*.launch.py"))),
        ("share/" + package_name + "/world", glob(os.path.join("world", "*.sdf"))),
        ("share/" + package_name + "/models", glob(os.path.join("models", "*.*"))),
        ('share/' + package_name + '/templates', glob(os.path.join('templates', '*.html'))),
    ] + package_files("models"),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ldh',
    maintainer_email='ldh9632@gmail.com',
    description='ROS 2 package for the final project',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "spawn_cube = final_project.spawn_cube:main",
            "aruco_detect = final_project.aruco_detect:main",
            "robot_control = final_project.robot_control:main",
            "dashboard = final_project.dashboard:main",
            "world_reboot = final_project.world_reboot:main",
            "spawn_marker = final_project.spawn_marker:main",
            
        ],
    },
)
