from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='real_robot',
            executable='yolo_depth_publisher',
            name='yolo_depth_publisher',
            output='screen',
        ),
    ])
