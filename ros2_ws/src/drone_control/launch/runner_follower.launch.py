import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("drone_control"),
        "config",
        "follower.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            Node(
                package="drone_control",
                executable="runner_follower",
                name="runner_follower",
                parameters=[LaunchConfiguration("config")],
                output="screen",
            ),
        ]
    )
