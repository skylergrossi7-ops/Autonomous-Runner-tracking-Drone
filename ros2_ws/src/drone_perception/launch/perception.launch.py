import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("drone_perception"),
        "config",
        "perception.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            Node(
                package="drone_perception",
                executable="perception_node",
                name="perception_node",
                parameters=[LaunchConfiguration("config")],
                output="screen",
            ),
        ]
    )
