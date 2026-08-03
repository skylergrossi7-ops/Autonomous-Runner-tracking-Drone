import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    perception_config = os.path.join(
        get_package_share_directory("drone_perception"),
        "config",
        "perception.yaml",
    )
    follower_config = os.path.join(
        get_package_share_directory("drone_control"),
        "config",
        "follower.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/image_raw",
                description="ROS image topic produced by ros_gz_image.",
            ),
            DeclareLaunchArgument(
                "forward_commands_enabled",
                default_value="false",
                description="Allow the controller to command forward motion.",
            ),
            DeclareLaunchArgument(
                "publish_to_mavros",
                default_value="false",
                description="Forward safe commands to the MAVROS velocity topic.",
            ),
            Node(
                package="drone_perception",
                executable="perception_node",
                name="perception_node",
                parameters=[
                    perception_config,
                    {"image_topic": LaunchConfiguration("image_topic")},
                ],
                output="screen",
            ),
            Node(
                package="drone_control",
                executable="runner_follower",
                name="runner_follower",
                parameters=[
                    follower_config,
                    {
                        "forward_commands_enabled": ParameterValue(
                            LaunchConfiguration(
                                "forward_commands_enabled"
                            ),
                            value_type=bool,
                        ),
                        "publish_to_mavros": ParameterValue(
                            LaunchConfiguration("publish_to_mavros"),
                            value_type=bool,
                        ),
                    },
                ],
                output="screen",
            ),
        ]
    )
