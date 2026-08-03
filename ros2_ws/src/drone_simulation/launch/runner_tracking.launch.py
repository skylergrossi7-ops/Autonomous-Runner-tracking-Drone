import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory("drone_simulation")
    world_file = os.path.join(package_share, "worlds", "runner_tracking.sdf")
    package_models = os.path.join(package_share, "models")
    ardupilot_models = os.path.expanduser("~/ardupilot_gazebo/models")
    ardupilot_plugins = os.path.expanduser("~/ardupilot_gazebo/build")

    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_paths = [package_models, ardupilot_models]
    if existing_resource_path:
        resource_paths.append(existing_resource_path)

    existing_plugin_path = os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
    plugin_paths = [ardupilot_plugins]
    if existing_plugin_path:
        plugin_paths.append(existing_plugin_path)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=world_file,
                description="Absolute path to the Gazebo world file.",
            ),
            DeclareLaunchArgument(
                "render_engine",
                default_value="ogre",
                description="Gazebo rendering engine. Ogre works more reliably in WSL.",
            ),
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                os.pathsep.join(resource_paths),
            ),
            SetEnvironmentVariable(
                "GZ_SIM_SYSTEM_PLUGIN_PATH",
                os.pathsep.join(plugin_paths),
            ),
            SetEnvironmentVariable("QT_QPA_PLATFORM", "xcb"),
            ExecuteProcess(
                cmd=[
                    "gz",
                    "sim",
                    "-v4",
                    "-r",
                    LaunchConfiguration("world"),
                    "--render-engine",
                    LaunchConfiguration("render_engine"),
                ],
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "ros_gz_bridge",
                    "parameter_bridge",
                    "/tracking/gimbal_pitch@std_msgs/msg/Float64]gz.msgs.Double",
                ],
                output="screen",
            ),
        ]
    )
