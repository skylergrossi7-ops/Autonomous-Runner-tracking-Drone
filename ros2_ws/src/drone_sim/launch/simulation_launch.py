import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_index_directories():
    pass


def generate_launch_description():
    package_share = get_package_share_directory("drone_sim")
    drone_control_share = get_package_share_directory("drone_control")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    world_file = os.path.join(
        package_share,
        "worlds",
        "person_drone.sdf",
    )

    # Path to your custom Nav2 parameters file
    params_file = os.path.join(drone_control_share, "config", "nav2_params.yaml")

    gazebo = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "-r",
            "-v",
            "3",
            world_file,
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        output="screen",
    )

    # Include Nav2 bringup stack and link it to your config file
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "True",
            "params_file": params_file,
            "autostart": "True",
        }.items(),
    )

    return LaunchDescription([
        gazebo,
        bridge,
        nav2_bringup,
    ])