import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    simulation_share = get_package_share_directory("drone_sim")
    control_share = get_package_share_directory("drone_control")
    world = os.path.join(simulation_share, "worlds", "person_drone.sdf")
    nav2_params = os.path.join(control_share, "config", "nav2_params.yaml")

    start_depth_ai = LaunchConfiguration("start_depth_ai")
    model_code_path = LaunchConfiguration("model_code_path")
    checkpoint_path = LaunchConfiguration("checkpoint_path")
    headless = LaunchConfiguration("headless")

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_depth_ai", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument(
                "model_code_path",
                default_value=os.path.expanduser(
                    "~/Depth-Anything-V2/metric_depth"
                ),
            ),
            DeclareLaunchArgument(
                "checkpoint_path",
                default_value=os.path.expanduser(
                    "~/Depth-Anything-V2/metric_depth/checkpoints/"
                    "depth_anything_v2_metric_vkitti_vits.pth"
                ),
            ),
            ExecuteProcess(
                cmd=["gz", "sim", "-r", "-v", "3", world],
                condition=UnlessCondition(headless),
                output="screen",
            ),
            ExecuteProcess(
                cmd=["gz", "sim", "-s", "-r", "-v", "3", world],
                condition=IfCondition(headless),
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="camera_bridge",
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                    "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
                ],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="odom_to_base_link",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "1.2",
                    "--roll", "0", "--pitch", "0", "--yaw", "0",
                    "--frame-id", "odom", "--child-frame-id", "base_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_to_camera_optical",
                arguments=[
                    "--x", "0.25", "--y", "0", "--z", "0.05",
                    "--roll", "-1.5707963", "--pitch", "0",
                    "--yaw", "-1.5707963", "--frame-id", "base_link",
                    "--child-frame-id", "camera_optical_frame",
                ],
            ),
            Node(
                package="runner_perception",
                executable="camera_info_node",
                name="camera_info_node",
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="runner_perception",
                executable="perception_node",
                name="runner_perception",
                parameters=[{"show_window": False, "use_sim_time": True}],
                additional_env={
                    "OMP_NUM_THREADS": "2",
                    "MKL_NUM_THREADS": "2",
                    "OPENBLAS_NUM_THREADS": "2",
                },
                output="screen",
            ),
            Node(
                package="runner_perception",
                executable="depth_anything_node",
                name="depth_anything_node",
                condition=IfCondition(start_depth_ai),
                parameters=[
                    {
                        "model_code_path": model_code_path,
                        "checkpoint_path": checkpoint_path,
                        "use_sim_time": True,
                    }
                ],
                additional_env={
                    "OMP_NUM_THREADS": "2",
                    "MKL_NUM_THREADS": "2",
                    "OPENBLAS_NUM_THREADS": "2",
                },
                output="screen",
            ),
            Node(
                package="runner_perception",
                executable="yolo_masking_node",
                name="yolo_masking_node",
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
            LifecycleNode(
                package="nav2_costmap_2d",
                executable="nav2_costmap_2d",
                namespace="local_costmap",
                name="local_costmap",
                parameters=[nav2_params],
                output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                namespace="local_costmap",
                name="lifecycle_manager",
                parameters=[
                    {
                        "use_sim_time": True,
                        "autostart": True,
                        "bond_timeout": 0.0,
                        "node_names": ["local_costmap"],
                    }
                ],
                output="screen",
            ),
        ]
    )
