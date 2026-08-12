import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    simulation_share = get_package_share_directory("drone_simulation")
    control_share = get_package_share_directory("drone_control")
    world = os.path.join(simulation_share, "worlds", "runner_tracking.sdf")
    package_models = os.path.join(simulation_share, "models")
    ardupilot_models = os.path.expanduser("~/ardupilot_gazebo/models")
    ardupilot_plugins = os.path.expanduser("~/ardupilot_gazebo/build")
    nav2_params = os.path.join(control_share, "config", "nav2_params.yaml")

    resource_paths = [package_models, ardupilot_models]
    if os.environ.get("GZ_SIM_RESOURCE_PATH"):
        resource_paths.append(os.environ["GZ_SIM_RESOURCE_PATH"])
    plugin_paths = [ardupilot_plugins]
    if os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH"):
        plugin_paths.append(os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"])

    headless = LaunchConfiguration("headless")
    start_rviz = LaunchConfiguration("start_rviz")
    use_live_odometry = LaunchConfiguration("use_live_odometry")
    model_code_path = LaunchConfiguration("model_code_path")
    checkpoint_path = LaunchConfiguration("checkpoint_path")

    common_ai_environment = {
        "OMP_NUM_THREADS": "2",
        "MKL_NUM_THREADS": "2",
        "OPENBLAS_NUM_THREADS": "2",
    }

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("start_rviz", default_value="false"),
        DeclareLaunchArgument("use_live_odometry", default_value="false"),
        DeclareLaunchArgument(
            "model_code_path",
            default_value=os.path.expanduser("~/Depth-Anything-V2/metric_depth"),
        ),
        DeclareLaunchArgument(
            "checkpoint_path",
            default_value=os.path.expanduser(
                "~/Depth-Anything-V2/metric_depth/checkpoints/"
                "depth_anything_v2_metric_vkitti_vits.pth"
            ),
        ),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.pathsep.join(resource_paths)),
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", os.pathsep.join(plugin_paths)),
        SetEnvironmentVariable("QT_QPA_PLATFORM", "xcb"),
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
            name="iris_camera_bridge",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="odom_to_iris_base",
            condition=UnlessCondition(use_live_odometry),
            arguments=[
                "--x", "0", "--y", "0", "--z", "0.195",
                "--roll", "0", "--pitch", "0", "--yaw", "1.5707963",
                "--frame-id", "odom", "--child-frame-id", "base_link",
            ],
        ),
        Node(
            package="drone_control",
            executable="mavros_pose_tf",
            name="mavros_pose_tf",
            condition=IfCondition(use_live_odometry),
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="iris_base_to_camera_optical",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0.06",
                "--roll", "-1.3707963", "--pitch", "0",
                "--yaw", "-1.5707963", "--frame-id", "base_link",
                "--child-frame-id", "camera_optical_frame",
            ],
        ),
        Node(
            package="runner_perception",
            executable="camera_info_node",
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="runner_perception",
            executable="perception_node",
            name="runner_perception",
            parameters=[{
                "show_window": False,
                # Keep enough actor detail for reliable detection after the
                # vehicle climbs to its two-metre tracking altitude.
                # 320 is the lowest resolution that still detects this actor;
                # the measured confidence is about 0.115.  Keeping inference
                # here prevents neural processing from starving MAVLink
                # heartbeats on a laptop CPU.
                "inference_image_size": 320,
                "confidence_threshold": 0.08,
                "use_sim_time": True,
            }],
            additional_env=common_ai_environment,
            output="screen",
        ),
        Node(
            package="runner_perception",
            executable="depth_anything_node",
            parameters=[{
                "model_code_path": model_code_path,
                "checkpoint_path": checkpoint_path,
                "depth_scale": 0.5,
                "input_size": 256,
                "pointcloud_stride": 4,
                "use_sim_time": True,
            }],
            additional_env=common_ai_environment,
            output="screen",
        ),
        Node(
            package="runner_perception",
            executable="yolo_masking_node",
            parameters=[{
                "use_sim_time": True,
                "max_bbox_age": 4.0,
                "max_z": 20.0,
            }],
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
            parameters=[{
                "use_sim_time": True,
                "autostart": True,
                "bond_timeout": 0.0,
                "node_names": ["local_costmap"],
            }],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="iris_navigation_rviz",
            condition=IfCondition(start_rviz),
            arguments=["-d", os.path.join(simulation_share, "config", "iris_ai_navigation.rviz")],
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
    ])
