"""Infer metric monocular depth and publish an organized XYZ point cloud."""

import os
import sys
import time

import cv2
import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


MODEL_CONFIGS = {
    "vits": {
        "encoder": "vits",
        "features": 64,
        "out_channels": [48, 96, 192, 384],
    },
    "vitb": {
        "encoder": "vitb",
        "features": 128,
        "out_channels": [96, 192, 384, 768],
    },
    "vitl": {
        "encoder": "vitl",
        "features": 256,
        "out_channels": [256, 512, 1024, 1024],
    },
}

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class DepthAnythingNode(Node):
    """Run the official metric Depth Anything V2 model on camera images."""

    def __init__(self):
        super().__init__("depth_anything_node")
        self.declare_parameter("model_size", "vits")
        self.declare_parameter("model_code_path", "~/Depth-Anything-V2/metric_depth")
        self.declare_parameter(
            "checkpoint_path",
            "~/Depth-Anything-V2/metric_depth/checkpoints/"
            "depth_anything_v2_metric_vkitti_vits.pth",
        )
        self.declare_parameter("max_depth_metres", 80.0)
        self.declare_parameter("maximum_publish_depth_metres", 20.0)
        self.declare_parameter("depth_scale", 1.0)
        self.declare_parameter("pointcloud_stride", 2)
        self.declare_parameter("input_size", 350)
        self.declare_parameter("maximum_inference_rate", 5.0)
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("depth_topic", "/camera/depth_ai/depth_image")
        self.declare_parameter("points_topic", "/camera/depth_ai/points")
        self.declare_parameter("pointcloud_frame", "camera_optical_frame")

        self._bridge = CvBridge()
        self._intrinsics = None
        self._last_inference_at = 0.0
        self._model = self._load_model()

        self._depth_publisher = self.create_publisher(
            Image, str(self.get_parameter("depth_topic").value), SENSOR_QOS
        )
        self._points_publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("points_topic").value),
            SENSOR_QOS,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._camera_info_callback,
            SENSOR_QOS,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self._image_callback,
            SENSOR_QOS,
        )
        self.get_logger().info(
            f"Metric Depth Anything V2 ready on {self._device}; "
            f"publishing at up to {self.get_parameter('maximum_inference_rate').value} Hz"
        )

    def _load_model(self):
        model_size = str(self.get_parameter("model_size").value)
        if model_size not in MODEL_CONFIGS:
            raise ValueError(f"unsupported model_size: {model_size}")
        code_path = os.path.abspath(
            os.path.expanduser(str(self.get_parameter("model_code_path").value))
        )
        checkpoint = os.path.abspath(
            os.path.expanduser(str(self.get_parameter("checkpoint_path").value))
        )
        if not os.path.isdir(code_path):
            raise FileNotFoundError(
                f"Depth Anything metric code not found: {code_path}"
            )
        if not os.path.isfile(checkpoint):
            raise FileNotFoundError(
                f"Depth Anything metric checkpoint not found: {checkpoint}"
            )
        if code_path not in sys.path:
            sys.path.insert(0, code_path)
        from depth_anything_v2.dpt import DepthAnythingV2

        self._device = "cuda" if torch.cuda.is_available() else "cpu"
        config = dict(MODEL_CONFIGS[model_size])
        config["max_depth"] = float(
            self.get_parameter("max_depth_metres").value
        )
        model = DepthAnythingV2(**config)
        state = torch.load(checkpoint, map_location="cpu")
        model.load_state_dict(state)
        return model.to(self._device).eval()

    def _camera_info_callback(self, message):
        fx, fy = float(message.k[0]), float(message.k[4])
        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().warning("Ignoring invalid camera intrinsics")
            return
        self._intrinsics = (fx, fy, float(message.k[2]), float(message.k[5]))

    def _image_callback(self, message):
        if self._intrinsics is None:
            self.get_logger().warning(
                "Waiting for /camera/camera_info", throttle_duration_sec=2.0
            )
            return
        maximum_rate = float(
            self.get_parameter("maximum_inference_rate").value
        )
        now = time.monotonic()
        if maximum_rate > 0.0 and now - self._last_inference_at < 1.0 / maximum_rate:
            return
        self._last_inference_at = now

        try:
            bgr = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            with torch.inference_mode():
                depth = self._model.infer_image(
                    bgr, int(self.get_parameter("input_size").value)
                )
            depth = np.asarray(depth, dtype=np.float32)
            depth *= float(self.get_parameter("depth_scale").value)
            if depth.shape != bgr.shape[:2]:
                depth = cv2.resize(
                    depth,
                    (bgr.shape[1], bgr.shape[0]),
                    interpolation=cv2.INTER_LINEAR,
                )
            maximum = float(
                self.get_parameter("maximum_publish_depth_metres").value
            )
            depth[~np.isfinite(depth) | (depth <= 0.0) | (depth > maximum)] = np.nan
            depth_message = self._bridge.cv2_to_imgmsg(depth, encoding="32FC1")
            depth_message.header = message.header
            depth_message.header.frame_id = str(
                self.get_parameter("pointcloud_frame").value
            )
            if not rclpy.ok():
                return
            self._depth_publisher.publish(depth_message)
            self._points_publisher.publish(
                self._depth_to_pointcloud(depth, depth_message.header)
            )
        except Exception as error:
            self.get_logger().error(f"Depth inference failed: {error}")

    def _depth_to_pointcloud(self, depth, header):
        height, width = depth.shape
        fx, fy, cx, cy = self._intrinsics
        stride = max(1, int(self.get_parameter("pointcloud_stride").value))
        u, v = np.meshgrid(
            np.arange(0, width, stride, dtype=np.float32),
            np.arange(0, height, stride, dtype=np.float32),
        )
        sampled_depth = depth[::stride, ::stride]
        sampled_height, sampled_width = sampled_depth.shape
        xyz = np.empty((sampled_height, sampled_width, 3), dtype=np.float32)
        xyz[..., 2] = sampled_depth
        xyz[..., 0] = (u - cx) * sampled_depth / fx
        xyz[..., 1] = (v - cy) * sampled_depth / fy

        cloud = PointCloud2()
        cloud.header = header
        cloud.height = sampled_height
        cloud.width = sampled_width
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = sampled_width * cloud.point_step
        cloud.is_dense = False
        cloud.data = xyz.tobytes()
        return cloud


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DepthAnythingNode()
        rclpy.spin(node)
    except (FileNotFoundError, ValueError) as error:
        rclpy.logging.get_logger("depth_anything_node").fatal(str(error))
        raise
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
