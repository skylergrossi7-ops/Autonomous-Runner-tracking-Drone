"""Asynchronous ROS 2 perception node with YOLO reacquisition."""

import os
import threading
import time
from typing import Optional

# Avoid CPU oversubscription when Gazebo, MAVROS, OpenCV, and Torch coexist.
os.environ.setdefault("OMP_NUM_THREADS", "2")
os.environ.setdefault("MKL_NUM_THREADS", "2")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "2")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "2")

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3Stamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from .fast_box_tracker import FastBoxTracker
from .models import Detection
from .person_detector import PersonDetector


class PerceptionNode(Node):
    """Run slow YOLO and fast optical-flow tracking concurrently."""

    def __init__(self) -> None:
        super().__init__("perception_node")
        self.get_logger().info("Perception node initialization started")

        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("inference_image_size", 640)
        self.declare_parameter("person_class_id", 0)
        self.declare_parameter("device", "")
        self.declare_parameter("yolo_reacquire_seconds", 1.0)
        self.declare_parameter("maximum_prediction_frames", 3)
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter(
            "annotated_topic", "/perception/annotated_image"
        )
        self.declare_parameter(
            "detections_topic", "/perception/detections"
        )
        self.declare_parameter("runner_topic", "/perception/runner")
        self.declare_parameter(
            "target_topic", "/perception/runner_target"
        )

        self._detector = PersonDetector(
            model_path=str(self.get_parameter("model_path").value),
            confidence_threshold=float(
                self.get_parameter("confidence_threshold").value
            ),
            person_class_id=int(
                self.get_parameter("person_class_id").value
            ),
            device=str(self.get_parameter("device").value),
            inference_image_size=int(
                self.get_parameter("inference_image_size").value
            ),
        )
        self.get_logger().info("YOLO model loaded")
        self._fast_tracker = FastBoxTracker(
            maximum_prediction_frames=int(
                self.get_parameter(
                    "maximum_prediction_frames"
                ).value
            )
        )
        self._reacquire_seconds = float(
            self.get_parameter("yolo_reacquire_seconds").value
        )
        self._bridge = CvBridge()
        self._condition = threading.Condition()
        self._tracker_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._latest_frame = None
        self._latest_message: Optional[Image] = None
        self._frame_generation = 0
        self._tracking_generation = 0
        self._tracking_times = []

        self._annotated_publisher = self.create_publisher(
            Image, str(self.get_parameter("annotated_topic").value), 10
        )
        self._detections_publisher = self.create_publisher(
            Detection2DArray,
            str(self.get_parameter("detections_topic").value),
            10,
        )
        self._runner_publisher = self.create_publisher(
            Detection2D,
            str(self.get_parameter("runner_topic").value),
            10,
        )
        self._target_publisher = self.create_publisher(
            Vector3Stamped,
            str(self.get_parameter("target_topic").value),
            10,
        )
        self._inference_latency_publisher = self.create_publisher(
            Float32, "/perception/inference_ms", 10
        )
        self._tracking_rate_publisher = self.create_publisher(
            Float32, "/perception/tracking_hz", 10
        )
        self.get_logger().info("Perception publishers created")

        image_topic = str(self.get_parameter("image_topic").value)
        self._image_subscription = self.create_subscription(
            Image,
            image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info("Camera subscription created")
        self._tracking_thread = threading.Thread(
            target=self._tracking_loop,
            name="optical-flow-tracker",
            daemon=True,
        )
        self._detector_thread = threading.Thread(
            target=self._detector_loop,
            name="yolo-reacquisition",
            daemon=True,
        )
        self._tracking_thread.start()
        self._detector_thread.start()
        self.get_logger().info(
            f"Asynchronous perception waiting on {image_topic}"
        )

    def _image_callback(self, image_message: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(
                image_message,
                desired_encoding="bgr8",
            )
        except Exception as error:
            self.get_logger().error(f"Image conversion failed: {error}")
            return
        with self._condition:
            self._latest_frame = frame
            self._latest_message = image_message
            self._frame_generation += 1
            self._condition.notify_all()

    def _tracking_loop(self) -> None:
        while not self._stop_event.is_set():
            item = self._wait_for_new_frame()
            if item is None:
                continue
            frame, image_message = item
            with self._tracker_lock:
                runner = self._fast_tracker.update(frame)
            if runner is None:
                continue
            self._publish_runner(runner, frame, image_message)
            self._publish_tracking_rate()

    def _detector_loop(self) -> None:
        while not self._stop_event.is_set():
            item = self._latest_snapshot()
            if item is None:
                self._stop_event.wait(0.1)
                continue
            frame, image_message = item
            started = time.monotonic()
            try:
                detections = tuple(self._detector.detect(frame))
            except Exception as error:
                self.get_logger().error(f"YOLO inference failed: {error}")
                self._stop_event.wait(self._reacquire_seconds)
                continue
            latency_ms = (time.monotonic() - started) * 1000.0
            self._inference_latency_publisher.publish(
                Float32(data=float(latency_ms))
            )
            self._publish_detections(detections, image_message)
            runner = max(
                detections,
                key=lambda item: item.confidence,
                default=None,
            )
            if runner is not None:
                with self._tracker_lock:
                    self._fast_tracker.initialize(frame, runner)
                self._publish_runner(runner, frame, image_message)
            self._stop_event.wait(self._reacquire_seconds)

    def _wait_for_new_frame(self):
        with self._condition:
            self._condition.wait_for(
                lambda: (
                    self._stop_event.is_set()
                    or self._frame_generation > self._tracking_generation
                ),
                timeout=0.5,
            )
            if self._stop_event.is_set() or self._latest_frame is None:
                return None
            self._tracking_generation = self._frame_generation
            return self._latest_frame.copy(), self._latest_message

    def _latest_snapshot(self):
        with self._condition:
            if self._latest_frame is None:
                return None
            return self._latest_frame.copy(), self._latest_message

    def _publish_detections(
        self,
        detections,
        image_message: Image,
    ) -> None:
        message = Detection2DArray()
        message.header = image_message.header
        message.detections = [
            self._to_ros_detection(item, image_message)
            for item in detections
        ]
        self._detections_publisher.publish(message)

    def _publish_runner(
        self,
        runner: Detection,
        frame,
        image_message: Image,
    ) -> None:
        self._runner_publisher.publish(
            self._to_ros_detection(runner, image_message)
        )
        self._target_publisher.publish(
            self._to_target_message(
                runner,
                frame.shape[1],
                frame.shape[0],
                image_message,
            )
        )
        annotated = frame.copy()
        center = (frame.shape[1] // 2, frame.shape[0] // 2)
        runner_center = tuple(int(value) for value in runner.center)
        cv2.rectangle(
            annotated,
            (runner.x1, runner.y1),
            (runner.x2, runner.y2),
            (0, 0, 255),
            2,
        )
        cv2.line(annotated, center, runner_center, (255, 255, 0), 2)
        annotated_message = self._bridge.cv2_to_imgmsg(
            annotated,
            encoding="bgr8",
        )
        annotated_message.header = image_message.header
        self._annotated_publisher.publish(annotated_message)

    def _publish_tracking_rate(self) -> None:
        now = time.monotonic()
        self._tracking_times.append(now)
        self._tracking_times = [
            item for item in self._tracking_times if now - item <= 2.0
        ]
        if len(self._tracking_times) < 2:
            return
        elapsed = self._tracking_times[-1] - self._tracking_times[0]
        if elapsed > 0.0:
            rate = (len(self._tracking_times) - 1) / elapsed
            self._tracking_rate_publisher.publish(
                Float32(data=float(rate))
            )

    @staticmethod
    def _to_ros_detection(
        detection: Detection,
        image_message: Image,
    ) -> Detection2D:
        message = Detection2D()
        message.header = image_message.header
        center_x, center_y = detection.center
        size_x, size_y = detection.size
        message.bbox.center.position.x = center_x
        message.bbox.center.position.y = center_y
        message.bbox.size_x = size_x
        message.bbox.size_y = size_y
        message.id = (
            str(detection.track_id)
            if detection.track_id is not None
            else ""
        )
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = str(detection.class_id)
        hypothesis.hypothesis.score = detection.confidence
        message.results.append(hypothesis)
        return message

    @staticmethod
    def _to_target_message(
        runner: Detection,
        width: int,
        height: int,
        image_message: Image,
    ) -> Vector3Stamped:
        message = Vector3Stamped()
        message.header = image_message.header
        center_x, center_y = runner.center
        box_width, box_height = runner.size
        message.vector.x = (2.0 * center_x / float(width)) - 1.0
        message.vector.y = (2.0 * center_y / float(height)) - 1.0
        message.vector.z = (
            box_width * box_height / float(width * height)
        )
        return message

    def destroy_node(self):
        self._stop_event.set()
        with self._condition:
            self._condition.notify_all()
        self._tracking_thread.join(timeout=2.0)
        self._detector_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
