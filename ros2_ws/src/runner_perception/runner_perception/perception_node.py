import cv2
import rclpy

from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import Detection2D
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

DETECTION_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class PerceptionNode(Node):

    def __init__(self):
        super().__init__("runner_perception")

        self.declare_parameter(
            "model_path",
            "/home/skyler/models/yolov8n.pt",
        )

        self.declare_parameter(
            "confidence_threshold",
            0.45,
        )

        self.declare_parameter(
            "show_window",
            True,
        )

        self.declare_parameter(
            "inference_image_size",
            320,
        )

        model_path = self.get_parameter(
            "model_path"
        ).value

        self.confidence_threshold = self.get_parameter(
            "confidence_threshold"
        ).value

        self.show_window = self.get_parameter(
            "show_window"
        ).value

        self.inference_image_size = int(
            self.get_parameter("inference_image_size").value
        )

        self.bridge = CvBridge()
        self.model = YOLO(model_path)

        self.image_subscription = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            SENSOR_QOS,
        )

        self.detection_publisher = self.create_publisher(
            Detection2DArray,
            "/perception/detections",
            DETECTION_QOS,
        )
        self.runner_publisher = self.create_publisher(
            Detection2D,
            "/perception/runner",
            DETECTION_QOS,
        )

        self.debug_image_publisher = self.create_publisher(
            Image,
            "/perception/debug_image",
            SENSOR_QOS,
        )

        self.get_logger().info(
            "Perception node started"
        )

        self.get_logger().info(
            "Waiting for images on /camera/image_raw"
        )

    def image_callback(self, image_message):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                image_message,
                desired_encoding="bgr8",
            )
        except Exception as error:
            self.get_logger().error(
                f"Image conversion failed: {error}"
            )
            return

        results = self.model.track(
            frame,
            persist=True,
            verbose=False,
            classes=[0],
            conf=self.confidence_threshold,
            imgsz=self.inference_image_size,
        )

        detections_message = Detection2DArray()
        detections_message.header = image_message.header

        debug_frame = frame.copy()

        if results:
            boxes = results[0].boxes

            for box in boxes:
                x1, y1, x2, y2 = map(
                    float,
                    box.xyxy[0],
                )

                confidence = float(
                    box.conf.item()
                )

                tracking_id = -1

                if box.id is not None:
                    tracking_id = int(
                        box.id.item()
                    )

                detection = Detection2D()
                detection.header = image_message.header
                detection.id = str(tracking_id)

                detection.bbox.center.position.x = (
                    x1 + x2
                ) / 2.0

                detection.bbox.center.position.y = (
                    y1 + y2
                ) / 2.0

                detection.bbox.size_x = x2 - x1
                detection.bbox.size_y = y2 - y1

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = "person"
                hypothesis.hypothesis.score = confidence

                detection.results.append(
                    hypothesis
                )

                detections_message.detections.append(
                    detection
                )

                cv2.rectangle(
                    debug_frame,
                    (int(x1), int(y1)),
                    (int(x2), int(y2)),
                    (0, 0, 255),
                    3,
                )

                label = (
                    f"runner {tracking_id} "
                    f"{confidence:.2f}"
                )

                cv2.putText(
                    debug_frame,
                    label,
                    (
                        int(x1),
                        max(30, int(y1) - 10),
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2,
                )

        if not rclpy.ok():
            return

        self.detection_publisher.publish(
            detections_message
        )
        if detections_message.detections:
            runner = max(
                detections_message.detections,
                key=lambda detection: detection.results[0].hypothesis.score,
            )
            self.runner_publisher.publish(runner)

        debug_message = self.bridge.cv2_to_imgmsg(
            debug_frame,
            encoding="bgr8",
        )

        debug_message.header = image_message.header

        self.debug_image_publisher.publish(
            debug_message
        )

        if self.show_window:
            cv2.imshow(
                "Runner Perception",
                debug_frame,
            )

            cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
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
