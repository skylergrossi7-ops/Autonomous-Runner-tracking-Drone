"""Record an exact-duration MP4 from a ROS image topic while tracking is active."""

import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class ActiveFollowVideoRecorder(Node):
    def __init__(
        self,
        image_topic: str,
        output_path: str,
        duration_seconds: float,
        output_fps: float = 10.0,
    ) -> None:
        super().__init__("active_follow_video_recorder")
        self._bridge = CvBridge()
        self._output_path = output_path
        self._duration = duration_seconds
        self._output_fps = output_fps
        self._latest_frame = None
        self._active = False
        self._started_at = None
        self._writer = None
        self._frames_written = 0

        self.create_subscription(Image, image_topic, self._image_callback, 10)
        self.create_subscription(
            Bool, "/tracking/enabled", self._activation_callback, 10
        )
        self.create_timer(1.0 / output_fps, self._sample_frame)
        print("VIDEO_READY", flush=True)

    def _image_callback(self, message: Image) -> None:
        self._latest_frame = self._bridge.imgmsg_to_cv2(
            message, desired_encoding="bgr8"
        ).copy()

    def _activation_callback(self, message: Bool) -> None:
        self._active = bool(message.data)

    def _sample_frame(self) -> None:
        if not self._active or self._latest_frame is None:
            return

        if self._writer is None:
            height, width = self._latest_frame.shape[:2]
            self._writer = cv2.VideoWriter(
                self._output_path,
                cv2.VideoWriter_fourcc(*"mp4v"),
                self._output_fps,
                (width, height),
            )
            if not self._writer.isOpened():
                raise RuntimeError(
                    f"Could not open video output: {self._output_path}"
                )
            self._started_at = time.monotonic()
            print("VIDEO_RECORDING_STARTED", flush=True)

        self._writer.write(self._latest_frame)
        self._frames_written += 1

        if time.monotonic() - self._started_at >= self._duration:
            self._finish()
            rclpy.shutdown()

    def _finish(self) -> None:
        if self._writer is not None:
            self._writer.release()
            self._writer = None
            print(
                f"VIDEO_COMPLETE frames={self._frames_written} "
                f"path={self._output_path}",
                flush=True,
            )

    def destroy_node(self):
        self._finish()
        return super().destroy_node()


def main() -> None:
    if len(sys.argv) != 4:
        raise SystemExit(
            "usage: record_ros_video.py IMAGE_TOPIC OUTPUT.mp4 DURATION_SECONDS"
        )
    rclpy.init()
    node = ActiveFollowVideoRecorder(
        image_topic=sys.argv[1],
        output_path=sys.argv[2],
        duration_seconds=float(sys.argv[3]),
    )
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
