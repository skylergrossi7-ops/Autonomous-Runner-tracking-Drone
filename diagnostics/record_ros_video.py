"""Record an exact-duration MP4 from unique ROS frames while tracking is active."""

import sys
import math

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
        self._required_frames = math.ceil(duration_seconds * output_fps)
        self._active = False
        self._writer = None
        self._frames_written = 0

        self.create_subscription(Image, image_topic, self._image_callback, 10)
        self.create_subscription(
            Bool, "/tracking/enabled", self._activation_callback, 10
        )
        print("VIDEO_READY", flush=True)

    def _image_callback(self, message: Image) -> None:
        if not self._active:
            return

        frame = self._bridge.imgmsg_to_cv2(
            message, desired_encoding="bgr8"
        ).copy()

        if self._writer is None:
            height, width = frame.shape[:2]
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
            print("VIDEO_RECORDING_STARTED", flush=True)

        self._writer.write(frame)
        self._frames_written += 1

        if self._frames_written >= self._required_frames:
            self._finish()
            rclpy.shutdown()

    def _activation_callback(self, message: Bool) -> None:
        self._active = bool(message.data)

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
    if len(sys.argv) not in (4, 5):
        raise SystemExit(
            "usage: record_ros_video.py IMAGE_TOPIC OUTPUT.mp4 "
            "DURATION_SECONDS [OUTPUT_FPS]"
        )
    rclpy.init()
    node = ActiveFollowVideoRecorder(
        image_topic=sys.argv[1],
        output_path=sys.argv[2],
        duration_seconds=float(sys.argv[3]),
        output_fps=float(sys.argv[4]) if len(sys.argv) == 5 else 10.0,
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
