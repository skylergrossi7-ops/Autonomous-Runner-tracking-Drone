"""Standalone laptop-camera runner perception test.

Run:

    python run_camera.py

Press q in the preview window to exit.
"""

import cv2

from perception.camera_source import CameraSource
from perception.frame_annotator import FrameAnnotator
from perception.person_detector import PersonDetector
from perception.pipeline import PerceptionPipeline
from perception.runner_tracker import RunnerTracker


WINDOW_NAME = "Drone Camera Test"


def main() -> None:
    camera = CameraSource(source=0, width=960, height=540)
    detector = PersonDetector(
        model_path="yolov8n.pt",
        confidence_threshold=0.45,
        person_class_id=0,
    )
    tracker = RunnerTracker(maximum_lost_seconds=2.0)
    pipeline = PerceptionPipeline(detector, tracker, FrameAnnotator())

    try:
        camera.open()
        print(
            "Camera and YOLO started. "
            "Press q in the preview window to stop."
        )

        while True:
            frame = camera.read()
            result, annotated_frame = pipeline.process(frame)
            cv2.imshow(WINDOW_NAME, annotated_frame)

            if result.tracking.runner is not None:
                print(
                    f"\rID={result.tracking.locked_track_id} "
                    f"offset_x={result.offset_x:+.2f} "
                    f"offset_y={result.offset_y:+.2f}",
                    end="",
                    flush=True,
                )

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()
        print()


if __name__ == "__main__":
    main()
