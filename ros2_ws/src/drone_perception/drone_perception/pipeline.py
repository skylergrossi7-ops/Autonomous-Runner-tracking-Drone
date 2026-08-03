"""Coordinate person detection, runner tracking, and annotation."""

from .models import PerceptionResult


class PerceptionPipeline:
    def __init__(self, detector, tracker, annotator) -> None:
        self.detector = detector
        self.tracker = tracker
        self.annotator = annotator

    def process(self, frame):
        height, width = frame.shape[:2]
        detections = tuple(self.detector.detect(frame))
        tracking = self.tracker.update(detections)

        offset_x = None
        offset_y = None
        if tracking.runner is not None:
            runner_x, runner_y = tracking.runner.center
            offset_x = (runner_x - width / 2.0) / (width / 2.0)
            offset_y = (runner_y - height / 2.0) / (height / 2.0)

        result = PerceptionResult(
            detections=detections,
            tracking=tracking,
            frame_width=width,
            frame_height=height,
            offset_x=offset_x,
            offset_y=offset_y,
        )
        return result, self.annotator.annotate(frame, result)
