"""Fast inter-frame person-box tracking with optical flow and prediction."""

from typing import Optional

import cv2
import numpy as np

from .models import Detection


class FastBoxTracker:
    """Translate a YOLO box between detections using sparse optical flow."""

    def __init__(
        self,
        maximum_prediction_frames: int = 3,
        minimum_features: int = 6,
    ) -> None:
        self.maximum_prediction_frames = maximum_prediction_frames
        self.minimum_features = minimum_features
        self._previous_gray = None
        self._points = None
        self._detection: Optional[Detection] = None
        self._velocity = np.zeros(2, dtype=np.float32)
        self._prediction_frames = 0

    @property
    def active(self) -> bool:
        return self._detection is not None

    def reset(self) -> None:
        self._previous_gray = None
        self._points = None
        self._detection = None
        self._velocity[:] = 0.0
        self._prediction_frames = 0

    def initialize(self, frame, detection: Detection) -> None:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self._detection = self._clamp(detection, gray.shape)
        self._previous_gray = gray
        self._points = self._find_features(gray, self._detection)
        self._velocity[:] = 0.0
        self._prediction_frames = 0

    def update(self, frame) -> Optional[Detection]:
        if self._detection is None or self._previous_gray is None:
            return None

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tracked = self._track_features(gray)
        if tracked is not None:
            previous_points, current_points = tracked
            displacement = np.median(
                current_points - previous_points,
                axis=0,
            )
            self._velocity = (
                0.6 * displacement + 0.4 * self._velocity
            ).astype(np.float32)
            self._detection = self._translated(
                self._detection,
                float(displacement[0]),
                float(displacement[1]),
                confidence_scale=0.995,
            )
            self._prediction_frames = 0
        elif (
            self._prediction_frames
            < self.maximum_prediction_frames
        ):
            self._detection = self._translated(
                self._detection,
                float(self._velocity[0]),
                float(self._velocity[1]),
                confidence_scale=0.95,
            )
            self._prediction_frames += 1
        else:
            self.reset()
            return None

        self._detection = self._clamp(
            self._detection,
            gray.shape,
        )
        self._previous_gray = gray
        self._points = self._find_features(gray, self._detection)
        return self._detection

    def _track_features(self, current_gray):
        if self._points is None or len(self._points) < self.minimum_features:
            return None
        current_points, status, _ = cv2.calcOpticalFlowPyrLK(
            self._previous_gray,
            current_gray,
            self._points,
            None,
            winSize=(21, 21),
            maxLevel=3,
            criteria=(
                cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                20,
                0.03,
            ),
        )
        if current_points is None or status is None:
            return None
        valid = status.reshape(-1) == 1
        previous = self._points.reshape(-1, 2)[valid]
        current = current_points.reshape(-1, 2)[valid]
        if len(current) < self.minimum_features:
            return None
        return previous, current

    @staticmethod
    def _find_features(gray, detection: Detection):
        mask = np.zeros_like(gray)
        mask[detection.y1:detection.y2, detection.x1:detection.x2] = 255
        return cv2.goodFeaturesToTrack(
            gray,
            maxCorners=80,
            qualityLevel=0.01,
            minDistance=4,
            mask=mask,
            blockSize=5,
        )

    @staticmethod
    def _translated(
        detection: Detection,
        dx: float,
        dy: float,
        confidence_scale: float,
    ) -> Detection:
        return Detection(
            x1=int(round(detection.x1 + dx)),
            y1=int(round(detection.y1 + dy)),
            x2=int(round(detection.x2 + dx)),
            y2=int(round(detection.y2 + dy)),
            class_id=detection.class_id,
            class_name=detection.class_name,
            confidence=detection.confidence * confidence_scale,
            track_id=detection.track_id,
        )

    @staticmethod
    def _clamp(detection: Detection, shape) -> Detection:
        height, width = shape[:2]
        x1 = max(0, min(width - 2, detection.x1))
        y1 = max(0, min(height - 2, detection.y1))
        x2 = max(x1 + 1, min(width - 1, detection.x2))
        y2 = max(y1 + 1, min(height - 1, detection.y2))
        return Detection(
            x1=x1,
            y1=y1,
            x2=x2,
            y2=y2,
            class_id=detection.class_id,
            class_name=detection.class_name,
            confidence=detection.confidence,
            track_id=detection.track_id,
        )
