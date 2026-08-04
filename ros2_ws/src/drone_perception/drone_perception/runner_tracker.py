"""Select and maintain a lock on one YOLO-tracked runner."""

import time
from typing import Iterable, Optional

from .models import Detection, TrackingResult, TrackingState


class RunnerTracker:
    def __init__(self, maximum_lost_seconds: float = 2.0) -> None:
        if maximum_lost_seconds <= 0:
            raise ValueError("maximum_lost_seconds must be positive")
        self.maximum_lost_seconds = maximum_lost_seconds
        self._target_id: Optional[int] = None
        self._last_seen_time: Optional[float] = None

    @staticmethod
    def select_initial_target(
        detections: Iterable[Detection],
    ) -> Optional[Detection]:
        return max(
            detections,
            key=lambda item: item.confidence,
            default=None,
        )

    @staticmethod
    def find_existing_target(
        detections: Iterable[Detection], target_id: int
    ) -> Optional[Detection]:
        if target_id == -1:
            return max(
                detections,
                key=lambda item: item.confidence,
                default=None,
            )
        return next(
            (
                detection
                for detection in detections
                if detection.track_id == target_id
            ),
            None,
        )

    def update(
        self,
        detections: Iterable[Detection],
        current_time: Optional[float] = None,
    ) -> TrackingResult:
        now = time.monotonic() if current_time is None else current_time
        detections = tuple(detections)

        if self._target_id is None:
            target = self.select_initial_target(detections)
            if target is None:
                return TrackingResult(TrackingState.SEARCHING, None, None)
            # Ultralytics may omit IDs when the simulation frame rate is
            # low. Use -1 as a confidence-selection fallback until a stable
            # tracker ID is available.
            self._target_id = (
                target.track_id if target.track_id is not None else -1
            )
            self._last_seen_time = now
            return TrackingResult(
                TrackingState.TRACKING, target, self._target_id
            )

        target = self.find_existing_target(detections, self._target_id)
        if target is not None:
            self._last_seen_time = now
            return TrackingResult(
                TrackingState.TRACKING, target, self._target_id
            )

        lost_duration = (
            now - self._last_seen_time
            if self._last_seen_time is not None
            else self.maximum_lost_seconds
        )
        if lost_duration <= self.maximum_lost_seconds:
            return TrackingResult(
                TrackingState.TEMPORARILY_LOST, None, self._target_id
            )

        old_target_id = self._target_id
        self._target_id = None
        self._last_seen_time = None
        return TrackingResult(TrackingState.LOST, None, old_target_id)
