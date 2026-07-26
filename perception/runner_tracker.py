"""Select and maintain a lock on one YOLO-tracked runner."""

from __future__ import annotations

import time
from typing import Iterable, Optional

from .models import Detection, TrackingResult, TrackingState


class RunnerTracker:
    """Preserve the target-selection behavior from the original script."""

    def __init__(self, maximum_lost_seconds: float = 2.0) -> None:
        if maximum_lost_seconds <= 0:
            raise ValueError("maximum_lost_seconds must be positive")
        self.maximum_lost_seconds = maximum_lost_seconds
        self._target_id: Optional[int] = None
        self._last_seen_time: Optional[float] = None

    @property
    def target_id(self) -> Optional[int]:
        return self._target_id

    @staticmethod
    def select_initial_target(
        detections: Iterable[Detection],
    ) -> Optional[Detection]:
        """Select the highest-confidence person with a tracking ID."""
        trackable = [
            detection
            for detection in detections
            if detection.track_id is not None
        ]
        return max(trackable, key=lambda item: item.confidence, default=None)

    @staticmethod
    def find_existing_target(
        detections: Iterable[Detection], target_id: int
    ) -> Optional[Detection]:
        """Find the detection matching the currently locked tracking ID."""
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
        """Update the runner lock using detections from the newest frame."""
        now = time.monotonic() if current_time is None else current_time
        detections = tuple(detections)

        if self._target_id is None:
            target = self.select_initial_target(detections)
            if target is None:
                return TrackingResult(TrackingState.SEARCHING, None, None)

            self._target_id = target.track_id
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

    def reset(self) -> None:
        self._target_id = None
        self._last_seen_time = None
