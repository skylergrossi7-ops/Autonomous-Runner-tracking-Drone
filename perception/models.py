"""Plain data objects passed between perception components."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple


class TrackingState(str, Enum):
    SEARCHING = "SEARCHING"
    TRACKING = "TRACKING"
    TEMPORARILY_LOST = "TEMPORARILY_LOST"
    LOST = "LOST"


@dataclass(frozen=True)
class Detection:
    x1: int
    y1: int
    x2: int
    y2: int
    class_id: int
    class_name: str
    confidence: float
    track_id: Optional[int] = None

    @property
    def center(self) -> Tuple[float, float]:
        return ((self.x1 + self.x2) / 2.0, (self.y1 + self.y2) / 2.0)


@dataclass(frozen=True)
class TrackingResult:
    state: TrackingState
    runner: Optional[Detection]
    locked_track_id: Optional[int]


@dataclass(frozen=True)
class PerceptionResult:
    detections: Tuple[Detection, ...]
    tracking: TrackingResult
    frame_width: int
    frame_height: int
    offset_x: Optional[float]
    offset_y: Optional[float]
