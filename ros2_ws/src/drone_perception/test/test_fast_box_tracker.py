import cv2
import numpy as np

from drone_perception.fast_box_tracker import FastBoxTracker
from drone_perception.models import Detection


def textured_frame(dx=0, dy=0):
    frame = np.zeros((120, 160, 3), dtype=np.uint8)
    for y in range(35, 76, 8):
        for x in range(45, 86, 8):
            cv2.circle(frame, (x + dx, y + dy), 2, (255, 255, 255), -1)
    return frame


def person_box():
    return Detection(40, 30, 90, 82, 0, "person", 0.9)


def test_tracker_follows_inter_frame_translation():
    tracker = FastBoxTracker(minimum_features=4)
    tracker.initialize(textured_frame(), person_box())

    tracked = tracker.update(textured_frame(dx=5, dy=3))

    assert tracked is not None
    assert abs(tracked.center[0] - (person_box().center[0] + 5)) <= 1
    assert abs(tracked.center[1] - (person_box().center[1] + 3)) <= 1
    assert tracked.confidence < person_box().confidence


def test_tracker_stops_after_prediction_limit():
    tracker = FastBoxTracker(
        maximum_prediction_frames=2,
        minimum_features=100,
    )
    tracker.initialize(textured_frame(), person_box())

    assert tracker.update(textured_frame()) is not None
    assert tracker.update(textured_frame()) is not None
    assert tracker.update(textured_frame()) is None
    assert not tracker.active
