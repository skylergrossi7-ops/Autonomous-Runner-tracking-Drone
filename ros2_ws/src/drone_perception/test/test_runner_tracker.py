from drone_perception.models import Detection, TrackingState
from drone_perception.runner_tracker import RunnerTracker


def person(confidence: float, track_id: int) -> Detection:
    return Detection(10, 20, 50, 80, 0, "person", confidence, track_id)


def test_runner_lock_and_timeout():
    tracker = RunnerTracker(maximum_lost_seconds=2.0)
    selected = tracker.update(
        [person(0.60, 3), person(0.95, 8)], current_time=10.0
    )
    assert selected.locked_track_id == 8
    assert selected.state == TrackingState.TRACKING
    assert (
        tracker.update([], current_time=11.0).state
        == TrackingState.TEMPORARILY_LOST
    )
    assert (
        tracker.update([], current_time=12.1).state
        == TrackingState.LOST
    )


def test_idless_detection_is_available_at_low_frame_rate():
    tracker = RunnerTracker(maximum_lost_seconds=2.0)
    detection = person(0.80, None)
    selected = tracker.update([detection], current_time=10.0)
    assert selected.state == TrackingState.TRACKING
    assert selected.runner == detection
    assert selected.locked_track_id == -1
