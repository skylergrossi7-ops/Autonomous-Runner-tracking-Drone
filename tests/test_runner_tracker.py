from perception.models import Detection, TrackingState
from perception.runner_tracker import RunnerTracker


def person(confidence: float, track_id: int) -> Detection:
    return Detection(10, 20, 50, 80, 0, "person", confidence, track_id)


def test_selects_highest_confidence_trackable_person():
    tracker = RunnerTracker()
    result = tracker.update(
        [person(0.60, 3), person(0.95, 8), person(0.80, 4)],
        current_time=10.0,
    )
    assert result.state == TrackingState.TRACKING
    assert result.locked_track_id == 8


def test_keeps_existing_runner_and_releases_after_timeout():
    tracker = RunnerTracker(maximum_lost_seconds=2.0)
    tracker.update([person(0.90, 7)], current_time=10.0)

    temporary = tracker.update([], current_time=11.0)
    assert temporary.state == TrackingState.TEMPORARILY_LOST
    assert temporary.locked_track_id == 7

    lost = tracker.update([], current_time=12.1)
    assert lost.state == TrackingState.LOST
    assert tracker.target_id is None
