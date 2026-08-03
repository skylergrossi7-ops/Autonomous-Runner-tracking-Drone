"""Draw runner tracking information with OpenCV."""

import cv2

from .models import PerceptionResult, TrackingState


class FrameAnnotator:
    TARGET_COLOR = (0, 0, 255)
    OFFSET_COLOR = (255, 255, 0)

    def annotate(self, frame, result: PerceptionResult):
        output = frame.copy()
        center = (result.frame_width // 2, result.frame_height // 2)
        cv2.drawMarker(
            output, center, (255, 255, 255), cv2.MARKER_CROSS, 18, 2
        )

        state_color = (
            (0, 200, 0)
            if result.tracking.state == TrackingState.TRACKING
            else (0, 165, 255)
        )
        cv2.putText(
            output,
            f"State: {result.tracking.state.value}",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            state_color,
            2,
        )

        runner = result.tracking.runner
        if runner is None:
            return output

        cv2.rectangle(
            output,
            (runner.x1, runner.y1),
            (runner.x2, runner.y2),
            self.TARGET_COLOR,
            2,
        )
        cv2.putText(
            output,
            f"TARGET ID {runner.track_id} conf={runner.confidence:.2f}",
            (runner.x1, max(20, runner.y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            self.TARGET_COLOR,
            2,
        )
        runner_center = tuple(int(value) for value in runner.center)
        cv2.circle(output, runner_center, 5, self.TARGET_COLOR, -1)
        cv2.line(output, center, runner_center, self.OFFSET_COLOR, 2)
        cv2.putText(
            output,
            f"offset x={result.offset_x:+.2f} y={result.offset_y:+.2f}",
            (20, result.frame_height - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            self.OFFSET_COLOR,
            2,
        )
        return output
