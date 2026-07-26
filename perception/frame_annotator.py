"""Draw runner tracking information with OpenCV."""

from __future__ import annotations

import cv2

from .models import PerceptionResult, TrackingState


class FrameAnnotator:
    TARGET_COLOR = (0, 0, 255)
    OFFSET_LINE_COLOR = (255, 255, 0)

    def annotate(self, frame, result: PerceptionResult):
        """Return a copy of the frame with tracking graphics."""
        output = frame.copy()
        runner = result.tracking.runner

        self._draw_frame_center(output)
        self._draw_state(output, result)

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

        runner_x, runner_y = runner.center
        frame_center = (result.frame_width // 2, result.frame_height // 2)
        runner_center = (int(runner_x), int(runner_y))
        cv2.circle(output, runner_center, 5, self.TARGET_COLOR, -1)
        cv2.line(
            output,
            frame_center,
            runner_center,
            self.OFFSET_LINE_COLOR,
            2,
        )
        cv2.putText(
            output,
            f"offset x={result.offset_x:+.2f} y={result.offset_y:+.2f}",
            (20, result.frame_height - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            self.OFFSET_LINE_COLOR,
            2,
        )
        return output

    @staticmethod
    def _draw_frame_center(frame) -> None:
        height, width = frame.shape[:2]
        center = (width // 2, height // 2)
        cv2.drawMarker(
            frame,
            center,
            (255, 255, 255),
            cv2.MARKER_CROSS,
            18,
            2,
        )

    @staticmethod
    def _draw_state(frame, result: PerceptionResult) -> None:
        state = result.tracking.state
        color = (
            (0, 200, 0)
            if state == TrackingState.TRACKING
            else (0, 165, 255)
        )
        cv2.putText(
            frame,
            f"State: {state.value}",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2,
        )
