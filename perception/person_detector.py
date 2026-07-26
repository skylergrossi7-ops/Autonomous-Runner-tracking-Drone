"""YOLO person detection and tracking adapter."""

from __future__ import annotations

from typing import List

from .models import Detection


class PersonDetector:
    """Run Ultralytics YOLO and return only person detections."""

    def __init__(
        self,
        model_path: str = "yolov8n.pt",
        confidence_threshold: float = 0.45,
        person_class_id: int = 0,
    ) -> None:
        if not 0.0 <= confidence_threshold <= 1.0:
            raise ValueError("confidence_threshold must be between 0 and 1")

        try:
            from ultralytics import YOLO
        except ImportError as error:
            raise RuntimeError(
                "Ultralytics is not installed. Run: "
                "python -m pip install -r requirements.txt"
            ) from error

        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.person_class_id = person_class_id
        self._model = YOLO(model_path)

    def detect(self, frame) -> List[Detection]:
        """Detect and track people in one OpenCV BGR frame."""
        results = self._model.track(
            frame,
            persist=True,
            verbose=False,
            conf=self.confidence_threshold,
            classes=[self.person_class_id],
        )

        if not results or results[0].boxes is None:
            return []

        detections = []
        class_names = results[0].names
        for box in results[0].boxes:
            class_id = int(box.cls.item())
            if class_id != self.person_class_id:
                continue

            x1, y1, x2, y2 = (
                int(value) for value in box.xyxy[0].tolist()
            )
            track_id = int(box.id.item()) if box.id is not None else None
            detections.append(
                Detection(
                    x1=x1,
                    y1=y1,
                    x2=x2,
                    y2=y2,
                    class_id=class_id,
                    class_name=str(class_names[class_id]),
                    confidence=float(box.conf.item()),
                    track_id=track_id,
                )
            )
        return detections
