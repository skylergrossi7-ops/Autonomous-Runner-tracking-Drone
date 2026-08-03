"""Ultralytics YOLO person detector."""

import os
from typing import List

from .models import Detection

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class PersonDetector:
    def __init__(
        self,
        model_path: str = "yolov8n.pt",
        confidence_threshold: float = 0.45,
        person_class_id: int = 0,
        device: str = "",
        inference_image_size: int = 320,
    ) -> None:
        if not 0.0 <= confidence_threshold <= 1.0:
            raise ValueError("confidence_threshold must be between 0 and 1")

        if YOLO is None:
            raise RuntimeError(
                "Ultralytics is missing. Install it with pip in the "
                "ROS-compatible virtual environment."
            )

        self._model = YOLO(os.path.expanduser(model_path))
        self._confidence_threshold = confidence_threshold
        self._person_class_id = person_class_id
        self._device = device or None
        self._inference_image_size = inference_image_size

    def detect(self, frame) -> List[Detection]:
        results = self._model.predict(
            frame,
            verbose=False,
            conf=self._confidence_threshold,
            classes=[self._person_class_id],
            device=self._device,
            imgsz=self._inference_image_size,
        )
        if not results or results[0].boxes is None:
            return []

        class_names = results[0].names
        detections = []
        for box in results[0].boxes:
            class_id = int(box.cls.item())
            if class_id != self._person_class_id:
                continue

            x1, y1, x2, y2 = (
                int(value) for value in box.xyxy[0].tolist()
            )
            detections.append(
                Detection(
                    x1=x1,
                    y1=y1,
                    x2=x2,
                    y2=y2,
                    class_id=class_id,
                    class_name=str(class_names[class_id]),
                    confidence=float(box.conf.item()),
                    track_id=(
                        int(box.id.item()) if box.id is not None else None
                    ),
                )
            )
        return detections
