#!/usr/bin/env python3
import sys

from ultralytics import YOLO


model = YOLO("/home/skyler/models/yolov8n.pt")
image_size = int(sys.argv[2]) if len(sys.argv) > 2 else 640
result = model.predict(
    sys.argv[1], classes=[0], conf=0.01, imgsz=image_size, verbose=False
)[0]
for confidence, box in zip(result.boxes.conf, result.boxes.xyxy):
    print(float(confidence), [round(float(value), 1) for value in box])
