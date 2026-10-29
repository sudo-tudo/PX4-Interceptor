#!/usr/bin/env python3.11
from ultralytics import YOLO

model = YOLO("yolo11n.pt")
# Export to ONNX with 'simplify' to clean up the graph for mobile/edge CPUs
model.export(format="onnx", simplify=True)
