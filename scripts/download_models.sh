#!/bin/bash

echo "Downloading ML models for security detection..."

# Create models directories
mkdir -p models/yolo
mkdir -p models/pointpillars

# Download YOLO models
echo "Downloading YOLO models..."
cd models/yolo

# Download YOLOv8 models
wget -O yolov8n.pt https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.pt
wget -O yolov8s.pt https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8s.pt
wget -O yolov8m.pt https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8m.pt

echo "YOLO models downloaded successfully!"

# Note: PointPillars models would need to be trained or downloaded from specific sources
echo "PointPillars models need to be trained or obtained from specific sources"
echo "For now, 3D detection will use simulation"

cd ../..
echo "Model download complete!"
