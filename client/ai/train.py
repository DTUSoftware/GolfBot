from ultralytics import YOLO
import os

VIDEO_INPUT = int(os.environ.get('VIDEO_INPUT', 1))
CURRENT_MODEL = os.environ.get("CURRENT_MODEL", "models/26041010")
PRETRAINED_MODEL = os.environ.get("PRETRAINED_MODEL", "yolov8n.pt")
DATA = os.environ.get("DATA", "datasets/RoboFlow1904/data.yaml")
EPOCHS = int(os.environ.get("EPOCHS", 3))
IMGSZ = int(os.environ.get("IMGSZ", 640))  # needs to be a multiple of 32

## Lots of example code and documentation from 
## https://github.com/ultralytics/ultralytics

# Load a model
## pretrained models (small/fastest -> largest/slowest):
## YOLOv8n	YOLOv8s	YOLOv8m	YOLOv8l	YOLOv8x
model = YOLO(PRETRAINED_MODEL)  # load a pretrained model (recommended for training)

# Use the model
model.train(data=DATA, epochs=EPOCHS)  # train the model
# metrics = model.val()  # evaluate model performance on the validation set
success = model.export(format="onnx")  # export the model to ONNX format
success = model.export(format="pt")  # export the model to pt format too
