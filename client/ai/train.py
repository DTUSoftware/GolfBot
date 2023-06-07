import os

import torch
from ultralytics import YOLO

PRETRAINED_MODEL = os.environ.get("PRETRAINED_MODEL", "yolov8n.pt")
DATA = os.environ.get("DATA", "datasets/RoboFlow0606")
EPOCHS = int(os.environ.get("EPOCHS", 300))
IMGSZ = int(os.environ.get("IMGSZ", 640))  # needs to be a multiple of 32


def train():
    # Set device for AI processing
    torch_device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
    device = 0 if "cuda" in torch_device.type else "cpu"
    print(f"Using device: {device} ({torch_device.type}: index {torch_device.index})")

    ## Lots of example code and documentation from
    ## https://github.com/ultralytics/ultralytics

    # Load a model
    ## pretrained models (small/fastest -> largest/slowest):
    ## YOLOv8n	YOLOv8s	YOLOv8m	YOLOv8l	YOLOv8x
    model = YOLO(PRETRAINED_MODEL)  # load a pretrained model (recommended for training)

    # Use the model
    model.train(data=DATA + "/data.yaml", epochs=EPOCHS, device=device)  # train the model
    # metrics = model.val()  # evaluate model performance on the validation set
    success = model.export(format="onnx")  # export the model to ONNX format
    # success = model.export(format="pt")  # export the model to pt format too


if __name__ == "__main__":
    train()
