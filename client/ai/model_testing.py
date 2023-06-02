import cv2
import os
import torch
from ultralytics import YOLO
from ultralytics.yolo.utils.plotting import Annotator

VIDEO_INPUT = int(os.environ.get('VIDEO_INPUT', 1))
CURRENT_MODEL = os.environ.get("CURRENT_MODEL", "models/20230601")
PRETRAINED_MODEL = os.environ.get("PRETRAINED_MODEL", "yolov8n.pt")
DATA = os.environ.get("DATA", "datasets/RoboFlow1904/data.yaml")
EPOCHS = int(os.environ.get("EPOCHS", 3))
IMGSZ = int(os.environ.get("IMGSZ", 640))  # needs to be a multiple of 32

# Set device for AI processing
device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
print(f"Using device: {device.type} (index {device.index})")

# Load the model from the local .pt file
model = YOLO(CURRENT_MODEL + ".pt")

# Open a connection to the webcam
cap = cv2.VideoCapture(VIDEO_INPUT)

while True:
    # Capture a frame from the webcam
    ret, frame = cap.read()

    # Send the frame to the model for prediction
    results = model.predict(frame, device=device)

    # Draw bounding boxes and labels on the image
    annotator = Annotator(frame)
    for r in results:
        boxes = r.boxes
        for box in boxes:
            b = box.xyxy[0]  # get box coordinates in (top, left, bottom, right) format
            c = box.cls
            conf = box.conf.item()
            annotator.box_label(b, model.names[int(c)])
            print(f"Class: {model.names[int(c)]}, Confidence: {conf:.2f}, x: {b[0]}, y: {b[1]}")
        frame = annotator.result()

    cv2.imshow('Camera Feed', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam when done
cap.release()
# Destroy
cv2.destroyAllWindows()
