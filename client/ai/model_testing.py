import cv2
import config
from ultralytics import YOLO
from ultralytics.yolo.utils.plotting import Annotator

# Load the model from the local .pt file
model = YOLO(config.CURRENT_MODEL + ".pt")

# Open a connection to the webcam
cap = cv2.VideoCapture(1)

while True:
    # Capture a frame from the webcam
    ret, frame = cap.read()

    # Send the frame to the model for prediction
    results = model.predict(frame)

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
