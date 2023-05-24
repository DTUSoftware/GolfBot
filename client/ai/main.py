import cv2
import config
from ultralytics import YOLO
from ultralytics.yolo.utils.plotting import Annotator
from threading import Event
from queue import Queue


# To be run as a thread
def run_ai(queue: Queue):
    # Load the model from the local .pt file
    model = YOLO(config.CURRENT_MODEL + ".pt")

    # Open a connection to the webcam
    cap = cv2.VideoCapture(1)

    while cap.isOpened():
        # Capture a frame from the webcam
        success, frame = cap.read()

        if not success:
            break

        # Send the frame to the model for prediction
        results = model.predict(frame)

        # Send the results to the driving algorithm
        evt = Event()
        queue.put((results, evt))

        # Draw bounding boxes and labels on the image
        # annotator = Annotator(frame)
        # for r in results:
        #     boxes = r.boxes
        #     for box in boxes:
        #         b = box.xyxy[0]  # get box coordinates in (top, left, bottom, right) format
        #         c = box.cls
        #         conf = box.conf.item()
        #         annotator.box_label(b, model.names[int(c)])
        #         print(f"Class: {model.names[int(c)]}, Confidence: {conf:.2f}, x: {b[0]}, y: {b[1]}")
        #     frame = annotator.result()
        frame = results[0].plot()

        cv2.imshow('Camera Feed', frame)

        # Wait for processing of data
        evt.wait()

        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break

    # Release the webcam when done and close window
    cap.release()
    cv2.destroyAllWindows()
