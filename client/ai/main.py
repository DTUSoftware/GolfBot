import os
import cv2
from ultralytics import YOLO
# from ultralytics.yolo.utils.plotting import Annotator
# import multiprocessing
import queue

VIDEO_INPUT = int(os.environ.get('VIDEO_INPUT', 1))
CURRENT_MODEL = os.environ.get("CURRENT_MODEL", "models/26041010")
PRETRAINED_MODEL = os.environ.get("PRETRAINED_MODEL", "yolov8n.pt")
DATA = os.environ.get("DATA", "datasets/RoboFlow1904/data.yaml")
EPOCHS = int(os.environ.get("EPOCHS", 3))
IMGSZ = int(os.environ.get("IMGSZ", 640))  # needs to be a multiple of 32


# To be run as a thread
def run_ai(queue: queue.Queue):
    # Load the model from the local .pt file
    print("[AI] Loading model...")
    global CURRENT_MODEL
    if not os.path.exists(CURRENT_MODEL + ".pt"):
        if os.path.exists("./ai/" + CURRENT_MODEL + ".pt"):
            CURRENT_MODEL = "./ai/" + CURRENT_MODEL
        else:
            print("[AI] No model found!")
            return
    model = YOLO(CURRENT_MODEL + ".pt")

    # Open a connection to the webcam
    print("[AI] Opening video capture...")
    cap = cv2.VideoCapture(VIDEO_INPUT)

    while cap.isOpened():
        # Capture a frame from the webcam
        print("[AI] Reading frame...")
        success, frame = cap.read()

        if not success:
            print("[AI] Breaking!")
            break

        # Send the frame to the model for prediction
        print("[AI] Predict")
        results = model.predict(frame)

        # Send the results to the driving algorithm
        print("[AI] Sending event to driving algorithm")
        # evt = multiprocessing.Event()
        # queue.put((results, evt))
        queue.put(results)

        # Draw bounding boxes and labels on the image
        print("[AI] Drawing boxes...")
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
        # print("[AI] Wait for processing of data...")
        # evt.wait()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the webcam when done and close window
    print("[AI] Releasing!")
    cap.release()
    cv2.destroyAllWindows()
