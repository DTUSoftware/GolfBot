import os
import sys
import cv2
from ultralytics import YOLO
import torch
import multiprocessing
import queue

VIDEO_INPUT = int(os.environ.get('VIDEO_INPUT', 1))
CURRENT_MODEL = os.environ.get("CURRENT_MODEL", "models/20230601_2")
DISABLE_LOGGING = "true" in os.environ.get('DISABLE_LOGGING', "True").lower()
DEBUG = ("true" in os.environ.get('DEBUG', "True").lower()) and not DISABLE_LOGGING


# To be run as a thread
def run_ai(cameraQueue: multiprocessing.JoinableQueue):
    if DISABLE_LOGGING:
        # THIS DISABLES LOGGING
        if sys.platform == "win32":
            sys.stdout = open('nul', 'w')
            # sys.stderr = open('nul', 'w')
        else:
            sys.stdout = open('/dev/null', 'w')
            # sys.stderr = open('/dev/null', 'w')

    # Set device for AI processing
    torch_device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
    device = 0 if "cuda" in torch_device.type else "cpu"
    if DEBUG:
        print(f"[AI] Using device: {device} ({torch_device.type}: index {torch_device.index})")

    # Load the model from the local .pt file
    if DEBUG:
        print("[AI] Loading model...")
    global CURRENT_MODEL
    if not os.path.exists(CURRENT_MODEL + ".pt"):
        if os.path.exists("./ai/" + CURRENT_MODEL + ".pt"):
            CURRENT_MODEL = "./ai/" + CURRENT_MODEL
        else:
            if DEBUG:
                print("[AI] No model found!")
            return
    model = YOLO(CURRENT_MODEL + ".pt")

    # Open a connection to the webcam
    if DEBUG and not DISABLE_LOGGING:
        print("[AI] Opening video capture...")
    cap = cv2.VideoCapture(VIDEO_INPUT)

    while cap.isOpened():
        # Capture a frame from the webcam
        # if DEBUG:
        #     print("[AI] Reading frame...")
        success, frame = cap.read()

        if not success:
            if DEBUG:
                print("[AI] Breaking!")
            break

        # Send the frame to the model for prediction
        # if DEBUG:
        #     print("[AI] Predict")
        results = model.predict(frame, device=device)

        # Send the results to the driving algorithm
        # if DEBUG:
        #     print("[AI] Sending event to driving algorithm")
        try:
            cameraQueue.put_nowait(results)
        except queue.Full:
            if DEBUG:
                print("[AI] Queue full, cannot store image")

        # Draw bounding boxes and labels on the image
        if DEBUG:
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
        # await asyncio.sleep(0)

    # Release the webcam when done and close window
    if DEBUG:
        print("[AI] Releasing!")
    cap.release()
    cv2.destroyAllWindows()


# if __name__ == "__main__":
#     asyncio.run(run_ai(asyncio.Queue(maxsize=1)))
