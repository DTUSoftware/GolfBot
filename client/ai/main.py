import logging
import os
import cv2
import torch
from ultralytics import YOLO
from client.Utils.opencv_helpers import draw_object

VIDEO_INPUT = int(os.environ.get('VIDEO_INPUT', 1))
CURRENT_MODEL = os.environ.get("CURRENT_MODEL", "models/20230601_2")
DISABLE_LOGGING = "true" in os.environ.get('DISABLE_LOGGING', "True").lower()
DEBUG = ("true" in os.environ.get('DEBUG', "True").lower()) and not DISABLE_LOGGING


# To be run as a thread
def run_ai(camera_queue: torch.multiprocessing.JoinableQueue, path_queue: torch.multiprocessing.JoinableQueue):
    if DISABLE_LOGGING:
        # THIS DISABLES LOGGING FOR YOLO
        logging.getLogger("ultralytics").setLevel(logging.WARNING)

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

    # Store current path for drawing
    current_objects = []

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

        # results.share_memory()

        if not camera_queue.full():
            # Put everything in shared memory
            shared_results = []
            for result in results:
                shared_results.append(result.cpu())

            try:
                camera_queue.put_nowait(shared_results)
            except Exception as e:
                if DEBUG:
                    print("[AI] Queue full, cannot store image")
                    print(str(e))
        else:
            if DEBUG:
                print("Queue full, will not store elements.")

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

        if DEBUG:
            print("[AI] Getting result from path queue")
        if not path_queue.empty():
            try:
                main_objects: dict = path_queue.get_nowait()
                current_objects = [{"type": "path", "path": main_objects["path"]},
                                   {"type": "small_goal", "path": main_objects["small_goal"]},
                                   {"type": "big_goal", "path": main_objects["big_goal"]}]
                for _obj in main_objects["obstacles"]:
                    current_objects.append({"type": "obstacle", "path": _obj})

                path_queue.task_done()
            except:
                pass

        # Draw fetched objects
        if current_objects:
            for _obj in current_objects:
                draw_object(frame, _obj["type"], _obj["path"])

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


if __name__ == "__main__":
    torch.multiprocessing.set_start_method('spawn', force=True)
    # manager = torch.multiprocessing.Manager()
    queue_1 = torch.multiprocessing.JoinableQueue(maxsize=1)
    queue_2 = torch.multiprocessing.JoinableQueue(maxsize=1)
    process = torch.multiprocessing.Process(target=run_ai, args=(queue_1, queue_2))
    process.start()
    process.join()
    # run_ai(queue)
