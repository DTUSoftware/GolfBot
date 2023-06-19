import os
import logging
import sys
from threading import Event
import cv2
import torch
from ultralytics import YOLO

from ..Utils.opencv_helpers import draw_object
from .model_testing import save_result

# The webcam to use
VIDEO_INPUT = int(os.environ.get('VIDEO_INPUT', 1))
# The model to use
CURRENT_MODEL = os.environ.get("CURRENT_MODEL", "models/20231906-probbetter")
# If logging should be disabled
DISABLE_LOGGING = "true" in os.environ.get('DISABLE_LOGGING', "True").lower()
# If debugging should be enabled
DEBUG = ("true" in os.environ.get('DEBUG', "True").lower()) and not DISABLE_LOGGING

logger = logging.getLogger(__name__)
logger.addHandler(logging.StreamHandler(sys.stdout))
if DEBUG:
    logger.setLevel(logging.DEBUG)


# To be run as a thread
def run_ai(camera_queue: torch.multiprocessing.JoinableQueue, path_queue: torch.multiprocessing.JoinableQueue,
           ai_event: Event):
    """
    Run the AI
    :param camera_queue: the queue to send the results from the AI to
    :param path_queue: the queue to get the path from the robot from
    :param ai_event: the event to let the AI know that the robot has processed the results
    :return: None
    """
    if DISABLE_LOGGING:
        # THIS DISABLES LOGGING FOR YOLO
        logging.getLogger("ultralytics").setLevel(logging.WARNING)

    # Set device for AI processing
    torch_device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
    device = 0 if "cuda" in torch_device.type else "cpu"
    logger.debug(f"Using device: {device} ({torch_device.type}: index {torch_device.index})")

    # Load the model from the local .pt file
    logger.debug("Loading model...")
    global CURRENT_MODEL
    if not os.path.exists(CURRENT_MODEL + ".pt"):
        if os.path.exists("./ai/" + CURRENT_MODEL + ".pt"):
            CURRENT_MODEL = "./ai/" + CURRENT_MODEL
        else:
            if os.path.exists("./app/ai/" + CURRENT_MODEL + ".pt"):
                CURRENT_MODEL = "./app/ai/" + CURRENT_MODEL
            else:
                logger.debug("No model found!")
                return
    model = YOLO(CURRENT_MODEL + ".pt")

    # Open a connection to the webcam
    logger.debug("Opening video capture...")
    # Open the video stream from camera using DirectShow
    cap = cv2.VideoCapture(VIDEO_INPUT, cv2.CAP_DSHOW)

    # Set FPS to 60
    cap.set(cv2.CAP_PROP_FPS, 60)

    # Default width and height
    width = 1080.0
    height = 720.0

    # Set width and height of video stream
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    # Store current path for drawing
    current_objects = []

    while cap.isOpened():
        # Capture a frame from the webcam
        # if DEBUG:
        #     print("Reading frame...")
        success, frame = cap.read()

        if not success:
            logger.debug("Breaking!")
            break

        # Send the frame to the model for prediction
        # if DEBUG:
        #     print("Predict")
        results = model.predict(frame, device=device)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            save_result(results, frame)

        # Send the results to the driving algorithm
        # if DEBUG:
        #     print("Sending event to driving algorithm")

        # results.share_memory()

        if ai_event.is_set():
            if not camera_queue.full():
                # Put everything in shared memory
                shared_results = []
                for result in results:
                    shared_results.append(result.cpu())

                ai_event.clear()
                try:
                    camera_queue.put_nowait(shared_results)
                except Exception as e:
                    ai_event.set()  # set the flag back
                    logger.debug("Queue full, cannot store image")
                    logger.debug(str(e))
            else:
                logger.debug("Queue full, will not store elements.")
        else:
            logger.debug("Waiting for event to get set, will not store elements.")

        # Draw bounding boxes and labels on the image
        logger.debug("Drawing boxes...")
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

        logger.debug("Getting result from path queue")
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
        # print("Wait for processing of data...")
        # evt.wait()

        # await asyncio.sleep(0)

    # Release the webcam when done and close window
    logger.debug("Releasing!")
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
