import cv2
import os
import torch
from datetime import datetime
from PIL import Image
from ultralytics import YOLO
from ultralytics.yolo.utils.plotting import Annotator

VIDEO_INPUT = int(os.environ.get('VIDEO_INPUT', 0))
CURRENT_MODEL = os.environ.get("CURRENT_MODEL", "models/20230605_nico1")
DATA = os.environ.get("DATA", "datasets/RoboFlow0506-1")


def save_result(results):
    print("Saving result...")
    if not os.path.exists(DATA + "/custom/images"):
        os.makedirs(DATA + "/custom/images", exist_ok=True)
    if not os.path.exists(DATA + "/custom/labels"):
        os.makedirs(DATA + "/custom/labels", exist_ok=True)

    for result in results:
        image_name: str = datetime.now().strftime("CustomImage-%Y-%m-%d_%H%M%S")
        image = Image.fromarray(result.orig_img)
        if not image:
            continue
        image.save(f"{DATA}/custom/images/{image_name}.jpg")
        print(f"Saved image as {image_name}.jpg!")

        labels = []
        for box in result.boxes:
            labels.append(box.cls + " " + " ".join(box.xywhn) + "\n")
        if not labels:
            continue
        with open(f"{DATA}/custom/labels/{image_name}.txt", "w+") as file:
            file.write("\n".join(labels))
            print(f"Saved image labels as {image_name}.txt!")


def test_model():
    # Set device for AI processing
    torch_device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
    device = 0 if "cuda" in torch_device.type else "cpu"
    print(f"Using device: {device} ({torch_device.type}: index {torch_device.index})")

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

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            save_result(results)

    # Release the webcam when done
    cap.release()
    # Destroy
    cv2.destroyAllWindows()


if __name__ == "__main__":
    test_model()
