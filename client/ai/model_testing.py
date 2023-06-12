import os
from datetime import datetime

import cv2
import torch
from PIL import Image
from ultralytics import YOLO

# Which webcam to use
VIDEO_INPUT = int(os.environ.get('VIDEO_INPUT', 1))
# The model to use
CURRENT_MODEL = os.environ.get("CURRENT_MODEL", "models/20231206-plztest")
# The directory to save the captured results to
DATA = os.environ.get("DATA", "datasets/RoboFlow0506-1")


def save_result(results, frame=None, missing=False):
    """
    Save the results from the model to the local filesystem
    :param results: the results from the model
    :param frame: the frame to save
    :param missing: true if we should save without annotations
    :return: None
    """
    print("Saving result...")
    if not os.path.exists(DATA + "/custom/images"):
        os.makedirs(DATA + "/custom/images", exist_ok=True)
    if not os.path.exists(DATA + "/custom/labels"):
        os.makedirs(DATA + "/custom/labels", exist_ok=True)
    if missing and not os.path.exists(DATA + "/custom/missing_images"):
        os.makedirs(DATA + "/custom/missing_images", exist_ok=True)

    for result in results:
        image_name: str = datetime.now().strftime("CustomImage-%Y-%m-%d_%H%M%S")

        if frame is None:
            image = Image.fromarray(result.orig_img)
            if not image:
                continue
            if missing:
                cv2.imwrite(f"{DATA}/custom/missing_images/{image_name}.jpg", frame)
            else:
                image.save(f"{DATA}/custom/images/{image_name}.jpg")
        else:
            if missing:
                cv2.imwrite(f"{DATA}/custom/missing_images/{image_name}.jpg", frame)
            else:
                cv2.imwrite(f"{DATA}/custom/images/{image_name}.jpg", frame)
        print(f"Saved image as {image_name}.jpg!")

        if missing:
            continue

        labels = []
        for box in result.boxes:
            # 1D Tensor
            arr = box.xywhn.cpu().numpy()
            labels.append(str(int(box.cls)) + " " + ' '.join(map(str, arr[0])))
        if not labels:
            continue
        with open(f"{DATA}/custom/labels/{image_name}.txt", "w+") as file:
            file.write("\n".join(labels))
            print(f"Saved image labels as {image_name}.txt!")


def test_model():
    """
    Test the model by running it on the webcam
    :return: None
    """
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

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            save_result(results, frame)
        elif key == ord('m'):
            save_result(results, frame, missing=True)

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

    # Release the webcam when done
    cap.release()
    # Destroy
    cv2.destroyAllWindows()


if __name__ == "__main__":
    test_model()
