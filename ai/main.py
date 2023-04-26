from ultralytics import YOLO
import config

## Lots of example code and documentation from 
## https://github.com/ultralytics/ultralytics

# Load a model
## pretrained models (small/fastest -> largest/slowest):
## YOLOv8n	YOLOv8s	YOLOv8m	YOLOv8l	YOLOv8x
model = YOLO(config.PRETRAINED_MODEL)  # load a pretrained model (recommended for training)

# Use the model
model.train(data=config.DATA, epochs=config.EPOCHS)  # train the model
# metrics = model.val()  # evaluate model performance on the validation set
success = model.export(format="onnx")  # export the model to ONNX format
success = model.export(format="pt")  # export the model to pt format too
