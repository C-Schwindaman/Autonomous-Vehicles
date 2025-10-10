from ultralytics import YOLO
import os

# Load a model
model = YOLO("yolo11n.pt")

yaml_file_path = os.path.expanduser('~/av/schwind7_av/lab7_yolo/traffic_sign_dataset/sign.yaml')   # This would ideally be ~/av/<student_repo>/lab7_yolo/traffic_sign_dataset/sign.yaml. os.path.expanduser() can read `~` and convert it to the absolute home path.

# Train the model
train_results = model.train(
    data=yaml_file_path,  # path to sign.yaml in your dataset
    epochs=400,           # number of training epochs
    imgsz=256,            # training image size
    device="cpu",  
)

