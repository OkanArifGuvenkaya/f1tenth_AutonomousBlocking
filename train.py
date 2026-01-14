from ultralytics import YOLO
import torch

def train():
    # GPU check
    if torch.cuda.is_available():
        device = 0  # Use first GPU
        print(f"✅ Using GPU: {torch.cuda.get_device_name(0)}")
        batch = 16  # Larger batch size for GPU
        epochs = 50  # More epochs for GPU
    else:
        device = 'cpu'
        print("⚠️ GPU not found, using CPU")
        batch = 4
        epochs = 10
    
    model = YOLO("yolov8n.pt")  # yolov8n.pt is the model to train
    model.train(
        data="dataset/data.yaml",  # name of the dataset
        epochs=epochs,  # number of epochs (GPU: 50, CPU: 10)
        imgsz=640,  # image size
        batch=batch,  # batch size (GPU: 16, CPU: 4)
        device=device,  # GPU or CPU
        name="mini_test_gpu"  # name of the model folder
    )

if __name__ == "__main__":
    train()
