from ultralytics import YOLO
import torch

def main():
    # GPU check
    device = 0 if torch.cuda.is_available() else 'cpu'
    print(f"🎮 Device in use: {'GPU - ' + torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU'}")
    
    # Load trained model (mini_test_gpu for GPU, mini_test for CPU)
    model_path = "runs/detect/mini_test_gpu/weights/best.pt" if torch.cuda.is_available() else "runs/detect/mini_test/weights/best.pt"
    model = YOLO(model_path)

    # 1) Test on train images
    model.predict(
        source="dataset/images/train",
        conf=0.018,  # More reasonable confidence threshold
        save=True,
        save_conf=True,
        device=device,
        name="train_pred_gpu" if torch.cuda.is_available() else "train_pred_cpu"
    )

    # 2) Test on validation images
    model.predict(
        source="dataset/images/val",
        conf=0.018,  # More reasonable confidence threshold
        save=True,
        save_conf=True,
        device=device,
        name="val_pred_gpu" if torch.cuda.is_available() else "val_pred_cpu"
    )

if __name__ == "__main__":
    main()
