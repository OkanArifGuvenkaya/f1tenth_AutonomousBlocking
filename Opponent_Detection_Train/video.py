from ultralytics import YOLO
import torch

def predict_video():
    # GPU check
    device = 0 if torch.cuda.is_available() else 'cpu'
    print(f"🎮 Device in use: {'GPU - ' + torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU'}")
    
    # Load trained model (mini_test_gpu for GPU, mini_test for CPU)
    model_path = "runs/detect/mini_test_gpu/weights/best.pt" if torch.cuda.is_available() else "runs/detect/mini_test/weights/best.pt"
    model = YOLO(model_path)
    
    # Predict on video
    print("🎬 Starting video processing...")
    results = model.predict(
        source="data/raw_video_data/Video_17.12.2026.mp4",
        conf=0.018,           # Confidence threshold (reasonable value)
        save=True,           # Save results
        save_conf=True,      # Show confidence scores
        show=False,          # Live display (may cause issues on Linux)
        device=device,       # GPU or CPU
        stream=True,         # Stream mode to prevent RAM overflow
        name="video_pred_gpu" if torch.cuda.is_available() else "video_pred_cpu"
    )
    
    # Consume the generator to process all frames
    frame_count = 0
    for r in results:
        frame_count += 1
        if frame_count % 500 == 0:  # Progress update every 500 frames
            print(f"   Processed {frame_count} frames...")
    
    print(f"✅ Video processing completed! Total frames: {frame_count}")

if __name__ == "__main__":
    predict_video()