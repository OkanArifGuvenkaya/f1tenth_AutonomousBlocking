#!/usr/bin/env python3
"""
Real-time ZED Camera Detection Script
Captures video from ZED camera and runs YOLO detection
"""

import cv2
import numpy as np
from ultralytics import YOLO
import sys


def main():
    # Configuration
    MODEL_PATH = "/home/okanarif/repositories/f1tenth_final_project/runs/detect/mini_test_gpu/weights/best.pt"
    CONFIDENCE_THRESHOLD = 0.0136
    
    print("=" * 60)
    print("ZED Real-Time Detection")
    print("=" * 60)
    
    # Load YOLO model
    print(f"Loading YOLO model from: {MODEL_PATH}")
    try:
        model = YOLO(MODEL_PATH)
        print("✅ Model loaded successfully!")
    except Exception as e:
        print(f"❌ Failed to load model: {e}")
        print("Please update MODEL_PATH in the script")
        return
    
    # Open ZED camera
    # Try different camera indices to find ZED
    print("\nSearching for ZED camera...")
    cap = None
    
    for camera_index in range(5):  # Try indices 0-4
        print(f"Trying camera index {camera_index}...")
        test_cap = cv2.VideoCapture(camera_index)
        
        if test_cap.isOpened():
            # Read a test frame to check resolution
            ret, frame = test_cap.read()
            if ret:
                height, width = frame.shape[:2]
                print(f"  Camera {camera_index}: {width}x{height}")
                
                # ZED cameras typically have higher resolution (1280x720 or higher)
                if width >= 1280:
                    print(f"✅ Found ZED camera at index {camera_index}")
                    cap = test_cap
                    break
                else:
                    test_cap.release()
            else:
                test_cap.release()
    
    if cap is None or not cap.isOpened():
        print("❌ Failed to find ZED camera")
        print("Make sure ZED camera is connected")
        print("\nManually select camera index:")
        try:
            idx = int(input("Enter camera index (0-4): "))
            cap = cv2.VideoCapture(idx)
            if not cap.isOpened():
                print("❌ Failed to open selected camera")
                return
        except:
            return
    
    # Set camera resolution (ZED default: 1280x720)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    print("✅ Camera opened successfully!")
    print("\nPress 'q' to quit")
    print("=" * 60)
    
    frame_count = 0
    total_detections = 0
    
    while True:
        # Capture frame
        ret, frame = cap.read()
        
        if not ret:
            print("❌ Failed to grab frame")
            break
        
        # Run YOLO detection
        results = model.predict(
            frame,
            conf=CONFIDENCE_THRESHOLD,
            verbose=False
        )[0]
        
        # Draw detections on frame
        annotated_frame = results.plot()
        
        # Count detections and extract bounding boxes
        num_detections = 0
        detections_info = []
        
        if results.boxes is not None:
            num_detections = len(results.boxes)
            total_detections += num_detections
            
            # Extract bounding box information
            for i, box in enumerate(results.boxes):
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                confidence = float(box.conf[0])
                width = x2 - x1
                height = y2 - y1
                center_x = (x1 + x2) / 2.0
                center_y = (y1 + y2) / 2.0
                
                detections_info.append({
                    'id': i + 1,
                    'bbox': (int(x1), int(y1), int(x2), int(y2)),
                    'center': (int(center_x), int(center_y)),
                    'size': (int(width), int(height)),
                    'confidence': confidence
                })
        
        # Display info on frame
        info_text = f"Frame: {frame_count} | Detections: {num_detections} | Total: {total_detections}"
        cv2.putText(
            annotated_frame, 
            info_text, 
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )
        
        # Show frame
        cv2.imshow("ZED Real-Time Detection", annotated_frame)
        
        # Log detections with bounding box info
        if num_detections > 0:
            print(f"\nFrame {frame_count}: Detected {num_detections} opponent car(s)")
            for det in detections_info:
                print(f"  Detection {det['id']}:")
                print(f"    Bounding Box: [{det['bbox'][0]}, {det['bbox'][1]}, {det['bbox'][2]}, {det['bbox'][3]}]")
                print(f"    Center: ({det['center'][0]}, {det['center'][1]})")
                print(f"    Size: {det['size'][0]}x{det['size'][1]} px")
                print(f"    Confidence: {det['confidence']:.3f}")
        
        frame_count += 1
        
        # Check for quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("\nQuitting...")
            break
    
    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    
    print("\n" + "=" * 60)
    print(f"Session Summary:")
    print(f"  Total frames: {frame_count}")
    print(f"  Total detections: {total_detections}")
    print("=" * 60)


if __name__ == "__main__":
    main()
