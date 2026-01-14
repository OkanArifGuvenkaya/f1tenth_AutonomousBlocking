import cv2
import os
from pathlib import Path


def extract_frames(video_path, output_dir, frame_interval=30):
    """
    Extract frames from a video at specified intervals and save them.
    
    Args:
        video_path (str): Path to the video file
        output_dir (str): Directory where frames will be saved
        frame_interval (int): Extract every nth frame (default: 30)
    """
    # Create output directory
    Path(output_dir).mkdir(parents=True, exist_ok=True)
    
    # Open video file
    video = cv2.VideoCapture(video_path)
    
    if not video.isOpened():
        print(f"Error: Could not open video file: {video_path}")
        return
    
    # Get video information
    fps = video.get(cv2.CAP_PROP_FPS)
    total_frames = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
    duration = total_frames / fps if fps > 0 else 0
    
    print(f"Video information:")
    print(f"  FPS: {fps}")
    print(f"  Total frames: {total_frames}")
    print(f"  Duration: {duration:.2f} seconds")
    print(f"  Frame interval: {frame_interval}")
    print(f"  Frames to extract: {total_frames // frame_interval}")
    
    frame_count = 0
    saved_count = 0
    
    while True:
        ret, frame = video.read()
        
        if not ret:
            break
        
        # Save frames at specified intervals
        if frame_count % frame_interval == 0:
            frame_filename = os.path.join(output_dir, f"frame_{saved_count:05d}.jpg")
            cv2.imwrite(frame_filename, frame)
            saved_count += 1
            
            if saved_count % 10 == 0:
                print(f"  {saved_count} frames saved...")
        
        frame_count += 1
    
    video.release()
    print(f"\nCompleted! Total {saved_count} frames saved.")
    print(f"Output directory: {output_dir}")


if __name__ == "__main__":
    # Video and output directory paths
    video_name = "Video_17.12.2026.mp4"
    video_path = f"/home/okanarif/repositories/f1tenth_final_project/data/raw_video_data/{video_name}"
    output_dir = f"/home/okanarif/repositories/f1tenth_final_project/data/frames/{video_name}"    
    
    # Extract frames (every 30th frame)
    # You can change frame_interval to extract more or fewer frames
    extract_frames(video_path, output_dir, frame_interval=30)
