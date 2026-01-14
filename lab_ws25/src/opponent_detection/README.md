# F1Tenth Opponent Detection Package

Real-time opponent car detection using YOLO and ROS2.

**Note:** This package was previously named `perception`.

## 📦 Package Structure

```
opponent_detection/
├── opponent_detection/
│   ├── __init__.py
│   └── opponent_detector.py        # Main ROS2 detection node
├── config/
│   └── opponent_detector_params.yaml  # Required configuration file
├── models/
│   └── best.pt                     # YOLO trained model
├── launch/
│   └── opponent_detector.launch.py  # Launch file
├── package.xml                     # ROS2 package manifest
├── setup.py                        # Python package setup
└── README.md                       # This file
```

## 🚀 Quick Start

### 1. Build the Package

```bash
cd perception/
colcon build --packages-select perception
source install/setup.bash
```

### 2. Run the Node

**Option A: Using Launch File (Recommended)**
```bash
ros2 launch perception opponent_detector.launch.py
```

**Option B: Direct Run with Config**
```bash
ros2 run perception opponent_detector --ros-args --params-file config/opponent_detector_params.yaml
```

**Option C: Manual Python Run (Development Only)**
```bash
# Must be in perception directory
cd perception/
python3 perception/opponent_detector.py
```

## ⚙️ Configuration

**All parameters must be defined in `config/opponent_detector_params.yaml`**

The node has **NO default values** - if config is missing or incomplete, it will fail with a clear error message.

### Required Parameters:

```yaml
opponent_yolo_detector:
  ros__parameters:
    model_path: "models/best.pt"              # Path to YOLO model (relative or absolute)
    confidence_threshold: 0.0136              # Detection confidence (0.0-1.0)
    use_gpu: true                             # Use GPU acceleration
    camera_topic: "/camera/image_raw"         # Input camera topic
    detection_topic: "/opponent_detections"   # Output detection topic
    visualization_topic: "/opponent_detections/visualization"
    publish_visualization: true               # Publish annotated images
    log_frequency: 1.0                        # Log interval in seconds
```

## 📡 ROS2 Topics

### Subscribed Topics:
- **`/camera/image_raw`** (`sensor_msgs/Image`)
  - Camera input images

### Published Topics:
- **`/opponent_detections`** (`vision_msgs/Detection2DArray`)
  - Detection results with bounding boxes and confidence scores
  
- **`/opponent_detections/visualization`** (`sensor_msgs/Image`)
  - Annotated images with drawn detections (if enabled)

## 🧩 Dependencies

### ROS2 Packages:
- `rclpy`
- `sensor_msgs`
- `vision_msgs`
- `std_msgs`
- `cv_bridge`
- `ament_index_python`

### Python Packages:
- `opencv-python`
- `torch`
- `ultralytics` (YOLO)
- `numpy`

## 🏁 F1Tenth Deployment

### Steps to Deploy on F1Tenth Car:

1. **Copy perception package to F1Tenth:**
   ```bash
   scp -r perception/ f1tenth@car_ip:~/ros2_ws/src/
   ```

2. **SSH into F1Tenth and build:**
   ```bash
   ssh f1tenth@car_ip
   cd ~/ros2_ws
   colcon build --packages-select perception
   source install/setup.bash
   ```

3. **Verify model exists:**
   ```bash
   ls ~/ros2_ws/install/perception/share/perception/models/best.pt
   ```

4. **Run the node:**
   ```bash
   ros2 launch perception opponent_detector.launch.py
   ```

## 🔧 Troubleshooting

### Error: "Missing required parameter"
- **Cause**: Config file not loaded or parameter missing
- **Solution**: Always run with config file:
  ```bash
  ros2 run perception opponent_detector --ros-args --params-file config/opponent_detector_params.yaml
  ```

### Error: "Model file not found"
- **Cause**: Model file missing from `models/` directory
- **Solution**: Ensure `best.pt` exists in `perception/models/`

### Error: "Camera topic timeout"
- **Cause**: Camera node not publishing
- **Solution**: Start camera publisher first:
  ```bash
  python3 scripts/camera_publisher.py
  ```

## 📝 Development Notes

- **Config-driven**: All parameters come from YAML file (no hardcoded defaults)
- **Path resolution**: Model path can be relative (resolved from package share directory) or absolute
- **Error handling**: Clear error messages guide troubleshooting
- **Validation**: All parameters are type-checked and validated on startup
- **Portable**: Package is self-contained with model included

## 📄 License

MIT License
