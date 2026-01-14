# F1Tenth Sensors Package

Sensor drivers for F1Tenth autonomous racing platform. Currently includes ZED camera publisher.

## 📦 Package Structure

```
sensors/
├── sensors/
│   ├── __init__.py
│   └── camera_publisher.py        # ZED camera ROS2 node
├── config/
│   └── camera_params.yaml         # Camera configuration
├── launch/
│   └── camera.launch.py           # Camera launch file
├── package.xml                    # ROS2 package manifest
├── setup.py                       # Python package setup
└── README.md                      # This file
```

## 🚀 Quick Start

### 1. Build the Package

```bash
cd sensors/
colcon build --packages-select sensors
source install/setup.bash
```

### 2. Run the Camera Publisher

**Option A: Using Launch File (Recommended)**
```bash
ros2 launch sensors camera.launch.py
```

**Option B: Direct Run with Config**
```bash
ros2 run sensors camera_publisher --ros-args --params-file config/camera_params.yaml
```

**Option C: Manual Python Run (Development Only)**
```bash
cd sensors/
python3 sensors/camera_publisher.py --ros-args --params-file config/camera_params.yaml
```

## ⚙️ Configuration

**All parameters must be defined in `config/camera_params.yaml`**

The node has **NO default values** - if config is missing or incomplete, it will fail with a clear error message.

### Required Parameters:

```yaml
camera_publisher:
  ros__parameters:
    camera_topic: "/camera/image_raw"  # Output topic
    frame_id: "camera"                 # TF frame ID
    publish_rate: 30.0                 # Publishing frequency (Hz)
    image_width: 1280                  # Image width
    image_height: 720                  # Image height
    camera_index_start: 0              # Start camera search index
    camera_index_end: 4                # End camera search index
    min_camera_width: 1280             # Minimum width for ZED detection
```

## 📡 ROS2 Topics

### Published Topics:
- **`/camera/image_raw`** (`sensor_msgs/Image`)
  - Camera images at configured resolution and rate
  - Frame ID: `camera` (configurable)

## 🧩 Dependencies

### ROS2 Packages:
- `rclpy`
- `sensor_msgs`
- `cv_bridge`
- `std_msgs`
- `ament_index_python`

### Python Packages:
- `opencv-python`

## 🔧 Camera Detection

The node automatically searches for ZED camera:

1. **Scans camera indices** from `camera_index_start` to `camera_index_end`
2. **Checks resolution** - cameras with width >= `min_camera_width` are considered ZED
3. **Opens first matching camera**
4. **Sets desired resolution** from config

### Troubleshooting Camera Detection

If camera is not found:

1. **Check connection**: `ls /dev/video*`
2. **Test manually**: `v4l2-ctl --list-devices`
3. **Verify permissions**: Add user to `video` group
   ```bash
   sudo usermod -a -G video $USER
   ```
4. **Adjust search range** in config if camera is at different index

## 🏁 F1Tenth Deployment

### Steps to Deploy on F1Tenth Car:

1. **Copy sensors package to F1Tenth:**
   ```bash
   scp -r sensors/ f1tenth@car_ip:~/ros2_ws/src/
   ```

2. **SSH into F1Tenth and build:**
   ```bash
   ssh f1tenth@car_ip
   cd ~/ros2_ws
   colcon build --packages-select sensors
   source install/setup.bash
   ```

3. **Run the camera:**
   ```bash
   ros2 launch sensors camera.launch.py
   ```

## 🧪 Testing

### Test camera output:
```bash
# Terminal 1: Start camera
ros2 launch sensors camera.launch.py

# Terminal 2: View images
ros2 run rqt_image_view rqt_image_view
# OR
ros2 topic echo /camera/image_raw
```

### Check camera info:
```bash
ros2 topic info /camera/image_raw
ros2 topic hz /camera/image_raw
```

## 🔍 Error Messages

### Error: "Missing required parameter"
- **Cause**: Config file not loaded or parameter missing
- **Solution**: Always run with config file:
  ```bash
  ros2 run sensors camera_publisher --ros-args --params-file config/camera_params.yaml
  ```

### Error: "Camera not found"
- **Cause**: ZED camera not detected
- **Solution**: 
  1. Check physical connection
  2. Verify camera index range in config
  3. Check camera permissions

### Warning: "Failed to capture frame"
- **Cause**: Camera disconnected or busy
- **Solution**: Restart node or check camera status

## 📝 Development Notes

- **Config-driven**: All parameters come from YAML file (no hardcoded defaults)
- **Auto-detection**: Automatically finds ZED camera by resolution
- **Error handling**: Clear error messages guide troubleshooting
- **Validation**: All parameters are type-checked and validated on startup
- **Portable**: Package is self-contained and follows ROS2 best practices

## 🔗 Integration

This package is designed to work with:
- **perception**: Opponent detection using YOLO
- **f1tenth_bringup**: Full system launch (future)

Example integration:
```python
# In your launch file
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription('sensors/launch/camera.launch.py'),
        IncludeLaunchDescription('perception/launch/opponent_detector.launch.py'),
    ])
```

## 📄 License

MIT License
