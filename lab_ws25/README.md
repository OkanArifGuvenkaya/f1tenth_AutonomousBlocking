# F1Tenth Lab Workspace

ROS2 workspace for F1Tenth autonomous racing system with opponent detection and sensor integration.

## 📁 Workspace Structure

```
lab_ws25/
├── src/
│   ├── opponent_detection/    # YOLO-based opponent car detection (formerly 'perception')
│   └── sensors/               # Camera and sensor drivers (ZED)
├── scripts/
│   ├── camera_publisher.py           # Standalone camera test
│   ├── zed_realtime_detection.py     # Direct YOLO test
│   ├── view_detections.py            # Detection viewer
│   ├── frame_extractor_from_video.py # Video frame extraction
│   └── build_and_test.sh             # Build automation script
├── launch/
│   └── full_system.launch.py   # Launch all components together
├── build/                      # Colcon build outputs
├── install/                    # Installed packages
├── log/                        # Build and runtime logs
└── setup_workspace.sh          # Environment setup script
```

## 🚀 Quick Start

### 1. Setup Environment

```bash
cd ~/repositories/f1tenth_AutonomousBlocking/lab_ws25/
source setup_workspace.sh
```

### 2. Build Packages

```bash
# Build all packages
colcon build --packages-select sensors opponent_detection

# Or use the automated script
chmod +x scripts/build_and_test.sh
./scripts/build_and_test.sh

# Source the workspace
source install/setup.bash
```

### 3. Run the System

**Option A: Full System (Camera + Detection)**
```bash
ros2 launch full_system.launch.py
```

**Option B: Individual Components**
```bash
# Terminal 1: Camera
ros2 launch sensors camera.launch.py

# Terminal 2: Opponent Detection
ros2 launch opponent_detection opponent_detector.launch.py
```

## 📦 Packages

### opponent_detection
Real-time YOLO-based opponent car detection.
- **Previously named:** `perception`
- **Topics:** 
  - Subscribes: `/camera/image_raw`
  - Publishes: `/opponent_detections`, `/opponent_detections/visualization`
- **Config:** `src/opponent_detection/config/opponent_detector_params.yaml`

### sensors
ZED camera publisher and sensor drivers.
- **Topics:**
  - Publishes: `/camera/image_raw`
- **Config:** `src/sensors/config/camera_params.yaml`

## 🛠️ Development Workflow

### Building After Changes

```bash
# Clean build
rm -rf build/ install/ log/
colcon build --packages-select sensors opponent_detection
source install/setup.bash
```

### Running Scripts

```bash
# Make scripts executable
chmod +x scripts/*.sh

# Setup environment
source setup_workspace.sh

# Build and test
./scripts/build_and_test.sh
```

### Testing Individual Components

```bash
# Test camera only
ros2 launch sensors camera.launch.py

# Test detection only (requires camera or pre-recorded images)
ros2 launch opponent_detection opponent_detector.launch.py

# View topics
ros2 topic list
ros2 topic echo /opponent_detections
ros2 topic hz /camera/image_raw
```

## 📡 ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | sensor_msgs/Image | Raw camera feed from ZED |
| `/opponent_detections` | vision_msgs/Detection2DArray | Detected opponent cars |
| `/opponent_detections/visualization` | sensor_msgs/Image | Annotated detection images |

## 🔧 Configuration Files

### Camera Configuration
`src/sensors/config/camera_params.yaml`
- Camera index, resolution, frame rate
- Topic names and frame IDs

### Detection Configuration
`src/opponent_detection/config/opponent_detector_params.yaml`
- YOLO model path
- Confidence threshold
- GPU usage
- Topic names

## 🧪 Testing & Debugging

### Check Package Installation
```bash
ros2 pkg list | grep -E "sensors|opponent"
```

### View Images
```bash
# Install rqt_image_view if not available
sudo apt-get install ros-humble-rqt-image-view

# View camera feed
ros2 run rqt_image_view rqt_image_view /camera/image_raw

# View detections
ros2 run rqt_image_view rqt_image_view /opponent_detections/visualization
```

### Monitor Performance
```bash
# Check topic rates
ros2 topic hz /camera/image_raw
ros2 topic hz /opponent_detections

# Check node info
ros2 node info /camera_publisher
ros2 node info /opponent_detector
```

## 🐛 Troubleshooting

### Build Errors
```bash
# Clean workspace
rm -rf build/ install/ log/

# Source ROS2
source /opt/ros/humble/setup.bash

# Rebuild
colcon build --packages-select sensors opponent_detection
```

### Runtime Errors

**Camera not found:**
```bash
# List video devices
ls /dev/video*

# Check camera config
cat src/sensors/config/camera_params.yaml
```

**Model not found:**
```bash
# Verify model exists
ls -la src/opponent_detection/models/best.pt

# Copy model if missing
cp /path/to/best.pt src/opponent_detection/models/
```

**GPU issues:**
```bash
# Check CUDA
python3 -c "import torch; print(torch.cuda.is_available())"

# Set environment variable
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1
```

## 📋 Package Migration Notes

The `perception` package has been renamed to `opponent_detection`:
- ✅ Package names updated in all files
- ✅ Launch files corrected
- ✅ Import statements fixed
- ✅ Config files compatible
- ✅ No breaking changes for external dependencies

## 🔄 Dependencies

### System Requirements
- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+
- CUDA (optional, for GPU acceleration)

### Python Dependencies
```bash
pip install torch torchvision ultralytics opencv-python
```

### ROS2 Dependencies
```bash
sudo apt-get install \
  ros-humble-vision-msgs \
  ros-humble-cv-bridge \
  ros-humble-rqt-image-view
```

## 📝 Environment Variables

Set these in your shell or add to `~/.bashrc`:

```bash
# ROS2 Humble
source /opt/ros/humble/setup.bash

# Workspace
source ~/repositories/f1tenth_AutonomousBlocking/lab_ws25/install/setup.bash

# PyTorch ARM64 compatibility
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1
```

## 📄 License

MIT License

## 👥 Contributors

F1Tenth Team - Autonomous Racing Project
