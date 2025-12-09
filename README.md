# VLN_BOT - Vision Language Navigation Robot

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10+-green)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

Autonomous navigation system with natural language commands via LLM running on NVIDIA Jetson Orin Nano, YOLOv8 detection, monocular depth estimation, and Nav2 for intelligent object following with search recovery.

## Features

- **Natural Language Commands**: LLM-based command parsing (Ollama + TinyLlama on Jetson)
- **Distributed Architecture**: LLM on Jetson Orin Nano, Vision/Nav on main system
- **YOLOv8 Detection**: Real-time multi-object detection (80 COCO classes)
- **Depth Estimation**: Monocular distance calculation from bounding boxes
- **Autonomous Navigation**: Nav2 path planning with obstacle avoidance  
- **Smart Search**: 360° rotation + forward exploration when target lost
- **SLAM Mapping**: Cartographer-based environment mapping

## Prerequisites

**Main System (Desktop/Laptop):**
- Ubuntu 22.04 + ROS2 Humble
- Python 3.10+, Gazebo 11
- TurtleBot3, Nav2, Cartographer packages
- 8GB RAM minimum (16GB recommended)

**Edge Device (NVIDIA Jetson Orin Nano):**
- JetPack 5.0+ (Ubuntu 20.04/22.04)
- Ollama + TinyLlama model
- ROS2 Humble (for topic communication)

## Installation

### Main System Setup

```bash
# Install dependencies
sudo apt install ros-humble-navigation2 ros-humble-cartographer \
  ros-humble-cartographer-ros ros-humble-turtlebot3*

# Clone and setup
cd ~/ros2_ws/src
git clone https://github.com/Sourav0607/VLN_BOT.git
cd VLN_BOT/ros2_ws/src/vision_language_nav
pip install -r requirements.txt

# Download YOLOv8 model
cd ~/ros2_ws
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt

# Build
colcon build --packages-select vision_language_nav
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
```

### Jetson Orin Nano Setup (LLM Parser)

```bash
# Install Ollama
curl -fsSL https://ollama.com/install.sh | sh
ollama pull tinyllama

# Install ROS2 Humble (if not already installed)
# Follow: https://docs.ros.org/en/humble/Installation.html

# Clone repository
cd ~/ros2_ws/src
git clone https://github.com/Sourav0607/VLN_BOT.git
cd VLN_BOT/ros2_ws/src/vision_language_nav
pip install -r requirements.txt

# Build LLM parser node
cd ~/ros2_ws
colcon build --packages-select vision_language_nav
source install/setup.bash
```

**Network Setup:** Ensure both systems are on the same ROS2 domain and can communicate via DDS.

## Usage

### Create Map (One-time)

```bash
# T1: Launch Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# T2: Start SLAM
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

# T3: Teleop to explore
ros2 run turtlebot3_teleop teleop_keyboard

# T4: Save map
cd ~/ros2_ws/src/vision_language_nav/maps
ros2 run nav2_map_server map_saver_cli -f my_map
```

### Run System (Distributed)

**On Main System:**
```bash
# T1: Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# T2: Navigation (update map path)
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=True map:=<path/to/map.yaml>

# T3: Detector
ros2 run vision_language_nav person_detector

# T4: Navigator  
ros2 run vision_language_nav person_navigator
```

**On Jetson Orin Nano:**
```bash
# Run LLM Parser (runs Ollama/TinyLlama)
ros2 run vision_language_nav llm_parser
```

**Set initial pose in RViz2 using "2D Pose Estimate"**

### Send Natural Language Commands

**From Jetson (or any system on ROS2 network):**
```bash
# Send commands via topic
ros2 topic pub /voice_command std_msgs/String "data: 'go to the person'"
ros2 topic pub /voice_command std_msgs/String "data: 'find the chair'"
ros2 topic pub /voice_command std_msgs/String "data: 'navigate to the bottle'"
```

## How It Works

### Distributed System Architecture

```
┌─────────────────────┐         ┌──────────────────────┐
│  Jetson Orin Nano   │  ROS2   │    Main System       │
│                     │ Topics  │                      │
│  ┌───────────────┐  │────────▶│  ┌────────────────┐  │
│  │ LLM Parser    │  │         │  │ YOLOv8 Detector│  │
│  │ (TinyLlama)   │  │         │  │ Nav2 Navigator │  │
│  └───────────────┘  │         │  │ Gazebo/RViz    │  │
│  /voice_command     │         │  └────────────────┘  │
│  /target_object     │         │  /camera/image_raw   │
└─────────────────────┘         └──────────────────────┘
```

### System Pipeline

1. **User Input** → Natural language command sent to `/voice_command` (on Jetson)
2. **LLM Parser (Jetson)** → Ollama/TinyLlama extracts target object
3. **Target Published** → `/target_object` published across ROS2 network
4. **Object Detection (Main)** → YOLOv8 detects target in camera feed
5. **Distance Estimation** → Calculate depth from bounding box
6. **Navigation** → Nav2 plans path to target
7. **Search Recovery** → Rotate and explore if target lost

**Example:**
```
Jetson: "go to the person" → LLM: "person" → Main System: YOLOv8 detect → Nav2 navigate
```

### Distance Estimation

Monocular depth from bounding box height using pinhole camera model:

$$d = \frac{f \cdot H_{real}}{H_{pixel}} - c$$

- $f = 250$ px (focal length)
- $H_{real} = 1.7$ m (person height)
- $H_{pixel}$ = bbox height
- $c = 0.8$ m (calibration offset)

**Implementation:**
```python
estimated_depth = (1.7 * 250) / bbox_height - 0.8
```

### Navigation Logic

1. **Command** → User sends natural language command
2. **Parse** → LLM extracts target object from 80 COCO classes
3. **Detect** → YOLOv8 finds target, calculates distance
4. **Navigate** → If >0.35m away, send Nav2 goal to approach within 0.01m
5. **Search** → If lost >5s: rotate 360° + move forward 0.5m (max 3 cycles)

## Configuration

**Key Parameters** (`person_navigator.py`):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `safe_margin` | 0.01m | Stop distance from person |
| `min_navigation_distance` | 0.35m | Min distance to trigger nav |
| `person_lost_timeout` | 5.0s | Time before search starts |
| `max_search_attempts` | 3 | Number of search cycles |

**Detection** (`person_detector.py`):
- `confidence_threshold`: 0.5
- Distance formula: `(1.7 * 250) / bbox_height - 0.8`

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/voice_command` | `std_msgs/String` | Input: Natural language commands |
| `/target_object` | `std_msgs/String` | LLM output: Parsed object name |
| `/camera/image_raw` | `sensor_msgs/Image` | Camera input |
| `/camera/image_detected` | `sensor_msgs/Image` | Annotated output |
| `/target/distance` | `std_msgs/Float32` | Distance to target object |
| `/detected_object` | `std_msgs/String` | Currently detected object class |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | Nav2 goals |


## Troubleshooting

| Issue | Solution |
|-------|----------|
| Robot not moving | Set "2D Pose Estimate" in RViz2 |
| No detection | Check: `ros2 topic hz /camera/image_raw` |
| Wrong distance | Recalibrate focal length (250) & offset (0.8) |
| Search not working | Verify `/cmd_vel` output |

**Monitor:**
```bash
# View detection
ros2 run rqt_image_view rqt_image_view /camera/image_detected

# Check distance
ros2 topic echo /target/distance
```

## Project Structure

```
ros2_ws/
├── yolov8n.pt                       # YOLOv8 weights (main system)
└── src/vision_language_nav/
    ├── vision_language_nav/
    │   ├── llm_command_parser.py   # Runs on Jetson Orin Nano
    │   ├── person_detector.py      # Runs on main system
    │   └── person_navigator.py     # Runs on main system
    ├── maps/                        # SLAM maps
    └── worlds/                      # Gazebo worlds
```

## Hardware Architecture

**Deployment:**
- **Jetson Orin Nano**: LLM inference (CPU-only TinyLlama) + ROS2 publisher
- **Main System**: Gazebo simulation, YOLOv8 detection, Nav2 navigation
- **Communication**: ROS2 DDS over WiFi/Ethernet

## Limitations

- LLM inference on CPU (~2-10s per command with TinyLlama)
- Monocular depth accuracy degrades beyond 3m
- Depth estimation assumes upright objects
- Static maps only (no dynamic environments)
- Single target tracking (closest detected object)

## References

- **Ollama**: [Ollama LLM Runtime](https://ollama.com/)
- **TinyLlama**: [Lightweight LLM](https://github.com/jzhang38/TinyLlama)
- **YOLOv8**: [Ultralytics](https://github.com/ultralytics/ultralytics)
- **Nav2**: [Navigation2](https://navigation.ros.org/)
- **TurtleBot3**: [ROBOTIS](https://emanual.robotis.com/docs/en/platform/turtlebot3/)

## Contributing

Contributions welcome! Fork → Branch → Commit → Push → Pull Request

Areas for improvement:
- Kalman filtering for smoother estimates
- Multi-person tracking
- Adaptive search strategies
- Real hardware validation

## Citation

```bibtex
@software{vln_bot_2025,
  author = {Sourav},
  title = {VLN_BOT: Vision Language Navigation Robot},
  year = {2025},
  url = {https://github.com/Sourav0607/VLN_BOT}
}
```

## License

MIT License - See [LICENSE](LICENSE)

## Author

**Sourav** - [@Sourav0607](https://github.com/Sourav0607)

---

⭐ **Star this repo if you find it useful!**
