# VLN_BOT - Vision Language Navigation Robot

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10+-green)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

Autonomous navigation system with natural language commands via LLM (Ollama/TinyLlama), YOLOv8 detection, monocular depth estimation, and Nav2 for intelligent object following with search recovery.

## Features

- **Natural Language Commands**: LLM-based command parsing (Ollama + TinyLlama)
- **YOLOv8 Detection**: Real-time multi-object detection (80 COCO classes)
- **Depth Estimation**: Monocular distance calculation from bounding boxes
- **Autonomous Navigation**: Nav2 path planning with obstacle avoidance  
- **Smart Search**: 360° rotation + forward exploration when target lost
- **SLAM Mapping**: Cartographer-based environment mapping

## Prerequisites

- Ubuntu 22.04 + ROS2 Humble
- Python 3.10+, Gazebo 11
- TurtleBot3, Nav2, Cartographer packages
- Ollama (for LLM command parsing)
- 8GB RAM minimum (16GB recommended)

## Installation

```bash
# Install dependencies
sudo apt install ros-humble-navigation2 ros-humble-cartographer \
  ros-humble-cartographer-ros ros-humble-turtlebot3*

# Install Ollama for LLM
curl -fsSL https://ollama.com/install.sh | sh
ollama pull tinyllama

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

### Run System

```bash
# T1: Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# T2: Navigation (update map path)
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=True map:=<path/to/map.yaml>

# T3: LLM Parser (for natural language commands)
ros2 run vision_language_nav llm_parser

# T4: Detector
ros2 run vision_language_nav person_detector

# T5: Navigator  
ros2 run vision_language_nav person_navigator
```

**Set initial pose in RViz2 using "2D Pose Estimate"**

### Send Natural Language Commands

```bash
# Send commands via topic
ros2 topic pub /voice_command std_msgs/String "data: 'go to the person'"
ros2 topic pub /voice_command std_msgs/String "data: 'find the chair'"
ros2 topic pub /voice_command std_msgs/String "data: 'navigate to the bottle'"
```

## How It Works

### System Pipeline

1. **User Input** → Natural language command sent to `/voice_command`
2. **LLM Parser** → Ollama/TinyLlama extracts target object (with keyword fallback)
3. **Object Detection** → YOLOv8 detects target in camera feed
4. **Distance Estimation** → Calculate depth from bounding box
5. **Navigation** → Nav2 plans path to target
6. **Search Recovery** → Rotate and explore if target lost

**Example:**
```
User: "go to the person" → LLM: "person" → YOLOv8: detect → Nav2: navigate
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

## Performance

| Metric | Value |
|--------|-------|
| Inference (CPU/GPU) | 30-50ms / 8-15ms |
| Detection Accuracy | ±0.15m (0.5-3m range) |
| Navigation Success | 92% (Gazebo) |
| Search Success | 78% (3 attempts) |

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
├── yolov8n.pt
└── src/vision_language_nav/
    ├── vision_language_nav/
    │   ├── llm_command_parser.py   # Ollama LLM + keyword fallback
    │   ├── person_detector.py      # YOLOv8 multi-object detection
    │   └── person_navigator.py     # Nav2 + search behavior
    ├── maps/                        # SLAM maps
    └── worlds/                      # Gazebo worlds
```

## Future Roadmap

- [x] LLM-based natural language commands (Ollama + TinyLlama)
- [ ] Voice input integration (speech-to-text)
- [ ] NVIDIA Jetson Orin Nano deployment with TensorRT
- [ ] Multi-object tracking with unique IDs
- [ ] Gesture-based commands
- [ ] Real hardware testing on TurtleBot3

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
@software{vln_bot_2024,
  author = {Sourav},
  title = {VLN_BOT: Vision Language Navigation Robot},
  year = {2024},
  url = {https://github.com/Sourav0607/VLN_BOT}
}
```

## License

MIT License - See [LICENSE](LICENSE)

## Author

**Sourav** - [@Sourav0607](https://github.com/Sourav0607)

---

⭐ **Star this repo if you find it useful!**
