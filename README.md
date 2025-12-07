# VLN_BOT - Vision Language Navigation Robot

A ROS2 Humble-based vision system for TurtleBot3 that detects persons using YOLOv8 and estimates their distance in Gazebo simulation.

## Features

- **Person Detection**: YOLOv8-nano for real-time person detection
- **Distance Estimation**: Pixel-based depth calculation from bounding box height
- **Gazebo Simulation**: Custom world with TurtleBot3 and person models
- **ROS2 Integration**: Publishes distance data and annotated images

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+
- Gazebo Classic 11

## Installation

1. Clone the repository:
```bash
git clone git@github.com:Sourav0607/VLN_BOT.git
cd VLN_BOT
```

2. Install Python dependencies:
```bash
cd ros2_ws/src/vision_language_nav
pip install -r requirements.txt
```

3. Build the workspace:
```bash
cd ~/VLN_BOT/ros2_ws
colcon build --packages-select vision_language_nav
source install/setup.bash
```

## Usage

1. Launch Gazebo simulation with custom world:
```bash
ros2 launch vision_language_nav my_custom_world.launch.py
```

2. In a new terminal, run the person detector:
```bash
cd ~/VLN_BOT/ros2_ws
source install/setup.bash
ros2 run vision_language_nav person_detector
```

3. View the annotated camera feed:
```bash
ros2 run rqt_image_view rqt_image_view /camera/image_detected
```

4. Monitor distance data:
```bash
ros2 topic echo /person/distance
```

## Topics

- **Subscribed**:
  - `/camera/image_raw` - RGB camera feed from TurtleBot3

- **Published**:
  - `/camera/image_detected` - Annotated image with bounding boxes and distance labels
  - `/person/distance` - Distance to detected person (std_msgs/Float32)

## Project Structure

```
ros2_ws/
├── src/
│   └── vision_language_nav/
│       ├── launch/
│       │   └── my_custom_world.launch.py
│       ├── worlds/
│       │   └── my_world.world
│       ├── vision_language_nav/
│       │   └── person_detector.py
│       └── requirements.txt
```

## Distance Estimation

The system uses pixel-based depth estimation:
- Formula: `depth (m) = (1.7 * 250) / bbox_height - 0.8`
- Based on average person height (~1.7m) and camera FOV (~60°)
- Calibrated for typical indoor scenarios

## License

MIT

## Author

Sourav
