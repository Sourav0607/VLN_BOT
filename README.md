# VLN_BOT - Vision Language Navigation Robot

A ROS2 Humble-based autonomous navigation system for TurtleBot3 that detects persons using YOLOv8, estimates distance, and navigates to them using Nav2 with fallback search behavior.

## Features

- **Person Detection**: YOLOv8-nano for real-time person detection
- **Distance Estimation**: Pixel-based depth calculation from bounding box height
- **Autonomous Navigation**: Nav2-based navigation to detected persons
- **Fallback Search**: 360° rotation search when person is lost, with forward movement between attempts
- **SLAM Mapping**: Cartographer-based mapping capability
- **Wide FOV**: 110° camera field of view for better person tracking
- **Safety Distance**: Maintains configurable safe distance from person

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+
- Gazebo Classic 11
- TurtleBot3 packages
- Nav2 navigation stack
- Cartographer SLAM

## Installation

1. Install ROS2 dependencies:
```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-turtlebot3*
```

2. Clone the repository:
```bash
mkdir -p ~/vision_navigation/ros2_ws/src
cd ~/vision_navigation/ros2_ws/src
git clone git@github.com:Sourav0607/VLN_BOT.git vision_language_nav
```

3. Install Python dependencies:
```bash
cd ~/vision_navigation/ros2_ws/src/vision_language_nav
pip install -r requirements.txt
```

4. Download YOLOv8 model:
```bash
cd ~/vision_navigation/ros2_ws
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt
```

5. Build the workspace:
```bash
cd ~/vision_navigation/ros2_ws
colcon build --packages-select vision_language_nav
source install/setup.bash
```

6. Set TurtleBot3 model:
```bash
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc
```

## Usage

### Step 1: Create Map (One-time setup)

1. Launch TurtleBot3 in Gazebo:
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

2. Start Cartographer SLAM (new terminal):
```bash
cd ~/vision_navigation/ros2_ws
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

3. Control robot to explore (new terminal):
```bash
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard
```

4. Save the map when done:
```bash
cd ~/vision_navigation/ros2_ws/src/vision_language_nav/maps
ros2 run nav2_map_server map_saver_cli -f my_map
```

### Step 2: Autonomous Person Navigation

Launch all components in separate terminals:

**Terminal 1 - Gazebo:**
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Terminal 2 - Navigation Stack:**
```bash
cd ~/vision_navigation/ros2_ws
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/vision_navigation/ros2_ws/src/vision_language_nav/maps/my_map.yaml
```

**Terminal 3 - Person Detector:**
```bash
cd ~/vision_navigation/ros2_ws
source install/setup.bash
ros2 run vision_language_nav person_detector
```

**Terminal 4 - Person Navigator:**
```bash
cd ~/vision_navigation/ros2_ws
source install/setup.bash
ros2 run vision_language_nav person_navigator
```

**In RViz2:** Set the initial robot pose using "2D Pose Estimate" button

### Monitoring

View annotated camera feed:
```bash
ros2 run rqt_image_view rqt_image_view /camera/image_detected
```

Monitor distance:
```bash
ros2 topic echo /person/distance
```

## How It Works

1. **Detection Phase**: YOLOv8 continuously detects persons in camera view
2. **Distance Calculation**: Estimates distance using bounding box height
3. **Navigation Trigger**: If person >0.35m away, triggers Nav2 navigation
4. **Approach**: Robot navigates to stop ~0.01m from person
5. **Fallback Search**: If person lost for 5 seconds:
   - Rotates 360° to search
   - If not found, moves forward 0.5m
   - Repeats up to 3 times
   - Stops search if person detected during rotation

## Configuration

### Navigation Parameters
Edit `person_navigator.py`:
- `safe_margin = 0.01` - Distance to maintain from person (meters)
- `min_navigation_distance = 0.35` - Minimum distance to trigger navigation
- `person_lost_timeout = 5.0` - Seconds before starting search
- `max_search_attempts = 3` - Number of search cycles

### Detection Parameters
Edit `person_detector.py`:
- Confidence threshold: `0.5`
- Distance formula: `depth = (1.7 * 250) / bbox_height - 0.8`

### Camera FOV
Modified in `~/.gazebo/models/turtlebot3_waffle_pi/model.sdf`:
- `horizontal_fov: 1.91986` radians (110°)

## Topics

**Subscribed:**
- `/camera/image_raw` - RGB camera feed
- `/person/distance` - Distance to person (Float32)

**Published:**
- `/camera/image_detected` - Annotated image with bounding boxes
- `/person/distance` - Distance to detected person
- `/cmd_vel` - Velocity commands during search
- `/navigate_to_pose` - Nav2 navigation goals

## Project Structure

```
ros2_ws/
├── yolov8n.pt (downloaded)
└── src/
    └── vision_language_nav/
        ├── worlds/
        │   └── my_world.world
        ├── maps/
        │   ├── my_map.yaml
        │   └── my_map.pgm
        ├── vision_language_nav/
        │   ├── person_detector.py
        │   └── person_navigator.py
        └── requirements.txt
```

## Troubleshooting

**Robot not navigating:**
- Ensure initial pose is set in RViz2
- Check map file path is correct
- Verify Nav2 nodes are running: `ros2 node list`

**Person not detected:**
- Check camera topic: `ros2 topic hz /camera/image_raw`
- View detection: `ros2 run rqt_image_view rqt_image_view /camera/image_detected`
- Verify YOLOv8 model exists: `ls ~/vision_navigation/ros2_ws/yolov8n.pt`

**Search not rotating:**
- Check cmd_vel: `ros2 topic echo /cmd_vel`
- Ensure no navigation conflict (only search when not navigating)

## License

MIT

## Author

Sourav - [GitHub](https://github.com/Sourav0607)

Sourav
