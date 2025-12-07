# Vision Language Navigation (VLN_BOT)

A ROS2 package for autonomous person detection and navigation using YOLOv8, Cartographer SLAM, and Nav2 navigation stack.

## Features

- **Person Detection**: Real-time person detection using YOLOv8-nano
- **Depth Estimation**: Pixel-based depth calculation from bounding box height
- **SLAM Mapping**: Cartographer-based simultaneous localization and mapping
- **Autonomous Navigation**: Nav2-based navigation to detected persons
- **Safety**: Maintains 0.5m safety distance from detected persons

## System Requirements

- ROS2 Humble
- Ubuntu 22.04
- Gazebo Classic 11
- Python 3.10+

## Dependencies

```bash
# ROS2 packages
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-turtlebot3*

# Python packages
pip install ultralytics opencv-python
```

## Installation

1. Clone the repository:
```bash
cd ~/vision_navigation/ros2_ws/src
git clone git@github.com:Sourav0607/VLN_BOT.git vision_language_nav
```

2. Download YOLOv8 nano model:
```bash
cd ~/vision_navigation/ros2_ws
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt
```

3. Build the package:
```bash
cd ~/vision_navigation/ros2_ws
colcon build --packages-select vision_language_nav
source install/setup.bash
```

## Usage

### Step 1: Create a Map with Cartographer SLAM

First, launch Gazebo with the custom world:
```bash
source ~/vision_navigation/ros2_ws/install/setup.bash
ros2 launch vision_language_nav my_custom_world.launch.py
```

In a new terminal, start Cartographer for mapping:
```bash
source ~/vision_navigation/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch vision_language_nav cartographer_slam.launch.py
```

In another terminal, control the robot to explore and build the map:
```bash
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard
```

Once the map is complete, save it:
```bash
cd ~/vision_navigation/ros2_ws/src/vision_language_nav/maps
ros2 run nav2_map_server map_saver_cli -f my_world_map
```

### Step 2: Autonomous Person Navigation

After creating the map, launch the autonomous navigation system:

1. Launch Gazebo world:
```bash
source ~/vision_navigation/ros2_ws/install/setup.bash
ros2 launch vision_language_nav my_custom_world.launch.py
```

2. In a new terminal, launch autonomous navigation:
```bash
source ~/vision_navigation/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch vision_language_nav autonomous_person_navigation.launch.py
```

3. Set initial pose in RViz2:
   - Click "2D Pose Estimate" button
   - Click and drag on the map where the robot is located

The robot will now:
- Continuously detect persons in camera view
- Calculate distance to detected persons
- Automatically navigate to approach the person (stopping 0.5m away)

### Alternative: Run Components Separately

If you prefer to run components individually:

**Terminal 1** - Gazebo:
```bash
ros2 launch vision_language_nav my_custom_world.launch.py
```

**Terminal 2** - Navigation Stack:
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch vision_language_nav navigation.launch.py map:=/path/to/your/map.yaml
```

**Terminal 3** - Person Detector:
```bash
ros2 run vision_language_nav person_detector
```

**Terminal 4** - Person Navigator:
```bash
ros2 run vision_language_nav person_navigator
```

**Terminal 5** - RViz2 for visualization:
```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

## ROS Topics

### Published Topics

- `/camera/image_detected` (sensor_msgs/Image) - Annotated camera image with bounding boxes
- `/person/distance` (std_msgs/Float32) - Distance to detected person in meters
- `/goal_pose` (geometry_msgs/PoseStamped) - Navigation goal for Nav2

### Subscribed Topics

- `/camera/image_raw` (sensor_msgs/Image) - Raw camera feed from Gazebo
- `/scan` (sensor_msgs/LaserScan) - Laser scan data for navigation

## Configuration

### Person Detection

Edit `vision_language_nav/person_detector.py`:
- `CONFIDENCE_THRESHOLD = 0.5` - Detection confidence threshold
- Depth formula: `depth = (1.7 * 250) / bbox_height - 0.8`

### Navigation Parameters

Edit `config/nav2_params.yaml`:
- Robot radius: `0.17m`
- Max velocity: `0.22 m/s`
- Goal tolerance: `0.15m`

### Safety Distance

Edit `vision_language_nav/person_navigator.py`:
- `SAFETY_MARGIN = 0.5` - Distance to maintain from person (meters)

## Project Structure

```
vision_language_nav/
├── launch/
│   ├── my_custom_world.launch.py          # Gazebo world launcher
│   ├── cartographer_slam.launch.py        # SLAM mapping launcher
│   ├── navigation.launch.py               # Nav2 stack launcher
│   └── autonomous_person_navigation.launch.py  # Complete system launcher
├── vision_language_nav/
│   ├── person_detector.py                 # YOLOv8 person detector
│   └── person_navigator.py                # Navigation goal publisher
├── config/
│   └── nav2_params.yaml                   # Nav2 configuration
├── maps/
│   └── my_world_map.yaml/pgm             # Saved map files
├── worlds/
│   └── my_world.world                     # Custom Gazebo world
└── README.md
```

## Troubleshooting

### IMU Link Warnings
If you see warnings about "imu_link" during Cartographer SLAM, these can be safely ignored. The TurtleBot3 model doesn't include an IMU link, but 2D LIDAR-based SLAM works correctly.

### Navigation Not Starting
1. Ensure you've set an initial pose in RViz2
2. Verify the map file exists in `maps/` directory
3. Check that all Nav2 nodes are running: `ros2 node list`

### Person Not Detected
1. Verify YOLOv8 model file `yolov8n.pt` exists in workspace root
2. Check camera topic: `ros2 topic echo /camera/image_raw`
3. View detection output: `ros2 run rqt_image_view rqt_image_view /camera/image_detected`

### Robot Doesn't Navigate to Person
1. Confirm distance is published: `ros2 topic echo /person/distance`
2. Check goal pose: `ros2 topic echo /goal_pose`
3. Verify Nav2 is receiving goals: `ros2 topic echo /navigate_to_pose/_action/feedback`

## Future Enhancements

- [ ] Multi-person tracking and selection
- [ ] Dynamic goal replanning for moving persons
- [ ] Voice command integration
- [ ] Object recognition beyond persons
- [ ] Improved depth estimation using depth camera
- [ ] Person following behavior

## License

MIT License

## Authors

Sourav - [GitHub](https://github.com/Sourav0607)

## Repository

https://github.com/Sourav0607/VLN_BOT
