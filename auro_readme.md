# AURO Assessment - Autonomous Item Retrieval System

## Overview

This project implements an autonomous robotic system for item retrieval using TurtleBot3 Waffle Pi robots in a ROS 2 simulation environment. The system can deploy up to 3 robots to collect colored items (red, blue, green, yellow) and return them to their corresponding collection zones.

## System Requirements

- **ROS 2**: Humble Hawksbill
- **Gazebo**: Classic 11
- **Python**: 3.8+
- **Operating System**: Ubuntu 22.04 LTS (recommended)

## Dependencies

### Required ROS 2 Packages
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-rviz2
```

### Python Dependencies
```bash
pip3 install opencv-python
pip3 install pyyaml
pip3 install numpy
```

### Environment Variables
Add these lines to your `~/.bashrc`:
```bash
export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/your_workspace/src/auro_assessment/models
source /opt/ros/humble/setup.bash
source ~/your_workspace/install/setup.bash
```

## Package Structure

```
auro_assessment/
├── launch/
│   └── assessment_world_launch.py
├── src/
│   ├── item_detector_node.py
│   └── assessment_task_manager.py
├── worlds/
│   └── assessment_world.world
├── config/
│   └── assessment_world.yaml
├── rviz/
│   └── assessment_view.rviz
├── models/
│   └── [item and world models]
└── package.xml
```

## Installation and Build

1. **Clone/Copy the project** into your ROS 2 workspace:
```bash
cd ~/your_workspace/src
# Copy the auro_assessment package here
```

2. **Install dependencies**:
```bash
cd ~/your_workspace
rosdep install --from-paths src --ignore-src -r -y
```

3. **Build the workspace**:
```bash
colcon build --packages-select auro_assessment
source install/setup.bash
```

## Running the System

### Basic Single Robot Scenario

Launch the assessment world with one robot:
```bash
ros2 launch auro_assessment assessment_world_launch.py
```

This will start:
- Gazebo simulation with the assessment world
- One TurtleBot3 robot at position (0.0, 0.0)
- Item detector node for the robot
- Assessment task manager
- RViz visualization

### Multi-Robot Scenarios

#### Two Robot Configuration:
```bash
ros2 launch auro_assessment assessment_world_launch.py \
    robot_count:=2 \
    x_pose_1:=0.0 y_pose_1:=0.0 \
    x_pose_2:=2.0 y_pose_2:=0.0
```

#### Three Robot Configuration:
```bash
ros2 launch auro_assessment assessment_world_launch.py \
    robot_count:=3 \
    x_pose_1:=0.0 y_pose_1:=0.0 \
    x_pose_2:=2.0 y_pose_2:=0.0 \
    x_pose_3:=-2.0 y_pose_3:=0.0
```

### Custom Robot Positions

You can specify custom starting positions for each robot:
```bash
ros2 launch auro_assessment assessment_world_launch.py \
    robot_count:=2 \
    robot_name_1:=explorer_1 robot_name_2:=collector_2 \
    x_pose_1:=1.0 y_pose_1:=1.0 \
    x_pose_2:=-1.0 y_pose_2:=-1.0
```

## Test Scenarios

The following scenarios are recommended for testing and evaluation:

### Scenario 1: Single Robot Basic Collection
```bash
ros2 launch auro_assessment assessment_world_launch.py robot_count:=1
```
- **Purpose**: Test basic item detection and collection with one robot
- **Expected Behavior**: Robot should navigate, find items, and return them to correct zones
- **Evaluation**: Basic functionality and obstacle avoidance

### Scenario 2: Dual Robot Coordination
```bash
ros2 launch auro_assessment assessment_world_launch.py \
    robot_count:=2 \
    x_pose_1:=-3.0 y_pose_1:=0.0 \
    x_pose_2:=3.0 y_pose_2:=0.0
```
- **Purpose**: Test multi-robot coordination and task allocation
- **Expected Behavior**: Robots should work together efficiently without conflicts
- **Evaluation**: Coordination strategies and efficiency improvements

### Scenario 3: Maximum Capacity Test
```bash
ros2 launch auro_assessment assessment_world_launch.py \
    robot_count:=3 \
    x_pose_1:=0.0 y_pose_1:=2.0 \
    x_pose_2:=-2.0 y_pose_2:=-1.0 \
    x_pose_3:=2.0 y_pose_3:=-1.0
```
- **Purpose**: Test system with maximum number of robots
- **Expected Behavior**: All three robots coordinate effectively
- **Evaluation**: Scalability and advanced coordination

### Scenario 4: Corner Start Positions
```bash
ros2 launch auro_assessment assessment_world_launch.py \
    robot_count:=2 \
    x_pose_1:=7.0 y_pose_1:=7.0 \
    x_pose_2:=-7.0 y_pose_2:=-7.0
```
- **Purpose**: Test navigation from challenging starting positions
- **Expected Behavior**: Robots navigate efficiently from corners to center area
- **Evaluation**: Path planning and navigation robustness

## Monitoring and Debugging

### View ROS Graph
```bash
rqt_graph
```

### Monitor Item Detections
```bash
ros2 topic echo /tb3_0/item_detections
```

### Check Assessment Status
```bash
ros2 topic echo /assessment/item_status
ros2 topic echo /assessment/zone_status
ros2 topic echo /assessment/score
```

### Debug Camera Feed
```bash
ros2 run rqt_image_view rqt_image_view /item_detector/debug_image
```

## Key Topics and Services

### Published Topics
- `/tb3_0/item_detections` - Item detection results (JSON format)
- `/assessment/item_status` - Current status of all items
- `/assessment/zone_status` - Status of collection zones
- `/assessment/score` - Current task score

### Subscribed Topics
- `/tb3_0/camera/image_raw` - Robot camera feed
- `/tb3_0/camera/depth/image_raw` - Depth information
- `/collect_item` - Item collection requests
- `/return_item` - Item return requests

### Services
- `/spawn_entity` - Spawn items in Gazebo
- `/delete_entity` - Remove items from Gazebo

## Configuration Parameters

### Item Detector Parameters
- `min_contour_area`: Minimum area for item detection (default: 500)
- `max_contour_area`: Maximum area for item detection (default: 50000)
- `confidence_threshold`: Minimum confidence for detections (default: 0.7)

### Task Manager Parameters
- `max_items_per_type`: Maximum items per color type (default: 3)
- `item_spawn_radius`: Radius for item spawning (default: 8.0)
- `respawn_delay`: Delay before respawning collected items (default: 5.0)

## Troubleshooting

### Common Issues

1. **Gazebo fails to start**:
   - Ensure `GAZEBO_MODEL_PATH` is set correctly
   - Check that all TurtleBot3 packages are installed

2. **No item detections**:
   - Verify camera topics are publishing: `ros2 topic list | grep camera`
   - Check if items are visible in simulation

3. **Robots don't move**:
   - Ensure your robot control nodes are running
   - Check `/cmd_vel` topics are being published to

4. **Build errors**:
   - Run `rosdep install --from-paths src --ignore-src -r -y`
   - Ensure all Python dependencies are installed

### Performance Tips

1. **For better performance**: Close unnecessary applications and increase Gazebo's real-time factor
2. **For debugging**: Use RViz to visualize robot trajectories and sensor data
3. **For testing**: Start with single robot scenarios before moving to multi-robot

## Implementation Notes

- The `item_detector_node.py` processes camera images to detect colored items using OpenCV
- The `assessment_task_manager.py` manages the simulation environment and tracks task progress
- The launch file supports flexible robot deployment with configurable positions
- All item detection uses HSV color space for robust color recognition
- The system supports automatic item respawning after successful collection

## Assessment Integration

This code is designed to work with the AURO assessment framework and should not be modified. Your robot control implementation should:
1. Subscribe to `/tb3_*/item_detections` to receive item information
2. Publish to `/collect_item` to collect items
3. Publish to `/return_item` to return items to zones
4. Use the provided coordinate system and topic structure