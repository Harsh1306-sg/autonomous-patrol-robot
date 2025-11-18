# ROS2 Autonomous Patrol Robot

A ROS2 Humble–based autonomous patrol robot system that uses the Nav2 stack for waypoint navigation, real-time obstacle avoidance, multi-cycle patrol routines, and patrol data logging. The robot operates in simulation using TurtleBot3 and Gazebo and can be adapted for real-world deployment.

## Table of Contents
- [Features](#features)
- [Demo Overview](#demo-overview)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Configuration](#configuration)
- [Logs](#logs)
- [Troubleshooting](#troubleshooting)
- [Monitoring and Debugging](#monitoring-and-debugging)
- [Performance Tips](#performance-tips)
- [Contributing](#contributing)
- [Assignment Requirements](#assignment-requirements)
- [Learning Resources](#learning-resources)
- [License](#license)
- [Author](#author)
- [Acknowledgments](#acknowledgments)
- [Support](#support)

## Features

- **Autonomous Waypoint Patrol**
  Navigates through a predefined set of waypoints using ROS2 action interfaces.

- **Dynamic Obstacle Avoidance**
  Utilizes Nav2’s global and local planners for real-time path adjustments.

- **Return-to-Start Behavior**
  Automatically returns to its initial pose after completing all patrol cycles.

- **Structured Data Logging**
  Saves navigation logs in JSON format with timestamps, positions, and status messages.

- **Multi-Cycle Patrol**
  Supports configurable numbers of patrol loops.

- **RViz2 Visualization**
  Provides real-time visualization of the robot’s path, goals, and localization.

## Demo Overview

In the simulation environment (TurtleBot3 + Gazebo), the robot:
1. Initializes at the starting pose (0, 0).
2. Navigates through all defined waypoints in order.
3. Completes a configurable number of patrol cycles.
4. Returns to the starting position.
5. Saves a detailed patrol log to `~/patrol_logs/`.

## Prerequisites

**Operating System:** Ubuntu 22.04  
**ROS2 Distribution:** Humble Hawksbill  
**Python:** 3.8+

**Required ROS2 Packages:**
- ros-humble-desktop
- ros-humble-navigation2
- ros-humble-nav2-bringup
- ros-humble-turtlebot3*
- ros-humble-gazebo-ros-pkgs

## Installation

### 1. Install ROS2 Humble
Install ROS2 Humble following the official instructions.

### 2. Install Dependencies
```
sudo apt install ros-humble-turtlebot3* -y
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup -y
sudo apt install ros-humble-gazebo-ros-pkgs -y
sudo apt install python3-numpy python3-pip -y
```

### 3. Environment Setup
Add to `.bashrc`:
```
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
```

### 4. Clone the Repository
```
cd ~
mkdir -p autonomous-patrol-robot
cd autonomous-patrol-robot

git clone https://github.com/Harsh1306-sg/autonomous-patrol-robot.git
cd ~/autonomous-patrol-robot/src/patrol_robot_pkg/scripts

chmod +x simple_patrol.py
```

## Usage

### Terminal 1 — Launch Gazebo
```
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Terminal 2 — Launch Navigation2
```
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True
```

Set the initial pose in RViz2.

### Terminal 3 — Run Patrol Script
```
cd ~/autonomous-patrol-robot/src/patrol_robot_pkg/scripts
python3 simple_patrol.py
```
### Expected Output

```
[INFO] [simple_patrol]: 🤖 Simple Patrol Robot Started!
[INFO] [simple_patrol]: 📍 Total waypoints: 5
[INFO] [simple_patrol]: 🔄 Cycles to complete: 2
[INFO] [simple_patrol]: 🚀 Starting patrol in 3 seconds...
[INFO] [simple_patrol]: ⚠️  Make sure you set the initial pose in RViz!
[INFO] [simple_patrol]: 🎯 Going to waypoint 0: (0.0, 0.0)
[INFO] [simple_patrol]: ⏳ Goal accepted, moving...
[INFO] [simple_patrol]: ✅ Reached waypoint 0!
[INFO] [simple_patrol]: ✅ [0] WP0: (0.0, 0.0) - SUCCESS
```

## Project Structure

```
autonomous-patrol-robot/
├── src/
│   └── patrol_robot_pkg/
│       ├── scripts/
│       │   └── simple_patrol.py          # Main 
│       ├── launch/                       # Future launch files (optional)
│       ├── config/                       # Future config files (optional)
│       └── maps/                         # Future custom maps (optional)
├── README.md                             # This file
├── LICENSE                               # Apache 2.0 License
└── .gitignore                            # Git ignore file
```


## Configuration

### Editing Waypoints
Modify in `simple_patrol.py`:
```
self.waypoints = [
    [x, y, yaw_degrees],
]
```

### Patrol Cycles
```
self.max_cycles = 2
```

### RViz Coordinates
```
ros2 topic echo /clicked_point
```

## Logs

Patrol logs stored in `~/patrol_logs/`.

Example:
```
{
  "time": "2025-11-19T10:30:15.123456",
  "cycle": 0,
  "waypoint": 1,
  "x": 1.5,
  "y": 0.0,
  "yaw": 90.0,
  "status": "SUCCESS"
}
```

## Troubleshooting

Includes:
- Nav2 issues
- Robot not moving
- Missing packages
- Gazebo crashes

## Monitoring and Debugging
```
ros2 topic list
ros2 topic echo /odom
ros2 run tf2_tools view_frames
ros2 run rqt_console rqt_console
```

## Performance Tips

- Reduce simulation load
- Tune DWA parameters
- Increase goal tolerances
- Add intermediate waypoints

## Contributing

1. Fork
2. Create branch
3. Commit changes
4. Open PR

## Assignment Requirements

Implements:
- Waypoint patrol
- Dynamic navigation
- Return-to-start
- Multi-cycle patrol
- Logging

## Learning Resources

- ROS2 Documentation
- Nav2 Documentation
- TurtleBot3 Manual
- Gazebo Tutorials

## License

Apache 2.0 License.

## Author

**Harsh Pandey**  
GitHub: https://github.com/Harsh1306-sg

## Acknowledgments

- Automatic Addison tutorials
- TurtleBot3 (ROBOTIS)
- ROS2 & Nav2 communities
