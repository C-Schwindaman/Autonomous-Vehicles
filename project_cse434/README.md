# Autonomous Driving System (Team 22)

## Project Overview
This ROS 2 package implements an autonomous driving system for the TurtleBot3. It integrates three core subsystems:
1. Lane Following: PID-based control to navigate tracks.
2. Sign Recognition: YOLO-based detection to obey traffic rules.
3. Obstacle Avoidance: LiDAR-based detection to prevent collisions.

# Installation
1. Clone this repository into your ROS 2 workspace src folder:
```bash
cd ~/src
git clone https://gitlab.msu.edu/rahman641/AV_Project_22.git
```
2. Build the package:
```bash
cd ..
colcon build --symlink-install --packages-select autonomous_driving
```
3. Source the setup file:
```bash
source install/setup.bash
```

## Usage
To run the full autonomous demonstration, you will need three terminals.
You will need to run the sign detection in a separate terminal

Terminal 1: Launch the custom autorace world: Make the file path wherever the downloaded world is
```bash
gazebo ~/..../AV_Project_22/Final_Project.world  
```
Terminal 2: Run the sign detection weights:
```bash
    act yolo
    addpypath 
    ros2 run autonomous_driving sign_detector \--weights_dir ~/av/hyunjoey_av/AV_Project_22/runs/detect/train8/weights
```
Terminal 3: Launch the decision maker node through the autonomous driving launch file:

```bash
ros2 launch autonomous_driving autonomous_driving_launch.py
```

# Architecture
- Inputs: /camera/image_raw, /scan
- Internal Topics: /lane_vel, /signs/detected, /obstacles
- Output: /cmd_vel