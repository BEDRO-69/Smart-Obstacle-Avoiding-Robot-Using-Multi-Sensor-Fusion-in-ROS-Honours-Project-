# Smart Obstacle-Avoiding Robot Using Multi-Sensor Fusion in ROS2
## Project Overview
This project implements an autonomous obstacle-avoiding robot using multi-sensor fusion of LiDAR and ultrasonic sensors within the ROS2 framework. The target platform is the AgileX LIMO robot simulated in Gazebo Classic, running ROS2 Humble on Ubuntu 22.04.
The system fuses LiDAR and ultrasonic sensor data through a dual-layer Nav2 costmap architecture to produce more reliable obstacle detection than either sensor provides independently. Global path planning uses the **Smac Hybrid-A\*** algorithm with a Dubin motion model, and local trajectory following uses the **Regulated Pure Pursuit (RPP)** controller — both configured for the Ackermann kinematic constraints of the LIMO platform.
Robot:AgileX LIMO (Ackermann drive mode) 
OS: Ubuntu 22.04 LTS 
ROS:ROS2 Humble
Simulator:Gazebo Classic 11 
Visualisation:RViz2 

---

## Packages
`limo_msgs`:Custom ROS2 message types for LIMO hardware 
`limo_car`:Gazebo simulation launch files, world files, Nav2 config
`limo_description`: URDF and xacro robot description with sensor definitions
`limo_base`:Hardware driver for physical LIMO deployment
`limo_sensors`: Custom ultrasonic sensor publisher and relay nodes 

---


### System Requirements
- Ubuntu 22.04 LTS 
- ROS2 Humble
- Gazebo Classic 11


### Install ROS2 Humble
Follow the official guide:  
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

### Install Dependencies
```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-topic-tools \
  python3-colcon-common-extensions
```

---

## Installation

### 1. Clone the Repository
```bash
cd ~
git clone https://github.com/BEDRO-69/Smart-Obstacle-Avoiding-Robot-Using-Multi-Sensor-Fusion-in-ROS-Honours-Project-.git limo_ws
cd limo_ws
```

### 2. Build the Workspace
```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 3. Add to .bashrc (optional but recommended)
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/limo_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---


### Full Navigation Launch 

1. ros2 launch limo_car ackermann_gazebo.launch.py

2. ros2 launch slam_toolbox localization_launch.py \
  slam_params_file:=$HOME/limo_ws/install/limo_car/share/limo_car/config/slam_params.yaml \
  use_sim_time:=true

3. ros2 launch nav2_bringup navigation_launch.py \
  params_file:=$HOME/limo_ws/install/limo_car/share/limo_car/config/nav2_params.yaml \
  use_sim_time:=true
```
Wait for the terminal to show: `Managed nodes are active`

---

### Sending Navigation Goals

1. Open RViz2 (launches automatically with Gazebo)
2. Click 2D Pose Estimate in the toolbar and click on the map to set the robot's starting position
3. Click Nav2 Goal in the toolbar and click on the map to send a navigation goal
4. Monitor the Nav2 terminal for goal status and the RViz2 Nav2 panel for distance remaining

---

### Building a New Map (Optional)

If you want to rebuild the map from scratch:

 (replace localisation launch with mapping):

ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=$HOME/limo_ws/install/limo_car/share/limo_car/config/slam_params.yaml \
  use_sim_time:=true


Drive the robot manually using teleop to explore the environment:

ros2 run teleop_twist_keyboard teleop_twist_keyboard


Save the map when complete:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/limo_ws/maps/open_maze_map
```

---

## Configuration Files

All configuration files are in:
```
src/limo_ros2/limo_car/config/
nav2_params.yaml     # Nav2 full parameter set
slam_params.yaml     # SLAM Toolbox parameters
