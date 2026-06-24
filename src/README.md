# FusionBot Core 🚀

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue)
![C++](https://img.shields.io/badge/C++-17-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)

FusionBot Core is a full-stack robotics architecture demonstrating custom C++ algorithm development, ROS 2 integration, and high-fidelity physics simulation.

## 🌟 Key Features
- **Custom A* Global Planner:** A grid-based pathfinding algorithm built from scratch as a modern C++ `nav2_core::GlobalPlanner` plugin. Guarantees the shortest mathematical path avoiding `LETHAL_OBSTACLE` constraints.
- **Gazebo Harmonic Simulation:** A fully rigged URDF/Xacro robot chassis (`fusion_bot`) featuring a 360-degree LiDAR, IMU, Depth Camera, and `ros2_control` motor interfaces.
- **Advanced Nav2 Integration:** Deeply configured AMCL, local/global costmaps, and automated `behavior_server` recovery maneuvers (e.g., dynamically backing up and replanning when blocked by obstacles).
- **Foxglove Studio Telemetry:** Pre-configured architecture for real-time WebSocket visualization of the Particle Filter, TF Tree, and generated A* paths.

## 🛠️ Architecture
The workspace is cleanly separated into two core packages:
1. `fusion_bot`: Contains the URDF models, Gazebo bridge configurations, static map YAMLs, and launch files.
2. `fusion_bot_planner`: Contains the raw C++ source code for the custom A* pathfinding plugin.

## 🚀 Quick Start

### 1. Install Dependencies
This project requires ROS 2 Jazzy and Gazebo Harmonic.
```bash
sudo apt install ros-jazzy-nav2-bringup ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control ros-jazzy-foxglove-bridge
```

### 2. Build the Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <your-repo-link>
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch the Architecture
Open two terminals.

**Terminal 1: Physics Engine**
```bash
ros2 launch fusion_bot sim.launch.py
```

**Terminal 2: Navigation Stack (Nav2)**
```bash
ros2 launch fusion_bot navigation.launch.py
```

### 4. Visualize
1. Open Foxglove Studio and connect to `ws://localhost:8765`.
2. Add a 3D Panel and enable `/scan`, `/map`, `/robot_description`, and `/plan`.
3. Use the "Publish Pose" tool on `/goal_pose` to watch the A* algorithm dynamically navigate the robot!

## 🎥 Demonstration
*(Insert a GIF of your Foxglove Screencast here showing the green A* path!)*
