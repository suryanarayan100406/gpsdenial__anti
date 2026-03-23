# 🚁 GPS-Denied Drone Navigation System: Complete Documentation

Welcome to the comprehensive documentation for the **GPS-Denied Drone Navigation System**. This document provides an in-depth overview of the codebase architecture, the location of required Webots and ROS 2 files, and a step-by-step guide on how to transition this simulation into a **real-world physical drone deployment**.

---

## 📑 Table of Contents
1. [Codebase Overview](#1-codebase-overview)
2. [Locating Webots and ROS 2 Files](#2-locating-webots-and-ros-2-files)
3. [Deep Dive: How the ROS 2 Navigation works](#3-deep-dive-how-the-ros-2-navigation-works)
4. [Real Drone Implementation Guide](#4-real-drone-implementation-guide)

---

## 1. Codebase Overview

This repository contains two parallel approaches to drone simulation and navigation:

### A. Pure-Python Advanced Simulation (`advanced_drone_sim.py`)
- **Location**: Root directory (`advanced_drone_sim.py`, `metrics_evaluator.py`, `drone_telemetry_cmd.py`).
- **Description**: A highly mathematical, headless 3D visualization using `matplotlib`. It implements bleeding-edge algorithms from scratch (e.g., Unscented Kalman Filters, D* Lite, 60-rollout Model Predictive Control) without requiring a heavy physics engine. 
- **Use Case**: Rapid algorithm testing, parameter tuning, and metric evaluation without ROS 2 overhead.

### B. ROS 2 + Webots Production Stack (`drone_nav_2d/`)
- **Location**: Inside the `drone_nav_2d/` package directory.
- **Description**: A complete, production-ready ROS 2 (Humble/Jazzy) autonomous navigation stack mapped directly to the Webots R2025a physics simulator. It implements publish/subscribe architectures for sensor data, path planning, obstacle avoidance, and PID flight control.
- **Use Case**: Realistic simulation with accurate physics, sensor noise (LiDAR, Camera, IMU), and preparation for deployment on real hardware.

---

## 2. Locating Webots and ROS 2 Files

All production-grade robotic files are contained within the `drone_nav_2d` folder. 

### 🤖 Webots Files (Simulation Environment)
Webots is responsible for the 3D rendering, physics simulation, and simulated sensor data generation.
* **Worlds (`.wbt` files)**: Located in `drone_nav_2d/worlds/`
  - *Examples*: `drone_world_hard.wbt`, `drone_world_urban_3d.wbt`, `drone_world_realistic.wbt`
  - *Purpose*: Defines the environment, terrain, lighting, static obstacles (trees, buildings), and the starting position of the drone.
* **URDF Models**: Located in `drone_nav_2d/urdf/`
  - *Examples*: `drone.urdf`, `drone_3d.urdf`
  - *Purpose*: Defines the physical properties of the drone (mass, joints) and attaches the `webots_ros2` plugins that bridge the simulator sensors (LiDAR/Camera/IMU/GPS) to ROS 2 topics.
* **Webots Controllers**: Located in `drone_nav_2d/controllers/`
  - *Purpose*: Sometimes used for dynamic moving obstacles in the Webots world.

### 🐢 ROS 2 Files (Brain & Logic)
ROS 2 handles the "brain" of the drone—taking sensor data, computing paths, and sending velocity commands.
* **ROS 2 Nodes (Python)**: Located in `drone_nav_2d/drone_nav_2d/`
  - `path_planner.py` / `path_planner_3d.py`: Calculates A* and RRT routes.
  - `obstacle_avoidance.py` / `obstacle_avoidance_3d.py`: Calculates Potential Field reflexive movements.
  - `drone_controller.py` / `drone_controller_3d.py`: PID controllers converting planned paths into Twist velocity commands.
  - `map_publisher.py` / `voxel_grid.py`: Converts raw LiDAR/Depth data into occupancy grids.
* **Launch Files**: Located in `drone_nav_2d/launch/`
  - *Examples*: `drone_nav_launch.py`, `drone_nav_3d_launch.py`
  - *Purpose*: Starts Webots, loads the `.wbt` world, injects the URDF drone, and spins up all the ROS 2 nodes simultaneously.
* **Configuration / Params**: Located in `drone_nav_2d/config/`
  - *Examples*: `nav_params.yaml`, `nav_params_3d.yaml`
  - *Purpose*: YAML files containing PID gains, obstacle safety radii, and planner thresholds.

---

## 3. Deep Dive: How the ROS 2 Navigation works

When you run a launch file (e.g., `ros2 launch drone_nav_2d drone_nav_3d_launch.py`), the following pipeline occurs:

1. **Sensing (Webots)**: The `webots_ros2_driver` reads the simulated sensors from the URDF and publishes to topics:
   - `/scan` (from 2D LiDAR)
   - `/camera/depth/image_raw` (from 3D Depth Camera)
   - `/imu/data` (Orientation & Acceleration)
2. **Perception (ROS 2)**: `voxel_grid.py` subscribes to the depth camera and creates a 3D occupancy map of obstacles.
3. **Planning (ROS 2)**: `path_planner_3d.py` takes the goal and the occupancy grid, running an A* search to calculate a global path avoiding known obstacles.
4. **Reactive Avoidance (ROS 2)**: `obstacle_avoidance_3d.py` monitors the immediate `/scan` or voxel space. If a dynamic obstacle jumps in front, it applies a repulsive Artificial Potential Field force.
5. **Control (ROS 2)**: `drone_controller_3d.py` fuses the global path and the repulsive forces, calculates the required velocities using PID loops, and publishes to `/cmd_vel` (`geometry_msgs/Twist`).
6. **Actuation (Webots)**: The simulator receives `/cmd_vel` and applies thrust to the drone's rotors, moving it in the simulation.

---

## 4. Real Drone Implementation Guide

Transitioning this codebase from the Webots simulation to a physical drone (like a custom quadcopter running PX4 or ArduPilot, or an off-the-shelf DJI Matrice / ROS-enabled drone) requires bridging the gap between ROS 2 and physical hardware. 

The beauty of ROS 2 is that your **Navigation Logic** (Planners, Controllers, Avoidance) remains **exactly the same**. You only need to replace the simulated sensors with real sensors.

### Step 1: Hardware Requirements
To run this stack on a real drone, you need a companion computer mounted on the hardware.
- **Companion Computer**: Raspberry Pi 4/5, Jetson Nano, or Jetson Orin Nano (running Ubuntu 22.04 + ROS 2).
- **Flight Controller (FCU)**: Pixhawk 4/6C or Cube Orange (running PX4 Autopilot or ArduPilot).
- **Sensors**: 
  - 2D/3D LiDAR (e.g., RPLidar A2/A3, or Ouster).
  - Depth Camera (e.g., Intel RealSense D435i).
  - VIO Tracking Camera (for GPS-denied state estimation, e.g., Intel RealSense T265 or OAK-D).

### Step 2: Swap the `webots_ros2_driver` for Real Drivers
In the simulation, Webots publishes sensor data. On real hardware, you must install and run the official ROS 2 drivers for your sensors.
- **LiDAR**: Run `sllidar_ros2` (for RPLidar) to publish to `/scan`.
- **Depth Camera**: Run `realsense2_camera` to publish to `/camera/depth/image_raw`.
- **State Estimation**: Since you are in a **GPS-denied** environment, you cannot rely on `/gps`. You must use a VIO (Visual Inertial Odometry) camera or run a SLAM package (like `rtabmap_ros` or `slam_toolbox`) to publish the `/odom` and map frame transformations (`tf`).

### Step 3: Establish FCU Communication (Micro-ROS / MAVROS)
Instead of Webots subscribing to `/cmd_vel`, you must configure your Flight Controller to listen to your companion computer.
- **Standard Approach**: Use **MAVROS** to translate standard ROS 2 `/cmd_vel` topics into MAVLink commands that the Pixhawk understands.
- **Modern Approach (PX4)**: Use **micro-ROS** (XRCE-DDS Agent). You will map the ROS 2 `/cmd_vel` topic directly into PX4 uORB topics (like `TrajectorySetpoint`).

### Step 4: Update the Launch Files
You will create a new launch file (e.g., `drone_nav_real_world.launch.py`) that **excludes** Webots entirely.
```python
# Instead of launching Webots, you launch your hardware drivers:
IncludeLaunchDescription(launch_ros.actions.Node(package='realsense2_camera', executable='realsense2_camera_node'))
IncludeLaunchDescription(launch_ros.actions.Node(package='sllidar_ros2', executable='sllidar_node'))
IncludeLaunchDescription(launch_ros.actions.Node(package='mavros', executable='mavros_node'))

# Then, you launch the exact same navigation nodes from this project!
Node(package='drone_nav_2d', executable='voxel_grid'),
Node(package='drone_nav_2d', executable='path_planner_3d'),
Node(package='drone_nav_2d', executable='obstacle_avoidance_3d'),
Node(package='drone_nav_2d', executable='drone_controller_3d'),
```

### Step 5: Safety and Tuning
1. **PID Tuning**: Real-world dynamics (wind, inertia, distinct motor KV ratings) are different than Webots. You must meticulously re-tune the PID parameters in `nav_params.yaml`.
2. **Offboard Mode**: To accept `/cmd_vel` from ROS 2, the Pixhawk must be put into "OFFBOARD" (PX4) or "GUIDED" (ArduPilot) mode via your RC transmitter.
3. **Safety Switch**: Always have an RC transmitter in hand with a dedicated switch mapped to immediately regain manual control (Return to Land or Position mode) in case the navigation algorithm attempts to crash into an obstacle.

---
*By following this hardware abstraction conceptually, the advanced planning and avoidance scripts developed in `drone_nav_2d` will port beautifully to a physical quadcopter frame.*
