#!/bin/bash
# setup.sh - Environment setup for GPS-Denied Autonomous Drone Navigation System

echo "🚀 Starting Environment Setup..."

# Update and install dependencies
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3-pip python3-colcon-common-extensions python3-rosdep build-essential cmake

# Install necessary ROS2 packages
# Assuming ROS2 Humble is already installed, if not, user needs to follow official docs.
sudo apt install -y \
    ros-jazzy-webots-ros2 \
    ros-jazzy-robot-localization \
    ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-rosbridge-suite \
    ros-jazzy-tf-transformations \
    ros-jazzy-xacro \
    ros-jazzy-rviz2

# Install Python dependencies
pip3 install transform3d numpy pandas pylint black

# Create workspace structure (using current directory)
WORKSPACE_DIR=$(pwd)
echo "📁 Creating Workspace Structure at $WORKSPACE_DIR..."
mkdir -p $WORKSPACE_DIR/src
cd $WORKSPACE_DIR/src

# Create packages
ros2 pkg create --build-type ament_python perception_layer --dependencies rclpy robot_localization sensor_msgs nav_msgs geometry_msgs
ros2 pkg create --build-type ament_cmake slam_layer --dependencies rclcpp slam_toolbox nav_msgs sensor_msgs
ros2 pkg create --build-type ament_python navigation_layer --dependencies rclpy nav2_msgs geometry_msgs
ros2 pkg create --build-type ament_cmake control_layer --dependencies rclcpp std_msgs geometry_msgs sensor_msgs
ros2 pkg create --build-type ament_python fault_handler --dependencies rclpy std_msgs sensor_msgs
ros2 pkg create --build-type ament_python telemetry --dependencies rclpy std_msgs rosbridge_server

# Return to base workspace and build
echo "🔨 Building Workspace..."
cd $WORKSPACE_DIR
colcon build --symlink-install

# Source the workspace
echo "source $WORKSPACE_DIR/install/setup.bash" >> ~/.bashrc
source $WORKSPACE_DIR/install/setup.bash

echo "✅ Setup Complete. Please verify Webots bridge is working."
