FROM osrf/ros:humble-desktop

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    cmake \
    git \
 && rm -rf /var/lib/apt/lists/*

# Install specific ROS2 packages for the project
RUN apt-get update && apt-get install -y \
    ros-humble-webots-ros2 \
    ros-humble-robot-localization \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-rosbridge-suite \
    ros-humble-tf-transformations \
    ros-humble-xacro \
    ros-humble-rviz2 \
 && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install transform3d numpy pandas pylint black

# Setup working directory and copy source code
WORKDIR /drone_ws
COPY ./src /drone_ws/src
COPY ./worlds /drone_ws/worlds

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Setup entrypoint to source both workspaces
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
 && echo "source /drone_ws/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash", "-c", "source /drone_ws/install/setup.bash && ros2 launch src/launch/master_launch.py"]
