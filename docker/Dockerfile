FROM osrf/ros:jazzy-desktop

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
    ros-jazzy-webots-ros2 \
    ros-jazzy-robot-localization \
    ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-rosbridge-suite \
    ros-jazzy-tf-transformations \
    ros-jazzy-xacro \
    ros-jazzy-rviz2 \
 && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install transform3d numpy pandas pylint black

# Setup working directory and copy source code
WORKDIR /drone_ws
COPY ./src /drone_ws/src
COPY ./worlds /drone_ws/worlds

# Build the workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

# Setup entrypoint to source both workspaces
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
 && echo "source /drone_ws/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash", "-c", "source /drone_ws/install/setup.bash && ros2 launch src/launch/master_launch.py"]
