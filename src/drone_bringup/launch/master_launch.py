import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_controller import WebotsController

import subprocess

def get_windows_ip():
    """Dynamically find the Windows host IP from WSL."""
    try:
        # Check the default route first
        result = subprocess.run(['ip', 'route'], capture_output=True, text=True)
        for line in result.stdout.split('\n'):
            if 'default' in line:
                return line.split(' ')[2]
        # Fallback to nameserver
        result = subprocess.run(['grep', 'nameserver', '/etc/resolv.conf'], capture_output=True, text=True)
        return result.stdout.split(' ')[1].strip()
    except Exception:
        return '127.0.0.1'

def generate_launch_description():
    """Master launch file for GPS-Denied Drone Navigation System."""
    
    package_dir = get_package_share_directory('drone_bringup')
    
    # The URDF string for the Webots driver
    robot_description_path = os.path.join(package_dir, 'urdf', 'mavic_webots.urdf')
    with open(robot_description_path, 'r') as f:
        robot_description = f.read()

    # The WebotsController node connects to the <extern> robot in Webots
    # It communicates via TCP over the default port or WSL2 bridge.
    webots_driver = WebotsController(
        robot_name='mavic2pro',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
            {'ip_address': get_windows_ip()}
        ]
    )

    # Launch Perception Layer (EKF Fusion)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('perception_layer'), 'config', 'ekf_params.yaml'), {'use_sim_time': True}]
    )

    # Launch Control Layer (PID)
    control_node = Node(
        package='control_layer',
        executable='pid_controller',
        name='pid_controller',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    motor_mixer_node = Node(
        package='control_layer',
        executable='motor_mixer',
        name='motor_mixer',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        webots_driver,
        ekf_node,
        control_node,
        motor_mixer_node
    ])
