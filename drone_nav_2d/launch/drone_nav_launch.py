import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_unique_bag_output(bag_output_base: str) -> str:
    candidate = bag_output_base
    if os.path.exists(candidate):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        candidate = f'{bag_output_base}_{timestamp}'
    return candidate


def _build_actions(context):
    pkg_share = get_package_share_directory('drone_nav_2d')
    params_file = os.path.join(pkg_share, 'config', 'nav_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'drone_nav.rviz')
    world_easy = os.path.join(pkg_share, 'worlds', 'drone_world.wbt')
    world_hard = os.path.join(pkg_share, 'worlds', 'drone_world_hard.wbt')
    world_realistic = os.path.join(pkg_share, 'worlds', 'drone_world_realistic.wbt')
    urdf_path = os.path.join(pkg_share, 'urdf', 'drone.urdf')

    world_profile = LaunchConfiguration('world_profile').perform(context).strip().lower()
    bag_output_base = LaunchConfiguration('bag_output').perform(context).strip()
    webots_port = LaunchConfiguration('webots_port').perform(context).strip()
    bag_output = _resolve_unique_bag_output(bag_output_base)

    selected_world = world_realistic
    if world_profile == 'easy':
        selected_world = world_easy
    elif world_profile == 'hard':
        selected_world = world_hard

    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    webots_home = os.environ.get('WEBOTS_HOME', '')
    webots_executable = os.path.join(webots_home, 'webots') if webots_home else 'webots'

    webots = ExecuteProcess(
        cmd=[
            webots_executable,
            f'--port={webots_port}',
            selected_world,
            '--batch',
            '--mode=realtime',
        ],
        output='screen',
        name='webots',
    )

    bridge_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={
            'WEBOTS_CONTROLLER_URL': f'tcp://127.0.0.1:{webots_port}/drone',
        },
        parameters=[
            {'robot_description': robot_description},
        ],
        remappings=[
            ('/gps', '/webots/drone/gps'),
            ('/scan', '/webots/drone/scan'),
            ('/cmd_vel', '/cmd_vel'),
        ],
    )

    map_publisher = Node(
        package='drone_nav_2d',
        executable='map_publisher',
        name='map_publisher',
        output='screen',
        parameters=[params_file],
    )

    planner = Node(
        package='drone_nav_2d',
        executable='path_planner',
        name='path_planner',
        output='screen',
        parameters=[params_file],
    )

    controller = Node(
        package='drone_nav_2d',
        executable='drone_controller',
        name='drone_controller',
        output='screen',
        parameters=[params_file],
    )

    avoidance = Node(
        package='drone_nav_2d',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        output='screen',
        parameters=[params_file],
    )

    metrics = Node(
        package='drone_nav_2d',
        executable='metrics_logger',
        name='metrics_logger',
        output='screen',
        parameters=[params_file],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    rosbag = ExecuteProcess(
        cmd=[
            'ros2',
            'bag',
            'record',
            '-o',
            bag_output,
            '/map',
            '/planned_path',
            '/drone_pose',
            '/drone_trajectory',
            '/cmd_vel',
            '/obstacle_detected',
            '/replan_event',
            '/min_obstacle_distance',
            '/mission_complete',
            '/obstacle_markers',
        ],
        output='screen',
    )

    actions = [
        LogInfo(msg=[f'Launching drone navigation in {world_profile} scenario']),
        LogInfo(msg=[f'Rosbag output: {bag_output}']),
        LogInfo(msg=[f'Webots port: {webots_port}']),
        webots,
        bridge_driver,
        map_publisher,
        planner,
        controller,
        avoidance,
        metrics,
        rviz,
        rosbag,
    ]

    return actions


def generate_launch_description() -> LaunchDescription:
    actions = [
        DeclareLaunchArgument(
            'world_profile',
            default_value='realistic',
            description='Scenario world: easy | hard | realistic',
        ),
        DeclareLaunchArgument(
            'bag_output',
            default_value='bags/drone_nav_run',
            description='Output directory for rosbag2 recording',
        ),
        DeclareLaunchArgument(
            'webots_port',
            default_value='1234',
            description='TCP port used by Webots and webots_ros2_driver',
        ),
        OpaqueFunction(function=_build_actions),
    ]

    return LaunchDescription(actions)
