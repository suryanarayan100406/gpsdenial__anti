from setuptools import setup

package_name = 'drone_nav_2d'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', [
            'launch/drone_nav_launch.py',
            'launch/drone_nav_headless.py',
            'launch/drone_nav_with_video.py',
            'launch/drone_nav_3d_launch.py',
        ]),
        (f'share/{package_name}/config', [
            'config/nav_params.yaml',
            'config/nav_params_3d.yaml',
        ]),
        (f'share/{package_name}/rviz', ['rviz/drone_nav.rviz']),
        (f'share/{package_name}/worlds', [
            'worlds/drone_world.wbt',
            'worlds/drone_world_hard.wbt',
            'worlds/drone_world_realistic.wbt',
            'worlds/drone_world_urban_3d.wbt',
            'worlds/drone_world_forest_3d.wbt',
            'worlds/drone_world_warehouse_3d.wbt',
            'worlds/drone_world_mixed_3d.wbt',
        ]),
        (f'share/{package_name}/controllers/moving_wall_controller', ['controllers/moving_wall_controller/moving_wall_controller.py']),
        (f'share/{package_name}/urdf', [
            'urdf/drone.urdf',
            'urdf/drone_3d.urdf',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robothon Team',
    maintainer_email='team@example.com',
    description='Autonomous 2D drone navigation in Webots using ROS2 Humble.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_publisher = drone_nav_2d.map_publisher:main',
            'path_planner = drone_nav_2d.path_planner:main',
            'drone_controller = drone_nav_2d.drone_controller:main',
            'obstacle_avoidance = drone_nav_2d.obstacle_avoidance:main',
            'metrics_logger = drone_nav_2d.metrics_logger:main',
            'dynamic_environment = drone_nav_2d.dynamic_environment:main',
            'advanced_metrics = drone_nav_2d.advanced_metrics:main',
            'drone_visualizer = drone_nav_2d.drone_visualizer:main',
            # 3D Navigation Stack
            'map_publisher_3d = drone_nav_2d.map_publisher_3d:main',
            'path_planner_3d = drone_nav_2d.path_planner_3d:main',
            'drone_controller_3d = drone_nav_2d.drone_controller_3d:main',
            'obstacle_avoidance_3d = drone_nav_2d.obstacle_avoidance_3d:main',
            'voxel_grid = drone_nav_2d.voxel_grid:main',
        ],
    },
)
