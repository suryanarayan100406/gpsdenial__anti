import math

import numpy as np
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from .voxel_grid import VoxelGrid


class MapPublisher3D(Node):
    """Publishes 3D voxel grid as visualization markers for RViz.
    
    Shows obstacles as cubes in 3D space.
    """

    def __init__(self) -> None:
        super().__init__('map_publisher_3d')

        # Parameters
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('resolution', 0.2)
        self.declare_parameter('world_width_m', 10.0)
        self.declare_parameter('world_height_m', 8.0)
        self.declare_parameter('world_depth_m', 10.0)
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('inflation_radius_m', 0.4)

        frame_id = self.get_parameter('frame_id').value
        resolution = float(self.get_parameter('resolution').value)
        width = float(self.get_parameter('world_width_m').value)
        height = float(self.get_parameter('world_height_m').value)
        depth = float(self.get_parameter('world_depth_m').value)
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        inflation_rad = float(self.get_parameter('inflation_radius_m').value)

        # Initialize voxel grid
        self.voxel_grid = VoxelGrid(
            width_m=width,
            height_m=height,
            depth_m=depth,
            resolution=resolution,
        )
        self.voxel_grid.set_inflation_radius(inflation_rad)

        # Populate with sample obstacles
        self._populate_obstacles()

        # Publisher
        self.marker_pub = self.create_publisher(MarkerArray, '/obstacles_3d', 10)

        # Timer
        self.publish_timer = self.create_timer(max(0.05, 1.0 / publish_rate), self._publish_obstacles)

        self.frame_id = frame_id
        self.get_logger().info('3D Map publisher initialized.')

    def _populate_obstacles(self) -> None:
        """Add sample obstacles to voxel grid."""
        # Spheres (trees, poles)
        self.voxel_grid.paint_sphere(0.0, 0.0, 1.5, 0.4)
        self.voxel_grid.paint_sphere(2.0, 2.0, 1.0, 0.3)
        self.voxel_grid.paint_sphere(-2.0, -1.5, 1.2, 0.35)
        self.voxel_grid.paint_sphere(1.5, -2.5, 1.5, 0.3)

        # Boxes (buildings, obstacles)
        self.voxel_grid.paint_box(-3.0, -2.0, 1.0, 1.5, 1.0, 2.0)
        self.voxel_grid.paint_box(3.0, 1.5, 0.8, 1.2, 1.5, 1.8)
        self.voxel_grid.paint_box(-1.0, 3.0, 1.2, 1.0, 0.8, 2.5)

        # Cylinders (trees)
        self.voxel_grid.paint_cylinder(0.5, -1.0, 2.0, 0.25, 3.0)
        self.voxel_grid.paint_cylinder(-2.5, 2.0, 1.8, 0.2, 2.5)

    def _publish_obstacles(self) -> None:
        """Publish obstacle voxels as RViz markers."""
        msg = MarkerArray()

        # Get occupied voxels
        occupied = self.voxel_grid.get_occupied_voxels()

        # Create one marker per voxel (simplified - could be optimized with mesh)
        marker_id = 0
        for vz, vy, vx in occupied[::4]:  # Sample every 4th voxel to reduce marker count
            x, y, z = self.voxel_grid.voxel_to_world(vx, vy, vz)

            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'obstacles'
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = float(z)
            marker.pose.orientation.w = 1.0

            marker.scale.x = self.voxel_grid.resolution
            marker.scale.y = self.voxel_grid.resolution
            marker.scale.z = self.voxel_grid.resolution

            marker.color.r = 0.8
            marker.color.g = 0.2
            marker.color.b = 0.2
            marker.color.a = 0.6

            msg.markers.append(marker)
            marker_id += 1

        self.marker_pub.publish(msg)
        self.get_logger().debug(f'Published {len(msg.markers)} obstacle markers')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapPublisher3D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
