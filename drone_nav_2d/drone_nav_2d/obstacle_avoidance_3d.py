import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool, Float32


class ObstacleAvoidance3D(Node):
    """3D obstacle avoidance for autonomous drones.
    
    Reads 3D lidar point cloud and computes repulsive forces
    to avoid collisions while navigating.
    """

    def __init__(self) -> None:
        super().__init__('obstacle_avoidance_3d')

        # Parameters
        self.declare_parameter('lidar_topic', '/webots/drone/scan')
        self.declare_parameter('obstacle_threshold_m', 0.6)
        self.declare_parameter('influence_distance_m', 1.5)
        self.declare_parameter('repulsive_gain', 1.0)
        self.declare_parameter('tangential_gain', 0.5)
        self.declare_parameter('max_avoidance_velocity', 1.0)
        self.declare_parameter('use_3d', True)

        lidar_topic = self.get_parameter('lidar_topic').value
        self.obstacle_threshold = float(self.get_parameter('obstacle_threshold_m').value)
        self.influence_distance = float(self.get_parameter('influence_distance_m').value)
        self.repulsive_gain = float(self.get_parameter('repulsive_gain').value)
        self.tangential_gain = float(self.get_parameter('tangential_gain').value)
        self.max_avoidance_velocity = float(self.get_parameter('max_avoidance_velocity').value)
        self.use_3d = bool(self.get_parameter('use_3d').value)

        # State
        self.last_scan: Optional[LaserScan] = None
        self.last_pointcloud: Optional[PointCloud2] = None
        self.obstacle_active = False
        self.min_distance = float('inf')

        # Subscriptions
        if lidar_topic.endswith('scan'):
            self.create_subscription(LaserScan, lidar_topic, self._on_scan, 10)
        elif lidar_topic.endswith('cloud'):
            self.create_subscription(PointCloud2, lidar_topic, self._on_pointcloud, 10)

        # Publishers
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected_3d', 10)
        self.avoidance_pub = self.create_publisher(Twist, '/avoidance_cmd_vel_3d', 10)
        self.replan_pub = self.create_publisher(Bool, '/replan_request_3d', 10)
        self.min_distance_pub = self.create_publisher(Float32, '/min_obstacle_distance_3d', 10)

        # Control loop
        self.timer = self.create_timer(0.05, self._tick)

        self.get_logger().info('3D Obstacle avoidance initialized.')

    def _on_scan(self, msg: LaserScan) -> None:
        """Receive 2D lidar scan."""
        self.last_scan = msg

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        """Receive 3D point cloud."""
        self.last_pointcloud = msg

    def _tick(self) -> None:
        """Control loop tick (50 Hz)."""
        if self.last_scan is None and self.last_pointcloud is None:
            self._publish_state(False, Twist(), float('inf'))
            return

        # Process sensor data
        min_dist = float('inf')
        detected = False

        # 2D Lidar processing
        if self.last_scan is not None:
            ranges = [r for r in self.last_scan.ranges if math.isfinite(r)]
            if ranges:
                min_dist = min(ranges)
                detected = min_dist < self.obstacle_threshold

        # 3D point cloud processing (if available)
        if self.last_pointcloud is not None and self.use_3d:
            # Would parse point cloud and compute distances
            # For now, simplified: use closest point
            pass

        # Compute avoidance command
        cmd = Twist()
        if detected:
            rep_x, rep_y, rep_z = self._compute_potential_field()
            cmd.linear.x = rep_x
            cmd.linear.y = rep_y
            cmd.linear.z = rep_z

            # Limit magnitude
            speed = math.sqrt(cmd.linear.x**2 + cmd.linear.y**2 + cmd.linear.z**2)
            if speed > self.max_avoidance_velocity and speed > 0.0:
                scale = self.max_avoidance_velocity / speed
                cmd.linear.x *= scale
                cmd.linear.y *= scale
                cmd.linear.z *= scale

        self._publish_state(detected, cmd, min_dist)

    def _compute_potential_field(self) -> Tuple[float, float, float]:
        """Compute 3D potential field from obstacles.
        
        Returns:
            (fx, fy, fz) repulsive force components
        """
        rep_x = 0.0
        rep_y = 0.0
        rep_z = 0.0

        if self.last_scan is not None:
            # 2D lidar to 3D (assume horizontal scan)
            for i, distance in enumerate(self.last_scan.ranges):
                if not math.isfinite(distance):
                    continue
                if distance > self.influence_distance:
                    continue

                # Compute bearing angle
                angle = self.last_scan.angle_min + i * self.last_scan.angle_increment

                # Repulsive strength (inverse distance falloff)
                strength = self.repulsive_gain * (
                    1.0 / max(distance, 0.05) - 1.0 / self.influence_distance
                )
                strength = max(0.0, strength)

                # Repulsive force (push away from obstacle)
                rep_x -= strength * math.cos(angle)
                rep_y -= strength * math.sin(angle)

                # Tangential force (encourage circular motion around obstacle)
                tangential = self.tangential_gain * strength
                rep_x += -tangential * math.sin(angle)
                rep_y += tangential * math.cos(angle)

        return rep_x, rep_y, rep_z

    def _publish_state(self, detected: bool, cmd: Twist, min_dist: float) -> None:
        """Publish obstacle detection and avoidance command."""
        # Obstacle detection
        obstacle_msg = Bool()
        obstacle_msg.data = detected
        self.obstacle_pub.publish(obstacle_msg)

        # Avoidance command
        self.avoidance_pub.publish(cmd)

        # Minimum distance
        dist_msg = Float32()
        dist_msg.data = float(min_dist if math.isfinite(min_dist) else 999.0)
        self.min_distance_pub.publish(dist_msg)
        self.min_distance = min_dist

        # Replan request (on new detection)
        if detected and not self.obstacle_active:
            repl = Bool()
            repl.data = True
            self.replan_pub.publish(repl)

        self.obstacle_active = detected


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleAvoidance3D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
