import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class DroneVisualizer(Node):
    def __init__(self) -> None:
        super().__init__('drone_visualizer')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate_hz', 20.0)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        self.pose_sub = self.create_subscription(PoseStamped, '/drone_pose', self._pose_cb, 20)
        self.marker_pub = self.create_publisher(MarkerArray, '/drone_model', 10)

        self.current_pose = None
        self.timer = self.create_timer(max(0.02, 1.0 / publish_rate), self._publish_model)
        self.get_logger().info('Drone visualizer initialized.')

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.current_pose = msg

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _create_marker(self, marker_id: int, marker_type: int, ns: str, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _publish_model(self) -> None:
        if self.current_pose is None:
            return

        now = self.get_clock().now().to_msg()
        pose = self.current_pose.pose
        yaw = self._yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )

        markers = MarkerArray()

        body = self._create_marker(0, Marker.CYLINDER, 'drone_body', now)
        body.pose = pose
        body.scale.x = 0.36
        body.scale.y = 0.36
        body.scale.z = 0.12
        body.color.r = 0.18
        body.color.g = 0.18
        body.color.b = 0.22
        body.color.a = 0.95
        markers.markers.append(body)

        canopy = self._create_marker(1, Marker.SPHERE, 'drone_canopy', now)
        canopy.pose = pose
        canopy.pose.position.z += 0.08
        canopy.scale.x = 0.22
        canopy.scale.y = 0.22
        canopy.scale.z = 0.10
        canopy.color.r = 0.05
        canopy.color.g = 0.35
        canopy.color.b = 0.85
        canopy.color.a = 0.95
        markers.markers.append(canopy)

        arm_offsets = [
            (0.22, 0.22),
            (-0.22, 0.22),
            (-0.22, -0.22),
            (0.22, -0.22),
        ]

        for index, (dx_local, dy_local) in enumerate(arm_offsets):
            dx = dx_local * math.cos(yaw) - dy_local * math.sin(yaw)
            dy = dx_local * math.sin(yaw) + dy_local * math.cos(yaw)

            rotor = self._create_marker(10 + index, Marker.CYLINDER, 'drone_rotors', now)
            rotor.pose = pose
            rotor.pose.position.x += dx
            rotor.pose.position.y += dy
            rotor.pose.position.z += 0.07
            rotor.scale.x = 0.14
            rotor.scale.y = 0.14
            rotor.scale.z = 0.02
            rotor.color.r = 0.92
            rotor.color.g = 0.92
            rotor.color.b = 0.92
            rotor.color.a = 0.9
            markers.markers.append(rotor)

        shadow = self._create_marker(30, Marker.CYLINDER, 'drone_shadow', now)
        shadow.pose.position.x = pose.position.x
        shadow.pose.position.y = pose.position.y
        shadow.pose.position.z = 0.01
        shadow.scale.x = 0.45
        shadow.scale.y = 0.45
        shadow.scale.z = 0.005
        shadow.color.r = 0.0
        shadow.color.g = 0.0
        shadow.color.b = 0.0
        shadow.color.a = 0.25
        markers.markers.append(shadow)

        self.marker_pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DroneVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
