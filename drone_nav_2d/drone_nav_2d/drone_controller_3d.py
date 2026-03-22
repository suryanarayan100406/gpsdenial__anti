import math
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion


@dataclass
class PIDState:
    """PID controller state with anti-windup."""
    kp: float
    ki: float
    kd: float
    integral_limit: float
    integral: float = 0.0
    prev_error: float = 0.0

    def reset(self) -> None:
        """Reset integral and derivative states."""
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error: float, dt: float) -> float:
        """Update PID loop and return control output.
        
        Args:
            error: Current error signal
            dt: Time since last update (seconds)
            
        Returns:
            Control output
        """
        if dt <= 0.0:
            return self.kp * error

        # Accumulate integral with anti-windup (clamp)
        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))

        # Compute derivative
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        # PID output
        return self.kp * error + self.ki * self.integral + self.kd * derivative


@dataclass
class QuadrotorState:
    """Physical state of quadrotor (6-DoF)."""
    # Position (m)
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    # Velocity (m/s)
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0

    # Euler angles (rad): roll (φ), pitch (θ), yaw (ψ)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    # Angular velocity (rad/s)
    roll_rate: float = 0.0
    pitch_rate: float = 0.0
    yaw_rate: float = 0.0


class DroneController3D(Node):
    """6-DoF quadrotor controller with cascaded PID loops.
    
    Control hierarchy:
    1. Position PID (outer loop): desired position → desired velocity
    2. Velocity PID (middle loop): desired velocity → desired attitude
    3. Attitude PID (inner loop): desired attitude → motor thrust
    
    This mimics real quadrotor controllers (e.g., PX4, Betaflight).
    """

    def __init__(self) -> None:
        super().__init__('drone_controller_3d')

        # Parameters
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('waypoint_tolerance_m', 0.2)
        self.declare_parameter('max_velocity_xy_ms', 2.0)
        self.declare_parameter('max_velocity_z_ms', 1.5)
        self.declare_parameter('max_acceleration_mss', 5.0)
        self.declare_parameter('max_roll_rad', 0.785)  # 45 degrees
        self.declare_parameter('max_pitch_rad', 0.785)
        self.declare_parameter('max_yaw_rate_rads', 3.14)
        self.declare_parameter('gravity_ms2', 9.81)
        self.declare_parameter('mass_kg', 1.8)

        # Position PID gains
        self.declare_parameter('kp_x', 1.5)
        self.declare_parameter('ki_x', 0.05)
        self.declare_parameter('kd_x', 0.3)
        self.declare_parameter('kp_y', 1.5)
        self.declare_parameter('ki_y', 0.05)
        self.declare_parameter('kd_y', 0.3)
        self.declare_parameter('kp_z', 2.0)
        self.declare_parameter('ki_z', 0.08)
        self.declare_parameter('kd_z', 0.4)

        # Velocity PID gains
        self.declare_parameter('kp_vx', 1.2)
        self.declare_parameter('ki_vx', 0.04)
        self.declare_parameter('kd_vx', 0.2)
        self.declare_parameter('kp_vy', 1.2)
        self.declare_parameter('ki_vy', 0.04)
        self.declare_parameter('kd_vy', 0.2)
        self.declare_parameter('kp_vz', 1.5)
        self.declare_parameter('ki_vz', 0.05)
        self.declare_parameter('kd_vz', 0.3)

        # Attitude PID gains
        self.declare_parameter('kp_roll', 4.5)
        self.declare_parameter('kd_roll', 0.15)
        self.declare_parameter('kp_pitch', 4.5)
        self.declare_parameter('kd_pitch', 0.15)
        self.declare_parameter('kp_yaw', 4.0)
        self.declare_parameter('ki_yaw', 0.02)
        self.declare_parameter('kd_yaw', 0.1)

        # Get parameters
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance_m').value)
        self.max_vel_xy = float(self.get_parameter('max_velocity_xy_ms').value)
        self.max_vel_z = float(self.get_parameter('max_velocity_z_ms').value)
        self.max_accel = float(self.get_parameter('max_acceleration_mss').value)
        self.max_roll = float(self.get_parameter('max_roll_rad').value)
        self.max_pitch = float(self.get_parameter('max_pitch_rad').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate_rads').value)
        self.gravity = float(self.get_parameter('gravity_ms2').value)
        self.mass = float(self.get_parameter('mass_kg').value)

        # Initialize PID controllers (position)
        self.pid_x = PIDState(
            kp=float(self.get_parameter('kp_x').value),
            ki=float(self.get_parameter('ki_x').value),
            kd=float(self.get_parameter('kd_x').value),
            integral_limit=2.0,
        )
        self.pid_y = PIDState(
            kp=float(self.get_parameter('kp_y').value),
            ki=float(self.get_parameter('ki_y').value),
            kd=float(self.get_parameter('kd_y').value),
            integral_limit=2.0,
        )
        self.pid_z = PIDState(
            kp=float(self.get_parameter('kp_z').value),
            ki=float(self.get_parameter('ki_z').value),
            kd=float(self.get_parameter('kd_z').value),
            integral_limit=2.0,
        )

        # Velocity PIDs
        self.pid_vx = PIDState(
            kp=float(self.get_parameter('kp_vx').value),
            ki=float(self.get_parameter('ki_vx').value),
            kd=float(self.get_parameter('kd_vx').value),
            integral_limit=0.5,
        )
        self.pid_vy = PIDState(
            kp=float(self.get_parameter('kp_vy').value),
            ki=float(self.get_parameter('ki_vy').value),
            kd=float(self.get_parameter('kd_vy').value),
            integral_limit=0.5,
        )
        self.pid_vz = PIDState(
            kp=float(self.get_parameter('kp_vz').value),
            ki=float(self.get_parameter('ki_vz').value),
            kd=float(self.get_parameter('kd_vz').value),
            integral_limit=0.5,
        )

        # Attitude PIDs
        self.pid_roll = PIDState(
            kp=float(self.get_parameter('kp_roll').value),
            ki=0.0,  # Usually no integral for attitude
            kd=float(self.get_parameter('kd_roll').value),
            integral_limit=0.0,
        )
        self.pid_pitch = PIDState(
            kp=float(self.get_parameter('kp_pitch').value),
            ki=0.0,
            kd=float(self.get_parameter('kd_pitch').value),
            integral_limit=0.0,
        )
        self.pid_yaw = PIDState(
            kp=float(self.get_parameter('kp_yaw').value),
            ki=float(self.get_parameter('ki_yaw').value),
            kd=float(self.get_parameter('kd_yaw').value),
            integral_limit=0.3,
        )

        # State
        self.current_pose: Optional[PoseStamped] = None
        self.state = QuadrotorState()
        self.path_points: List[PoseStamped] = []
        self.current_index = 0
        self.goal_reached = False

        self.obstacle_override = False
        self.avoidance_cmd = Twist()

        self.last_control_time = self.get_clock().now()
        
        # Pose history for velocity calculation (3-pose buffer)
        self.pose_history: List[tuple] = []

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_3d', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/drone_pose_3d', 10)
        self.traj_pub = self.create_publisher(Path, '/drone_trajectory_3d', 10)
        self.mission_complete_pub = self.create_publisher(Bool, '/mission_complete_3d', 10)

        # Subscribers
        self.create_subscription(Path, '/planned_path_3d', self._on_path, 10)
        self.create_subscription(PoseStamped, '/webots/drone/pose', self._on_pose, 10)
        self.create_subscription(Twist, '/avoidance_cmd_vel_3d', self._on_avoidance_cmd, 10)
        self.create_subscription(Bool, '/obstacle_detected_3d', self._on_obstacle, 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.traj_msg = Path()
        self.traj_msg.header.frame_id = 'map'

        # Control loop timer
        self.timer = self.create_timer(max(0.02, 1.0 / self.control_rate_hz), self._control_loop)

        self.get_logger().info('6-DoF Drone controller 3D initialized.')

    def _on_path(self, msg: Path) -> None:
        """Receive planned path from path planner."""
        self.path_points = msg.poses
        self.current_index = 0
        self.goal_reached = False

        # Reset PIDs when new path received
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
        self.pid_vx.reset()
        self.pid_vy.reset()
        self.pid_vz.reset()

        self.get_logger().info(f'Received 3D path with {len(self.path_points)} waypoints.')

    def _on_pose(self, msg: PoseStamped) -> None:
        """Update drone pose from Webots."""
        self.current_pose = msg
        # Extract position and orientation
        self.state.x = msg.pose.position.x
        self.state.y = msg.pose.position.y
        self.state.z = msg.pose.position.z
        
        # Extract Euler angles from quaternion
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        roll, pitch, yaw = euler_from_quaternion([qx, qy, qz, qw])
        self.state.roll = roll
        self.state.pitch = pitch
        self.state.yaw = yaw
        
        # Calculate velocity from pose history
        now = self.get_clock().now()
        self.pose_history.append((msg, now))
        if len(self.pose_history) > 3:
            self.pose_history.pop(0)
        
        if len(self.pose_history) >= 2:
            prev_pose, prev_time = self.pose_history[-2]
            dt_vel = (now - prev_time).nanoseconds / 1e9
            if dt_vel > 0.001:  # More than 1ms
                self.state.vx = (msg.pose.position.x - prev_pose.pose.position.x) / dt_vel
                self.state.vy = (msg.pose.position.y - prev_pose.pose.position.y) / dt_vel
                self.state.vz = (msg.pose.position.z - prev_pose.pose.position.z) / dt_vel

    def _on_avoidance_cmd(self, msg: Twist) -> None:
        """Receive avoidance command from obstacle avoidance node."""
        self.avoidance_cmd = msg

    def _on_obstacle(self, msg: Bool) -> None:
        """Receive obstacle detection."""
        self.obstacle_override = msg.data

    def _control_loop(self) -> None:
        """Main control loop (20 Hz)."""
        now = self.get_clock().now()
        dt = (now - self.last_control_time).nanoseconds / 1e9
        self.last_control_time = now

        if dt <= 0.0:
            return

        # Use pose from Webots or fallback to simulated state
        if self.current_pose is not None:
            pose_msg = self.current_pose
        else:
            # Create default pose
            pose_msg = PoseStamped()
            pose_msg.header.stamp = now.to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position.x = self.state.x
            pose_msg.pose.position.y = self.state.y
            pose_msg.pose.position.z = self.state.z
            pose_msg.pose.orientation.w = 1.0

        # Publish current pose and TF
        self.pose_pub.publish(pose_msg)
        self._publish_tf(pose_msg)

        # Update trajectory
        self._update_trajectory(pose_msg)

        # Check if goal reached
        if self.goal_reached or not self.path_points:
            self._publish_stop()
            return

        if self.current_index >= len(self.path_points):
            self._mark_goal_reached()
            self._publish_stop()
            return

        # === CONTROL HIERARCHY ===

        # 1. POSITION CONTROL (outer loop)
        target = self.path_points[self.current_index].pose.position
        current = pose_msg.pose.position

        err_x = target.x - current.x
        err_y = target.y - current.y
        err_z = target.z - current.z

        # Check waypoint proximity
        dist_xy = math.hypot(err_x, err_y)
        if dist_xy <= self.waypoint_tolerance and abs(err_z) <= self.waypoint_tolerance:
            self.current_index += 1
            if self.current_index >= len(self.path_points):
                self._mark_goal_reached()
            return

        # Position PID → Desired velocity
        des_vx = self.pid_x.update(err_x, dt)
        des_vy = self.pid_y.update(err_y, dt)
        des_vz = self.pid_z.update(err_z, dt)

        # Limit desired velocity
        des_vx = max(-self.max_vel_xy, min(self.max_vel_xy, des_vx))
        des_vy = max(-self.max_vel_xy, min(self.max_vel_xy, des_vy))
        des_vz = max(-self.max_vel_z, min(self.max_vel_z, des_vz))

        # 2. VELOCITY CONTROL (middle loop)
        # Get current velocity (rough estimate from pose rate)
        curr_vx = self.state.vx
        curr_vy = self.state.vy
        curr_vz = self.state.vz

        vel_err_x = des_vx - curr_vx
        vel_err_y = des_vy - curr_vy
        vel_err_z = des_vz - curr_vz

        # Velocity PID → Desired acceleration
        des_ax = self.pid_vx.update(vel_err_x, dt)
        des_ay = self.pid_vy.update(vel_err_y, dt)
        des_az = self.pid_vz.update(vel_err_z, dt)

        # Add gravity compensation
        des_az += self.gravity

        # 3. ATTITUDE MAPPING (translate acceleration to attitude)
        # For small angles: roll ≈ ay/g, pitch ≈ -ax/g
        des_roll = des_ay / self.gravity
        des_pitch = -des_ax / self.gravity
        des_roll = max(-self.max_roll, min(self.max_roll, des_roll))
        des_pitch = max(-self.max_pitch, min(self.max_pitch, des_pitch))

        # 4. ATTITUDE CONTROL (inner loop)
        roll_err = des_roll - self.state.roll
        pitch_err = des_pitch - self.state.pitch
        yaw_err = 0  # Keep yaw fixed for now

        roll_control = self.pid_roll.update(roll_err, dt)
        pitch_control = self.pid_pitch.update(pitch_err, dt)
        yaw_control = self.pid_yaw.update(yaw_err, dt)

        # Compute motor thrust
        # Simplified: total thrust for altitude, differential for attitude
        thrust_total = des_az * self.mass / 4.0  # Divide by 4 motors

        # Motor outputs (PWM-like 0-1 scale)
        motor_1 = thrust_total + roll_control - pitch_control + yaw_control
        motor_2 = thrust_total - roll_control - pitch_control - yaw_control
        motor_3 = thrust_total - roll_control + pitch_control + yaw_control
        motor_4 = thrust_total + roll_control + pitch_control - yaw_control

        # Clamp and normalize motors
        motors = [motor_1, motor_2, motor_3, motor_4]
        min_motor = min(motors)
        max_motor = max(motors)

        if max_motor > 1.0:
            scale = 1.0 / max_motor
            motors = [m * scale for m in motors]

        if min_motor < 0.0:
            # Shift all motors up
            shift = -min_motor
            motors = [m + shift for m in motors]

        # Publish command as Twist (simplified for Webots)
        cmd = Twist()
        cmd.linear.x = des_vx
        cmd.linear.y = des_vy
        cmd.linear.z = des_vz
        cmd.angular.x = des_roll * 10  # Roll rate scaling
        cmd.angular.y = des_pitch * 10
        cmd.angular.z = des_az * 2  # Simplified yaw

        self.cmd_pub.publish(cmd)

    def _mark_goal_reached(self) -> None:
        """Signal mission complete."""
        if self.goal_reached:
            return
        
        self.goal_reached = True
        msg = Bool()
        msg.data = True
        self.mission_complete_pub.publish(msg)
        self.get_logger().info('Mission complete: goal reached (3D).')

    def _publish_stop(self) -> None:
        """Publish zero velocity command."""
        self.cmd_pub.publish(Twist())

    def _update_trajectory(self, pose_msg: PoseStamped) -> None:
        """Append current pose to trajectory and publish."""
        pose_copy = PoseStamped()
        pose_copy.header = pose_msg.header
        pose_copy.pose = pose_msg.pose
        self.traj_msg.header.stamp = pose_msg.header.stamp
        self.traj_msg.poses.append(pose_copy)
        
        # Trim trajectory history to prevent unbounded growth (keep last 500 poses)
        if len(self.traj_msg.poses) > 500:
            self.traj_msg.poses = self.traj_msg.poses[-500:]
        
        self.traj_pub.publish(self.traj_msg)

    def _publish_tf(self, pose_msg: PoseStamped) -> None:
        """Broadcast drone TF frame."""
        t = TransformStamped()
        t.header.stamp = pose_msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'drone_base_3d'
        t.transform.translation.x = pose_msg.pose.position.x
        t.transform.translation.y = pose_msg.pose.position.y
        t.transform.translation.z = pose_msg.pose.position.z
        t.transform.rotation = pose_msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DroneController3D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
