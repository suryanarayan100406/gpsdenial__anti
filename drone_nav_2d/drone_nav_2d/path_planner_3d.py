import heapq
import math
import random
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Bool

from .voxel_grid import VoxelGrid


VoxelCell = Tuple[int, int, int]


@dataclass
class Planner3DState:
    """State for 3D path planner."""
    voxel_grid: Optional[VoxelGrid] = None
    drone_radius_m: float = 0.2


class PathPlanner3D(Node):
    """3D A* path planner for autonomous drone navigation.
    
    Uses voxel-based grid and A* algorithm with 26-connectivity.
    Handles dynamic replanning when obstacles change or new goals are set.
    """

    def __init__(self) -> None:
        super().__init__('path_planner_3d')

        # Declare parameters
        self.declare_parameter('voxel_resolution', 0.2)
        self.declare_parameter('world_width_m', 10.0)
        self.declare_parameter('world_height_m', 8.0)
        self.declare_parameter('world_depth_m', 10.0)
        self.declare_parameter('inflation_radius_m', 0.4)
        self.declare_parameter('drone_radius_m', 0.2)
        self.declare_parameter('replan_rate_hz', 1.0)
        self.declare_parameter('start_xyz', [-4.0, 0.0, 1.0])
        self.declare_parameter('goal_xyz', [4.0, 0.0, 2.0])
        self.declare_parameter('rrt_max_iterations', 3000)
        self.declare_parameter('rrt_step_size_m', 0.4)
        self.declare_parameter('rrt_goal_sample_rate', 0.15)

        # Get parameters
        voxel_res = float(self.get_parameter('voxel_resolution').value)
        world_width = float(self.get_parameter('world_width_m').value)
        world_height = float(self.get_parameter('world_height_m').value)
        world_depth = float(self.get_parameter('world_depth_m').value)
        inflation_rad = float(self.get_parameter('inflation_radius_m').value)
        drone_rad = float(self.get_parameter('drone_radius_m').value)
        replan_rate = float(self.get_parameter('replan_rate_hz').value)
        self.start_xyz = tuple(self.get_parameter('start_xyz').value)
        self.goal_xyz = tuple(self.get_parameter('goal_xyz').value)

        # Initialize voxel grid
        self.state = Planner3DState()
        self.state.voxel_grid = VoxelGrid(
            width_m=world_width,
            height_m=world_height,
            depth_m=world_depth,
            resolution=voxel_res,
        )
        self.state.voxel_grid.set_inflation_radius(inflation_rad)
        self.state.drone_radius_m = drone_rad

        # Planning state
        self.current_pose: Optional[PoseStamped] = None
        self.replan_requested = True
        self.mission_completed = False
        self.last_path: List[Tuple[float, float, float]] = []
        self.last_start_cell: Optional[VoxelCell] = None
        self.last_goal_cell: Optional[VoxelCell] = None
        self.map_initialized = False

        # Publishers
        self.path_pub = self.create_publisher(Path, '/planned_path_3d', 10)
        self.replan_event_pub = self.create_publisher(Bool, '/replan_event_3d', 10)

        # Subscribers
        self.create_subscription(PoseStamped, '/drone_pose_3d', self._on_pose, 10)
        self.create_subscription(Bool, '/replan_request_3d', self._on_replan_request, 10)
        self.create_subscription(Bool, '/mission_complete_3d', self._on_mission_complete, 10)
        self.create_subscription(PoseStamped, '/goal_pose_3d', self._on_goal_pose, 10)
        self.create_subscription(PointStamped, '/clicked_point_3d', self._on_clicked_point, 10)

        # Timer for planning
        self.timer = self.create_timer(max(0.1, 1.0 / replan_rate), self._plan_if_needed)

        self.get_logger().info('3D Path planner initialized.')

    def _on_pose(self, msg: PoseStamped) -> None:
        """Update current drone pose."""
        self.current_pose = msg

    def _on_replan_request(self, msg: Bool) -> None:
        """Request replanning (e.g., obstacle detected)."""
        if msg.data:
            self.replan_requested = True

    def _on_mission_complete(self, msg: Bool) -> None:
        """Mark mission as complete."""
        if msg.data:
            self.mission_completed = True

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        """Receive new goal pose."""
        self.goal_xyz = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )
        self.replan_requested = True
        self.mission_completed = False
        self.get_logger().info(f'New goal: ({self.goal_xyz[0]:.2f}, {self.goal_xyz[1]:.2f}, {self.goal_xyz[2]:.2f})')

    def _on_clicked_point(self, msg: PointStamped) -> None:
        """Receive goal from RViz click."""
        self.goal_xyz = (msg.point.x, msg.point.y, msg.point.z)
        self.replan_requested = True
        self.mission_completed = False
        self.get_logger().info(f'New goal (RViz): ({self.goal_xyz[0]:.2f}, {self.goal_xyz[1]:.2f}, {self.goal_xyz[2]:.2f})')

    def _plan_if_needed(self) -> None:
        """Main planning loop: decide if replanning is needed and compute path."""
        if self.state.voxel_grid is None:
            return

        # Check if mission complete → skip planning
        if self.mission_completed and not self.replan_requested:
            return

        # Get current position
        if self.current_pose is not None:
            start_xyz = (
                self.current_pose.pose.position.x,
                self.current_pose.pose.position.y,
                self.current_pose.pose.position.z,
            )
        else:
            start_xyz = self.start_xyz

        # Convert to voxel coordinates
        start_cell = self.state.voxel_grid.world_to_voxel(*start_xyz)
        goal_cell = self.state.voxel_grid.world_to_voxel(*self.goal_xyz)

        # Check if replanning is necessary
        if (
            not self.replan_requested
            and self.last_goal_cell == goal_cell
            and self.last_start_cell is not None
            and self.last_start_cell == start_cell
        ):
            # No change in start/goal → skip
            return

        # Validate start/goal are free
        if not self.state.voxel_grid.is_free(*start_xyz, use_inflation=True):
            self.get_logger().warn('Start position blocked.')
            return

        if not self.state.voxel_grid.is_free(*self.goal_xyz, use_inflation=True):
            self.get_logger().warn('Goal position blocked.')
            return

        # Run A* planner
        path_voxels = self.a_star(start_cell, goal_cell)

        if not path_voxels:
            self.get_logger().warn('A* found no path; trying RRT...')
            path_voxels = self.rrt_3d(start_cell, goal_cell)

        if not path_voxels:
            self.get_logger().error('No valid path found.')
            return

        # Convert voxels to world coordinates
        path_world = [self.state.voxel_grid.voxel_to_world(*v) for v in path_voxels]

        # Publish path
        self.publish_path(path_world)
        self.last_path = path_world
        self.last_start_cell = start_cell
        self.last_goal_cell = goal_cell

        # Publish replan event
        event_msg = Bool()
        event_msg.data = True
        self.replan_event_pub.publish(event_msg)

        self.replan_requested = False

    def a_star(self, start: VoxelCell, goal: VoxelCell) -> List[VoxelCell]:
        """3D A* pathfinding using 26-connectivity (26-neighbor search).
        
        Args:
            start: Starting voxel (vx, vy, vz)
            goal: Goal voxel (vx, vy, vz)
            
        Returns:
            List of voxel coordinates from start to goal, or empty if no path
        """
        grid = self.state.voxel_grid

        open_heap: List[Tuple[float, VoxelCell]] = []
        heapq.heappush(open_heap, (0.0, start))

        came_from: Dict[VoxelCell, VoxelCell] = {}
        g_score: Dict[VoxelCell, float] = {start: 0.0}
        f_score: Dict[VoxelCell, float] = {start: self._heuristic_3d(start, goal)}
        open_set = {start}

        iterations = 0
        max_iterations = 100000

        while open_heap and iterations < max_iterations:
            iterations += 1
            _, current = heapq.heappop(open_heap)

            if current not in open_set:
                continue
            open_set.remove(current)

            if current == goal:
                return self._reconstruct_path(came_from, current)

            # Explore 26 neighbors
            for neighbor in grid.get_neighbors(*current, use_inflation=True):
                # Calculate step cost (varies by direction)
                step_cost = self._cost_3d(current, neighbor)
                tentative = g_score[current] + step_cost

                if tentative < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative
                    f_score[neighbor] = tentative + self._heuristic_3d(neighbor, goal)

                    if neighbor not in open_set:
                        open_set.add(neighbor)
                        heapq.heappush(open_heap, (f_score[neighbor], neighbor))

        return []  # No path found

    def rrt_3d(self, start: VoxelCell, goal: VoxelCell) -> List[VoxelCell]:
        """Rapidly-exploring Random Tree (RRT) planner for 3D space.
        
        Fallback when A* cannot find a path.
        
        Args:
            start: Starting voxel
            goal: Goal voxel
            
        Returns:
            List of voxel coordinates or empty if no path
        """
        max_iter = int(self.get_parameter('rrt_max_iterations').value)
        step_size_m = float(self.get_parameter('rrt_step_size_m').value)
        goal_sample_rate = float(self.get_parameter('rrt_goal_sample_rate').value)

        grid = self.state.voxel_grid
        step_cells = max(1, int(step_size_m / grid.resolution))

        tree: List[VoxelCell] = [start]
        parent: Dict[VoxelCell, Optional[VoxelCell]] = {start: None}

        for iteration in range(max_iter):
            # Sample random node or goal
            if random.random() < goal_sample_rate:
                sample = goal
            else:
                sample = (
                    random.randint(0, grid.width - 1),
                    random.randint(0, grid.depth - 1),
                    random.randint(0, grid.height - 1),
                )

            # Find nearest node in tree
            nearest = min(
                tree,
                key=lambda n: (n[0] - sample[0]) ** 2 + (n[1] - sample[1]) ** 2 + (n[2] - sample[2]) ** 2,
            )

            # Steer toward sample
            new_node = self._steer_3d(nearest, sample, step_cells)

            # Check validity
            if not grid.is_free(*grid.voxel_to_world(*new_node), use_inflation=True):
                continue
            
            if new_node in parent:
                continue

            tree.append(new_node)
            parent[new_node] = nearest

            # Check if goal reached
            if self._distance_3d(new_node, goal) <= step_cells:
                parent[goal] = new_node
                return self._rrt_reconstruct(parent, goal)

        return []  # No path found

    @staticmethod
    def _heuristic_3d(a: VoxelCell, b: VoxelCell) -> float:
        """3D Euclidean distance heuristic."""
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        dz = a[2] - b[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    @staticmethod
    def _cost_3d(a: VoxelCell, b: VoxelCell) -> float:
        """Calculate movement cost between two voxels (26-connectivity).
        
        Cardinal (6 directions): 1.0
        Face diagonal (12 directions): sqrt(2) ≈ 1.414
        Space diagonal (8 directions): sqrt(3) ≈ 1.732
        """
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        dz = abs(a[2] - b[2])
        
        # Count number of non-zero differences
        non_zero = (dx > 0) + (dy > 0) + (dz > 0)
        
        if non_zero == 1:
            return 1.0  # Cardinal
        elif non_zero == 2:
            return math.sqrt(2)  # Face diagonal
        else:  # non_zero == 3
            return math.sqrt(3)  # Space diagonal

    @staticmethod
    def _distance_3d(a: VoxelCell, b: VoxelCell) -> float:
        """3D Euclidean distance."""
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)

    def _steer_3d(self, from_node: VoxelCell, to_node: VoxelCell, step_cells: int) -> VoxelCell:
        """Steer from one node toward another by step_cells voxels."""
        dx = to_node[0] - from_node[0]
        dy = to_node[1] - from_node[1]
        dz = to_node[2] - from_node[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist <= step_cells:
            return to_node

        nx = int(round(from_node[0] + dx / dist * step_cells))
        ny = int(round(from_node[1] + dy / dist * step_cells))
        nz = int(round(from_node[2] + dz / dist * step_cells))

        grid = self.state.voxel_grid
        nx = max(0, min(grid.width - 1, nx))
        ny = max(0, min(grid.depth - 1, ny))
        nz = max(0, min(grid.height - 1, nz))

        return nx, ny, nz

    @staticmethod
    def _reconstruct_path(came_from: Dict[VoxelCell, VoxelCell], current: VoxelCell) -> List[VoxelCell]:
        """Reconstruct path from came_from map."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    @staticmethod
    def _rrt_reconstruct(parent: Dict[VoxelCell, Optional[VoxelCell]], goal: VoxelCell) -> List[VoxelCell]:
        """Reconstruct RRT path from parent map."""
        path = [goal]
        current = goal
        while parent[current] is not None:
            current = parent[current]
            path.append(current)
        path.reverse()
        return path

    def publish_path(self, points: List[Tuple[float, float, float]]) -> None:
        """Publish path as ROS message."""
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        for x, y, z in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(z)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.path_pub.publish(msg)
        self.get_logger().info(f'Published 3D path with {len(msg.poses)} waypoints.')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathPlanner3D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
