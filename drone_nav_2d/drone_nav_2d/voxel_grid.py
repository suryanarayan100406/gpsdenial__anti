import math
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from scipy.ndimage import binary_dilation


class VoxelGrid:
    """3D occupancy voxel grid for navigation planning.
    
    Represents a 3D space as a grid of voxels (3D pixels).
    Each voxel is either occupied (obstacle) or free (navigable).
    """

    def __init__(
        self,
        width_m: float = 10.0,
        height_m: float = 10.0,
        depth_m: float = 5.0,
        resolution: float = 0.2,
    ) -> None:
        """Initialize voxel grid.
        
        Args:
            width_m: X dimension in meters
            height_m: Z dimension (vertical) in meters
            depth_m: Y dimension in meters
            resolution: Voxel size in meters (0.2m = 20cm cubes)
        """
        self.width_m = width_m
        self.height_m = height_m
        self.depth_m = depth_m
        self.resolution = resolution

        # Grid dimensions in voxels
        self.width = int(math.ceil(width_m / resolution))
        self.depth = int(math.ceil(depth_m / resolution))
        self.height = int(math.ceil(height_m / resolution))

        # Origin (world coordinates of voxel (0, 0, 0))
        self.origin_x = -width_m / 2.0
        self.origin_y = -depth_m / 2.0
        self.origin_z = 0.0

        # Occupancy grid: True = occupied, False = free
        # Shape: (height, depth, width) → [z, y, x]
        self.grid = np.zeros((self.height, self.depth, self.width), dtype=bool)

        # Cached inflated grid (with safety margins)
        self.inflated_grid: Optional[np.ndarray] = None
        self.inflation_radius_m: float = 0.4

    def world_to_voxel(self, x: float, y: float, z: float) -> Tuple[int, int, int]:
        """Convert world coordinates to voxel indices.
        
        Args:
            x: Position along width (forward)
            y: Position along depth (side)
            z: Position along height (vertical)
            
        Returns:
            (vx, vy, vz) voxel indices, clamped to grid bounds
        """
        vx = int((x - self.origin_x) / self.resolution)
        vy = int((y - self.origin_y) / self.resolution)
        vz = int((z - self.origin_z) / self.resolution)

        # Clamp to valid range
        vx = max(0, min(self.width - 1, vx))
        vy = max(0, min(self.depth - 1, vy))
        vz = max(0, min(self.height - 1, vz))

        return vx, vy, vz

    def voxel_to_world(self, vx: int, vy: int, vz: int) -> Tuple[float, float, float]:
        """Convert voxel indices to world coordinates (center of voxel).
        
        Args:
            vx, vy, vz: Voxel indices
            
        Returns:
            (x, y, z) world coordinates at voxel center
        """
        x = self.origin_x + (vx + 0.5) * self.resolution
        y = self.origin_y + (vy + 0.5) * self.resolution
        z = self.origin_z + (vz + 0.5) * self.resolution
        return x, y, z

    def set_occupied(self, x: float, y: float, z: float) -> None:
        """Mark a world position as occupied."""
        vx, vy, vz = self.world_to_voxel(x, y, z)
        self.grid[vz, vy, vx] = True
        self.inflated_grid = None  # Invalidate cache

    def set_free(self, x: float, y: float, z: float) -> None:
        """Mark a world position as free."""
        vx, vy, vz = self.world_to_voxel(x, y, z)
        self.grid[vz, vy, vx] = False
        self.inflated_grid = None  # Invalidate cache

    def is_occupied(self, x: float, y: float, z: float) -> bool:
        """Check if a world position is occupied."""
        vx, vy, vz = self.world_to_voxel(x, y, z)
        return bool(self.grid[vz, vy, vx])

    def is_free(self, x: float, y: float, z: float, use_inflation: bool = True) -> bool:
        """Check if a world position is free (navigable).
        
        Args:
            x, y, z: World coordinates
            use_inflation: If True, check inflated grid (with safety margin)
            
        Returns:
            True if position is free (not occupied)
        """
        vx, vy, vz = self.world_to_voxel(x, y, z)

        if use_inflation:
            if self.inflated_grid is None:
                self._compute_inflation()
            return not bool(self.inflated_grid[vz, vy, vx])
        else:
            return not bool(self.grid[vz, vy, vx])

    def _compute_inflation(self) -> None:
        """Compute inflated grid with safety margin.
        
        Uses morphological dilation to expand obstacles by inflation_radius.
        This creates a safety buffer around obstacles.
        """
        inflation_voxels = max(1, int(math.ceil(self.inflation_radius_m / self.resolution)))

        # Create dilation kernel (cubic neighborhood)
        kernel_size = inflation_voxels * 2 + 1
        kernel = np.ones((kernel_size, kernel_size, kernel_size), dtype=bool)

        # Dilate obstacles
        self.inflated_grid = binary_dilation(self.grid, structure=kernel)

    def set_inflation_radius(self, radius_m: float) -> None:
        """Update inflation radius and invalidate cache."""
        self.inflation_radius_m = radius_m
        self.inflated_grid = None

    def paint_sphere(self, cx: float, cy: float, cz: float, radius: float) -> None:
        """Paint a spherical obstacle in the voxel grid.
        
        Args:
            cx, cy, cz: Center in world coordinates
            radius: Radius in meters
        """
        # Convert to voxel space
        cvx, cvy, cvz = self.world_to_voxel(cx, cy, cz)
        radius_voxels = max(1, int(math.ceil(radius / self.resolution)))

        # Create sphere mask
        y_idxs, x_idxs, z_idxs = np.ogrid[
            -radius_voxels : radius_voxels + 1,
            -radius_voxels : radius_voxels + 1,
            -radius_voxels : radius_voxels + 1,
        ]
        mask = x_idxs**2 + y_idxs**2 + z_idxs**2 <= radius_voxels**2

        # Apply mask to grid
        z_min = max(0, cvz - radius_voxels)
        z_max = min(self.height, cvz + radius_voxels + 1)
        y_min = max(0, cvy - radius_voxels)
        y_max = min(self.depth, cvy + radius_voxels + 1)
        x_min = max(0, cvx - radius_voxels)
        x_max = min(self.width, cvx + radius_voxels + 1)

        sub = self.grid[z_min:z_max, y_min:y_max, x_min:x_max]
        m = mask[
            (z_min - (cvz - radius_voxels)) : (z_max - (cvz - radius_voxels)),
            (y_min - (cvy - radius_voxels)) : (y_max - (cvy - radius_voxels)),
            (x_min - (cvx - radius_voxels)) : (x_max - (cvx - radius_voxels)),
        ]
        sub[m] = True
        self.inflated_grid = None

    def paint_box(
        self,
        cx: float,
        cy: float,
        cz: float,
        sx: float,
        sy: float,
        sz: float,
    ) -> None:
        """Paint a rectangular box obstacle.
        
        Args:
            cx, cy, cz: Center in world coordinates
            sx, sy, sz: Dimensions in meters
        """
        min_x, min_y, min_z = self.world_to_voxel(
            cx - sx / 2.0, cy - sy / 2.0, cz - sz / 2.0
        )
        max_x, max_y, max_z = self.world_to_voxel(
            cx + sx / 2.0, cy + sy / 2.0, cz + sz / 2.0
        )

        x0, x1 = sorted((min_x, max_x))
        y0, y1 = sorted((min_y, max_y))
        z0, z1 = sorted((min_z, max_z))

        self.grid[z0 : z1 + 1, y0 : y1 + 1, x0 : x1 + 1] = True
        self.inflated_grid = None

    def paint_cylinder(
        self,
        cx: float,
        cy: float,
        cz: float,
        radius: float,
        height: float,
    ) -> None:
        """Paint a vertical cylindrical obstacle (like a tree or pole).
        
        Args:
            cx, cy, cz: Center in world coordinates
            radius: Radius in X-Y plane
            height: Height in Z dimension
        """
        cvx, cvy, cvz = self.world_to_voxel(cx, cy, cz)
        radius_voxels = max(1, int(math.ceil(radius / self.resolution)))
        height_voxels = max(1, int(math.ceil(height / self.resolution)))

        # Create circular mask in X-Y plane
        y_idxs, x_idxs = np.ogrid[
            -radius_voxels : radius_voxels + 1,
            -radius_voxels : radius_voxels + 1,
        ]
        xy_mask = x_idxs**2 + y_idxs**2 <= radius_voxels**2

        # Apply to Z range
        z_min = max(0, cvz)
        z_max = min(self.height, cvz + height_voxels)
        y_min = max(0, cvy - radius_voxels)
        y_max = min(self.depth, cvy + radius_voxels + 1)
        x_min = max(0, cvx - radius_voxels)
        x_max = min(self.width, cvx + radius_voxels + 1)

        for z in range(z_min, z_max):
            sub = self.grid[z, y_min:y_max, x_min:x_max]
            m = xy_mask[
                (y_min - (cvy - radius_voxels)) : (y_max - (cvy - radius_voxels)),
                (x_min - (cvx - radius_voxels)) : (x_max - (cvx - radius_voxels)),
            ]
            sub[m] = True

        self.inflated_grid = None

    def get_neighbors(
        self,
        vx: int,
        vy: int,
        vz: int,
        use_inflation: bool = True,
    ) -> List[Tuple[int, int, int]]:
        """Get valid neighboring voxels (26-connectivity).
        
        8 cardinal directions + 12 face diagonals + 6 corner voxels = 26 neighbors
        
        Args:
            vx, vy, vz: Current voxel position
            use_inflation: Check against inflated grid
            
        Returns:
            List of (vx, vy, vz) tuples for valid neighbors
        """
        grid = self.inflated_grid if use_inflation and self.inflated_grid is not None else self.grid
        neighbors = []

        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue  # Skip self

                    nvx, nvy, nvz = vx + dx, vy + dy, vz + dz

                    # Check bounds
                    if nvx < 0 or nvx >= self.width:
                        continue
                    if nvy < 0 or nvy >= self.depth:
                        continue
                    if nvz < 0 or nvz >= self.height:
                        continue

                    # Check occupation
                    if grid[nvz, nvy, nvx]:
                        continue  # Occupied

                    neighbors.append((nvx, nvy, nvz))

        return neighbors

    def line_of_sight(
        self,
        x1: float,
        y1: float,
        z1: float,
        x2: float,
        y2: float,
        z2: float,
        use_inflation: bool = True,
    ) -> bool:
        """Check if a straight line between two points is collision-free.
        
        Uses Bresenham-like voxel traversal.
        
        Args:
            x1, y1, z1: Start position (world)
            x2, y2, z2: End position (world)
            use_inflation: Check against inflated grid
            
        Returns:
            True if line is free, False if collides
        """
        vx1, vy1, vz1 = self.world_to_voxel(x1, y1, z1)
        vx2, vy2, vz2 = self.world_to_voxel(x2, y2, z2)

        grid = self.inflated_grid if use_inflation and self.inflated_grid is not None else self.grid

        # Bresenham-like 3D traversal
        steps = max(abs(vx2 - vx1), abs(vy2 - vy1), abs(vz2 - vz1))
        if steps == 0:
            steps = 1

        for i in range(steps + 1):
            t = i / steps
            vx = int(round(vx1 + (vx2 - vx1) * t))
            vy = int(round(vy1 + (vy2 - vy1) * t))
            vz = int(round(vz1 + (vz2 - vz1) * t))

            if grid[vz, vy, vx]:
                return False  # Collision

        return True

    def get_occupied_voxels(self) -> np.ndarray:
        """Get array of occupied voxel coordinates for visualization.
        
        Returns:
            Array of shape (N, 3) with voxel indices
        """
        occupied = np.where(self.grid)
        return np.column_stack(occupied)  # (z, y, x) → Nx3


class VoxelGridPublisher(Node):
    """ROS2 node for managing 3D voxel grid (similar to map_publisher but 3D)."""

    def __init__(self) -> None:
        super().__init__('voxel_grid_publisher')

        # Parameters
        self.declare_parameter('width_m', 10.0)
        self.declare_parameter('height_m', 10.0)
        self.declare_parameter('depth_m', 10.0)
        self.declare_parameter('resolution', 0.2)
        self.declare_parameter('inflation_radius_m', 0.4)

        width_m = float(self.get_parameter('width_m').value)
        height_m = float(self.get_parameter('height_m').value)
        depth_m = float(self.get_parameter('depth_m').value)
        resolution = float(self.get_parameter('resolution').value)
        inflation_radius = float(self.get_parameter('inflation_radius_m').value)

        # Initialize voxel grid
        self.voxel_grid = VoxelGrid(
            width_m=width_m,
            height_m=height_m,
            depth_m=depth_m,
            resolution=resolution,
        )
        self.voxel_grid.set_inflation_radius(inflation_radius)

        # For now, publish stub (would integrate with actual obstacle detection)
        self.get_logger().info('Voxel grid publisher initialized.')


if __name__ == '__main__':
    # Test voxel grid functionality
    vg = VoxelGrid(width_m=10.0, height_m=10.0, depth_m=10.0, resolution=0.2)
    print(f'Grid dimensions: {vg.width} x {vg.depth} x {vg.height} voxels')
    print(f'Resolution: {vg.resolution} m per voxel')

    # Test painting obstacles
    vg.paint_sphere(0.0, 0.0, 2.0, 0.5)  # Sphere at center-ish
    vg.paint_box(-2.0, -2.0, 1.0, 1.0, 1.0, 2.0)  # Box

    # Test collision checking
    print(f'Is (0, 0, 0.5) free? {vg.is_free(0.0, 0.0, 0.5)}')
    print(f'Is (5, 5, 5) free? {vg.is_free(5.0, 5.0, 5.0)}')

    # Test line of sight
    print(f'LOS (0,0,1) to (5,5,5)? {vg.line_of_sight(0, 0, 1, 5, 5, 5)}')
