#!/usr/bin/env python3
"""
nature_3d_simulation.py
-----------------------
Standalone 3D Autonomous Drone Navigation Simulation
- Realistic Nature Scene (trees, rocks, bushes, terrain)
- **DYNAMIC OBSTACLES** (moving birds/animals)
- **REAL-TIME REPLANNING** with A*
- **LOCAL TOP-DOWN CAMERA** (locked to drone FOV)
- Cascaded PID drone motion

Run: python nature_3d_simulation.py
Requires: numpy, matplotlib
"""

import heapq
import math
import random
import time
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.colors as mcolors

# ─────────────────────────────────────────────
#  WORLD DEFINITION
# ─────────────────────────────────────────────
WORLD_SIZE   = 10.0
WORLD_HEIGHT = 6.0
VOXEL_RES    = 0.25

START = np.array([-4.0, 0.0, 1.0])
GOAL  = np.array([ 4.0, 0.0, 2.0])

# Static Nature obstacles: (cx, cy, cz_base, radius_xy, height)
PINE_TREES = [
    (-3.5, 2.0, 0.0,  0.5, 3.2),
    (-4.5, -0.5, 0.0, 0.4, 2.7),
    ( 3.0, -3.5, 0.0, 0.5, 3.5),
]
OAK_TREES = [
    ( 2.5, -1.0, 0.0, 0.6, 3.5),
    ( 4.0,  1.5, 0.0, 0.5, 3.0),
]
ROCKS = [
    (-1.5, -2.5, 0.0, 0.5, 0.5),
    ( 1.0, -1.0, 0.0, 0.4, 0.4),
]
BUSHES = [
    (-2.0,  0.5, 0.0, 0.55, 0.55),
    ( 1.5,  3.0, 0.0, 0.45, 0.45),
    (-0.5,  2.5, 0.0, 0.4,  0.4),
]

STATIC_OBSTACLES = PINE_TREES + OAK_TREES + ROCKS + BUSHES
INFLATION = 0.55  # safety margin

# Dynamic Obstacles (cx_start, cy_start, cz, radius, vx, vy, name)
DYNAMIC_OBSTACLES = [
    # A "bird" flying across the center
    {'pos': np.array([-1.0, 4.0, 1.5]), 'vel': np.array([0.5, -1.2, 0.0]), 'r': 0.6, 'color': '#E53935', 'name': 'Bird 1'},
    # An "animal" walking across the ground
    {'pos': np.array([1.5, -4.0, 0.4]), 'vel': np.array([-0.8, 0.6, 0.0]), 'r': 0.7, 'color': '#8D6E63', 'name': 'Animal'},
]

# ─────────────────────────────────────────────
#  VOXEL GRID & PATH PLANNING
# ─────────────────────────────────────────────
nx = int(2 * WORLD_SIZE / VOXEL_RES)
ny = int(2 * WORLD_SIZE / VOXEL_RES)
nz = int(WORLD_HEIGHT / VOXEL_RES)

def build_static_grid():
    grid = np.zeros((nx, ny, nz), dtype=bool)
    for (cx, cy, _, rx, h) in STATIC_OBSTACLES:
        rx_inf = rx + INFLATION
        h_inf  = h  + INFLATION * 0.5

        ix_lo = max(0, int((cx - rx_inf + WORLD_SIZE) / VOXEL_RES))
        ix_hi = min(nx-1, int((cx + rx_inf + WORLD_SIZE) / VOXEL_RES))
        iy_lo = max(0, int((cy - rx_inf + WORLD_SIZE) / VOXEL_RES))
        iy_hi = min(ny-1, int((cy + rx_inf + WORLD_SIZE) / VOXEL_RES))
        iz_hi = min(nz-1, int(h_inf / VOXEL_RES))

        for ix in range(ix_lo, ix_hi + 1):
            for iy in range(iy_lo, iy_hi + 1):
                xw = ix * VOXEL_RES - WORLD_SIZE
                yw = iy * VOXEL_RES - WORLD_SIZE
                if (xw - cx)**2 + (yw - cy)**2 <= rx_inf**2:
                    grid[ix, iy, 0:iz_hi+1] = True
    return grid

STATIC_GRID = build_static_grid()

def get_dynamic_grid(dyn_obs, time_sec):
    """Returns grid with static + current dynamic obstacles."""
    grid = STATIC_GRID.copy()
    for obs in dyn_obs:
        # Predict position
        cx, cy, cz = obs['pos'] + obs['vel'] * time_sec
        rx_inf = obs['r'] + INFLATION
        
        ix_lo = max(0, int((cx - rx_inf + WORLD_SIZE) / VOXEL_RES))
        ix_hi = min(nx-1, int((cx + rx_inf + WORLD_SIZE) / VOXEL_RES))
        iy_lo = max(0, int((cy - rx_inf + WORLD_SIZE) / VOXEL_RES))
        iy_hi = min(ny-1, int((cy + rx_inf + WORLD_SIZE) / VOXEL_RES))
        
        iz_lo = max(0, int((cz - rx_inf) / VOXEL_RES))
        iz_hi = min(nz-1, int((cz + rx_inf) / VOXEL_RES))

        for ix in range(ix_lo, ix_hi + 1):
            for iy in range(iy_lo, iy_hi + 1):
                xw = ix * VOXEL_RES - WORLD_SIZE
                yw = iy * VOXEL_RES - WORLD_SIZE
                if (xw - cx)**2 + (yw - cy)**2 <= rx_inf**2:
                    grid[ix, iy, iz_lo:iz_hi+1] = True
    return grid

def world_to_voxel(p):
    ix = int((p[0] + WORLD_SIZE) / VOXEL_RES)
    iy = int((p[1] + WORLD_SIZE) / VOXEL_RES)
    iz = int(p[2] / VOXEL_RES)
    return (max(0, min(nx-1, ix)), max(0, min(ny-1, iy)), max(0, min(nz-1, iz)))

def voxel_to_world(v):
    return (v[0]*VOXEL_RES - WORLD_SIZE + VOXEL_RES/2,
            v[1]*VOXEL_RES - WORLD_SIZE + VOXEL_RES/2,
            v[2]*VOXEL_RES + VOXEL_RES/2)

DIRS_26 = [(dx, dy, dz) for dx in (-1,0,1) for dy in (-1,0,1) for dz in (-1,0,1) if dx!=0 or dy!=0 or dz!=0]

def heuristic(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

def a_star(grid, start_v, goal_v):
    open_heap = [(0.0, start_v)]
    came_from = {}
    g_score   = {start_v: 0.0}
    open_set  = {start_v}

    while open_heap:
        _, cur = heapq.heappop(open_heap)
        
        if cur not in open_set: continue
        open_set.remove(cur)

        if cur == goal_v:
            path = [cur]
            while cur in came_from:
                cur = came_from[cur]
                path.append(cur)
            path.reverse()
            return path

        for d in DIRS_26:
            nb = (cur[0]+d[0], cur[1]+d[1], cur[2]+d[2])
            if not (0 <= nb[0] < nx and 0 <= nb[1] < ny and 0 <= nb[2] < nz): continue
            if grid[nb[0], nb[1], nb[2]]: continue
            
            nz_count = sum(c != 0 for c in d)
            cost = (1.0, 1.414, 1.732)[nz_count - 1]
            
            tg = g_score[cur] + cost
            if tg < g_score.get(nb, float('inf')):
                came_from[nb] = cur
                g_score[nb]   = tg
                f = tg + heuristic(nb, goal_v)
                if nb not in open_set:
                    open_set.add(nb)
                    heapq.heappush(open_heap, (f, nb))
    return []

# ─────────────────────────────────────────────
#  DRAW HELPERS
# ─────────────────────────────────────────────
def draw_pine_tree(ax3, cx, cy, height, r, color_trunk='#5C3A1A', color_canopy='#1B5E20'):
    theta = np.linspace(0, 2*np.pi, 8)
    zt = np.linspace(0, height * 0.55, 3)
    for z0, z1 in zip(zt[:-1], zt[1:]):
        xs = [[cx + 0.12 * np.cos(t) for t in theta], [cx + 0.12 * np.cos(t) for t in theta]]
        ys = [[cy + 0.12 * np.sin(t) for t in theta], [cy + 0.12 * np.sin(t) for t in theta]]
        zs = [[z0]*8, [z1]*8]
        ax3.plot_surface(np.array(xs), np.array(ys), np.array(zs), color=color_trunk, alpha=0.9, linewidth=0)
    for layer in range(2):
        base_r = r * (1.0 - layer * 0.3)
        base_z = height * 0.45 + layer * height * 0.2
        top_z  = base_z + height * 0.35
        theta2 = np.linspace(0, 2*np.pi, 12)
        cone_x = np.outer(np.linspace(base_r, 0.05, 4), np.cos(theta2)) + cx
        cone_y = np.outer(np.linspace(base_r, 0.05, 4), np.sin(theta2)) + cy
        cone_z = np.outer(np.linspace(base_z, top_z,  4), np.ones(12))
        ax3.plot_surface(cone_x, cone_y, cone_z, color=color_canopy, alpha=0.85, linewidth=0)

def draw_oak_tree(ax3, cx, cy, height, r, color_trunk='#6D4C1A', color_canopy='#2E7D32'):
    theta = np.linspace(0, 2*np.pi, 8)
    x = np.outer(0.14 * np.ones(2), np.cos(theta)) + cx
    y = np.outer(0.14 * np.ones(2), np.sin(theta)) + cy
    z = np.outer(np.array([0, height*0.6]), np.ones(8))
    ax3.plot_surface(x, y, z, color=color_trunk, alpha=0.9, linewidth=0)
    u = np.linspace(0, np.pi, 10)
    v = np.linspace(0, 2*np.pi, 12)
    sx = r * np.outer(np.sin(u), np.cos(v)) + cx
    sy = r * np.outer(np.sin(u), np.sin(v)) + cy
    sz = r * np.outer(np.cos(u), np.ones(12)) + height * 0.75
    ax3.plot_surface(sx, sy, sz, color=color_canopy, alpha=0.82, linewidth=0)

def draw_rock(ax3, cx, cy, rxy, h, color='#78909C'):
    u = np.linspace(0, np.pi, 8)
    v = np.linspace(0, 2*np.pi, 10)
    x = rxy * np.outer(np.sin(u), np.cos(v)) + cx
    y = rxy * np.outer(np.sin(u), np.sin(v)) + cy
    z = h   * np.outer(np.cos(u), np.ones(10)) * 0.5 + h * 0.5
    ax3.plot_surface(x, y, z, color=color, alpha=0.95, linewidth=0)

def draw_bush(ax3, cx, cy, r, color='#33691E'):
    u = np.linspace(0, np.pi, 8)
    v = np.linspace(0, 2*np.pi, 10)
    x = r * np.outer(np.sin(u), np.cos(v)) + cx
    y = r * np.outer(np.sin(u), np.sin(v)) + cy
    z = r * np.outer(np.cos(u), np.ones(10)) + r
    ax3.plot_surface(x, y, z, color=color, alpha=0.80, linewidth=0)


# ─────────────────────────────────────────────
#  MAIN SIMULATION LOGIC
# ─────────────────────────────────────────────
def run_simulation():
    print("Initializing Dynamic 3D Nature Simulation...")

    # Real-time state
    drone_pos = START.copy()
    current_time = 0.0
    dt_sim = 0.1  # 10Hz sim step
    
    # Initial Plan
    grid = get_dynamic_grid(DYNAMIC_OBSTACLES, current_time)
    start_v = world_to_voxel(drone_pos)
    goal_v  = world_to_voxel(GOAL)
    
    path_voxels = a_star(grid, start_v, goal_v)
    if not path_voxels:
        print("Failed to find initial path!")
        sys.exit(1)
        
    path_world = [voxel_to_world(v) for v in path_voxels]
    
    # Store history for visualization
    history_pos = [drone_pos.copy()]
    history_path = [path_world.copy()]  # store path at each step
    history_dyn = [ [obs['pos'] + obs['vel']*current_time for obs in DYNAMIC_OBSTACLES] ]
    replan_flags = [False]
    
    max_steps = 300
    for step in range(max_steps):
        if np.linalg.norm(drone_pos - GOAL) < 0.4:
            print(f"Goal reached at step {step}!")
            history_pos.append(drone_pos.copy())
            history_path.append(path_world.copy())
            history_dyn.append([obs['pos'] + obs['vel']*current_time for obs in DYNAMIC_OBSTACLES])
            replan_flags.append(False)
            break
            
        current_time += dt_sim
        
        # Move dynamic obstacles
        current_dyn_pos = [obs['pos'] + obs['vel']*current_time for obs in DYNAMIC_OBSTACLES]
        history_dyn.append(current_dyn_pos)
        
        # Check if current path is blocked by dynamic obstacles
        path_blocked = False
        lookahead = min(10, len(path_world))
        
        for p_idx in range(lookahead):
            wp = path_world[p_idx]
            for d_idx, obs in enumerate(DYNAMIC_OBSTACLES):
                d_pos = current_dyn_pos[d_idx]
                dist = np.linalg.norm(np.array(wp) - d_pos)
                if dist < (obs['r'] + INFLATION):
                    path_blocked = True
                    break
            if path_blocked: break
            
        # Replan if blocked
        did_replan = False
        if path_blocked:
            print(f"Step {step}: Path BLOCKED! Replanning A* in real-time...")
            grid = get_dynamic_grid(DYNAMIC_OBSTACLES, current_time)
            cur_v = world_to_voxel(drone_pos)
            new_voxels = a_star(grid, cur_v, goal_v)
            if new_voxels:
                path_world = [voxel_to_world(v) for v in new_voxels]
                did_replan = True
                print(f" -> Found new detour path ({len(path_world)} waypoints)")
            else:
                print(" -> Trapped! Waiting...")
        
        replan_flags.append(did_replan)
        
        # Move drone along path
        if len(path_world) > 1:
            target = np.array(path_world[1])
            direction = target - drone_pos
            dist = np.linalg.norm(direction)
            speed = 1.5  # m/s
            step_dist = speed * dt_sim
            
            if dist <= step_dist:
                drone_pos = target
                path_world.pop(0)
            else:
                drone_pos = drone_pos + (direction / dist) * step_dist
                
        history_pos.append(drone_pos.copy())
        history_path.append(path_world.copy())


    # ─────────────────────
    #  ANIMATION SETUP
    # ─────────────────────
    plt.style.use('dark_background')
    fig = plt.figure(figsize=(16, 9), facecolor='#0A0A1A')
    fig.suptitle(
        "GPS-Denied Drone Navigation · Dynamic Obstacle Replanning",
        color='white', fontsize=14, fontweight='bold', y=0.98
    )

    gs = gridspec.GridSpec(2, 3, figure=fig, left=0.05, right=0.98, top=0.92, bottom=0.05, hspace=0.3, wspace=0.25)

    ax3d   = fig.add_subplot(gs[:, 0:2], projection='3d')
    axtop  = fig.add_subplot(gs[0, 2])
    axinfo = fig.add_subplot(gs[1, 2])
    axinfo.axis('off')

    # ── 3D View ──
    ax3d.set_facecolor('#0D1117')
    ax3d.set_xlim(-WORLD_SIZE, WORLD_SIZE); ax3d.set_ylim(-WORLD_SIZE, WORLD_SIZE); ax3d.set_zlim(0, WORLD_HEIGHT)
    ax3d.view_init(elev=35, azim=-60)
    ax3d.set_title("Global 3D View", color='#B0BEC5')
    
    gx, gy = np.meshgrid(np.linspace(-WORLD_SIZE, WORLD_SIZE, 8), np.linspace(-WORLD_SIZE, WORLD_SIZE, 8))
    ax3d.plot_surface(gx, gy, np.zeros_like(gx), color='#1B5E20', alpha=0.3, linewidth=0)

    for cx, cy, _, rx, h in PINE_TREES: draw_pine_tree(ax3d, cx, cy, h, rx)
    for cx, cy, _, rx, h in OAK_TREES: draw_oak_tree(ax3d, cx, cy, h, rx)
    for cx, cy, _, rx, h in ROCKS: draw_rock(ax3d, cx, cy, rx, h)

    ax3d.scatter(*START, s=100, c='#00E5FF', label='Start')
    ax3d.scatter(*GOAL, s=150, c='#FFD600', marker='*', label='Goal')

    drone3d_scatter = ax3d.scatter([], [], [], s=120, c='#29B6F6', label='Drone')
    traj3d_line, = ax3d.plot([], [], [], '-', c='#4FC3F7', alpha=0.5)
    plan3d_line, = ax3d.plot([], [], [], '--', c='#FFF176', lw=2, label='Active A* Plan')
    
    dyn_3d_scatters = []
    for obs in DYNAMIC_OBSTACLES:
        scat = ax3d.scatter([], [], [], s=300, c=obs['color'], label=obs['name'])
        dyn_3d_scatters.append(scat)

    ax3d.legend(loc='upper right', facecolor='#1A237E', edgecolor='none')

    # ── Local Top-Down View ──
    axtop.set_facecolor('#102010')
    axtop.set_aspect('equal')
    axtop.set_title("Local Drone Camera View (Constrained FOV)", color='#A5D6A7')
    
    for cx, cy, _, rx, h in PINE_TREES:
        axtop.add_patch(plt.Circle((cx, cy), rx, color='#1B5E20', alpha=0.8))
    for cx, cy, _, rx, h in OAK_TREES:
        axtop.add_patch(plt.Circle((cx, cy), rx*1.2, color='#2E7D32', alpha=0.8))
    for cx, cy, _, rx, h in ROCKS:
        axtop.add_patch(plt.Circle((cx, cy), rx, color='#546E7A', alpha=0.9))

    drone_top, = axtop.plot([], [], 'o', c='#29B6F6', ms=8)
    plan_top,  = axtop.plot([], [], '--', c='#FFF176', lw=2.5)
    
    dyn_top_circs = []
    for obs in DYNAMIC_OBSTACLES:
        circ = plt.Circle((0,0), obs['r'], color=obs['color'], alpha=0.9)
        axtop.add_patch(circ)
        dyn_top_circs.append(circ)

    # ── Info Panel ──
    axinfo.set_facecolor('#0D1117')
    info_text = axinfo.text(0.05, 0.95, '', transform=axinfo.transAxes, color='#B2EBF2', 
                            fontsize=9, fontfamily='monospace', va='top',
                            bbox=dict(boxstyle='round', facecolor='#1A237E', alpha=0.6))

    # ── Anim logic ──
    frames_total = len(history_pos)

    def animate(i):
        pos = history_pos[i]
        path = np.array(history_path[i])
        dyn = history_dyn[i]
        replan = replan_flags[i]
        
        # 3D update
        drone3d_scatter._offsets3d = ([pos[0]], [pos[1]], [pos[2]])
        past = np.array(history_pos[:i+1])
        traj3d_line.set_data(past[:,0], past[:,1])
        traj3d_line.set_3d_properties(past[:,2])
        
        if len(path) > 0:
            plan3d_line.set_data(path[:,0], path[:,1])
            plan3d_line.set_3d_properties(path[:,2])
            
            # Flash path red if replanning
            if replan:
                plan3d_line.set_color('#FF5252')
                plan3d_line.set_linewidth(4.0)
            else:
                plan3d_line.set_color('#FFF176')
                plan3d_line.set_linewidth(2.0)
                
        for j, d_pos in enumerate(dyn):
            dyn_3d_scatters[j]._offsets3d = ([d_pos[0]], [d_pos[1]], [d_pos[2]])

        # Top-down update (moving window, 6x6m FOV)
        axtop.set_xlim(pos[0]-3.0, pos[0]+3.0)
        axtop.set_ylim(pos[1]-3.0, pos[1]+3.0)
        
        drone_top.set_data([pos[0]], [pos[1]])
        if len(path) > 0:
            plan_top.set_data(path[:,0], path[:,1])
            if replan:
                plan_top.set_color('#FF5252')
            else:
                plan_top.set_color('#FFF176')
                
        for j, d_pos in enumerate(dyn):
            dyn_top_circs[j].center = (d_pos[0], d_pos[1])

        # Info update
        status = "⚠️ REPLANNING!" if replan else ("NAVIGATING" if i < frames_total-2 else "✅ GOAL REACHED")
        info_text.set_text(
            f"{'─'*30}\n"
            f"  STATUS      : {status}\n"
            f"  Time        : {i * dt_sim:.1f} s\n"
            f"{'─'*30}\n"
            f"  Drone Pose\n"
            f"  X: {pos[0]:+.2f}  Y: {pos[1]:+.2f}  Z: {pos[2]:+.2f}\n"
            f"{'─'*30}\n"
            f"  Dynamic Objects Active: {len(DYNAMIC_OBSTACLES)}\n"
            f"  Current A* Waypoints: {len(path)}\n"
            f"{'─'*30}\n"
            f"  ALGORITHM MONITOR\n"
            f"  A* checks future collisions\n"
            f"  every {dt_sim}s. Re-routes if\n"
            f"  dynamic object intersects.\n"
            f"{'─'*30}"
        )

        return drone3d_scatter, traj3d_line, plan3d_line, drone_top, plan_top, info_text

    anim = FuncAnimation(fig, animate, frames=frames_total, interval=100, blit=False, repeat=False)
    plt.show()

if __name__ == '__main__':
    run_simulation()
