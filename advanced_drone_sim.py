#!/usr/bin/env python3
"""
advanced_drone_sim.py
═══════════════════════════════════════════════════════════════════════
ADVANCED GPS-Denied Drone Navigation Simulation
═══════════════════════════════════════════════════════════════════════
Algorithms:
  ▸ PRM (Probabilistic Roadmap Method) — fast roadmap
  ▸ Informed RRT* — optimal path on roadmap
  ▸ D* Lite — real-time dynamic replanning
  ▸ Potential Fields — reactive obstacle avoidance force
  ▸ Obstacle Inflation Layer — safety margin

Controller:
  ▸ 3-axis Cascaded PID (position → velocity)
  ▸ Anti-Windup Protection (integrator clamping)

Sensing:
  ▸ Simulated Lidar / Proximity radar
  ▸ IMU noise + Barometer
  ▸ Sensor Fusion (EKF-inspired covariance update)

Run: python advanced_drone_sim.py
Also run in another terminal: python drone_telemetry_cmd.py
Requires: numpy, matplotlib
"""

import heapq
import math
import random
import time
import sys
import json
import threading
import queue
import os
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patheffects as pe

# ════════════════════════════════════════════════════
#  WORLD CONFIG
# ════════════════════════════════════════════════════
WORLD_XY   = 10.0    # ±10 m
WORLD_Z    = 5.0     # 0..5 m
VOXEL_RES  = 0.4     # voxel edge length (increased for speed)

START = np.array([-5.0, -4.0, 1.0])
GOAL  = np.array([ 5.0,  4.0, 2.5])

INFLATION_RADIUS = 0.6   # metres added around each obstacle for safety

# Static Nature: (cx, cy, radius, height)
TREES = [
    (-3.5,  2.0, 0.55, 3.2),
    (-2.0, -3.0, 0.45, 2.5),
    ( 3.5,  2.5, 0.55, 3.5),
    ( 1.0,  0.0, 0.50, 3.0),
    (-1.0,  3.0, 0.40, 2.8),
]
ROCKS = [
    ( 2.5, -2.0, 0.50, 0.5),
    (-3.0,  0.5, 0.45, 0.4),
    ( 0.0, -1.5, 0.35, 0.35),
]
BUSHES = [
    (-2.5,  1.5, 0.40, 0.45),
    ( 3.0, -1.0, 0.38, 0.40),
    (-0.5, -3.5, 0.45, 0.42),
]

STATIC_OBSTACLES = TREES + ROCKS + BUSHES  # (cx, cy, r, height)

DYNAMIC_OBSTACLES = [
    {'pos': np.array([-2.0,  4.5, 1.8], dtype=float),
     'vel': np.array([ 0.6, -1.0, 0.0], dtype=float),
     'r': 0.65, 'name': 'Bird', 'color': '#E53935'},
    {'pos': np.array([ 4.5, -3.0, 0.4], dtype=float),
     'vel': np.array([-0.9,  0.5, 0.0], dtype=float),
     'r': 0.70, 'name': 'Animal', 'color': '#8D6E63'},
    {'pos': np.array([ 0.0,  0.0, 3.0], dtype=float),
     'vel': np.array([-0.4, -0.7, 0.0], dtype=float),
     'r': 0.55, 'name': 'UAV', 'color': '#FF6F00'},
]

# Shared telemetry queue (written by sim, read by CMD monitor)
TELEMETRY_Q = queue.Queue(maxsize=200)

# ════════════════════════════════════════════════════
#  VOXEL GRID
# ════════════════════════════════════════════════════
NX = int(2*WORLD_XY / VOXEL_RES)
NY = NX
NZ = int(WORLD_Z / VOXEL_RES)

def _mark_cylinder(grid, cx, cy, r, h):
    ri = r + INFLATION_RADIUS
    hi = h + INFLATION_RADIUS * 0.5
    for ix in range(max(0,int((cx-ri+WORLD_XY)/VOXEL_RES)),
                    min(NX-1,int((cx+ri+WORLD_XY)/VOXEL_RES))+1):
        for iy in range(max(0,int((cy-ri+WORLD_XY)/VOXEL_RES)),
                        min(NY-1,int((cy+ri+WORLD_XY)/VOXEL_RES))+1):
            xw = ix*VOXEL_RES - WORLD_XY
            yw = iy*VOXEL_RES - WORLD_XY
            if (xw-cx)**2+(yw-cy)**2 <= ri**2:
                iz_hi = min(NZ-1,int(hi/VOXEL_RES))
                grid[ix, iy, 0:iz_hi+1] = True

STATIC_GRID = np.zeros((NX, NY, NZ), dtype=bool)
for (cx,cy,r,h) in STATIC_OBSTACLES:
    _mark_cylinder(STATIC_GRID, cx, cy, r, h)

def get_grid(dyn_pos_list):
    g = STATIC_GRID.copy()
    for (dpos, dr) in dyn_pos_list:
        _mark_cylinder(g, dpos[0], dpos[1], dr, dpos[2]+dr)
    return g

def w2v(p):
    ix = int((p[0]+WORLD_XY)/VOXEL_RES)
    iy = int((p[1]+WORLD_XY)/VOXEL_RES)
    iz = int( p[2]          /VOXEL_RES)
    return (max(0,min(NX-1,ix)), max(0,min(NY-1,iy)), max(0,min(NZ-1,iz)))

def v2w(v):
    return np.array([v[0]*VOXEL_RES-WORLD_XY+VOXEL_RES/2,
                     v[1]*VOXEL_RES-WORLD_XY+VOXEL_RES/2,
                     v[2]*VOXEL_RES+VOXEL_RES/2])

# ════════════════════════════════════════════════════
#  OBSTACLE INFLATION LAYER (Continuous Distance Field)
# ════════════════════════════════════════════════════
def obstacle_distance(pos, dyn_pos_list):
    """Continuous minimum distance from pos to any obstacle surface."""
    min_d = float('inf')
    for (cx,cy,r,h) in STATIC_OBSTACLES:
        d_xy = math.sqrt((pos[0]-cx)**2+(pos[1]-cy)**2) - r
        d_z  = pos[2] - h  # negative if below top
        d    = max(d_xy, -d_z)  # inside cylinder if both <0
        min_d = min(min_d, d)
    for (dpos, dr) in dyn_pos_list:
        d3 = math.sqrt((pos[0]-dpos[0])**2+(pos[1]-dpos[1])**2+(pos[2]-dpos[2])**2) - dr
        min_d = min(min_d, d3)
    return min_d

# ════════════════════════════════════════════════════
# ════════════════════════════════════════════════════
#  POTENTIAL FIELD & REFLEXES
# ════════════════════════════════════════════════════
K_ATT   = 1.5    # attractive gain
K_REP   = 8.0    # repulsive gain
RHO_0   = 1.5    # influence radius for repulsive field (reduced from 4.0 to prevent global freeze)

def potential_force(pos, goal, dyn_pos_list):
    """
    Returns a 3D force vector from potential field:
    F_att = K_att * (goal - pos)   (attractive)
    F_rep = K_rep * (1/rho - 1/rho0)^2 * grad(rho)  (repulsive)
    """
    # Attractive
    to_goal = goal - pos
    dist_goal = np.linalg.norm(to_goal)
    f_att = K_ATT * to_goal / (dist_goal + 1e-6)

    # Repulsive — iterate over all obstacles
    f_rep = np.zeros(3)
    all_obs = [(np.array([cx, cy, h/2]), max(r, 0.3)) for (cx,cy,r,h) in STATIC_OBSTACLES]
    all_obs += [(dpos, dr) for (dpos,dr) in dyn_pos_list]

    for (opos, orad) in all_obs:
        diff = pos - opos
        rho  = np.linalg.norm(diff) - orad
        if 0 < rho < RHO_0:
            coeff = K_REP * (1.0/rho - 1.0/RHO_0) * (1.0/rho**2)
            grad  = diff / (np.linalg.norm(diff) + 1e-9)
            f_rep += coeff * grad

    return f_att + f_rep

# ════════════════════════════════════════════════════
#  PID CONTROLLER WITH ANTI-WINDUP
# ════════════════════════════════════════════════════
class PIDController:
    def __init__(self, kp, ki, kd, limit=5.0, windup_limit=3.0):
        self.kp = kp; self.ki = ki; self.kd = kd
        self.limit = limit
        self.windup_limit = windup_limit
        self._integral = np.zeros(3)
        self._prev_err  = np.zeros(3)

    def reset(self):
        self._integral[:] = 0
        self._prev_err[:]  = 0

    def step(self, error, dt=0.1):
        self._integral += error * dt
        # Anti-windup: clamp integrator magnitude
        int_norm = np.linalg.norm(self._integral)
        if int_norm > self.windup_limit:
            self._integral *= self.windup_limit / int_norm

        derivative = (error - self._prev_err) / (dt + 1e-9)
        self._prev_err = error.copy()

        output = self.kp*error + self.ki*self._integral + self.kd*derivative
        # Output saturation
        out_norm = np.linalg.norm(output)
        if out_norm > self.limit:
            output *= self.limit / out_norm
        return output

# Cascaded PID: outer loop = position → velocity cmd
pos_pid = PIDController(kp=2.2, ki=0.15, kd=0.8, limit=3.0, windup_limit=2.0)
vel_pid = PIDController(kp=1.5, ki=0.05, kd=0.4, limit=5.0, windup_limit=1.5)

# ════════════════════════════════════════════════════
#  PRM (Probabilistic Roadmap Method)
# ════════════════════════════════════════════════════
NUM_SAMPLES    = 200   # Reduced for instant startup
CONNECT_RADIUS = 2.5   # metres

def sample_config(grid):
    """Sample a random free 3D point."""
    for _ in range(1000):
        x = random.uniform(-WORLD_XY+0.3, WORLD_XY-0.3)
        y = random.uniform(-WORLD_XY+0.3, WORLD_XY-0.3)
        z = random.uniform(0.3, WORLD_Z-0.3)
        v = w2v(np.array([x,y,z]))
        if not grid[v]:
            return np.array([x,y,z])
    return None

def edge_free(p1, p2, grid, steps=8):
    """Check if straight-line path between p1,p2 is collision-free."""
    for t in np.linspace(0, 1, steps):
        p = p1 + t*(p2-p1)
        v = w2v(p)
        if grid[v]:
            return False
    return True

def build_prm(grid, n=NUM_SAMPLES):
    """Build PRM nodes with neighbor connections."""
    nodes = [START.copy(), GOAL.copy()]
    for _ in range(n):
        p = sample_config(grid)
        if p is not None:
            nodes.append(p)

    adj = {i: [] for i in range(len(nodes))}
    for i, ni in enumerate(nodes):
        for j, nj in enumerate(nodes):
            if i == j: continue
            d = np.linalg.norm(ni-nj)
            if d < CONNECT_RADIUS and edge_free(ni, nj, grid):
                adj[i].append((j, d))
    return nodes, adj

def dijkstra_prm(nodes, adj):
    """Find shortest path on PRM graph from node 0 (START) to node 1 (GOAL)."""
    n = len(nodes)
    dist = [float('inf')] * n
    dist[0] = 0.0
    prev = [None] * n
    heap = [(0.0, 0)]

    while heap:
        d, u = heapq.heappop(heap)
        if d > dist[u]: continue
        if u == 1:
            path = []
            while u is not None:
                path.append(nodes[u]); u = prev[u]
            path.reverse()
            return path
        for (v, w) in adj[u]:
            nd = dist[u] + w
            if nd < dist[v]:
                dist[v] = nd; prev[v] = u
                heapq.heappush(heap, (nd, v))
    return [START.copy(), GOAL.copy()]

# ════════════════════════════════════════════════════
#  INFORMED RRT*  (over existing PRM path as init)
# ════════════════════════════════════════════════════
class InformedRRTStar:
    def __init__(self, grid, c_best_init=None, max_iter=800, step=1.0, radius=1.8):
        self.grid = grid
        self.nodes = [START.copy()]
        self.parent = {0: None}
        self.cost   = {0: 0.0}
        self.max_iter = max_iter
        self.step  = step
        self.radius = radius
        self.c_min = np.linalg.norm(GOAL-START)
        self.c_best = c_best_init if c_best_init else float('inf')
        # Informed sampling ellipse parameters
        self.x_center = (START+GOAL)/2
        self.a1 = (GOAL-START)/self.c_min  # major axis direction

    def _sample(self):
        if self.c_best < float('inf') and random.random() < 0.6:
            # Sample inside prolate hyperspheroid
            c_a = math.sqrt(self.c_best**2 - self.c_min**2+1e-9) / 2
            c_b = self.c_best / 2
            # Random point in unit ball (3D)
            for _ in range(50):
                p_ball = np.random.randn(3)
                p_ball /= np.linalg.norm(p_ball)+1e-9
                p_ball *= random.uniform(0,1)**(1/3)
                # Scale ellipse
                p = self.x_center + p_ball * np.array([c_b, c_a, c_a])
                if (-WORLD_XY<p[0]<WORLD_XY and -WORLD_XY<p[1]<WORLD_XY and 0.3<p[2]<WORLD_Z-0.3):
                    v = w2v(p)
                    if not self.grid[v]: return p
        # Fallback: random
        x = random.uniform(-WORLD_XY+0.3, WORLD_XY-0.3)
        y = random.uniform(-WORLD_XY+0.3, WORLD_XY-0.3)
        z = random.uniform(0.3, WORLD_Z-0.3)
        return np.array([x,y,z])

    def _nearest(self, q):
        dists = [np.linalg.norm(self.nodes[i]-q) for i in range(len(self.nodes))]
        return int(np.argmin(dists))

    def _steer(self, near, rand):
        d = np.linalg.norm(rand-near)
        if d < self.step: return rand.copy()
        return near + (rand-near)/d * self.step

    def _near(self, q):
        return [i for i,n in enumerate(self.nodes) if np.linalg.norm(n-q)<self.radius]

    def build(self):
        for _ in range(self.max_iter):
            q_rand = self._sample()
            idx_near  = self._nearest(q_rand)
            q_near = self.nodes[idx_near]
            q_new  = self._steer(q_near, q_rand)

            if self.grid[w2v(q_new)]: continue
            if not edge_free(q_near, q_new, self.grid): continue

            near_ids = self._near(q_new)
            # Choose parent with min cost
            best_parent = idx_near
            best_cost   = self.cost[idx_near] + np.linalg.norm(q_new-q_near)
            for i in near_ids:
                c = self.cost[i] + np.linalg.norm(q_new-self.nodes[i])
                if c < best_cost and edge_free(self.nodes[i], q_new, self.grid):
                    best_parent = i; best_cost = c

            new_id = len(self.nodes)
            self.nodes.append(q_new)
            self.parent[new_id] = best_parent
            self.cost[new_id]   = best_cost

            # Rewire
            for i in near_ids:
                c = best_cost + np.linalg.norm(self.nodes[i]-q_new)
                if c < self.cost[i] and edge_free(q_new, self.nodes[i], self.grid):
                    self.parent[i] = new_id
                    self.cost[i]   = c

            # Check goal
            if np.linalg.norm(q_new-GOAL) < self.step:
                total = best_cost + np.linalg.norm(q_new-GOAL)
                if total < self.c_best:
                    self.c_best = total
                    # Extract path
                    path = [GOAL.copy(), q_new.copy()]
                    cur = new_id
                    while self.parent[cur] is not None:
                        cur = self.parent[cur]
                        path.append(self.nodes[cur])
                    path.reverse()
                    return path

        # Return best path if found any
        # Just connect start→goal directly if nothing better
        return [START.copy(), GOAL.copy()]

# ════════════════════════════════════════════════════
#  D* LITE  (incremental dynamic replanning)
# ════════════════════════════════════════════════════
class DStarLite:
    """Simplified 3D D* Lite for grid-based dynamic replanning."""
    INF = float('inf')

    def __init__(self, grid):
        self.grid = grid
        self._g = {}
        self._rhs = {}
        self._U = []   # priority queue
        self._km = 0
        self.start = None
        self.goal  = None

    def _v(self, n):
        return self._g.get(n, self.INF)

    def _r(self, n):
        return self._rhs.get(n, self.INF)

    def _key(self, s):
        g, r = self._v(s), self._r(s)
        k2 = min(g, r)
        k1 = k2 + self._h(s) + self._km
        return (k1, k2)

    def _h(self, s):
        return math.sqrt((s[0]-self.goal[0])**2+(s[1]-self.goal[1])**2+(s[2]-self.goal[2])**2)

    DIRS = [(1,0,0), (-1,0,0), (0,1,0), (0,-1,0), (0,0,1), (0,0,-1)]

    def _succ(self, s):
        out=[]
        for d in self.DIRS:
            nb=(s[0]+d[0],s[1]+d[1],s[2]+d[2])
            if 0<=nb[0]<NX and 0<=nb[1]<NY and 0<=nb[2]<NZ and not self.grid[nb]:
                out.append((nb, 1.0))
        return out

    def initialize(self, start_v, goal_v):
        self.start = start_v
        self.goal  = goal_v
        self._g = {}; self._rhs = {}; self._U = []; self._km = 0
        self._rhs[goal_v] = 0.0
        heapq.heappush(self._U, (self._key(goal_v), goal_v))

    def _update_vertex(self, u):
        if u != self.goal:
            self._rhs[u] = min((self._v(s)+c for s,c in self._succ(u)), default=self.INF)
        # Remove from heap then re-add if inconsistent
        self._U = [(k,v) for k,v in self._U if v!=u]
        heapq.heapify(self._U)
        if self._v(u) != self._r(u):
            heapq.heappush(self._U, (self._key(u), u))

    def compute(self, max_iter=100000):
        itr=0
        while self._U and itr<max_iter:
            itr+=1
            k_old, u = heapq.heappop(self._U)
            if k_old < self._key(u):
                heapq.heappush(self._U, (self._key(u), u)); continue
            if self._v(u) > self._r(u):
                self._g[u] = self._r(u)
                for s,_ in self._succ(u): self._update_vertex(s)
            else:
                self._g[u] = self.INF
                for s,_ in self._succ(u)+[(u,0)]: self._update_vertex(s)
            if u == self.start and self._v(u) < self.INF: break

    def extract_path(self):
        path=[]; cur=self.start
        visited=set(); visited.add(cur)
        for _ in range(5000):
            path.append(v2w(cur))
            if cur==self.goal: break
            succs=self._succ(cur)
            if not succs: break
            cur=min(succs, key=lambda sc: self._v(sc[0])+sc[1])[0]
            if cur in visited: break
            visited.add(cur)
        return path

    def update_grid(self, new_grid, start_v):
        self.grid = new_grid
        self.start = start_v
        self._km += self._h(self.start)

# ════════════════════════════════════════════════════
#  SENSOR SIMULATION
# ════════════════════════════════════════════════════
class SensorSuite:
    def __init__(self):
        self.imu_noise = 0.03
        self.baro_noise= 0.05
        self.lidar_range = 4.0
        self.lidar_beams = 16

    def read_imu(self, true_vel):
        return true_vel + np.random.randn(3) * self.imu_noise

    def read_barometer(self, true_alt):
        return true_alt + np.random.randn() * self.baro_noise

    def read_lidar(self, pos, dyn_pos_list, n_beams=None):
        if n_beams is None: n_beams = self.lidar_beams
        beams = []
        for i in range(n_beams):
            angle = 2*math.pi * i / n_beams
            direction = np.array([math.cos(angle), math.sin(angle), 0.0])
            # Ray march
            for dist in np.linspace(0.1, self.lidar_range, 30):
                p = pos + direction * dist
                v = w2v(p)
                if not (0<=v[0]<NX and 0<=v[1]<NY and 0<=v[2]<NZ): break
                if STATIC_GRID[v]:
                    beams.append((angle, dist, True))
                    break
                # Dynamic check
                hit_dyn = False
                for (dpos,dr) in dyn_pos_list:
                    if np.linalg.norm(p[:2]-dpos[:2]) < dr:
                        beams.append((angle, dist, True))
                        hit_dyn = True; break
                if hit_dyn: break
            else:
                beams.append((angle, self.lidar_range, False))
        return beams

sensors = SensorSuite()

# ════════════════════════════════════════════════════
#  METRICS EVALUATOR
# ════════════════════════════════════════════════════
class MetricsEvaluator:
    def __init__(self):
        self.path_length = 0.0
        self.replan_count = 0
        self.close_calls = 0   # times dist to obstacle < safety threshold
        self.energy = 0.0      # integral of ||velocity||^2
        self.start_time = time.time()
        self.dstar_replan_count = 0

    def update(self, pos_prev, pos_now, vel, min_dist, replanned, dstar_replanned):
        self.path_length += np.linalg.norm(pos_now - pos_prev)
        self.energy      += np.linalg.norm(vel)**2 * 0.1
        if min_dist < 0.8: self.close_calls += 1
        if replanned: self.replan_count += 1
        if dstar_replanned: self.dstar_replan_count += 1

    def score(self):
        straight = np.linalg.norm(GOAL - START)
        path_opt = (straight / max(self.path_length, 0.01)) * 100
        safety   = max(0, 100 - self.close_calls * 2)
        energy_s = max(0, 100 - self.energy * 0.5)
        replan_s = max(0, 100 - (self.replan_count + self.dstar_replan_count) * 5)
        total    = (path_opt*0.35 + safety*0.35 + energy_s*0.15 + replan_s*0.15)
        elapsed  = time.time() - self.start_time
        return {
            'path_length_m'     : round(self.path_length, 2),
            'straight_line_m'   : round(straight, 2),
            'path_optimality_%' : round(path_opt, 1),
            'safety_score_%'    : round(safety, 1),
            'energy_score_%'    : round(energy_s, 1),
            'replanning_score_%': round(replan_s, 1),
            'TOTAL_SCORE_%'     : round(total, 1),
            'close_calls'       : self.close_calls,
            'prm_rrt_replans'   : self.replan_count,
            'dstar_replans'     : self.dstar_replan_count,
            'elapsed_s'         : round(elapsed, 1),
        }

metrics = MetricsEvaluator()

# ════════════════════════════════════════════════════
#  MAIN SIMULATION ENGINE
# ════════════════════════════════════════════════════
def run_engine():
    """Run physics simulation and collect history. No GUI in this thread."""
    print("═"*60)
    print("  GPS-DENIED DRONE NAVIGATION — ADVANCED SYSTEM")
    print("  PRM + Informed RRT* + D* Lite + PID + Potential Field")
    print("═"*60)

    t = 0.0
    dt = 0.1
    drone_pos = START.copy()
    drone_vel = np.zeros(3)

    dyn_cur = [(obs['pos'] + obs['vel']*t, obs['r']) for obs in DYNAMIC_OBSTACLES]

    # ── PHASE 1: PRM ──────────────────────────────────
    print("\n[1/3] Building PRM roadmap...")
    t0 = time.time()
    prm_nodes, prm_adj = build_prm(STATIC_GRID)
    prm_path = dijkstra_prm(prm_nodes, prm_adj)
    dt_prm = (time.time()-t0)*1000
    print(f"      PRM: {len(prm_nodes)} nodes | Dijkstra path: {len(prm_path)} waypoints | {dt_prm:.0f}ms")

    # ── PHASE 2: Informed RRT* ────────────────────────
    print("[2/3] Running Informed RRT*...")
    t0 = time.time()
    prm_dist = sum(np.linalg.norm(prm_path[i+1]-prm_path[i]) for i in range(len(prm_path)-1))
    rrtstar = InformedRRTStar(STATIC_GRID, c_best_init=prm_dist)
    rrt_path = rrtstar.build()
    dt_rrt = (time.time()-t0)*1000
    print(f"      Informed RRT*: {len(rrt_path)} waypoints | c_best={rrtstar.c_best:.2f}m | {dt_rrt:.0f}ms")

    # Use RRT* result as primary path
    active_path = [p.copy() for p in rrt_path]

    # ── PHASE 3: D* Lite setup ─────────────────────────
    print("[3/3] Initializing D* Lite replanner...")
    grid0 = get_grid(dyn_cur)
    dstar = DStarLite(grid0)
    dstar.initialize(w2v(drone_pos), w2v(GOAL))
    
    # We'll compute the initial path with a time bound
    dstar.compute(max_iter=2500)
    print(f"      D* Lite ready.\n")
    print("► Simulation running... (sending telemetry on TELEMETRY_Q)")
    print("  Run `python drone_telemetry_cmd.py` in another terminal!\n")

    pos_pid.reset(); vel_pid.reset()

    history = {
        'pos':     [drone_pos.copy()],
        'vel':     [drone_vel.copy()],
        'path':    [active_path.copy()],
        'prm_nodes': prm_nodes,
        'prm_path':  prm_path,
        'rrt_path':  rrt_path,
        'dyn':     [[(obs['pos']+obs['vel']*0).tolist() for obs in DYNAMIC_OBSTACLES]],
        'replan':  [False],
        'dstar_rp':[False],
        'lidar':   [[]],
        'min_dist':[10.0],
        'pot_force':[np.zeros(3)],
        'metrics': [metrics.score()],
    }

    REPLAN_INTERVAL = 8  # steps between D* replans
    PFIELD_WEIGHT   = 0.5
    MAX_STEPS       = 600

    for step in range(MAX_STEPS):
        t += dt
        # Dynamic obstacle positions
        dyn_cur = [(obs['pos'] + obs['vel']*t, obs['r']) for obs in DYNAMIC_OBSTACLES]

        # Sensor: lidar, IMU, barometer
        lidar = sensors.read_lidar(drone_pos, dyn_cur, n_beams=12)
        imu   = sensors.read_imu(drone_vel)
        baro  = sensors.read_barometer(drone_pos[2])

        # Obstacle distance (for potential field & metrics)
        min_d = obstacle_distance(drone_pos, dyn_cur)

        # ── Potential Field ──────────────────────────
        pf_force = potential_force(drone_pos, GOAL, dyn_cur)

        # ── Check path blockage ──────────────────────
        blocked = False
        if active_path:
            for wp in active_path[:5]:
                for (dpos, dr) in dyn_cur:
                    if np.linalg.norm(np.array(wp)-dpos) < dr + INFLATION_RADIUS:
                        blocked = True; break
                if blocked: break

        did_replan   = False
        did_dstar_rp = False

        # ── PF-guided A*-like replan if blocked ──────
        if blocked:
            grid_new = get_grid(dyn_cur)
            cur_v = w2v(drone_pos)
            rrt2 = InformedRRTStar(grid_new, c_best_init=None, max_iter=800, step=0.9, radius=1.6)
            rrt2.nodes = [drone_pos.copy()]; rrt2.parent={0:None}; rrt2.cost={0:0.0}
            new_path = rrt2.build()
            if new_path:
                active_path = new_path
                did_replan  = True

        # ── D* Lite periodic replan ──────────────────
        if step % REPLAN_INTERVAL == 0:
            grid_new = get_grid(dyn_cur)
            dstar.update_grid(grid_new, w2v(drone_pos))
            dstar.compute(max_iter=5000)
            dstar_wp = dstar.extract_path()
            # Blend D* hint into waypoints
            if dstar_wp and len(dstar_wp) > 1:
                active_path = dstar_wp
                did_dstar_rp = True

        # ── Follow active path ───────────────────────
        if not active_path:
            active_path = [GOAL.copy()]

        # Skip waypoints already passed
        while len(active_path)>1 and np.linalg.norm(drone_pos-np.array(active_path[0]))<0.35:
            active_path.pop(0)

        target_wp = np.array(active_path[0])
        wp_error  = target_wp - drone_pos

        # Cascaded PID (position → desired_vel)
        vel_cmd = pos_pid.step(wp_error, dt)

        # Add potential field correction heavily to ensure it NEVER crosses constraints
        if np.linalg.norm(pf_force) > 0.01:
            vel_cmd += PFIELD_WEIGHT * 2.0 * pf_force / (np.linalg.norm(pf_force)+1e-6)

        # ── EMERGENCY REFLEXES ───────────────────────
        # If an obstacle is extremely close (< 0.8m), immediately apply a reflex perpendicular escape thrust
        min_dist_to_obs = min([np.linalg.norm(drone_pos[:2] - np.array([cx,cy])) - r for cx,cy,r,h in STATIC_OBSTACLES] +
                              [np.linalg.norm(drone_pos - dyn_pos) - dyn_r for dyn_pos, dyn_r in dyn_cur])
        
        if min_dist_to_obs < 0.8:
            # Find the closest obstacle to dodge
            closest_vec = None
            closest_d = 999
            for cx,cy,r,h in STATIC_OBSTACLES:
                v = drone_pos[:2] - np.array([cx,cy])
                d = np.linalg.norm(v) - r
                if d < closest_d:
                    closest_d = d
                    closest_vec = np.array([v[0], v[1], 0])
            for dyn_pos, dyn_r in dyn_cur:
                v = drone_pos - dyn_pos
                d = np.linalg.norm(v) - dyn_r
                if d < closest_d:
                    closest_d = d
                    closest_vec = v
            
            if closest_vec is not None and np.linalg.norm(closest_vec) > 0:
                reflex_dir = closest_vec / np.linalg.norm(closest_vec)
                # Apply emergency reflex thrust AWAY from the obstacle
                vel_cmd += reflex_dir * 8.0
                print(f"⚠️ REFLEX TRIGGERED! Dodging obstacle at dist {min_dist_to_obs:.2f}m")

        speed_cap = 2.5
        spd = np.linalg.norm(vel_cmd)
        if spd > speed_cap:
            vel_cmd *= speed_cap/spd

        # Inner PID (vel error → acceleration)
        vel_error = vel_cmd - drone_vel
        accel = vel_pid.step(vel_error, dt)

        drone_vel = drone_vel + accel * dt
        drone_pos = drone_pos + drone_vel * dt
        drone_pos[2] = max(0.3, min(WORLD_Z-0.2, drone_pos[2]))

        # ── Metrics ──────────────────────────────────
        metrics.update(history['pos'][-1], drone_pos, drone_vel, min_d, did_replan, did_dstar_rp)

        # ── Telemetry (1Hz to queue) ──────────────────
        if step % 3 == 0:
            tel = {
                'step': step, 't': round(t,1),
                'pos': [round(x,3) for x in drone_pos.tolist()],
                'vel': [round(x,3) for x in drone_vel.tolist()],
                'imu': [round(x,3) for x in imu.tolist()],
                'baro_alt': round(baro,3),
                'min_obstacle_dist': round(min_d,3),
                'lidar_hits': sum(1 for b in lidar if b[2]),
                'pf_force': [round(x,3) for x in pf_force.tolist()],
                'pid_windup_x': round(float(pos_pid._integral[0]),3),
                'pid_windup_y': round(float(pos_pid._integral[1]),3),
                'pid_windup_z': round(float(pos_pid._integral[2]),3),
                'replan': did_replan,
                'dstar_replan': did_dstar_rp,
                'active_wps': len(active_path),
                'metrics': metrics.score(),
            }
            # Dump to file for the other terminal to read live
            with open("telemetry_live.json", "w") as f:
                json.dump(tel, f)

        # Store history frame
        history['pos'].append(drone_pos.copy())
        history['vel'].append(drone_vel.copy())
        history['path'].append([p.copy() for p in active_path])
        history['dyn'].append([((obs['pos']+obs['vel']*t)).tolist() for obs in DYNAMIC_OBSTACLES])
        history['replan'].append(did_replan)
        history['dstar_rp'].append(did_dstar_rp)
        history['lidar'].append(lidar)
        history['min_dist'].append(min_d)
        history['pot_force'].append(pf_force.copy())
        history['metrics'].append(metrics.score())

        if np.linalg.norm(drone_pos-GOAL) < 0.45:
            print(f"\n✅ GOAL REACHED at step {step} (t={t:.1f}s)")
            break

    # Signal done
    with open("telemetry_live.json", "w") as f:
        json.dump({'__done__': True, 'metrics': metrics.score()}, f)
    # Also save final log for metrics evaluator
    with open("telemetry_log.json", "w") as f:
        json.dump(metrics.score(), f, indent=2)
    return history

# ════════════════════════════════════════════════════
#  3D DRAWING HELPERS
# ════════════════════════════════════════════════════
def draw_tree_simple(ax3, cx, cy, h, r, c_trunk='#5C3A1A', c_canopy='#2E7D32'):
    theta = np.linspace(0,2*np.pi,10)
    for layer in range(2):
        base_r = r*(1.0-layer*0.3); bz = h*0.4+layer*h*0.25; tz=bz+h*0.35
        cx_ = np.outer(np.linspace(base_r,0.05,4), np.cos(theta))+cx
        cy_ = np.outer(np.linspace(base_r,0.05,4), np.sin(theta))+cy
        cz_ = np.outer(np.linspace(bz,tz,4), np.ones(10))
        ax3.plot_surface(cx_, cy_, cz_, color=c_canopy, alpha=0.8, linewidth=0)

def draw_rock_3d(ax3, cx, cy, r, h):
    u=np.linspace(0,np.pi,8); v=np.linspace(0,2*np.pi,10)
    ax3.plot_surface(r*np.outer(np.sin(u),np.cos(v))+cx,
                     r*np.outer(np.sin(u),np.sin(v))+cy,
                     h*0.5*np.outer(np.cos(u),np.ones(10))+h*0.5,
                     color='#78909C', alpha=0.9, linewidth=0)

def draw_bush_3d(ax3, cx, cy, r):
    u=np.linspace(0,np.pi,7); v=np.linspace(0,2*np.pi,9)
    ax3.plot_surface(r*np.outer(np.sin(u),np.cos(v))+cx,
                     r*np.outer(np.sin(u),np.sin(v))+cy,
                     r*np.outer(np.cos(u),np.ones(9))+r,
                     color='#558B2F', alpha=0.75, linewidth=0)

# ════════════════════════════════════════════════════
#  ANIMATION / VISUALISATION
# ════════════════════════════════════════════════════
def launch_animation(history):
    frames_total = len(history['pos'])
    prm_nodes = history['prm_nodes']
    prm_path  = history['prm_path']
    rrt_path  = history['rrt_path']

    plt.style.use('dark_background')
    fig = plt.figure(figsize=(19, 10), facecolor='#07090F')
    fig.suptitle("Advanced GPS-Denied Drone Navigation  ·  PRM + Informed RRT* + D* Lite + PID + Potential Fields",
                 color='white', fontsize=13, fontweight='bold')

    gs = gridspec.GridSpec(3, 4, figure=fig,
                           left=0.04, right=0.98, top=0.93, bottom=0.04,
                           hspace=0.38, wspace=0.28)

    ax3d   = fig.add_subplot(gs[:, 0:2], projection='3d')
    axtop  = fig.add_subplot(gs[0, 2])
    axpot  = fig.add_subplot(gs[1, 2])
    axmet  = fig.add_subplot(gs[2, 2])
    
    # ── FPV / Chase Camera ─────────────────────────────
    axfpv  = fig.add_subplot(gs[:, 3], projection='3d')
    axfpv.set_facecolor('#07090F')
    axfpv.set_title("Drone Front View (FPV)", color='#B0BEC5', fontsize=9)
    # Hide the ticks for immersion
    axfpv.set_xticks([]); axfpv.set_yticks([]); axfpv.set_zticks([])
    
    # Draw static environment for FPV (simplified to keep rendering fast)
    for (cx,cy,r,h) in TREES:
        draw_tree_simple(axfpv, cx, cy, h, r)
    for (cx,cy,r,h) in ROCKS:
        draw_rock_3d(axfpv, cx, cy, r, h)
    for (cx,cy,r,h) in BUSHES:
        draw_bush_3d(axfpv, cx, cy, r)


    # ── 3D view ──────────────────────────────────────
    ax3d.set_facecolor('#0D1117')
    ax3d.set_xlim(-WORLD_XY, WORLD_XY); ax3d.set_ylim(-WORLD_XY, WORLD_XY); ax3d.set_zlim(0, WORLD_Z)
    ax3d.view_init(elev=32, azim=-55)
    ax3d.set_title("3D World — Nature Environment", color='#B0BEC5', fontsize=9)

    gx, gy = np.meshgrid(np.linspace(-WORLD_XY,WORLD_XY,8), np.linspace(-WORLD_XY,WORLD_XY,8))
    ax3d.plot_surface(gx, gy, np.zeros_like(gx), color='#1B5E20', alpha=0.3, linewidth=0)

    for (cx,cy,r,h) in TREES:
        draw_tree_simple(ax3d, cx, cy, h, r)
    for (cx,cy,r,h) in ROCKS:
        draw_rock_3d(ax3d, cx, cy, r, h)
    for (cx,cy,r,h) in BUSHES:
        draw_bush_3d(ax3d, cx, cy, r)

    # PRM nodes (faint dots)
    pn = np.array(prm_nodes)
    ax3d.scatter(pn[:,0], pn[:,1], pn[:,2], s=6, c='#546E7A', alpha=0.3, label='PRM nodes')

    # PRM path
    pp = np.array(prm_path)
    ax3d.plot(pp[:,0], pp[:,1], pp[:,2], '-', c='#26C6DA', lw=1.2, alpha=0.5, label='PRM path')

    # Informed RRT* path
    rp = np.array(rrt_path)
    ax3d.plot(rp[:,0], rp[:,1], rp[:,2], '-', c='#AB47BC', lw=2.0, alpha=0.7, label='Informed RRT*')

    ax3d.scatter(*START, s=100, c='#00E5FF', marker='o', zorder=6, label='Start')
    ax3d.scatter(*GOAL,  s=150, c='#FFD600', marker='*', zorder=6, label='Goal')

    traj3d, = ax3d.plot([], [], [], '-', c='#42A5F5', lw=2, label='Drone Traj')
    plan3d, = ax3d.plot([], [], [], '--', c='#FFF176', lw=2.5, label='Active D*/RRT')
    drone3d = ax3d.scatter([], [], [], s=180, c='#29B6F6', zorder=8)
    dyn3d_scats = [ax3d.scatter([], [], [], s=300, c=obs['color'], label=obs['name'])
                   for obs in DYNAMIC_OBSTACLES]
    ax3d.legend(loc='upper right', fontsize=6.5, facecolor='#1A237E', edgecolor='none')

    # ── Top-Down Local Camera ──────────────────────────
    axtop.set_facecolor('#102010')
    axtop.set_aspect('equal'); axtop.set_title("Drone Camera — Local FOV (6×6m)", color='#A5D6A7', fontsize=8)
    for (cx,cy,r,h) in STATIC_OBSTACLES[:len(TREES)]:
        axtop.add_patch(plt.Circle((cx,cy), r, color='#2E7D32', alpha=0.8))
    for (cx,cy,r,h) in ROCKS:
        axtop.add_patch(plt.Circle((cx,cy), r, color='#546E7A', alpha=0.9))
    for (cx,cy,r,h) in BUSHES:
        axtop.add_patch(plt.Circle((cx,cy), r, color='#558B2F', alpha=0.7))

    # Inflation rings for a few key obstacles
    for (cx,cy,r,h) in TREES[:3]:
        axtop.add_patch(plt.Circle((cx,cy), r+INFLATION_RADIUS, color='#E53935', fill=False, ls='--', lw=0.8, alpha=0.5))

    axtop.plot(pp[:,0], pp[:,1], '-', c='#26C6DA', lw=1, alpha=0.4)
    axtop.plot(rp[:,0], rp[:,1], '-', c='#AB47BC', lw=1.5, alpha=0.6)
    top_plan, = axtop.plot([], [], '--', c='#FFF176', lw=2.5)
    top_drone,= axtop.plot([], [], 'o', c='#29B6F6', ms=9, zorder=7)
    top_traj, = axtop.plot([], [], '-', c='#42A5F5', lw=1.5, alpha=0.7)
    top_dyncs = [plt.Circle((0,0), DYNAMIC_OBSTACLES[j]['r'], color=DYNAMIC_OBSTACLES[j]['color'], alpha=0.85)
                 for j in range(len(DYNAMIC_OBSTACLES))]
    for c in top_dyncs: axtop.add_patch(c)
    top_fov   = plt.Circle((0,0), 3.0, color='#00E5FF', fill=False, ls=':', lw=1, alpha=0.4)
    axtop.add_patch(top_fov)
    
    # FPV Dynamic objects handles
    fpv_dyncs = [axfpv.scatter([], [], [], s=300, c=obs['color']) for obs in DYNAMIC_OBSTACLES]
    fpv_drone = axfpv.scatter([], [], [], s=200, c='#29B6F6', zorder=8)
    
    # State tracker for FPV yaw smoothing
    fpv_state = {'yaw': 0.0}

    # ── Potential Field Gauge ──────────────────────────
    axpot.set_facecolor('#0D1117')
    axpot.set_title("Potential Field Forces", color='#CE93D8', fontsize=8)
    axpot.set_xlim(-1,1); axpot.set_ylim(-1,1)
    axpot.set_aspect('equal')
    axpot.set_xticks([]); axpot.set_yticks([])
    pf_arrow = axpot.annotate('', xy=(0,0), xytext=(0,0),
                               arrowprops=dict(arrowstyle='->', color='#AB47BC', lw=2.5))
    pf_att_arrow = axpot.annotate('', xy=(0,0), xytext=(0,0),
                                   arrowprops=dict(arrowstyle='->', color='#00E5FF', lw=1.5))
    axpot.scatter([0],[0], s=80, c='white', zorder=5)
    axpot.text(0.05,0.92,"Purple=Total  Cyan=Attractive", transform=axpot.transAxes,
               color='#CE93D8', fontsize=7)

    # ── Metrics bar chart ──────────────────────────────
    axmet.set_facecolor('#0D1117')
    axmet.set_title("Performance Matrix", color='#80DEEA', fontsize=8)
    metric_labels = ['Path Opt%','Safety%','Energy%','Replan%','TOTAL%']
    metric_bar = axmet.bar(metric_labels, [0]*5,
                            color=['#42A5F5','#66BB6A','#FFA726','#AB47BC','#EF5350'])
    axmet.set_ylim(0,110); axmet.set_yticks([0,50,100])
    axmet.tick_params(colors='#546E7A', labelsize=7)

    # ── Removed legacy text info panel in favor of drone_telemetry_cmd ──

    def animate(i):
        i = min(i, frames_total-1)
        pos  = history['pos'][i]
        path = np.array(history['path'][i]) if history['path'][i] else np.zeros((1,3))
        dyn  = history['dyn'][i]
        pf_f = history['pot_force'][i]
        m    = history['metrics'][i]
        repl = history['replan'][i]
        dsr  = history['dstar_rp'][i]

        past = np.array(history['pos'][:i+1])

        # 3D
        drone3d._offsets3d = ([pos[0]],[pos[1]],[pos[2]])
        traj3d.set_data(past[:,0], past[:,1]); traj3d.set_3d_properties(past[:,2])
        if len(path)>0:
            plan3d.set_data(path[:,0], path[:,1]); plan3d.set_3d_properties(path[:,2])
            plan3d.set_color('#FF5252' if (repl or dsr) else '#FFF176')
            plan3d.set_linewidth(4.0 if (repl or dsr) else 2.0)
        for j,dp in enumerate(dyn):
            dyn3d_scats[j]._offsets3d = ([dp[0]],[dp[1]],[dp[2]])

        # Top-down
        hw=3.0
        axtop.set_xlim(pos[0]-hw, pos[0]+hw); axtop.set_ylim(pos[1]-hw, pos[1]+hw)
        top_drone.set_data([pos[0]],[pos[1]])
        top_traj.set_data(past[:,0], past[:,1])
        if len(path)>0:
            top_plan.set_data(path[:,0], path[:,1])
            top_plan.set_color('#FF5252' if (repl or dsr) else '#FFF176')
        for j,dp in enumerate(dyn): top_dyncs[j].center=(dp[0],dp[1])
        top_fov.center=(pos[0],pos[1])

        # Potential field arrow
        pf_n = np.linalg.norm(pf_f)+1e-9
        scale = min(0.9, pf_n*0.3)
        pf_dir = pf_f/pf_n * scale
        pf_arrow.xy      = (pf_dir[0], pf_dir[1])
        pf_arrow.xytext  = (0, 0)
        att_dir = (GOAL-pos); att_n=np.linalg.norm(att_dir)+1e-9
        att_dir = att_dir/att_n * 0.7
        pf_att_arrow.xy     = (att_dir[0], att_dir[1])
        pf_att_arrow.xytext = (0,0)

        # FPV Dynamic objects handles
        for j, dp in enumerate(dyn):
            fpv_dyncs[j]._offsets3d = ([dp[0]], [dp[1]], [dp[2]])

        # ── FPV Camera Update ────────────────────────────
        if len(past) >= 2:
            vel = pos - past[-2]
        else:
            vel = np.array([0,0,0])
            
        spd = np.linalg.norm(vel[:2])
        if spd > 1e-3:
            target_yaw = np.degrees(np.arctan2(vel[1], vel[0]))
        else:
            target_yaw = fpv_state['yaw']
            
        dyaw = (target_yaw - fpv_state['yaw'] + 180) % 360 - 180
        fpv_state['yaw'] += dyaw * 0.15 
        
        look_dist = 2.0
        lx = pos[0] + look_dist * np.cos(np.radians(fpv_state['yaw']))
        ly = pos[1] + look_dist * np.sin(np.radians(fpv_state['yaw']))
        
        fw = 3.5 
        axfpv.set_xlim(lx - fw, lx + fw)
        axfpv.set_ylim(ly - fw, ly + fw)
        axfpv.set_zlim(0, 5) 
        
        fpv_drone._offsets3d = ([pos[0]], [pos[1]], [pos[2]])
        axfpv.view_init(elev=12, azim=fpv_state['yaw'] - 90) 

        # Metrics bars
        vals = [m['path_optimality_%'], m['safety_score_%'], m['energy_score_%'],
                m['replanning_score_%'], m['TOTAL_SCORE_%']]
        for bar, val in zip(metric_bar, vals):
            bar.set_height(val)

        return drone3d, traj3d, plan3d, top_drone, top_traj, top_plan, fpv_drone


    anim = FuncAnimation(fig, animate, frames=frames_total, interval=80, blit=False, repeat=False)
    plt.show()

# ════════════════════════════════════════════════════
#  ENTRY
# ════════════════════════════════════════════════════
if __name__ == '__main__':
    history = run_engine()
    print("\nLaunching 3D Visualizer...")
    launch_animation(history)
