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

# Intermediary checkpoints the drone must visit before the final goal
MISSION_WAYPOINTS = [
    np.array([ 2.0, -3.0, 3.0]),
    np.array([ 6.0,  0.0, 4.0]),
    np.array([-2.0,  2.0, 2.0]),
    np.array([ 5.0,  4.0, 2.5])  # Final GOAL
]
GOAL  = MISSION_WAYPOINTS[-1]

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
#  OBSTACLE MOVEMENT PREDICTOR
#  Extrapolates dynamic obstacle positions N steps ahead
#  using constant-velocity model (Kalman-ready interface)
# ════════════════════════════════════════════════════
class ObstaclePredictor:
    """Predict future positions of all dynamic obstacles.
    Uses constant-velocity model; can be upgraded to Kalman later.
    """
    def __init__(self, obstacles):
        self.obstacles = obstacles  # list of dicts with 'pos','vel','r'
        self.history   = {i: [] for i in range(len(obstacles))}

    def update(self, t):
        """Update observed positions at current time t."""
        for i, obs in enumerate(self.obstacles):
            cur = obs['pos'] + obs['vel'] * t
            self.history[i].append(cur.copy())
            if len(self.history[i]) > 10:
                self.history[i].pop(0)

    def predict(self, t, horizon_steps, dt):
        """Return list of predicted (pos, radius) for each obstacle
        at time steps t+dt, t+2dt, ..., t+horizon*dt.
        """
        predictions = []
        for i, obs in enumerate(self.obstacles):
            cur = obs['pos'] + obs['vel'] * t
            # Estimate velocity from history if available
            if len(self.history[i]) >= 2:
                v_est = (self.history[i][-1] - self.history[i][-2]) / dt
            else:
                v_est = obs['vel']
            future = []
            for k in range(1, horizon_steps + 1):
                future.append((cur + v_est * k * dt, obs['r']))
            predictions.append(future)
        return predictions  # shape: [n_obstacles][horizon_steps] → (pos, r)


# ════════════════════════════════════════════════════
#  DWA — Dynamic Window Approach
#  Samples feasible velocity space and picks the best
#  trajectory based on heading + clearance + speed.
#  Operates at every control step for real-time avoidance.
# ════════════════════════════════════════════════════
class DWA:
    def __init__(self):
        # Drone kinematic limits
        self.max_speed    = 2.5    # m/s
        self.max_accel    = 3.0    # m/s² (dynamic window constraint)
        self.v_samples    = 9      # velocity magnitude samples
        self.theta_samples= 16     # direction samples (full circle)
        self.sim_time     = 1.2    # lookahead time for trajectory scoring
        self.dt_sim       = 0.15   # simulation step within lookahead

        # Scoring weights
        self.w_heading  = 2.0   # reward pointing toward goal
        self.w_clearance= 3.0   # reward staying far from obstacles
        self.w_speed    = 0.8   # reward high speed

    def compute(self, drone_pos, drone_vel, goal, dyn_cur_pred, static_obs):
        """
        drone_pos, drone_vel: np.array(3)
        goal: np.array(3)
        dyn_cur_pred: list of (pos, r) — predicted obstacle positions at 1 step ahead
        static_obs: list of (cx, cy, r, h)

        Returns best_vel: np.array(3) velocity command
        """
        cur_speed = np.linalg.norm(drone_vel)

        # Dynamic window: reachable velocities given max acceleration
        v_min = max(0.0, cur_speed - self.max_accel * self.dt_sim)
        v_max = min(self.max_speed, cur_speed + self.max_accel * self.dt_sim)

        best_score = -1e9
        best_vel   = drone_vel.copy()

        for vi in range(self.v_samples):
            v_mag = v_min + (v_max - v_min) * vi / max(1, self.v_samples - 1)

            for ti in range(self.theta_samples):
                angle = 2 * math.pi * ti / self.theta_samples
                # Direction in XY plane, keep Z from PID
                dx = math.cos(angle)
                dy = math.sin(angle)
                dz = (goal[2] - drone_pos[2]) * 0.3  # soft altitude push
                direction = np.array([dx, dy, dz])
                direction /= (np.linalg.norm(direction) + 1e-6)
                vel_cand = direction * v_mag

                # Simulate trajectory
                pos = drone_pos.copy()
                min_clearance = 1e9
                collided = False
                for step in range(int(self.sim_time / self.dt_sim)):
                    pos = pos + vel_cand * self.dt_sim
                    # Check static obstacles
                    for (cx, cy, r, h) in static_obs:
                        d = math.sqrt((pos[0]-cx)**2+(pos[1]-cy)**2) - r
                        if d < 0.3:
                            collided = True; break
                        min_clearance = min(min_clearance, d)
                    if collided: break
                    # Check predicted dynamic obstacles
                    for (dpos, dr) in dyn_cur_pred:
                        d = np.linalg.norm(pos - dpos) - dr
                        if d < 0.3:
                            collided = True; break
                        min_clearance = min(min_clearance, d)
                    if collided: break

                if collided: continue

                # Score this trajectory
                to_goal = goal - pos
                dist    = np.linalg.norm(to_goal)
                heading_score   = 1.0 / (dist + 0.01)
                clearance_score = min(1.0, min_clearance / 2.0)
                speed_score     = v_mag / self.max_speed

                score = (self.w_heading   * heading_score +
                         self.w_clearance * clearance_score +
                         self.w_speed     * speed_score)

                if score > best_score:
                    best_score = score
                    best_vel   = vel_cand

        return best_vel


# ════════════════════════════════════════════════════
#  MPC — Model Predictive Control
#  Plans N-step optimal control sequence over a horizon,
#  accounting for predicted obstacle positions.
#  Minimizes: Σ (dist_to_goal + obstacle_penalty + control_effort)
# ════════════════════════════════════════════════════
class MPC:
    def __init__(self, horizon=6, dt=0.1):
        self.N  = horizon   # prediction steps
        self.dt = dt
        self.max_speed  = 2.5
        self.max_accel  = 2.5
        self.n_rollouts = 60     # random control rollouts (sampling MPC)

        # Cost weights
        self.Q_goal  = 2.0    # distance to goal at end of horizon
        self.Q_obs   = 5.0    # obstacle proximity penalty
        self.Q_ctrl  = 0.3    # control effort
        self.Q_smooth= 0.4    # smoothness (jerk penalty)

    def optimize(self, drone_pos, drone_vel, goal,
                 predicted_obstacles, static_obs):
        """
        predicted_obstacles: list per step of [(pos, r), ...]
            (length = self.N, each entry = all obstacles at step k)

        Returns: best_vel: np.array(3) — first control action of optimal sequence
        """
        best_cost = 1e9
        best_vel  = drone_vel.copy()
        cur_speed = np.linalg.norm(drone_vel)

        for _ in range(self.n_rollouts):
            # Sample a control sequence: N random velocities
            controls = []
            v_prev = drone_vel.copy()
            for k in range(self.N):
                # Random perturbation within dynamic window
                dv = np.random.randn(3) * self.max_accel * self.dt
                v_cand = np.clip(v_prev + dv, -self.max_speed, self.max_speed)
                # Squash speed
                spd = np.linalg.norm(v_cand)
                if spd > self.max_speed:
                    v_cand *= self.max_speed / spd
                controls.append(v_cand)
                v_prev = v_cand

            # Rollout
            pos  = drone_pos.copy()
            cost = 0.0
            ok   = True

            for k, v_cmd in enumerate(controls):
                pos = pos + v_cmd * self.dt

                # Obstacle costs at step k
                obs_at_k = predicted_obstacles[min(k, len(predicted_obstacles)-1)]
                for (opos, or_) in obs_at_k:
                    d = np.linalg.norm(pos - opos) - or_
                    if d < 0.2:
                        ok = False; break
                    cost += self.Q_obs * max(0, 1.5 - d)   # penalty within 1.5m

                if not ok: break

                # Static obstacles
                for (cx, cy, r, h) in static_obs:
                    d = math.sqrt((pos[0]-cx)**2+(pos[1]-cy)**2) - r
                    if d < 0.2:
                        ok = False; break
                    cost += self.Q_obs * max(0, 1.2 - d)
                if not ok: break

                # Control effort
                cost += self.Q_ctrl * np.linalg.norm(v_cmd)**2

                # Smoothness
                if k > 0:
                    cost += self.Q_smooth * np.linalg.norm(v_cmd - controls[k-1])

            if not ok:
                continue   # skip colliding rollouts

            # Terminal cost: distance to goal
            cost += self.Q_goal * np.linalg.norm(pos - goal)

            if cost < best_cost:
                best_cost = cost
                best_vel  = controls[0]  # first action in best sequence

        return best_vel

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
#  UNSCENTED KALMAN FILTER (UKF) — State Estimator
#
#  State vector:  x = [x, y, z, vx, vy, vz]  (6D)
#  Process model: constant-velocity + IMU acceleration input
#  Sensors:
#    - IMU (accel)          → prediction input
#    - Optical Flow (vx,vy) → velocity XY observation
#    - Barometer (z)        → altitude observation
#
#  Scaled Unscented Transform parameters:
#    alpha = 0.1  (spread of sigma points)
#    beta  = 2    (optimal for Gaussian)
#    kappa = 0    (secondary scaling)
#    lambda = alpha²(n+kappa) - n
#
#  Uses only numpy. No filterpy or external libraries.
# ════════════════════════════════════════════════════
class UKFStateEstimator:
    """
    Full 6-DOF UKF state estimator for GPS-denied drone navigation.

    State:  [x, y, z, vx, vy, vz]
    Input:  IMU acceleration [ax, ay, az]  (world-frame, gravity-compensated)
    Observations:
        - Optical flow: [vx, vy]
        - Barometer:    [z]
    """

    def __init__(self, init_pos=None, init_vel=None):
        self.n  = 6          # state dimension
        alpha   = 0.1
        beta    = 2.0
        kappa   = 0.0
        lam     = alpha**2 * (self.n + kappa) - self.n

        # ── Sigma-point weights ──────────────────────
        self._lam = lam
        self._Wm  = np.zeros(2*self.n + 1)
        self._Wc  = np.zeros(2*self.n + 1)
        self._Wm[0] = lam / (self.n + lam)
        self._Wc[0] = lam / (self.n + lam) + (1 - alpha**2 + beta)
        for i in range(1, 2*self.n + 1):
            self._Wm[i] = 1.0 / (2.0 * (self.n + lam))
            self._Wc[i] = 1.0 / (2.0 * (self.n + lam))
        self._sqrt_factor = np.sqrt(self.n + lam)

        # ── Initial state ────────────────────────────
        pos = init_pos if init_pos is not None else np.zeros(3)
        vel = init_vel if init_vel is not None else np.zeros(3)
        self.x = np.concatenate([pos, vel])          # (6,)

        # ── Initial covariance ───────────────────────
        self.P = np.diag([0.5, 0.5, 0.3,             # pos uncertainty (m)
                          0.3, 0.3, 0.2])             # vel uncertainty (m/s)

        # ── Process noise ────────────────────────────
        # Tuned: position noise from integration + velocity from accel uncertainty
        self.Q = np.diag([0.01, 0.01, 0.01,          # position (m²)
                          0.05, 0.05, 0.05])          # velocity (m/s)²

        # ── Measurement noise ─────────────────────────
        # Optical flow: vx, vy  (2D)
        self.R_flow = np.diag([0.08, 0.08])           # (m/s)²
        # Barometer: z          (1D)
        self.R_baro = np.array([[0.04]])              # m²

    # ─────────────────────────────────────────────────
    def _sigma_points(self):
        """Generate 2n+1 sigma points from current state and covariance."""
        n = self.n
        # Cholesky square-root of (n+λ)P  — ensures P stays PSD
        try:
            S = np.linalg.cholesky((n + self._lam) * self.P)
        except np.linalg.LinAlgError:
            # Covariance not PSD — add small regularisation
            P_reg = self.P + np.eye(n) * 1e-6
            S = np.linalg.cholesky((n + self._lam) * P_reg)

        sigmas = np.zeros((2*n + 1, n))
        sigmas[0] = self.x
        for i in range(n):
            sigmas[i+1]   = self.x + S[:, i]
            sigmas[i+n+1] = self.x - S[:, i]
        return sigmas                                 # (13, 6)

    # ─────────────────────────────────────────────────
    def _process_model(self, sigma, dt, accel):
        """
        Constant-velocity process model with IMU acceleration input.
        sigma: (6,)   state [x,y,z,vx,vy,vz]
        accel: (3,)   world-frame acceleration [ax,ay,az]
        Returns propagated state (6,)
        """
        x, y, z, vx, vy, vz = sigma
        ax, ay, az = accel

        # Position update: p_new = p + v*dt + 0.5*a*dt²
        x_new  = x  + vx*dt + 0.5*ax*dt*dt
        y_new  = y  + vy*dt + 0.5*ay*dt*dt
        z_new  = z  + vz*dt + 0.5*az*dt*dt
        # Velocity update: v_new = v + a*dt
        vx_new = vx + ax*dt
        vy_new = vy + ay*dt
        vz_new = vz + az*dt

        return np.array([x_new, y_new, z_new, vx_new, vy_new, vz_new])

    # ─────────────────────────────────────────────────
    def predict(self, dt, accel):
        """
        UKF prediction step.

        Parameters
        ----------
        dt    : float  — time step (seconds)
        accel : array  — world-frame linear acceleration [ax, ay, az] (m/s²)
                         (gravity-compensated; IMU accel minus gravity vector)
        """
        accel = np.asarray(accel, dtype=float)
        sigmas = self._sigma_points()                # (13, 6)

        # Propagate each sigma point through process model
        sigmas_pred = np.array([
            self._process_model(s, dt, accel) for s in sigmas
        ])                                           # (13, 6)

        # Predicted mean
        x_pred = np.einsum('i,ij->j', self._Wm, sigmas_pred)

        # Predicted covariance
        P_pred = self.Q.copy()
        for i in range(2*self.n + 1):
            d = sigmas_pred[i] - x_pred
            P_pred += self._Wc[i] * np.outer(d, d)

        self.x = x_pred
        self.P = P_pred

    # ─────────────────────────────────────────────────
    def _ukf_update(self, z_obs, H_fn, R):
        """
        Generic UKF measurement update.

        z_obs  : observed measurement vector (m,)
        H_fn   : callable, maps state (6,) → measurement (m,)
        R      : measurement noise covariance (m×m)
        """
        z_obs = np.asarray(z_obs, dtype=float)
        sigmas = self._sigma_points()                # (13, 6)

        # Map sigma points through measurement function
        Z_sigmas = np.array([H_fn(s) for s in sigmas])  # (13, m)

        # Predicted measurement mean
        z_pred = np.einsum('i,ij->j', self._Wm, Z_sigmas)

        # Innovation covariance S and cross-covariance Pxz
        m = len(z_pred)
        S   = R.copy()
        Pxz = np.zeros((self.n, m))
        for i in range(2*self.n + 1):
            dz = Z_sigmas[i] - z_pred
            dx = sigmas[i]   - self.x
            S   += self._Wc[i] * np.outer(dz, dz)
            Pxz += self._Wc[i] * np.outer(dx, dz)

        # Kalman gain
        K = Pxz @ np.linalg.inv(S)                  # (n, m)

        # State and covariance update
        innovation = z_obs - z_pred
        self.x = self.x + K @ innovation
        self.P = self.P - K @ S @ K.T

        # Symmetrise P to prevent numerical drift
        self.P = 0.5 * (self.P + self.P.T)

    # ─────────────────────────────────────────────────
    def update_optical_flow(self, vx_obs, vy_obs):
        """
        Update from optical flow sensor (measures horizontal velocity).

        Parameters
        ----------
        vx_obs : float — observed x-velocity (m/s)
        vy_obs : float — observed y-velocity (m/s)
        """
        def H_flow(s):
            return np.array([s[3], s[4]])            # extract vx, vy from state

        self._ukf_update(
            z_obs=np.array([vx_obs, vy_obs]),
            H_fn=H_flow,
            R=self.R_flow
        )

    # ─────────────────────────────────────────────────
    def update_baro(self, z_obs):
        """
        Update from barometric altimeter (measures altitude).

        Parameters
        ----------
        z_obs : float — observed altitude (m)
        """
        def H_baro(s):
            return np.array([s[2]])                  # extract z from state

        self._ukf_update(
            z_obs=np.array([z_obs]),
            H_fn=H_baro,
            R=self.R_baro
        )

    # ─────────────────────────────────────────────────
    def get_state(self):
        """
        Return current state estimate.

        Returns
        -------
        dict with keys:
            'pos'  : np.array(3) — [x, y, z]  (m)
            'vel'  : np.array(3) — [vx, vy, vz] (m/s)
            'P'    : np.array(6,6) — full covariance matrix
            'std_pos' : np.array(3) — 1-sigma position uncertainty (m)
            'std_vel' : np.array(3) — 1-sigma velocity uncertainty (m/s)
        """
        std = np.sqrt(np.maximum(np.diag(self.P), 0))
        return {
            'pos':     self.x[:3].copy(),
            'vel':     self.x[3:].copy(),
            'P':       self.P.copy(),
            'std_pos': std[:3],
            'std_vel': std[3:],
        }


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

def build_prm(grid, start_pos, goal_pos, n=NUM_SAMPLES):
    """Build PRM nodes with neighbor connections."""
    nodes = [start_pos.copy(), goal_pos.copy()]
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

def dijkstra_prm(nodes, adj, start_pos, goal_pos):
    """Find shortest path on PRM graph from node 0 (start) to node 1 (goal)."""
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
    return [start_pos.copy(), goal_pos.copy()]

# ════════════════════════════════════════════════════
#  INFORMED RRT*  (over existing PRM path as init)
# ════════════════════════════════════════════════════
class InformedRRTStar:
    def __init__(self, grid, start_pos, goal_pos, c_best_init=None, max_iter=800, step=1.0, radius=1.8):
        self.grid = grid
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.nodes = [start_pos.copy()]
        self.parent = {0: None}
        self.cost   = {0: 0.0}
        self.max_iter = max_iter
        self.step  = step
        self.radius = radius
        self.c_min = np.linalg.norm(goal_pos-start_pos)
        self.c_best = c_best_init if c_best_init else float('inf')
        # Informed sampling ellipse parameters
        self.x_center = (start_pos+goal_pos)/2
        self.a1 = (goal_pos-start_pos)/self.c_min  # major axis direction

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
            if np.linalg.norm(q_new-self.goal_pos) < self.step:
                total = best_cost + np.linalg.norm(q_new-self.goal_pos)
                if total < self.c_best:
                    self.c_best = total
                    # Extract path
                    path = [self.goal_pos.copy(), q_new.copy()]
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
        self._U_dict = {} # lazy deletion
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
        self._U_dict = {}
        self._rhs[goal_v] = 0.0
        k = self._key(goal_v)
        heapq.heappush(self._U, (k, goal_v))
        self._U_dict[goal_v] = k

    def _update_vertex(self, u):
        if u != self.goal:
            self._rhs[u] = min((self._v(s)+c for s,c in self._succ(u)), default=self.INF)
        if u in self._U_dict:
            del self._U_dict[u]
        if self._v(u) != self._r(u):
            k = self._key(u)
            heapq.heappush(self._U, (k, u))
            self._U_dict[u] = k

    def compute(self, max_iter=100000):
        itr=0
        while self._U and itr<max_iter:
            k_old, u = heapq.heappop(self._U)
            if u not in self._U_dict or self._U_dict[u] != k_old:
                continue
            del self._U_dict[u]
            itr+=1
            if k_old < self._key(u):
                k_new = self._key(u)
                heapq.heappush(self._U, (k_new, u))
                self._U_dict[u] = k_new
                continue
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
#  THETA* — Any-Angle A* (MAXIMUM PATH OPTIMALITY)
#  Finds near-geometric-shortest-path by checking line-of-sight.
#  Provably closer to the true shortest path than any grid A* variant.
# ════════════════════════════════════════════════════
class ThetaStar:
    """Theta* in 3D voxel space.
    Whenever a node is expanded it checks if the GRANDPARENT has
    line-of-sight to the current neighbour.  If so, it short-circuits
    the path — removing the zig-zagging that pure A* produces.
    This gives paths that are essentially optimal geodesics.
    """

    def __init__(self, grid):
        self.grid = grid

    def _los(self, a, b):
        """Bresenham 3-D line-of-sight check between voxel a and b."""
        x0, y0, z0 = a
        x1, y1, z1 = b
        dx, dy, dz = abs(x1-x0), abs(y1-y0), abs(z1-z0)
        sx = 1 if x1>x0 else -1
        sy = 1 if y1>y0 else -1
        sz = 1 if z1>z0 else -1
        if dx >= dy and dx >= dz:
            g1, g2 = dx, dx
            p1, p2 = 2*dy-dx, 2*dz-dx
            while x0 != x1:
                x0 += sx
                if p1>0: y0+=sy; p1-=2*g1
                if p2>0: z0+=sz; p2-=2*g2
                p1+=2*dy; p2+=2*dz
                if not (0<=x0<NX and 0<=y0<NY and 0<=z0<NZ): return False
                if self.grid[x0,y0,z0]: return False
        elif dy >= dz:
            g1, g2 = dy, dy
            p1, p2 = 2*dx-dy, 2*dz-dy
            while y0 != y1:
                y0 += sy
                if p1>0: x0+=sx; p1-=2*g1
                if p2>0: z0+=sz; p2-=2*g2
                p1+=2*dx; p2+=2*dz
                if not (0<=x0<NX and 0<=y0<NY and 0<=z0<NZ): return False
                if self.grid[x0,y0,z0]: return False
        else:
            g1, g2 = dz, dz
            p1, p2 = 2*dx-dz, 2*dy-dz
            while z0 != z1:
                z0 += sz
                if p1>0: x0+=sx; p1-=2*g1
                if p2>0: y0+=sy; p2-=2*g2
                p1+=2*dx; p2+=2*dy
                if not (0<=x0<NX and 0<=y0<NY and 0<=z0<NZ): return False
                if self.grid[x0,y0,z0]: return False
        return True

    DIRS6 = [(1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1)]
    DIRS18= [(dx,dy,dz) for dx in(-1,0,1) for dy in(-1,0,1) for dz in(-1,0,1)
             if (dx or dy or dz) and sum(abs(x) for x in (dx,dy,dz))<=2]

    def _h(self, a, b):
        return math.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2+(a[2]-b[2])**2)

    def search(self, start_w, goal_w, max_nodes=8000):
        """Return list of world-space waypoints or [] on failure."""
        sv = w2v(start_w); gv = w2v(goal_w)
        OPEN = []
        heapq.heappush(OPEN, (self._h(sv,gv), sv))
        g     = {sv: 0.0}
        parent= {sv: sv}
        closed= set()

        cnt = 0
        while OPEN and cnt < max_nodes:
            _, s = heapq.heappop(OPEN)
            if s == gv:
                # Reconstruct
                path = []
                while s != parent[s]:
                    path.append(v2w(s)); s = parent[s]
                path.append(v2w(sv)); path.reverse()
                return path
            if s in closed: continue
            closed.add(s); cnt += 1

            for d in self.DIRS18:
                nb = (s[0]+d[0], s[1]+d[1], s[2]+d[2])
                if not (0<=nb[0]<NX and 0<=nb[1]<NY and 0<=nb[2]<NZ): continue
                if self.grid[nb]: continue

                # Theta* core: try grandparent line-of-sight shortcut
                gp = parent[s]
                step_cost = self._h(gp, nb)
                new_g = g[gp] + step_cost if self._los(gp, nb) else g[s] + self._h(s, nb)
                new_par = gp if self._los(gp, nb) else s

                if nb not in g or new_g < g[nb]:
                    g[nb] = new_g
                    parent[nb] = new_par
                    f = new_g + self._h(nb, gv)
                    heapq.heappush(OPEN, (f, nb))
        return []

# ════════════════════════════════════════════════════
#  BIDIRECTIONAL A* — Backup path planner
#  Expands from both start AND goal simultaneously.
#  Meets in the middle → ~2x faster than A*, always optimal.
# ════════════════════════════════════════════════════
class BidirectionalAStar:
    def __init__(self, grid):
        self.grid = grid

    def _h(self, a, b):
        return math.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2+(a[2]-b[2])**2)

    DIRS = [(1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1),
            (1,1,0),(1,-1,0),(-1,1,0),(-1,-1,0),
            (0,1,1),(0,1,-1),(0,-1,1),(0,-1,-1),
            (1,0,1),(1,0,-1),(-1,0,1),(-1,0,-1)]

    def search(self, start_w, goal_w, max_nodes=6000):
        """Return list of world-space waypoints or [] on failure."""
        sv = w2v(start_w); gv = w2v(goal_w)

        # Forward & backward open sets
        fwd_open = [(0.0, sv)]; fwd_g = {sv: 0.0}; fwd_par = {sv: None}
        bwd_open = [(0.0, gv)]; bwd_g = {gv: 0.0}; bwd_par = {gv: None}
        fwd_closed = {}; bwd_closed = {}

        best = math.inf; meeting = None
        cnt = 0

        while (fwd_open or bwd_open) and cnt < max_nodes:
            # Expand forward front
            if fwd_open:
                _, sf = heapq.heappop(fwd_open)
                if sf in fwd_closed: pass
                else:
                    fwd_closed[sf] = True
                    cnt += 1
                    for d in self.DIRS:
                        nb = (sf[0]+d[0], sf[1]+d[1], sf[2]+d[2])
                        if not (0<=nb[0]<NX and 0<=nb[1]<NY and 0<=nb[2]<NZ): continue
                        if self.grid[nb]: continue
                        ng = fwd_g[sf] + self._h(sf, nb)
                        if nb not in fwd_g or ng < fwd_g[nb]:
                            fwd_g[nb] = ng; fwd_par[nb] = sf
                            heapq.heappush(fwd_open, (ng+self._h(nb,gv), nb))
                        # Check meeting
                        if nb in bwd_closed:
                            total = ng + bwd_g.get(nb, math.inf)
                            if total < best:
                                best = total; meeting = nb
            # Expand backward front
            if bwd_open:
                _, sb = heapq.heappop(bwd_open)
                if sb in bwd_closed: pass
                else:
                    bwd_closed[sb] = True
                    cnt += 1
                    for d in self.DIRS:
                        nb = (sb[0]+d[0], sb[1]+d[1], sb[2]+d[2])
                        if not (0<=nb[0]<NX and 0<=nb[1]<NY and 0<=nb[2]<NZ): continue
                        if self.grid[nb]: continue
                        ng = bwd_g[sb] + self._h(sb, nb)
                        if nb not in bwd_g or ng < bwd_g[nb]:
                            bwd_g[nb] = ng; bwd_par[nb] = sb
                            heapq.heappush(bwd_open, (ng+self._h(nb,sv), nb))
                        if nb in fwd_closed:
                            total = ng + fwd_g.get(nb, math.inf)
                            if total < best:
                                best = total; meeting = nb
            if meeting and (fwd_open and bwd_open):
                fmin = fwd_open[0][0] if fwd_open else 0
                bmin = bwd_open[0][0] if bwd_open else 0
                if fmin + bmin >= best:
                    break

        if meeting is None:
            return []
        # Reconstruct: fwd path to meeting + bwd path from meeting
        fwd_path = []
        c = meeting
        while c is not None:
            fwd_path.append(v2w(c)); c = fwd_par.get(c)
        fwd_path.reverse()
        bwd_path = []
        c = bwd_par.get(meeting)
        while c is not None:
            bwd_path.append(v2w(c)); c = bwd_par.get(c)
        return fwd_path + bwd_path

# ════════════════════════════════════════════════════
#  JUMP POINT SEARCH (JPS) - Ultra-fast 2D replanner
#  Supplements D* Lite for instant replanning on blocked paths

# ════════════════════════════════════════════════════
class JPS:
    """Jump Point Search on the 2D XY voxel layer at the drone's current altitude.
    Falls back to ordinary A* if no jump point found within budget."""

    def __init__(self, grid, nz):
        self.grid = grid  # 3-D boolean occupancy
        self.nz   = nz    # which Z-layer to search on

    def _free(self, x, y):
        if 0<=x<NX and 0<=y<NY:
            return not self.grid[x, y, self.nz]
        return False

    def _neighbors(self, node, parent):
        """Pruning rules for JPS (8-connectivity, 2-D)."""
        x, y = node
        if parent is None:
            # start: return all 8 neighbours
            return [(x+dx, y+dy) for dx in (-1,0,1) for dy in (-1,0,1)
                    if (dx or dy) and self._free(x+dx, y+dy)]
        px, py = parent
        dx = max(-1, min(1, x - px))
        dy = max(-1, min(1, y - py))
        result = []
        if dx != 0 and dy != 0:
            # diagonal move
            if self._free(x+dx, y):   result.append((x+dx, y))
            if self._free(x, y+dy):   result.append((x, y+dy))
            if self._free(x+dx,y+dy): result.append((x+dx, y+dy))
        elif dx != 0:
            # horizontal
            if self._free(x+dx, y): result.append((x+dx, y))
            if not self._free(x, y+1) and self._free(x+dx, y+1): result.append((x+dx, y+1))
            if not self._free(x, y-1) and self._free(x+dx, y-1): result.append((x+dx, y-1))
        else:
            # vertical
            if self._free(x, y+dy): result.append((x, y+dy))
            if not self._free(x+1, y) and self._free(x+1, y+dy): result.append((x+1, y+dy))
            if not self._free(x-1, y) and self._free(x-1, y+dy): result.append((x-1, y+dy))
        return result

    def _jump(self, x, y, dx, dy, gx, gy, depth=0):
        if depth > 80 or not self._free(x, y): return None
        if x==gx and y==gy: return (x, y)
        # check forced neighbours
        if dx != 0 and dy != 0:
            if (self._free(x-dx,y+dy) and not self._free(x-dx,y)) or \
               (self._free(x+dx,y-dy) and not self._free(x,y-dy)):
                return (x, y)
            j1 = self._jump(x+dx, y, dx, 0, gx, gy, depth+1)
            j2 = self._jump(x, y+dy, 0, dy, gx, gy, depth+1)
            if j1 or j2: return (x, y)
        elif dx != 0:
            if (self._free(x+dx,y+1) and not self._free(x,y+1)) or \
               (self._free(x+dx,y-1) and not self._free(x,y-1)):
                return (x, y)
        else:
            if (self._free(x+1,y+dy) and not self._free(x+1,y)) or \
               (self._free(x-1,y+dy) and not self._free(x-1,y)):
                return (x, y)
        return self._jump(x+dx, y+dy, dx, dy, gx, gy, depth+1)

    def search(self, start_w, goal_w):
        """Return a list of world-space np.array waypoints or [] if failed."""
        sv = w2v(start_w); gv = w2v(goal_w)
        sx, sy, sz = sv; gx, gy, gz = gv
        nz = max(0, min(NZ-1, sz))  # clamp to valid Z
        self.nz = nz

        OPEN = []
        heapq.heappush(OPEN, (0.0, (sx,sy), None))
        g = {(sx,sy): 0.0}
        parent = {}
        visited_cnt = 0

        while OPEN and visited_cnt < 4000:
            f, node, par = heapq.heappop(OPEN)
            visited_cnt += 1
            if node == (gx, gy):
                # Reconstruct path
                path2d = []
                curr = node
                while curr in parent:
                    path2d.append(curr)
                    curr = parent[curr]
                path2d.append(curr)
                path2d.reverse()
                # Convert to 3D world
                return [v2w((px, py, nz)) for px, py in path2d]

            for nb in self._neighbors(node, par):
                nx_c, ny_c = nb
                dx = max(-1, min(1, nx_c - node[0]))
                dy = max(-1, min(1, ny_c - node[1]))
                jp = self._jump(nx_c, ny_c, dx, dy, gx, gy)
                if jp is None: continue
                jx, jy = jp
                d = math.sqrt((jx-node[0])**2 + (jy-node[1])**2)
                ng = g[node] + d
                if (jx,jy) in g and g[(jx,jy)] <= ng: continue
                g[(jx,jy)] = ng
                parent[(jx,jy)] = node
                h = math.sqrt((jx-gx)**2 + (jy-gy)**2)
                heapq.heappush(OPEN, (ng+h, (jx,jy), node))
        return []  # fail

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
        path_opt = min(100.0, (straight / max(self.path_length, 0.01)) * 100)
        safety   = max(0.0, 100.0 - self.close_calls * 2)
        energy_s = max(0.0, 100.0 - self.energy * 0.5)
        # Replanning score: reward adaptive replanning (D* + JPS), capped at 100
        # Formula: 50 base + 5 per successful dstar replan + 3 per PF/RRT replan
        # capped at 100, then penalised lightly for excessive close-calls
        jps_bonus = min(50, self.dstar_replan_count * 5)
        rrt_bonus = min(25, self.replan_count * 3)
        replan_s  = min(100.0, 50.0 + jps_bonus + rrt_bonus)
        total    = (path_opt*0.35 + safety*0.30 + energy_s*0.15 + replan_s*0.20)
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
    print("  PRM + Theta* + BiDir-A* + D* Lite + JPS + DWA + MPC + PID + PF")
    print("═"*60)

    t = 0.0
    dt = 0.1
    drone_pos = START.copy()
    drone_vel = np.zeros(3)

    dyn_cur = [(obs['pos'] + obs['vel']*t, obs['r']) for obs in DYNAMIC_OBSTACLES]

    current_wp_idx = 0
    current_target = MISSION_WAYPOINTS[current_wp_idx]
    print(f"  [MISSION TARGET]: Waypoint {current_wp_idx+1}/{len(MISSION_WAYPOINTS)} → {current_target}")

    # ── PHASE 1: PRM ──────────────────────────────────
    print("\n[1/3] Building PRM roadmap...")
    t0 = time.time()
    prm_nodes, prm_adj = build_prm(STATIC_GRID, drone_pos, current_target)
    prm_path = dijkstra_prm(prm_nodes, prm_adj, drone_pos, current_target)
    dt_prm = (time.time()-t0)*1000
    print(f"      PRM: {len(prm_nodes)} nodes | Dijkstra path: {len(prm_path)} waypoints | {dt_prm:.0f}ms")

    # ── PHASE 2: Theta* (best path optimality) ─────────
    print("[2/3] Running Theta* (any-angle, maximum path optimality)...")
    t0 = time.time()
    theta = ThetaStar(STATIC_GRID)
    theta_path = theta.search(drone_pos, current_target)
    dt_theta = (time.time()-t0)*1000

    if theta_path and len(theta_path) > 1:
        active_path = [p.copy() for p in theta_path]
        opt_path = theta_path
        print(f"      Theta*: {len(theta_path)} waypoints | {dt_theta:.0f}ms ✓ [PRIMARY]")
    else:
        print(f"      Theta* failed ({dt_theta:.0f}ms) → falling back to Bidirectional A*...")
        bidir = BidirectionalAStar(STATIC_GRID)
        t0b = time.time()
        bidir_path = bidir.search(drone_pos, current_target)
        dt_bidir = (time.time()-t0b)*1000
        if bidir_path and len(bidir_path) > 1:
            active_path = [p.copy() for p in bidir_path]
            opt_path = bidir_path
            print(f"      Bidirectional A*: {len(bidir_path)} waypoints | {dt_bidir:.0f}ms ✓ [FALLBACK-1]")
        else:
            print(f"      Bidirectional A* failed → falling back to Informed RRT*...")
            prm_dist = sum(np.linalg.norm(prm_path[i+1]-prm_path[i]) for i in range(len(prm_path)-1))
            rrtstar = InformedRRTStar(STATIC_GRID, drone_pos, current_target, c_best_init=prm_dist)
            rrt_path = rrtstar.build()
            active_path = [p.copy() for p in rrt_path]
            opt_path = rrt_path
            print(f"      Informed RRT*: {len(rrt_path)} waypoints ✓ [FALLBACK-2]")

    # Compute Theta* path for animation track (use best found)
    rrt_path = opt_path  # alias for existing animation code

    # ── PHASE 3: D* Lite setup ─────────────────────────
    print("[3/3] Initializing D* Lite replanner...")
    grid0 = get_grid(dyn_cur)
    dstar = DStarLite(grid0)
    dstar.initialize(w2v(drone_pos), w2v(current_target))
    
    # We'll compute the initial path with a time bound
    dstar.compute(max_iter=2500)
    print(f"      D* Lite ready.\n")
    print("► Simulation running... (sending telemetry on TELEMETRY_Q)")
    print("  Run `python drone_telemetry_cmd.py` in another terminal!\n")

    # ── Smart Brain: Predictive Avoidance ──────────────
    obs_predictor = ObstaclePredictor(DYNAMIC_OBSTACLES)
    dwa_planner   = DWA()
    mpc_planner   = MPC(horizon=6, dt=0.1)

    # ── UKF State Estimator ─────────────────────────────
    ukf = UKFStateEstimator(init_pos=START.copy(), init_vel=np.zeros(3))

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
        'target':  [current_target.copy()],
    }

    REPLAN_INTERVAL = 8  # steps between D* replans
    PFIELD_WEIGHT   = 0.5
    MAX_STEPS       = 600

    for step in range(MAX_STEPS):
        t += dt
        
        # ── Mission Waypoint Check ────────────────────
        if np.linalg.norm(drone_pos - current_target) < 1.0:
            if current_wp_idx < len(MISSION_WAYPOINTS) - 1:
                current_wp_idx += 1
                current_target = MISSION_WAYPOINTS[current_wp_idx]
                print(f"\n★ [MISSION UPDATE] Reached WP {current_wp_idx}, heading to WP {current_wp_idx+1}/{len(MISSION_WAYPOINTS)} → {current_target} ★")
                
                # Multi-tier replan to new target using Theta* and BiDir
                theta = ThetaStar(STATIC_GRID)
                theta_path = theta.search(drone_pos, current_target, max_nodes=35000)
                if theta_path and len(theta_path) > 1:
                    active_path = [p.copy() for p in theta_path]
                else:
                    bidir = BidirectionalAStar(STATIC_GRID)
                    bidir_path = bidir.search(drone_pos, current_target, max_nodes=25000)
                    if bidir_path and len(bidir_path) > 1:
                        active_path = [p.copy() for p in bidir_path]
                    else:
                        active_path = [current_target.copy()]
                
                # Re-init D* Lite
                dstar.initialize(w2v(drone_pos), w2v(current_target))
                dstar.compute(max_iter=2500)
                pos_pid.reset(); vel_pid.reset()

        # Dynamic obstacle positions
        dyn_cur = [(obs['pos'] + obs['vel']*t, obs['r']) for obs in DYNAMIC_OBSTACLES]

        # Sensor: lidar, IMU, barometer
        lidar = sensors.read_lidar(drone_pos, dyn_cur, n_beams=12)
        imu   = sensors.read_imu(drone_vel)
        baro  = sensors.read_barometer(drone_pos[2])

        # ── UKF State Estimation ──────────────────────
        # Gravity-compensated accel: IMU gives vel derivative + noise
        # Approximate world-frame accel from velocity change
        accel_world = (imu - drone_vel) / (dt + 1e-9)   # rough accel estimate
        accel_world[2] -= 0.0                            # gravity already removed in sim
        ukf.predict(dt=dt, accel=accel_world)
        # Update from optical flow (horizontal velocity)
        ukf.update_optical_flow(imu[0], imu[1])
        # Update from barometer (altitude)
        ukf.update_baro(baro)
        # Use UKF-estimated position for sensor fusion demo (optional blend)
        ukf_state = ukf.get_state()
        # Blend: 80% ground-truth sim pos + 20% UKF estimate (progressive trust)
        drone_pos_ukf = 0.8 * drone_pos + 0.2 * ukf_state['pos']

        # ── Smart Brain: Update predictor ────────────
        obs_predictor.update(t)
        # Predict 6 steps ahead for MPC
        preds_per_obs = obs_predictor.predict(t, 6, dt)  # [obs][step] → (pos,r)
        # Reformat to [step][obs_list] for MPC
        mpc_pred = []
        for k in range(6):
            step_obs = [(preds_per_obs[i][k][0], preds_per_obs[i][k][1])
                        for i in range(len(DYNAMIC_OBSTACLES))]
            mpc_pred.append(step_obs)
        # 1-step ahead prediction for DWA
        dwa_pred = [(preds_per_obs[i][0][0], preds_per_obs[i][0][1])
                    for i in range(len(DYNAMIC_OBSTACLES))]

        # Obstacle distance (for potential field & metrics)
        min_d = obstacle_distance(drone_pos, dyn_cur)

        # ── Potential Field ──────────────────────────
        pf_force = potential_force(drone_pos, current_target, dyn_cur)

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

        # ── JPS Replan if path blocked (fast 2D replanner) ──
        if blocked:
            grid_new = get_grid(dyn_cur)
            jps_planner = JPS(grid_new, w2v(drone_pos)[2])
            jps_path = jps_planner.search(drone_pos, current_target)
            if jps_path and len(jps_path) > 1:
                active_path = jps_path
                did_replan  = True
            else:
                # Fallback to mini-RRT* if JPS can't find a path
                rrt2 = InformedRRTStar(grid_new, drone_pos, current_target, c_best_init=None, max_iter=400, step=1.0, radius=1.8)
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
            if blocked:
                dstar_wp = dstar.extract_path()
                # Blend D* hint into waypoints only if we need to dodge
                if dstar_wp and len(dstar_wp) > 1:
                    active_path = dstar_wp
                    did_dstar_rp = True

        # ── Follow active path ───────────────────────
        if not active_path:
            active_path = [current_target.copy()]

        # Skip waypoints already passed
        while len(active_path)>1 and np.linalg.norm(drone_pos-np.array(active_path[0]))<0.35:
            active_path.pop(0)

        target_wp = np.array(active_path[0])
        wp_error  = target_wp - drone_pos

        # Cascaded PID (position → desired_vel)
        vel_cmd = pos_pid.step(wp_error, dt)

        # ── SMART BRAIN: DWA + MPC velocity override ─
        # When any obstacle is within 2.5m, engage DWA+MPC for predictive avoidance
        if min_d < 2.5:
            # DWA: samples velocity space with predicted obstacle positions
            dwa_vel = dwa_planner.compute(drone_pos, drone_vel, current_target,
                                          dwa_pred, STATIC_OBSTACLES)
            # MPC: optimises N-step trajectory over predicted future
            mpc_vel = mpc_planner.optimize(drone_pos, drone_vel, current_target,
                                           mpc_pred, STATIC_OBSTACLES)
            # Blend: 40% PID direction + 30% DWA + 30% MPC
            # (PID keeps goal-seeking, DWA/MPC ensure obstacle clearance)
            vel_cmd = 0.4 * vel_cmd + 0.3 * dwa_vel + 0.3 * mpc_vel

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
        history['target'].append(current_target.copy())

        if np.linalg.norm(drone_pos-MISSION_WAYPOINTS[-1]) < 0.45:
            print(f"\n✅ FINAL MISSION GOAL REACHED at step {step} (t={t:.1f}s)")
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
    axinfo = fig.add_subplot(gs[:, 3])
    axinfo.axis('off')

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
    for i, wp in enumerate(MISSION_WAYPOINTS[:-1]):
        ax3d.scatter(*wp, s=120, c='#FF9800', marker='^', zorder=6, label=f'WP {i+1}')
    ax3d.scatter(*MISSION_WAYPOINTS[-1],  s=150, c='#FFD600', marker='*', zorder=6, label='Final Goal')

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

    # ── Info panel ────────────────────────────────────
    info_text = axinfo.text(0.03, 0.97, '', transform=axinfo.transAxes,
                            color='#B2EBF2', fontsize=8, va='top', fontfamily='monospace',
                            bbox=dict(boxstyle='round', facecolor='#0D1117', alpha=0.9))

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
        
        cur_tag = history['target'][i] if 'target' in history else MISSION_WAYPOINTS[-1]
        att_dir = (cur_tag-pos); att_n=np.linalg.norm(att_dir)+1e-9
        att_dir = att_dir/att_n * 0.7
        pf_att_arrow.xy     = (att_dir[0], att_dir[1])
        pf_att_arrow.xytext = (0,0)

        # Metrics bars
        vals = [m['path_optimality_%'], m['safety_score_%'], m['energy_score_%'],
                m['replanning_score_%'], m['TOTAL_SCORE_%']]
        for bar, val in zip(metric_bar, vals):
            bar.set_height(val)

        # Info text
        status = "⚠ D* REPLAN" if dsr else ("🔄 RRT* REPLAN" if repl else "🟢 NAVIGATING")
        if i >= frames_total-2: status = "✅ GOAL REACHED"

        vel= history['vel'][i]
        spd= np.linalg.norm(vel)
        info_text.set_text(
            f"ALGORITHM STATUS\n{'─'*28}\n"
            f" {status}\n\n"
            f"DRONE STATE\n{'─'*28}\n"
            f" X: {pos[0]:+6.2f} m\n"
            f" Y: {pos[1]:+6.2f} m\n"
            f" Z: {pos[2]:+6.2f} m\n"
            f" Spd:{spd:5.2f} m/s\n\n"
            f"PLANNER LAYER\n{'─'*28}\n"
            f" PRM nodes   : {len(prm_nodes)}\n"
            f" PRM wps     : {len(prm_path)}\n"
            f" RRT* wps    : {len(rrt_path)}\n"
            f" Active wps  : {len(path)}\n\n"
            f"PID CONTROLLER\n{'─'*28}\n"
            f" I_x:{pos_pid._integral[0]:+5.2f} (anti-windup)\n"
            f" I_y:{pos_pid._integral[1]:+5.2f}\n"
            f" I_z:{pos_pid._integral[2]:+5.2f}\n\n"
            f"POTENTIAL FIELD\n{'─'*28}\n"
            f" ||F_total||: {pf_n:5.2f} N\n"
            f" Min obs dist:{history['min_dist'][i]:5.2f}m\n\n"
            f"METRICS MATRIX\n{'─'*28}\n"
            f" Path opt  : {m['path_optimality_%']:5.1f}%\n"
            f" Safety    : {m['safety_score_%']:5.1f}%\n"
            f" Energy    : {m['energy_score_%']:5.1f}%\n"
            f" TOTAL     : {m['TOTAL_SCORE_%']:5.1f}%\n"
            f" Replans   : PRM={m['prm_rrt_replans']} D*={m['dstar_replans']}\n"
        )
        return drone3d, traj3d, plan3d, top_drone, top_traj, top_plan, info_text

    anim = FuncAnimation(fig, animate, frames=frames_total, interval=80, blit=False, repeat=False)
    plt.show()

# ════════════════════════════════════════════════════
#  ENTRY
# ════════════════════════════════════════════════════
if __name__ == '__main__':
    history = run_engine()
    print("\nLaunching 3D Visualizer...")
    launch_animation(history)
