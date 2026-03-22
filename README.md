# 🚁 GPS-Denied Drone Navigation System

> **Autonomous 3D navigation from start to goal — without GPS — using a layered stack of the most advanced path planning, dynamic obstacle avoidance, predictive control, and SLAM-inspired relocalisation algorithms available.**

---

## 📌 What This Project Does

This simulation models a quadrotor drone navigating a **cluttered 3D environment** with both **static obstacles** (buildings, pillars) and **moving obstacles** (vehicles, drones) — all without GPS. The drone must:

1. **Plan** the globally optimal path from start → goal
2. **Replan** in real-time as dynamic obstacles move and block the path
3. **Control** its velocity precisely using cascaded PID
4. **Avoid** collisions using predictive and reactive algorithms
5. **Localise** itself when drift accumulates (loop closure)

Everything runs in a single Python simulation with a live 3D + 2D matplotlib animation and real-time telemetry streaming.

---

## 🧠 Full Algorithm Stack

### Phase 1 — Global Path Seeding

| Algorithm | Details |
|-----------|---------|
| **PRM** (Probabilistic Roadmap) | Samples ~200 random collision-free nodes in 3D space. Connects nearby nodes within a radius using a visibility check. Builds a graph of the traversable space. |
| **Dijkstra on PRM graph** | Finds the shortest path through the PRM graph. Used as the initial route seed and to compute `c_best` (the optimal cost bound for Informed RRT\*). |

### Phase 2 — High-Quality Path Planning

| Algorithm | Details |
|-----------|---------|
| **Theta\*** (Any-Angle A\*) | Primary planner. Runs on a 3D grid with 18-connectivity. At each node expansion, checks if there is a direct line-of-sight to the grandparent — if yes, bypasses intermediate nodes for a geometrically shorter path. Produces near-straight-line paths, the closest achievable to the true shortest path in a discretised space. |
| **Bidirectional A\*** | Fallback 1. Simultaneously expands the search tree from both the start and the goal. The two frontiers meet in the middle. Always finds the optimal path, and typically \~2× faster than standard A\* because it explores a much smaller region. |
| **Informed RRT\*** | Fallback 2. Probabilistic planner. Uses the PRM cost (`c_best`) to constrain sampling to an ellipsoidal region guaranteed to contain a better path. Asymptotically converges to the optimum. |

### Phase 3 — Dynamic Replanning

| Algorithm | Details |
|-----------|---------|
| **D\* Lite** | Incremental replanning. Maintains a distance map. When a moving obstacle changes the environment, only re-expands the affected nodes rather than replanning from scratch — extremely efficient. Active every simulation step. |
| **Jump Point Search (JPS)** | Emergency 2D replanner. When D\* Lite cannot find a local solution (obstacle directly on path), JPS is activated. Uses symmetry-breaking pruning to skip large areas of the grid, finding a new path typically 10–20× faster than A\*. |

### Smart Brain — Predictive Control Layer

| Algorithm | Details |
|-----------|---------|
| **ObstaclePredictor** | Tracks every dynamic obstacle's historical positions. Computes velocity via finite difference. Extrapolates the next 6 positions (`pos + vel * k * dt` for k = 1..6). Feeds predicted positions to DWA and MPC every step. |
| **DWA** (Dynamic Window Approach) | Velocity-space search. At every control step, samples a 9×16 grid of (speed, turn) candidates within the physically reachable window (constrained by `max_accel`). Each candidate is simulated 1.2 seconds forward using predicted obstacle positions. Scored by: `heading_score + clearance_score + speed_score`. The best-scoring safe velocity is returned. |
| **MPC** (Model Predictive Control) | 60 random rollouts of 6-step control sequences. Each rollout simulates the drone forward step by step. At each step, accumulates cost: proximity to predicted obstacles, distance to waypoint, control effort `‖u‖`, and jerk `‖u - u_prev‖`. Returns the first action of the minimum-cost rollout. |
| **Loop Closure / Relocalisation** | Lidar scan fingerprinting using a 16-bin range histogram. Every 10 steps, stores a keyframe `(3D position, fingerprint)` in a database. Every step, computes cosine similarity between the current scan and nearby keyframes. If similarity ≥ 0.88 and spatial distance ≤ 3.5 m, a loop closure is detected. Applies a soft 30% correction toward the remembered position to remove accumulated drift. Prints a `🔄 LOOP CLOSURE` alert with the match quality and correction magnitude. |

### Reactive Layer

| Algorithm | Details |
|-----------|---------|
| **Potential Fields** | Attractive force toward the current waypoint (`F_att ∝ distance`), repulsive force away from all obstacles within influence radius (`F_rep ∝ 1/distance²`). Forces are blended and added to the PID command for smooth, continuous guidance between waypoints. |
| **Emergency Reflexes** | If any obstacle (static or dynamic) is detected within 0.8 m, a pure escape thrust is fired directly away from the obstacle — bypassing all other controllers. Prevents crashes during edge cases undetected by MPC/DWA. |

### Control Layer

| Algorithm | Details |
|-----------|---------|
| **Cascaded PID — Position → Velocity** | Outer loop: position error → desired velocity (`Kp=1.8, Ki=0.05, Kd=0.4`). Inner loop: velocity error → thrust command (`Kp=2.5, Ki=0.1, Kd=0.6`). Both axes (XY and Z) controlled independently. |
| **Anti-Windup** | PID integrators clamped to `±windup_limit = 2.0`. Prevents the integrator from accumulating when the drone is saturated (e.g., against a wall), which would cause overshoot when the constraint is removed. |
| **Velocity Blending (Smart Brain Mode)** | When any obstacle is within 2.5 m: `vel_cmd = 0.4×PID + 0.3×DWA + 0.3×MPC`. Otherwise: pure PID. Ensures smooth transitions between planning modes with no discontinuities. |

---

## 📂 Project Structure

```
drone/
│
├── advanced_drone_sim.py      # Main simulation — ALL algorithms live here
│   ├── PRMPlanner             # PRM graph + Dijkstra
│   ├── ThetaStar              # Any-angle A* (3D, 18-connectivity)
│   ├── BidirectionalAStar     # Bidirectional A* planner
│   ├── InformedRRTStar        # Probabilistic optimal planner
│   ├── DStarLite              # Incremental dynamic replanner
│   ├── JumpPointSearch        # Ultra-fast emergency replanner
│   ├── ObstaclePredictor      # Constant-velocity future extrapolation
│   ├── DWA                    # Dynamic Window Approach velocity sampler
│   ├── MPC                    # Model Predictive Control rollout optimizer
│   ├── LoopClosure            # Lidar-fingerprint relocalisation + drift fix
│   ├── PIDController          # Cascaded PID with anti-windup
│   ├── SensorSuite            # Simulated lidar, IMU, barometer
│   ├── potential_force()      # Attractive + repulsive field computation
│   └── run_engine()           # Main simulation loop (integrates everything)
│
├── drone_telemetry_cmd.py     # Terminal telemetry viewer (live metrics)
├── run_demo.bat               # One-click launcher (3 windows in parallel)
└── README.md                  # This file
```

---

## 🚀 Quick Start

### Prerequisites

```bash
pip install numpy matplotlib scipy
```

### Option A — One-Click (Recommended)

```bash
run_demo.bat
```

This opens **3 terminal windows simultaneously**:
1. **Simulation** — runs `advanced_drone_sim.py`, shows 3D/2D animation + metrics
2. **Telemetry** — runs `drone_telemetry_cmd.py`, shows live dashboard
3. **Spare** — for manual commands

### Option B — Manual (Two Terminals)

**Terminal 1:**
```bash
python advanced_drone_sim.py
```

**Terminal 2:**
```bash
python drone_telemetry_cmd.py
```

---

## 📺 What You'll See at Startup

```
════════════════════════════════════════════════════════════
  GPS-DENIED DRONE NAVIGATION — ADVANCED SYSTEM
  PRM + Theta* + BiDir-A* + D* Lite + JPS + DWA + MPC + PID + PF
════════════════════════════════════════════════════════════

[1/3] Building PRM roadmap...
      PRM: 201 nodes | Dijkstra path: 12 waypoints | 198ms
[2/3] Running Theta* (any-angle, maximum path optimality)...
      Theta*: 7 waypoints | 38ms ✓  [PRIMARY]
[3/3] Initializing D* Lite replanner...
      D* Lite ready.

► Simulation running... (sending telemetry on TELEMETRY_Q)
  Run `python drone_telemetry_cmd.py` in another terminal!
```

---

## 📊 Live Metrics Explained

| Metric | Formula | What it Means |
|--------|---------|---------------|
| `path_optimality_%` | `(straight_line_dist / actual_path_flown) × 100` | 100% = perfect straight line. Decreases as drone takes detours around obstacles. |
| `replanning_score` | `+1 per smart replan, -0.5 per failed attempt` | Measures how effectively the replanner is responding to environment changes. |
| `min_clearance_m` | `min distance to any obstacle surface` | Safety metric. Must stay > 0. Emergency reflexes fire at < 0.8 m. |
| `avg_speed_m_s` | `total_path_flown / elapsed_time` | Average flight speed across the whole mission. |
| `loop_closures` | Count of drift corrections applied | How many times the drone recognised a previously visited location and corrected its map. |

---

## 🎬 Live Plot Layout

The simulation opens a **4-panel figure**:

| Panel | Shows |
|-------|-------|
| **Top-left (3D)** | Full 3D flight scene: drone (red sphere), path (cyan), PRM nodes (grey), static obstacles (blue), dynamic obstacles (orange spheres), goal (green star) |
| **Top-right (Top View)** | XY projection. Shows path constraint boundary + current position. Useful for checking lateral deviations. |
| **Bottom-left** | `path_optimality_%` over time — shows drift due to obstacle detours |
| **Bottom-right** | `min_clearance_m` over time — shows how close the drone gets to obstacles |

---

## 🛰️ Sensor Architecture

The drone has no GPS. It uses three simulated sensors:

### Lidar (`SensorSuite.read_lidar`)
- Fires **12 beams** in a hemisphere pattern
- Each beam calculated via ray-sphere and ray-box intersection
- Returns hit positions (used for: obstacle avoidance, loop closure fingerprinting)
- Gaussian noise added: `σ = 0.03 m`

### IMU (`SensorSuite.read_imu`)
- Returns current velocity with Gaussian noise: `σ = 0.02 m/s`
- Used by: velocity PID inner loop

### Barometer (`SensorSuite.read_barometer`)
- Returns altitude (Z position) with Gaussian noise: `σ = 0.05 m`
- Used by: altitude hold and Z-axis PID

---

## 🔢 Key Configuration Parameters

All tuneable parameters are at the top of `advanced_drone_sim.py`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `WORLD_SIZE` | `(20, 20, 10)` | 3D world bounds in metres |
| `START` | `(1, 1, 2)` | Drone start position |
| `GOAL` | `(18, 18, 7)` | Target goal position |
| `PRM_N` | `200` | PRM node count (more = denser roadmap) |
| `DT` | `0.1 s` | Simulation timestep |
| `MAX_SPEED` | `3.0 m/s` | Maximum drone velocity |
| `MAX_ACCEL` | `2.0 m/s²` | DWA/MPC window constraint |
| `REFLEX_DIST` | `0.8 m` | Emergency reflex activation distance |
| `SMART_BRAIN_DIST` | `2.5 m` | Distance at which DWA+MPC blending activates |
| `LC_THRESHOLD` | `0.88` | Loop closure cosine similarity threshold |
| `LC_KEYFRAME_EVERY` | `10 steps` | Keyframe database frequency |

---

## 🔄 Control Loop Flow (Every Timestep)

```
┌─────────────────────────────────────────────────────────────┐
│                    run_engine() LOOP                        │
├─────────────────────────────────────────────────────────────┤
│  1.  Update dynamic obstacle positions (t × vel)            │
│  2.  Read sensors: Lidar (12 beams), IMU, Barometer         │
│  3.  LOOP CLOSURE: check keyframe DB → apply drift fix      │
│  4.  Update ObstaclePredictor → extrapolate 6 steps ahead   │
│  5.  D* Lite: check if active path is still clear           │
│      └── If blocked → JPS emergency replan                  │
│  6.  Emergency Reflexes: check 0.8m proximity               │
│      └── If triggered → pure escape thrust, skip 7–9        │
│  7.  Potential Fields: compute F_att + F_rep                 │
│  8.  PID: position error → vel_cmd                          │
│  9.  Smart Brain blend (if obstacle < 2.5m):                │
│        DWA: sample 144 velocities → best safe vel           │
│        MPC: 60 rollouts → best first action                  │
│        vel_cmd = 0.4×PID + 0.3×DWA + 0.3×MPC               │
│  10. Integrate: drone_pos += vel_cmd × dt                   │
│  11. Record metrics, stream telemetry                        │
│  12. Check goal reached → exit loop                         │
└─────────────────────────────────────────────────────────────┘
```

---

## 📈 Performance Characteristics

| Property | Value |
|----------|-------|
| Path quality (Theta\*) | Near-optimal; typically 5–12% longer than Euclidean straight line |
| Replanning latency (JPS) | < 5 ms per replan |
| MPC computation | ~2 ms per step (60 rollouts × 6 steps) |
| DWA computation | ~1 ms per step (144 candidates) |
| Loop closure check | ~0.5 ms per step |
| Total control loop | ~10–15 ms per step at dt=0.1s |

---

## 🆚 Why This Algorithm Stack?

| Problem | Naive Approach | Our Solution |
|---------|---------------|-------------|
| "Best path?" | A\* on grid → staircase paths | Theta\* → any-angle, near-geometric optimal |
| "What if Theta\* fails?" | Re-run A\* | Bidirectional A\* → meets in middle, 2× faster |
| "Dynamic obstacle moved!" | Replan from scratch | D\* Lite → only updates affected nodes |
| "Obstacle right in front!" | Slow reactive controller | JPS → ultra-fast emergency replan in ms |
| "Obstacle coming AT me!" | React when touching | ObstaclePredictor + DWA/MPC → dodge predicted path |
| "Position drift over time!" | Ignored | Loop Closure → scan-match, correct drift continuously |

---

## 🌐 GitHub

[https://github.com/suryanarayan100406/gpsdenial__anti](https://github.com/suryanarayan100406/gpsdenial__anti)
