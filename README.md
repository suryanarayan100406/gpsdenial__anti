# 🚁 GPS-Denied Drone Navigation System

> **Autonomous 3D navigation from start to goal — without GPS — using a 10-layer algorithm stack covering global planning, dynamic replanning, predictive control, reactive avoidance, and SLAM-inspired relocalisation.**

---

## 📌 What This Project Does

A quadrotor drone navigates a cluttered 3D outdoor world (trees, rocks, bushes + 3 moving obstacles: a bird, an animal, and a rival UAV) without any GPS signal. It must:

1. **Build a roadmap** of the environment and plan the globally shortest path
2. **Fly** from `START = [-5, -4, 1]` → `GOAL = [5, 4, 2.5]` in a `±10m × ±10m × 5m tall` world
3. **Replan in real-time** as dynamic obstacles move and block the route
4. **Avoid collisions** using predictive velocity control (DWA + MPC)
5. **Correct drift** when the drone recognises a previously visited location

Everything runs in one Python file with a live 4-panel matplotlib animation and parallel telemetry streaming.

---

## 🌍 World Configuration

```
World bounds:  X: ±10m   Y: ±10m   Z: 0–5m
Voxel resolution: 0.4m  (grid: 50×50×12 = 30,000 cells)
Inflation radius: 0.6m  (safety buffer around every obstacle)

START = [-5.0, -4.0, 1.0]
GOAL  = [ 5.0,  4.0, 2.5]
Straight-line distance: ≈ 13.4m
```

### Static Obstacles (11 total)

| Type | Count | Height | Radius |
|------|-------|--------|--------|
| Trees | 5 | 2.5–3.5m | 0.40–0.55m |
| Rocks | 3 | 0.35–0.5m | 0.35–0.50m |
| Bushes | 3 | 0.40–0.45m | 0.38–0.45m |

All static obstacles are modelled as **inflated cylinders** built into a 3D boolean voxel grid at startup.

### Dynamic Obstacles (3 moving)

| Name | Start Position | Velocity | Radius |
|------|---------------|---------|--------|
| 🔴 Bird | `[-2.0, 4.5, 1.8]` | `[+0.6, -1.0, 0]` m/s | 0.65m |
| 🟤 Animal | `[4.5, -3.0, 0.4]` | `[-0.9, +0.5, 0]` m/s | 0.70m |
| 🟠 UAV | `[0.0, 0.0, 3.0]` | `[-0.4, -0.7, 0]` m/s | 0.55m |

Dynamic obstacles use constant-velocity linear motion: `pos(t) = pos₀ + vel × t`

---

## 🧠 Full Algorithm Stack — Deep Detail

### 1. Voxel Grid (`STATIC_GRID`)

- **Size**: `NX=50, NY=50, NZ=12` (50×50×12 boolean numpy array)
- **Built at startup**: Every static obstacle is marked as a cylinder with `INFLATION_RADIUS = 0.6m` added to its radius
- **`get_grid(dyn_pos_list)`**: Returns a merged grid including both static obstacles AND current dynamic obstacle positions (used by D* Lite every step)
- **`w2v(pos)`** / **`v2w(voxel)`**: Convert between continuous world coordinates and discrete voxel indices
- **`obstacle_distance(pos, dyn)`**: Returns the exact continuous distance from any 3D point to the nearest obstacle surface (used by potential fields and metrics)

---

### 2. PRM — Probabilistic Roadmap Method

**File location**: `build_prm()` + `dijkstra_prm()` — lines 591–630

- Samples **200 random collision-free 3D points** (rejecting those that fall inside obstacles via voxel lookup)
- Always includes `START` and `GOAL` as nodes 0 and 1
- Connects any two nodes within `CONNECT_RADIUS = 2.5m` if the straight line between them is collision-free (`edge_free()` checks 8 intermediate points)
- Builds an adjacency list `adj[i] → [(j, distance), ...]`
- Runs **Dijkstra** on this graph to find the shortest path (gives the `c_best` cost bound for Informed RRT*)

**Output**: `prm_nodes` (list of 3D positions) + `prm_path` (list of waypoints from START→GOAL)

---

### 3. Theta* — Any-Angle A* (Primary Planner)

**File location**: `class ThetaStar` — lines 827–923

This is the highest-quality path planner in the system. It produces **near-geometric-shortest paths** in 3D voxel space.

**How it differs from A\*:**
- Standard A\* can only move along grid edges → paths look like staircases
- Theta\* checks **line-of-sight from the grandparent** using a 3D Bresenham algorithm
- If grandparent → neighbour is clear, it **short-circuits** the path, skipping intermediate grid nodes entirely
- Result: smooth, near-straight-line paths that are the closest possible approximation to the true shortest path in discrete space

**Key parameters:**
```python
connectivity = 18-directional  # 3×3×3 cube minus centre minus face-diagonals
max_nodes    = 8000            # expansion budget
heuristic    = Euclidean 3D distance
```

**Line-of-sight check** (`_los(a, b)`): 3D Bresenham traversal — checks every voxel the line passes through for obstacles. Handles all three axis-dominant directions.

**Fallback chain**:
1. Theta\* succeeds → use it ✓
2. Theta\* fails → try **Bidirectional A\***
3. BiDir A\* fails → try **Informed RRT\***

---

### 4. Bidirectional A\* (Fallback 1)

**File location**: `class BidirectionalAStar` — lines 930–1012

- Runs **two simultaneous A\* searches**: one from START→GOAL, one from GOAL→START
- Uses **18-directional** connectivity (same as Theta\*)
- Each iteration alternates: expand one node forward, expand one node backward
- Tracks a `meeting` node where both frontiers have visited
- Terminates when `f_min_fwd + f_min_bwd ≥ best_known_cost` (provably optimal termination)
- Path reconstruction: trace forward parents to meeting point → reverse + trace backward parents

**Budget**: `max_nodes = 6000` expanded nodes

---

### 5. Informed RRT\* (Fallback 2)

**File location**: `class InformedRRTStar` — lines 635–730

- Standard RRT\* with **ellipsoidal informed sampling**
- Once any path is found with cost `c_best`, samples are drawn from the prolate hyperspheroid region: `{x : d(x,START) + d(x,GOAL) ≤ c_best}`
- 60% of samples come from the ellipsoid, 40% are random (for diversity)
- **Rewiring**: when a new node is added, checks all nodes within `radius = 1.8m` and reroutes them through the new node if it's cheaper
- Uses PRM's Dijkstra cost as initial `c_best` to immediately restrict sampling

**Parameters**:
```python
max_iter  = 800       # tree expansion iterations
step      = 1.0m      # max extension per step
radius    = 1.8m      # rewire neighbourhood radius
c_min     = ‖GOAL - START‖ = 13.4m    # minimum possible path length
```

---

### 6. D\* Lite — Incremental Dynamic Replanner

**File location**: `class DStarLite` — lines 735–821

D\* Lite runs continuously in the background, updating the path every time the environment changes.

**How it works:**
- Maintains two values per cell: `g(s)` (current best cost) and `rhs(s)` (one-step-lookahead cost)
- A cell is **consistent** when `g = rhs`; **inconsistent** otherwise
- Only processes inconsistent cells — when a dynamic obstacle moves into a cell, only the cells affected by that change need to be reprocessed (not the entire map)
- `_km` counter accumulates heuristic offset as the robot moves (required for correctness)
- **`update_grid(new_grid, start_v)`**: Called every 5 steps to snapshot current dynamic obstacle positions and trigger consistency updates
- **`extract_path()`**: Greedy descent following minimum `g` values from current position to goal

**6-directional connectivity** (pure Manhattan: no diagonals)

**Initial compute budget**: 2500 iterations (fast startup)

---

### 7. Jump Point Search (JPS) — Emergency Replanner

**File location**: `class JPS` — lines 1019–1123

JPS is activated when D\* Lite loses its path (obstacle directly blocking). It finds a new path in **milliseconds** instead of the hundreds of milliseconds A\* would take.

**How it works:**
- Works on a **2D slice** of the voxel grid at the drone's current altitude (`nz` layer)
- Uses **symmetry-breaking pruning**: instead of expanding all neighbours, identifies "jump points" — the minimum set of nodes needed to guarantee optimality
- A jump point is a node where the path must turn (forced neighbour detected)
- By jumping over symmetrical corridors, it skips vast regions of the grid that A\* would have to visit
- Falls back to ordinary A\* if no jump point is found within 4000 node budget

**8-directional connectivity** in 2D XY plane

---

### 8. ObstaclePredictor — Constant-Velocity Extrapolation

**File location**: `class ObstaclePredictor` — lines 191–223

- Maintains a rolling **history buffer** (last 10 positions) for each dynamic obstacle
- `update(t)`: Computes `pos_i(t) = pos₀ + vel × t` for each obstacle and appends to history
- `predict(t, horizon=6, dt=0.1)`: Estimates velocity from last two history entries (`v_est = (h[-1] - h[-2]) / dt`). Extrapolates 6 steps: `pred_k = pos + v_est × k × dt` for k=1..6
- Returns a `[n_obstacles][6]` array of predicted `(position, radius)` tuples
- Interface designed to be **Kalman-filter upgradeable** — just swap `v_est` with Kalman output

---

### 9. DWA — Dynamic Window Approach

**File location**: `class DWA` — lines 232–316

At every control step, DWA performs a real-time velocity search in the drone's **reachable velocity space**.

**Parameters:**
```python
max_speed      = 2.5 m/s
max_accel      = 3.0 m/s²    # defines the dynamic window size
v_samples      = 9            # speed magnitudes to test
theta_samples  = 16           # directions (full 360°)
sim_time       = 1.2s         # forward simulation horizon
dt_sim         = 0.15s        # internal simulation timestep
w_heading      = 2.0          # weight: pointing toward goal
w_clearance    = 3.0          # weight: distance from obstacles
w_speed        = 0.8          # weight: travel speed
```

**Per step: 9 × 16 = 144 velocity candidates evaluated**

For each candidate:
1. Simulate trajectory for 1.2s (8 steps)
2. Check collision against static obstacles (min clearance > 0.3m required)
3. Check collision against predicted obstacle positions at 1 step ahead
4. Score: `w_heading × (1/dist_to_goal) + w_clearance × min(1, clearance/2) + w_speed × (v/v_max)`
5. Return the highest-scoring non-colliding velocity

---

### 10. MPC — Model Predictive Control

**File location**: `class MPC` — lines 325–409

MPC finds the optimal first control action by evaluating many possible future control sequences.

**Parameters:**
```python
N           = 6         # prediction horizon (steps)
dt          = 0.1s      # timestep
max_speed   = 2.5 m/s
max_accel   = 2.5 m/s²
n_rollouts  = 60        # random control sequences evaluated
Q_goal      = 2.0       # weight: terminal distance to goal
Q_obs       = 5.0       # weight: proximity to obstacles
Q_ctrl      = 0.3       # weight: control effort ‖v‖²
Q_smooth    = 0.4       # weight: jerk (‖v_k - v_{k-1}‖)
```

**Per step: 60 rollouts × 6 steps evaluated**

For each rollout:
1. Sample 6 random velocities within the dynamic window (each step bounded by `max_accel × dt` from previous)
2. Simulate drone forward 6 × 0.1s = 0.6s
3. At each step: penalise proximity to predicted obstacles (penalty within 1.5m for dynamic, 1.2m for static), control effort, and jerk
4. Discard rollouts where position gets within 0.2m of any obstacle
5. Add terminal cost: `Q_goal × ‖pos_final - goal‖`
6. Return `controls[0]` of the minimum-cost rollout

---

### 11. Loop Closure / Relocalisation

**File location**: `class LoopClosure` — lines 422–527

Inspired by SLAM (Simultaneous Localisation and Mapping) — detects when the drone revisits a location and corrects any accumulated position drift.

**Parameters:**
```python
FINGERPRINT_BINS = 16    # histogram resolution
MATCH_THRESHOLD  = 0.88  # cosine similarity for acceptance
MIN_KEYFRAME_DIST= 1.2m  # minimum spacing between stored keyframes
Spatial search radius = 3.5m   # only compare nearby keyframes
Correction factor = 0.30       # soft pull (30%) toward remembered pose
```

**Keyframe building** (every 10 simulation steps):
- Take current lidar scan hits
- Compute range from drone to each hit: `r_i = ‖hit_i - drone_pos‖`
- Build a 16-bin histogram over normalised ranges `r/r_max`
- L2-normalise histogram → fingerprint vector
- Add as keyframe only if no existing keyframe is within 1.2m

**Loop closure detection** (every step):
- Compute fingerprint of current lidar scan
- For all keyframes within 3.5m: compute cosine similarity = `(a·b) / (‖a‖‖b‖)`
- If `best_sim ≥ 0.88`: loop closure confirmed
- Apply correction: `drone_pos += (keyframe_pos - drone_pos) × 0.30`
- Log: `🔄 LOOP CLOSURE @step=N | sim=0.94 | drift-fix=0.3m ← matched keyframe #7`

---

### 12. Potential Fields

**File location**: `potential_force()` — lines 160–184

```python
K_ATT  = 1.5    # attractive gain
K_REP  = 8.0    # repulsive gain
RHO_0  = 1.5m   # influence radius
```

- **Attractive**: `F_att = K_att × (goal - pos) / ‖goal - pos‖`  (unit vector scaled)
- **Repulsive**: For each obstacle within `RHO_0`:
  `F_rep += K_rep × (1/ρ - 1/ρ₀) × (1/ρ²) × (pos - obs_center) / ‖pos - obs_center‖`
- Treats static obstacles as cylinders (centred at `[cx, cy, h/2]`)
- Treats dynamic obstacles as spheres (3D distance)
- Force is **added** to the PID velocity command for smooth continuous guidance

---

### 13. Emergency Reflexes

Built into the main loop, not a separate class.

```python
REFLEX_DIST = 0.8m   # activation threshold
```

If `obstacle_distance(drone_pos, dyn_cur) < 0.8m`:
- Compute unit vector away from nearest obstacle surface
- Apply `escape_vel = 2.5 × away_vector` directly as velocity command
- **Bypasses PID, DWA, MPC, and potential fields entirely**
- Recorded as a `close_call` in metrics (each costs -2 to safety score)

---

### 14. Cascaded PID Controller

**File location**: `class PIDController` — lines 532–563

Two PID instances in cascade:

```python
# Outer loop: position error → desired velocity
pos_pid = PIDController(kp=2.2, ki=0.15, kd=0.8, limit=3.0, windup_limit=2.0)

# Inner loop: velocity error → acceleration/thrust
vel_pid = PIDController(kp=1.5, ki=0.05, kd=0.4, limit=5.0, windup_limit=1.5)
```

**Anti-windup**: If `‖integral‖ > windup_limit`, the integral is rescaled: `integral *= windup_limit / ‖integral‖`

**Output saturation**: If `‖output‖ > limit`, scaled back: `output *= limit / ‖output‖`

**Velocity blending** (Smart Brain Mode):
```python
if any_obstacle_within_2.5m:
    vel_cmd = 0.4 × PID_output + 0.3 × DWA_output + 0.3 × MPC_output
```

---

### 15. Sensor Suite

**File location**: `class SensorSuite` — lines 1128–1166

| Sensor | Noise | Range | Beams | Used For |
|--------|-------|-------|-------|---------|
| **Lidar** | — | 4.0m | 12–16 | Loop closure fingerprinting, obstacle detection |
| **IMU** | σ=0.03 m/s | — | — | Velocity PID inner loop |
| **Barometer** | σ=0.05m | — | — | Altitude hold, Z-axis PID |

**Lidar ray marching**: For each of 12 beams (evenly spaced 360° in XY plane):
- Ray: `p(d) = pos + direction × d`, sampled at 30 points from 0.1m to 4.0m
- Checks voxel grid for static obstacles at each point
- Checks sphere intersection for each dynamic obstacle at each point
- Returns hit list: `[(angle, distance, hit_bool), ...]`

---

## 📊 MetricsEvaluator — Scoring System

**File location**: `class MetricsEvaluator` — lines 1171–1214

| Metric | Weight | Formula |
|--------|--------|---------|
| `path_optimality_%` | 35% | `(straight_line / path_flown) × 100` |
| `safety_score_%` | 30% | `100 - close_calls × 2` |
| `energy_score_%` | 15% | `100 - (Σ‖v‖² × dt) × 0.5` |
| `replanning_score_%` | 20% | `50 + min(50, dstar_replans×5) + min(25, rrt_replans×3)` |
| **TOTAL_SCORE_%** | — | Weighted sum of all four |

Additional tracked values: `path_length_m`, `straight_line_m`, `close_calls`, `prm_rrt_replans`, `dstar_replans`, `elapsed_s`

---

## 🔁 Full Control Loop (Every 0.1 seconds)

```
Step N:
  1.  t += 0.1  →  update all dynamic obstacle positions: pos_i(t) = pos₀ + vel×t
  2.  dyn_cur ← [(obs_pos, obs_r) for each obstacle]
  3.  Read sensors:
        lidar  = 12 ray-march beams → list of (angle, dist, hit) tuples
        imu    = drone_vel + N(0, 0.03)
        baro   = drone_pos[2] + N(0, 0.05)
  4.  LOOP CLOSURE:
        if step%10 == 0: add_keyframe(drone_pos, lidar)
        (matched, correction, info) = loop_closure.check(drone_pos, lidar, step)
        if matched: drone_pos += correction  →  print 🔄 alert
  5.  OBSTACLE PREDICTOR:
        obs_predictor.update(t)
        preds_per_obs = obs_predictor.predict(t, horizon=6, dt=0.1)
        mpc_pred[k] = [(obs_i_pos_at_step_k, obs_i_r) for i in obs]
        dwa_pred    = mpc_pred[0]  (1-step-ahead)
  6.  D* LITE (every 5 steps):
        new_grid = get_grid(dyn_cur)
        dstar.update_grid(new_grid, w2v(drone_pos))
        dstar.compute(max_iter=800)
        dstar_path = dstar.extract_path()
        if dstar_path blocked → JPS emergency replan
  7.  WAYPOINT ADVANCEMENT:
        if ‖drone_pos - active_path[wp_idx]‖ < 0.5m: wp_idx += 1
        current_waypoint = active_path[wp_idx]
  8.  EMERGENCY REFLEXES:
        min_dist = obstacle_distance(drone_pos, dyn_cur)
        if min_dist < 0.8m:
            vel_cmd = 2.5 × away_from_nearest_obstacle
            ↳ skip steps 9–11, go to step 12
  9.  POTENTIAL FIELDS:
        pf_force = potential_force(drone_pos, current_waypoint, dyn_cur)
  10. PID:
        pos_error = current_waypoint - drone_pos
        des_vel   = pos_pid.step(pos_error) + 0.3 × pf_force
        vel_error = des_vel - drone_vel
        vel_cmd   = vel_pid.step(vel_error)
  11. SMART BRAIN (if any obstacle within 2.5m):
        dwa_vel = dwa.compute(drone_pos, drone_vel, goal, dwa_pred, static_obs)
        mpc_vel = mpc.optimize(drone_pos, drone_vel, goal, mpc_pred, static_obs)
        vel_cmd = 0.4×vel_cmd + 0.3×dwa_vel + 0.3×mpc_vel
  12. INTEGRATION:
        drone_vel = vel_cmd  (direct velocity control)
        drone_pos += drone_vel × 0.1
  13. METRICS:
        metrics.update(prev_pos, drone_pos, drone_vel, min_dist, replanned, dstar_rep)
  14. TELEMETRY:
        push snapshot to TELEMETRY_Q → read by drone_telemetry_cmd.py
  15. CHECK GOAL:
        if ‖drone_pos - GOAL‖ < 0.6m: mission complete
```

---

## 📺 Live Visualisation (4-Panel Figure)

| Panel | Content |
|-------|---------|
| **3D scene** (top-left) | Drone (red sphere + trail), active path (cyan), PRM nodes (grey dots), PRM path (yellow), static obstacles (blue cylinders), dynamic obstacles (coloured spheres + name labels), goal (green star), lidar beams (thin white lines), potential field force arrow |
| **Top view** (top-right) | XY projection. World boundary box, waypoints, obstacle footprints, drone position cross-hair |
| **Path optimality %** (bottom-left) | Time-series plot, 0–100%, drops as detours are taken |
| **Min clearance m** (bottom-right) | Time-series plot, red dashed line at 0.8m (reflex threshold) |

Animation runs at **10 FPS** (1 frame per simulation step).

---

## 📡 Telemetry System

A background thread continuously writes JSON snapshots to `TELEMETRY_Q` (a Python `queue.Queue(maxsize=200)`).

`drone_telemetry_cmd.py` reads from this queue and renders a live terminal dashboard showing:
- Current position, velocity, altitude
- Distance to goal
- Active planner status
- All metric scores
- Last replan event

---

## 🚀 Quick Start

### Prerequisites
```bash
pip install numpy matplotlib
```

### Option A — One-Click
```bash
run_demo.bat
```
Opens 3 terminal windows simultaneously (sim + telemetry + spare).

### Option B — Manual (Two Terminals)
```bash
# Terminal 1
python advanced_drone_sim.py

# Terminal 2
python drone_telemetry_cmd.py
```

---

## 🔧 Tuning Guide

| Want to... | Change this |
|-----------|------------|
| Faster planning | Reduce `NUM_SAMPLES` (PRM), `max_iter` (RRT\*), `max_nodes` (Theta\*) |
| Better path quality | Increase `NUM_SAMPLES`, `CONNECT_RADIUS`, `max_nodes` |
| More aggressive avoidance | Increase `K_REP`, `RHO_0`, `Q_obs` (MPC), `w_clearance` (DWA) |
| Smoother flight | Increase `Q_smooth` (MPC), decrease `kp` (PID) |
| Earlier obstacle prediction | Increase MPC `horizon`, DWA `sim_time` |
| More sensitive loop closure | Decrease `MATCH_THRESHOLD` (e.g., 0.82) |
| Tighter safety | Decrease `REFLEX_DIST` from 0.8m (e.g., 1.0m) |

---

## 🆚 Why This Specific Stack?

| Problem | Why This Answer |
|---------|----------------|
| Path quality | Theta\* is provably closer to the true shortest path than any grid-A\* variant. Only continuous-space planners (RRT\*) beat it and they take longer. |
| Backup planning | Bidirectional A\* halves the search space. Always optimal, fast. |
| Dynamic obstacles | D\* Lite is the gold standard — only updates affected cells, not the whole map. |
| Speed emergencies | JPS is 10–20× faster than A\* in open corridors due to symmetry pruning. |
| Predictive avoidance | DWA + MPC together sample both velocity space AND trajectory space, covering what the other misses. |
| Position drift | Loop closure prevents the drone's internal map from diverging from reality over time. |

---

## 🌐 GitHub

[https://github.com/suryanarayan100406/gpsdenial__anti](https://github.com/suryanarayan100406/gpsdenial__anti)
