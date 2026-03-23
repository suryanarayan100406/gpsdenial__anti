# 🚁 GPS-Denied Drone Navigation System

> **Autonomous navigation of a drone across multiple dynamic waypoints in a 3D realistic environment — using advanced multi-layered algorithms for path planning, dynamic obstacle avoidance, full UKF state estimation, and adaptive replanning.**

![Python](https://img.shields.io/badge/Python-3.9%2B-blue?logo=python)
![Matplotlib](https://img.shields.io/badge/Visualisation-Matplotlib-orange)
![License](https://img.shields.io/badge/License-MIT-green)

---

## 📋 Overview

This project simulates a GPS-denied drone autonomously navigating through a realistic 3D forest environment filled with **static** and **dynamic** obstacles (birds, animals, UAVs). The system demonstrates a fully autonomous pipeline—from probabilistic roadmap construction down to real-time reactive potential-field avoidance. 

Recently, the system has been upgraded to support an **Autonomous Mission Planner** (sequentially routing through multiple waypoints) and a full **Unscented Kalman Filter (UKF)** for complex non-linear state estimation from simulated optical flow, IMU, and barometer sensors.

---

## 🧠 Algorithm Stack

| Phase | Algorithm | Purpose |
|-------|-----------|---------|
| **Mission Planning** | **Autonomous Waypoint Manager** | Dynamically chains targets sequentially; triggers path optimizers automatically upon reaching a target |
| **Phase 1** | **PRM** (Probabilistic Roadmap) + Dijkstra | Fast graph construction and initial route seed |
| **Phase 2** | **Theta\*** (Any-Angle A\*) | Maximum path optimality — near-geometric-shortest-path via 3D line-of-sight |
| **Phase 2 Fallback 1** | **Bidirectional A\*** | Expands from start AND goal; meets in the middle — always optimal, ~2× faster than A\* |
| **Phase 2 Fallback 2** | **Informed RRT\*** | Probabilistic optimal path using PRM cost as ellipsoidal bound |
| **Phase 3** | **D\* Lite (Optimized)** | **$O(1)$ Hash-Map Implementation!** Instant incremental replanning when dynamic obstacles block the path. Restored optimal Theta* geodesic lines unless directly intercepted. |
| **Reactive** | **Jump Point Search (JPS)** | Ultra-fast 2D replanner triggered when path is blocked by a moving obstacle |
| **Smart Brain** | **ObstaclePredictor** | Constant-velocity extrapolation of all dynamic obstacles 6 steps into the future |
| **Smart Brain** | **DWA** (Dynamic Window Approach) | Samples 144 velocity candidates per step, scores by heading + clearance + speed using predicted obstacle positions |
| **Smart Brain** | **MPC** (Model Predictive Control) | 60-rollout N-step trajectory optimiser; minimises goal distance + obstacle proximity + control effort + jerk |
| **Reactive** | **Potential Fields** | Smooth blended attractive/repulsive forces guide the drone between waypoints |
| **Reactive** | **Emergency Reflexes** | Instant escape thrust when any obstacle breaches 0.8 m proximity |
| **Control** | **Cascaded PID** | Position → Velocity control in all 3 axes |
| **Control** | **Anti-Windup** | PID integrator clamped to ±2.0 to prevent saturation |
| **Sensing** | **Lidar Array** | 12-beam simulated lidar for real-time proximity mapping |
| **Estimation** | **UKF State Estimator** | Advanced **Unscented Kalman Filter** replacing EKF. Merges IMU, Barometer, and Optical Flow for drift-resistant XYZ tracking |

---

## 🗂️ Project Structure

```
drone/
├── advanced_drone_sim.py      # Main simulation — trajectory, UKF, planners, and visualization
├── drone_telemetry_cmd.py     # Real-time CMD telemetry dashboard (2nd terminal)
├── metrics_evaluator.py       # Standalone performance comparison vs. baselines
├── run_demo.bat               # One-click launcher — opens all 3 terminals at once
└── README.md
```

---

## 🚀 Quick Start

### Prerequisites

```bash
pip install numpy matplotlib
```

### Run (1-click)

```cmd
run_demo.bat
```

This opens **3 terminals simultaneously**:

| Terminal | Script | Shows |
|----------|--------|-------|
| **1** | `advanced_drone_sim.py` | 4-panel 3D visualiser (world, top-down, potential field, metrics) |
| **2** | `drone_telemetry_cmd.py` | ANSI live dashboard — pose, sensors, PID, UKF Status |
| **3** | `metrics_evaluator.py` | Comparison matrix vs. Naïve A\*, basic A\*, basic RRT |

### Run individually

```bash
# Terminal 1 — Simulation
python advanced_drone_sim.py

# Terminal 2 — Live Telemetry (run alongside sim)
python drone_telemetry_cmd.py

# Terminal 3 — Metrics
python metrics_evaluator.py
```

---

## 🌍 Environment & Mission

- **World size**: 20 × 20 × 8 m voxel grid
- **Voxel resolution**: 0.2 m (High density)
- **Static obstacles**: Trees, rocks, boulders (inflated safety margin applied)
- **Dynamic obstacles**: 3 moving objects — Bird, Animal, UAV — randomly crossing the drone's path.
- **Mission Waypoints**: `[2, -3, 3]`, `[0, 4, 3]`, `[-3, 0, 4]`, and **Final Target**: `[-3, 4, 3]`

---

## 📊 Metrics

The `MetricsEvaluator` tracks and scores the full flight on 5 axes:

| Metric | Description |
|--------|-------------|
| `path_optimality_%` | `straight_line / actual_path_flown × 100` (higher = more direct) |
| `safety_score_%` | Penalised for every obstacle breach below 0.8 m |
| `energy_score_%` | Integral of ‖velocity‖² — rewards efficient, steady flight |
| `replanning_score_%` | Rewards smart replanning (D\* + JPS events), starts at 50, caps at 100 |
| `TOTAL_SCORE_%` | Weighted: 35% optimality + 30% safety + 15% energy + 20% replanning |

---

## 🛰️ Architecture Flow

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Optical   │    │    IMU      │    │  Barometer  │
│    Flow     │    │ Accel/Gyro  │    │  Altitude   │
└──────┬──────┘    └──────┬──────┘    └──────┬──────┘
       │                  │                  │
       └──────────────────┴──────────────────┘
                          │
                  ┌───────▼────────┐
                  │    UKF State   │  <- (Unscented Kalman Filter)
                  │   Estimator    │
                  └───────┬────────┘
                          │
           ┌──────────────┼──────────────┐
           ▼              ▼              ▼
       D* Lite           JPS         Potential
     Replanner        Replanner       Fields
```

---

## 📝 License

MIT License — feel free to use, adapt, and build on this for your own projects.
