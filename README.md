# 🚁 GPS-Denied Drone Navigation System

> **Autonomous navigation of a drone from start to goal in a 3D realistic environment — using advanced multi-layered algorithms for path planning, dynamic obstacle avoidance, and adaptive replanning.**

![Python](https://img.shields.io/badge/Python-3.9%2B-blue?logo=python)
![Matplotlib](https://img.shields.io/badge/Visualisation-Matplotlib-orange)
![License](https://img.shields.io/badge/License-MIT-green)

---

## 📋 Overview

This project simulates a GPS-denied drone autonomously navigating through a realistic 3D forest environment filled with **static** and **dynamic** obstacles (birds, animals, UAVs). It demonstrates a full multi-algorithm pipeline — from roadmap construction to real-time reactive avoidance — all visualised live.

---

## 🧠 Algorithm Stack

| Phase | Algorithm | Purpose |
|-------|-----------|---------|
| **Phase 1** | **PRM** (Probabilistic Roadmap) + Dijkstra | Fast graph construction and initial route seed |
| **Phase 2** | **Theta\*** (Any-Angle A\*) | Maximum path optimality — near-geometric-shortest-path via 3D line-of-sight |
| **Phase 2 Fallback 1** | **Bidirectional A\*** | Expands from start AND goal; meets in the middle — always optimal, ~2× faster than A\* |
| **Phase 2 Fallback 2** | **Informed RRT\*** | Probabilistic optimal path using PRM cost as ellipsoidal bound |
| **Phase 3** | **D\* Lite** | Incremental replanning when dynamic obstacles change the environment |
| **Reactive** | **Jump Point Search (JPS)** | Ultra-fast 2D replanner triggered when path is blocked by a moving obstacle |
| **Smart Brain** | **ObstaclePredictor** | Constant-velocity extrapolation of all dynamic obstacles 6 steps into the future |
| **Smart Brain** | **DWA** (Dynamic Window Approach) | Samples 144 velocity candidates per step, scores by heading + clearance + speed using predicted obstacle positions |
| **Smart Brain** | **MPC** (Model Predictive Control) | 60-rollout N-step trajectory optimiser; minimises goal distance + obstacle proximity + control effort + jerk |
| **Reactive** | **Potential Fields** | Smooth blended attractive/repulsive forces guide the drone between waypoints |
| **Reactive** | **Emergency Reflexes** | Instant escape thrust when any obstacle breaches 0.8 m proximity |
| **Control** | **Cascaded PID** | Position → Velocity control in all 3 axes |
| **Control** | **Anti-Windup** | PID integrator clamped to ±2.0 to prevent saturation |
| **Sensing** | **Lidar + IMU + Barometer** | Simulated sensor fusion providing data to the replanner |

---

## 🗂️ Project Structure

```
drone/
├── advanced_drone_sim.py      # Main simulation — all algorithms, physics, animation
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
| **2** | `drone_telemetry_cmd.py` | ANSI live dashboard — pose, sensors, PID, replanning events |
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

## 🌍 Environment

- **World size**: 20 × 20 × 8 m voxel grid
- **Voxel resolution**: 0.5 m
- **Static obstacles**: Trees, rocks, boulders (inflated safety margin applied)
- **Dynamic obstacles**: 3 moving objects — Bird, Animal, UAV — crossing the drone's path
- **Start**: `[1, 1, 2]` m
- **Goal**: `[18, 18, 5]` m

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

## 🛰️ Sensor Architecture

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Lidar     │    │    IMU      │    │  Barometer  │
│ 12 beams    │    │ vel + noise │    │ alt + noise │
│ range: 4.0m │    │             │    │             │
└──────┬──────┘    └──────┬──────┘    └──────┬──────┘
       │                  │                  │
       └──────────────────┴──────────────────┘
                          │
                  ┌───────▼────────┐
                  │  Sensor Fusion │
                  │  Obstacle Map  │
                  └───────┬────────┘
                          │
           ┌──────────────┼──────────────┐
           ▼              ▼              ▼
       D* Lite          JPS         Potential
      Replanner      Replanner       Fields
```

---

## 🎛️ Key Configuration (in `advanced_drone_sim.py`)

```python
VOXEL_RES        = 0.5     # Grid resolution (m)
INFLATION_RADIUS = 0.4     # Safety margin around obstacles
K_ATT            = 1.5     # Potential field attractive gain
K_REP            = 8.0     # Potential field repulsive gain
RHO_0            = 1.5     # Repulsive influence radius (m)
PFIELD_WEIGHT    = 0.6     # Blend weight for potential field in velocity command
REPLAN_INTERVAL  = 5       # D* Lite replan every N steps
NUM_SAMPLES      = 200     # PRM node count
```

---

## 📡 Live Telemetry (CMD Dashboard)

When `drone_telemetry_cmd.py` is running in a second terminal it shows:

```
╔══════════════ DRONE TELEMETRY ══════════════╗
║  Pos:  [12.4, 11.2,  4.8] m               ║
║  Vel:  [0.82, 0.76, 0.12] m/s   Spd: 1.13 ║
║  Alt:  4.80 m (baro)                       ║
║  IMU:  [0.83, 0.77, 0.13]                  ║
║  Lidar hits: 3/12    Min dist: 1.42 m      ║
║  PF force:   1.87    Anti-windup: OFF      ║
║  D* replans: 4       JPS replans: 1        ║
╠═══════════════ SCORES ══════════════════════╣
║  Path Opt: 74.2%   Safety: 96.0%           ║
║  Energy:   81.5%   Replanning: 85.0%       ║
║  TOTAL:    82.4%                            ║
╚═════════════════════════════════════════════╝
```

---

## 📝 License

MIT License — feel free to use, adapt, and build on this for your own projects.
