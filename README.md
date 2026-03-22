# GPS-Denied Drone Navigation System 🚁

> **Advanced autonomous drone navigation without GPS** — using multi-layer sensor fusion, a full algorithm stack for path planning, dynamic obstacle avoidance, and real-time LiDAR reflexes.

---

## 🏆 Algorithm Stack

### Path Planning (Hierarchical Cascade)

| Layer | Algorithm | Role | Quality |
|---|---|---|---|
| 1 | **PRM** (Probabilistic Roadmap) | Global roadmap construction | Fast, probabilistic |
| 2 | **Theta\*** (Any-Angle A\*) | Optimal initial path | Near-geometric optimal ✅ |
| 2a | **Bidirectional A\*** | Backup if Theta\* fails | Provably optimal, 2× faster than A\* |
| 2b | **Informed RRT\*** | Last-resort fallback | Asymptotically optimal |
| 3 | **D\* Lite** | Global dynamic replanning (1 Hz) | Handles map changes |
| 4 | **JPS** (Jump Point Search) | Fast blocked-path replan | 10× faster than RRT\* on grids |

### Why Theta\*?
Theta\* is the best algorithm for **maximum path optimality**. Unlike grid-based A\* which forces zig-zagged grid-edge movement, Theta\* performs **line-of-sight checks between ancestors**, allowing movement at any angle. This produces near-geometric shortest paths — provably closer to the true optimal than any standard grid algorithm.

---

## ⚡ Dual-Rate Obstacle Avoidance Architecture

```
┌─────────────────────────────────────────────────────┐
│  FAST LAYER: LiDAR Reflex — 20 Hz (every step)     │
│  • Real-time LiDAR scan (12 beams, 360°)            │
│  • Detects popup/ dynamic obstacles instantly       │
│  • Fires Potential Field repulsion immediately      │
│  • Emergency dodge if obstacle < 0.8m              │
│  • Triggers JPS replan same step it detects         │
└─────────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────────┐
│  SLOW LAYER: D* Lite Replanner — 1 Hz (every 10s)  │
│  • Bakes popup obstacles into full voxel grid       │
│  • Global optimal replan across the whole map       │
│  • Updates active waypoint path completely          │
└─────────────────────────────────────────────────────┘
```

---

## 🌲 World & Obstacle Types

| Type | Count | Behaviour |
|---|---|---|
| Trees (cylinders) | 5 | Static |
| Rocks (low cylinders) | 3 | Static |
| Dynamic Obstacles | 4 | Moving, bounce off walls |
| **Pop-up Obstacles** | 2 | Appear suddenly mid-flight at steps 80 & 160 |

Pop-up obstacles test the drone's instant LiDAR reflex response — they materialise directly in the drone's path without warning, forcing real-time course correction.

---

## 🧠 Sensor Architecture

| Sensor | Rate | Data |
|---|---|---|
| LiDAR | 20 Hz | 12-beam radial distance scan |
| IMU | 20 Hz | Linear velocity + acceleration |
| Barometer | 20 Hz | Altitude (Z position) |

Sensor data drives the **Potential Field + Reflex** layer directly, while the **path planner** uses the voxel grid model updated from the same sensor feed.

---

## 🎮 PID Controller

Cascaded PID architecture with **anti-windup protection**:

```
Position Error → [Pos PID] → Velocity Command
                               → [Vel PID] → Force Command
                                              → Potential Field blending
                                              → Motor command (clamped)
```

**Obstacle Inflation**: All obstacles are inflated by `INFLATION_RADIUS = 0.6m` in the grid so the drone always plans with safety margin.

---

## 📊 Metrics Evaluator

Real-time scoring across 5 dimensions:

| Metric | Formula | Best |
|---|---|---|
| Path Optimality % | `Theta* length / actual flown` | 100% |
| Obstacle Clearance % | Based on min distance to obstacles | 100% |
| Goal Progress % | How far toward goal | 100% |
| Replanning Score % | `50 + 5×D*replans + 3×JPS replans` (capped 100) | 100% |
| Overall Score | Weighted average | 100% |

---

## 🚀 Quick Start

### Run Everything at Once
```bat
run_demo.bat
```
This launches 3 terminals simultaneously:
1. **Main Simulation** — `advanced_drone_sim.py` (3D + top-down GUI)
2. **Telemetry Monitor** — `drone_telemetry_cmd.py` (live data feed, CMD-style)
3. **Metrics Evaluator** — `metrics_evaluator.py` (live scoring dashboard)

### Run Individually
```bash
python advanced_drone_sim.py      # Main simulation
python drone_telemetry_cmd.py     # Live telemetry (run in parallel)
python metrics_evaluator.py       # Scoring dashboard (run in parallel)
```

### Requirements
```bash
pip install numpy matplotlib
```

---

## 📁 Files

| File | Purpose |
|---|---|
| `advanced_drone_sim.py` | Main simulation engine — all algorithms, physics, 3D GUI |
| `drone_telemetry_cmd.py` | CMD-style live telemetry display |
| `metrics_evaluator.py` | Scoring dashboard |
| `run_demo.bat` | One-click launcher for all 3 components |
| `QUICK_START.md` | Quick reference |
| `ROBOTHON_SUBMISSION_SUMMARY.md` | Full submission documentation |

---

## 🏗️ Architecture Overview

```
┌───────────────────────────────────────────────────────────────┐
│                    Advanced Drone Sim                         │
│                                                               │
│  START ──► [PRM] ──► [Theta*] ──► Active Waypoint Path       │
│                         ↑                                     │
│              [Bidirectional A*] (fallback 1)                 │
│              [Informed RRT*]    (fallback 2)                 │
│                                                               │
│  Per Step (20Hz):                                            │
│    LiDAR → Reflex Layer → Potential Field Force              │
│    Popup obstacle? → JPS instant replan                       │
│    Emergency (<0.8m)? → Immediate dodge thrust                │
│                                                               │
│  Per 10 Steps (1Hz):                                         │
│    D* Lite global replan with popup obstacles baked in        │
│                                                               │
│  PID → Vel Command → Clamped acceleration → Physics           │
│                                                               │
│  Telemetry → telemetry_live.json → drone_telemetry_cmd.py    │
└───────────────────────────────────────────────────────────────┘
```

---

## 🔬 Key Constants

| Parameter | Value | Effect |
|---|---|---|
| `VOXEL_RES` | 0.4 m | Grid resolution |
| `INFLATION_RADIUS` | 0.6 m | Safety margin around obstacles |
| `RHO_0` | 1.5 m | Potential field influence radius |
| `K_REP` | 3.0 | Repulsive force strength |
| `REPLAN_INTERVAL` | 10 steps | D\* Lite slow replan rate (1 Hz) |
| `FAST_REFLEX_HZ` | 20 Hz | LiDAR reflex rate |
| `MAX_STEPS` | 600 | Simulation length |

---

*GPS-Denied Drone Navigation — Advanced System*
*Algorithms: PRM · Theta\* · Bidirectional A\* · Informed RRT\* · D\* Lite · JPS · PID · Potential Field*
