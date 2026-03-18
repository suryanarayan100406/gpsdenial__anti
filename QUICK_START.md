# 🚀 Quick Start Guide - Robothon Submission

## 5-Minute Setup

```bash
# 1. Clone repository
git clone https://github.com/suryanarayan100406/GPS-Denied-Drone-Navigation-System.git
cd GPS-Denied-Drone-Navigation-System

# 2. Setup environment (in WSL/Linux terminal)
source /opt/ros/jazzy/setup.bash

# 3. Build package
colcon build --packages-select drone_nav_2d
source install/setup.bash

# 4. Run demo
python3 robothon_demo.py
```

---

## 📊 Running Tests

### Option 1: Automated Evaluation (Recommended)
```bash
python3 robothon_tester.py
# Runs all difficulty levels and generates evaluation reports
# Results saved to: results/evaluation_suite_*.json
```

### Option 2: Interactive Demo
```bash
python3 robothon_demo.py
# Shows all features, lets you choose test scenarios
# Great for presentations and demonstrations
```

### Option 3: Manual Launch
```bash
# Easy environment (30s test)
ros2 launch drone_nav_2d drone_nav_launch.py use_hard_world:=false

# Medium difficulty (60s test)
ros2 launch drone_nav_2d drone_nav_launch.py \
  difficulty_level:=medium \
  enable_dynamic_obstacles:=true

# Hard environment (90s test)
ros2 launch drone_nav_2d drone_nav_launch.py use_hard_world:=true
```

---

## 📈 View Results

### After test completes:
```bash
# See all reports
ls -la results/evaluation*.json

# Pretty-print latest report
python3 -m json.tool results/evaluation*.json | head -100

# Check overall score
cat results/evaluation*.json | grep overall_score
```

---

## 🎯 Robothon Evaluation Checklist

- ✅ **Simulation**: Demonstrates navigation in Webots simulator
- ✅ **Environments**: 3 difficulty levels with explicit constraints  
- ✅ **Metrics**: 15+ quantitative metrics tracked automatically
- ✅ **Reports**: JSON evaluation reports generated after each test
- ✅ **Package**: Complete ROS2 package ready to deploy
- ✅ **Documentation**: README with installation, setup, running instructions
- ✅ **Features**: 5 unique advanced capabilities included
- ✅ **Reproducibility**: Automated test suite for consistent results

---

## 🔧 Key Features

### 1. Dynamic Environment Manager
- Multiple difficulty levels (Easy/Medium/Hard)
- Runtime obstacle generation
- Battery/energy modeling
- Real-time constraint enforcement

### 2. Advanced Metrics Evaluator  
- Comprehensive performance tracking
- Safety scoring system
- Energy efficiency analysis
- Computational profiling (CPU/memory)

### 3. Testing Suite
- Automated multi-level testing
- Pre-configured scenarios
- Evaluation report generation
- Quantitative scoring

### 4. Adaptive Navigation
- Dynamic replanning capabilities
- Constraint compliance
- Battery-aware planning
- Safety margin enforcement

### 5. Visualization Tools
- Real-time RViz monitoring
- Rosbag trajectory recording
- Post-run analysis capabilities

---

## 📞 Support

### For Setup Issues
1. Check: `ROBOTHON_SUBMISSION_SUMMARY.md` (project overview)
2. Check: `drone_nav_2d/README.md` (detailed guide)
3. Run: `colcon build --packages-select drone_nav_2d` (rebuild package)

### For Test Issues
1. Verify ROS2 sourcing: `echo $ROS_DISTRO`
2. Check Webots install: `echo $WEBOTS_HOME` (or install via auto-prompt)
3. Review logs: `~/.ros/log/` (latest run directory)

### For Evaluation
1. Run automated suite: `python3 robothon_tester.py`
2. Check results: `cat results/evaluation_*.json`
3. Review metrics: Looking for >80 overall score across difficulty levels

---

## 🎯 Expected Results

### Easy Level (30s)
- ✅ Mission completion
- ✅ Safety score: >95
- ✅ Overall score: 85-95

### Medium Level (60s)
- ✅ Mission completion
- ✅ Safety score: >80
- ✅ Overall score: 75-85

### Hard Level (90s)
- ✅ Mission completion
- ✅ Safety score: >70  
- ✅ Overall score: 65-75

---

## 🚀 Presentation Ready

The system includes:
- ✅ Complete source code (7 ROS2 nodes)
- ✅ Architecture diagrams
- ✅ Algorithm documentation
- ✅ Environment specifications
- ✅ Performance metrics dashboards
- ✅ Automated evaluation reports
- ✅ Video generation capabilities
- ✅ Interactive demo script

---

## 📋 Repository Contents

```
GPS-Denied-Drone-Navigation-System/
├── drone_nav_2d/                    # Core package
│   ├── README.md                    # Comprehensive guide
│   ├── drone_nav_2d/                # Implementation (7 nodes)
│   ├── launch/                      # ROS2 launch files (3 variants)
│   ├── worlds/                      # Webots environments (2 levels)
│   ├── config/                      # Tunable parameters
│   └── rviz/                        # Visualization config
├── robothon_tester.py              # Automated test suite
├── robothon_demo.py                # Interactive demo
├── ROBOTHON_SUBMISSION_SUMMARY.md   # Project summary
├── create_demo_video.py             # Video generation
└── .gitignore                       # Clean repository
```

---

## ✨ Unique Selling Points (USP)

1. **Dynamic Environments**: Difficulty scaling beyond static obstacles
2. **Comprehensive Metrics**: 15+ tracked dimensions (not just completion)
3. **Adaptive Planning**: Real-time replanning in response to changes
4. **Energy Modeling**: Battery-aware mission planning
5. **Automated Evaluation**: One-command testing with complete reports

---

## 🎓 Technical Highlights

- **Algorithms**: A* planning, RRT fallback, PID control, potential fields
- **Languages**: Python 3 (100% native ROS2 implementation)
- **Frameworks**: ROS2 Jazzy, Webots 2025a
- **Metrics**: Safety, efficiency, optimality, computational load
- **Testing**: 3-level difficulty progression with constraints

---

## 📊 Performance Summary

| Metric | Target | Status |
|--------|--------|--------|
| Overall Score | >80 | ✅ Ready |
| Safety Score | >90 | ✅ Ready |
| Path Optimality | >0.85 | ✅ Ready |
| Energy Efficiency | >5 m/% | ✅ Ready |
| CPU Usage | <30% | ✅ Ready |
| Completion Rate | 100% | ✅ Ready |

---

**Version**: 1.0  
**Status**: ✅ Ready for Robothon Shortlisting  
**Last Updated**: March 18, 2026  
**GitHub**: https://github.com/suryanarayan100406/GPS-Denied-Drone-Navigation-System
