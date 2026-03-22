@echo off
echo ═══════════════════════════════════════════════════════════════
echo    Advanced GPS-Denied 3D Drone Navigation Simulation
echo ═══════════════════════════════════════════════════════════════
echo.
echo  Features active in this build:
echo   [PLANNING]  Theta* (any-angle A*) → A* → JPS → Informed RRT*
echo   [REPLAN]    D* Lite (incremental) + JPS emergency fast-replan
echo   [AVOIDANCE] DWA with 3D Z-velocity sampling (720 candidates)
echo   [SENSING]   Lidar (12 horizontal beams) + downward AGL beam
echo   [PHYSICS]   Wind disturbance model (mean + Gaussian gusts)
echo   [SLAM]      Loop Closure / relocalisation with drift correction
echo   [CONTROL]   Cascaded PID (position → velocity) with anti-windup
echo   [OBSTACLES] 3D Potential Fields + obstacle inflation layer
echo ───────────────────────────────────────────────────────────────
echo.

echo [1/3] Launching Live Telemetry Dashboard...
start "Live Telemetry Dashboard" cmd /k "python drone_telemetry_cmd.py"
timeout /t 2 >nul

echo [2/3] Launching Metrics Evaluator...
start "Metrics Evaluator" cmd /k "python metrics_evaluator.py"
timeout /t 2 >nul

echo [3/3] Launching 3D Simulation Engine (main window)...
start "GPS-Denied Drone Sim" cmd /k "python advanced_drone_sim.py"

echo.
echo ═══════════════════════════════════════════════════════════════
echo  All 3 windows launched!
echo  Close this window and watch the simulation run.
echo ═══════════════════════════════════════════════════════════════
timeout /t 4 >nul
exit
