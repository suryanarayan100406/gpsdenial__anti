@echo off
echo =======================================================
echo    Advanced 3D Drone Navigation Simulation
echo =======================================================
echo.
echo Launching Telemetry Dashboard...
start "Live Telemetry Dashboard" cmd /k "python drone_telemetry_cmd.py"
timeout /t 1 > nul

echo Launching Metrics Evaluator...
start "Metrics Evaluator" cmd /k "python metrics_evaluator.py"
timeout /t 1 > nul

echo Launching 3D Simulation Engine...
start "Main 3D Simulation" cmd /k "python advanced_drone_sim.py"

echo.
echo All systems launched successfully!
echo You can close this window.
timeout /t 3 > nul
exit
