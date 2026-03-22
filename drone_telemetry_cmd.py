#!/usr/bin/env python3
"""
drone_telemetry_cmd.py
═══════════════════════════════════════════════════════════════════════
Real-Time CMD Telemetry Monitor for Advanced Drone Simulation
═══════════════════════════════════════════════════════════════════════
Shows live sensor data, algorithm state, PID internals, and metrics.
Run this in a SEPARATE terminal while advanced_drone_sim.py is running.

Usage:
  Terminal 1: python advanced_drone_sim.py
  Terminal 2: python drone_telemetry_cmd.py

If no live feed is detected, reads from telemetry_log.json instead.
"""

import time
import os
import sys
import json
import math

# Polling live JSON from the main simulation
LIVE_FILE = "telemetry_live.json"
LOG_FILE = "telemetry_log.json"

# ── ANSI Colors ───────────────────────────────────────────────────────
class C:
    RESET  = "\033[0m"
    BOLD   = "\033[1m"
    RED    = "\033[91m"
    GREEN  = "\033[92m"
    YELLOW = "\033[93m"
    BLUE   = "\033[94m"
    MAGENTA= "\033[95m"
    CYAN   = "\033[96m"
    WHITE  = "\033[97m"
    DIM    = "\033[2m"
    BG_DARK= "\033[40m"
    BG_BLU = "\033[44m"

def cls():
    os.system('cls' if os.name=='nt' else 'clear')

def bar(val, total=100, width=20, color=C.GREEN):
    filled = int(width * min(val,total) / total)
    return color + '█'*filled + C.DIM + '░'*(width-filled) + C.RESET

def fmt_vec(v, unit='m'):
    return f"[{C.CYAN}{v[0]:+7.3f}{C.RESET}, {C.CYAN}{v[1]:+7.3f}{C.RESET}, {C.CYAN}{v[2]:+7.3f}{C.RESET}] {unit}"

def fmt_num(v, unit='', color=C.CYAN):
    return f"{color}{v:+8.3f}{C.RESET} {unit}"

def status_color(status):
    if 'REPLAN' in status: return C.RED + C.BOLD
    if 'GOAL' in status: return C.GREEN + C.BOLD
    return C.GREEN

def render(data):
    cls()
    t    = data.get('t', 0)
    step = data.get('step', 0)
    pos  = data.get('pos', [0,0,0])
    vel  = data.get('vel', [0,0,0])
    imu  = data.get('imu', [0,0,0])
    baro = data.get('baro_alt', 0)
    min_d= data.get('min_obstacle_dist', 99)
    hits = data.get('lidar_hits', 0)
    pf   = data.get('pf_force', [0,0,0])
    iwx  = data.get('pid_windup_x', 0)
    iwy  = data.get('pid_windup_y', 0)
    iwz  = data.get('pid_windup_z', 0)
    repl = data.get('replan', False)
    dsr  = data.get('dstar_replan', False)
    awps = data.get('active_wps', 0)
    m    = data.get('metrics', {})

    status = "⚠  D* LITE REPLAN" if dsr else ("🔄  RRT* REPLAN" if repl else "🟢  NAVIGATING")
    sc = status_color(status)

    spd = math.sqrt(vel[0]**2+vel[1]**2+vel[2]**2)
    pf_mag = math.sqrt(pf[0]**2+pf[1]**2+pf[2]**2)

    windup_ok = all(abs(v)<2.0 for v in [iwx,iwy,iwz])

    print(f"""{C.BOLD}{C.BG_DARK}
╔══════════════════════════════════════════════════════════════════════════╗
║      GPS-DENIED DRONE NAVIGATION — LIVE TELEMETRY DASHBOARD             ║
║      PRM + Informed RRT* + D* Lite + PID + Potential Field              ║
╚══════════════════════════════════════════════════════════════════════════╝{C.RESET}

{C.BOLD}[TIME]{C.RESET}  t = {C.YELLOW}{t:6.1f}s{C.RESET}   Step = {C.YELLOW}{step:4d}{C.RESET}   Status: {sc}{status}{C.RESET}

{C.BOLD}┌─ DRONE POSITION  ─────────────────────────────────────────────────────┐{C.RESET}
│  Pose   {fmt_vec(pos, 'm')}
│  Velocity {fmt_vec(vel, 'm/s')}
│  Speed   {C.CYAN}{spd:6.3f}{C.RESET} m/s  {bar(spd*25, 100, 20, C.BLUE)}
│  Altitude (Baro): {fmt_num(baro, 'm', C.YELLOW)}
{C.BOLD}└───────────────────────────────────────────────────────────────────────┘{C.RESET}

{C.BOLD}┌─ SENSOR SUITE ────────────────────────────────────────────────────────┐{C.RESET}
│  IMU (accel/gyro) {fmt_vec(imu)}
│  Lidar hits (12 beams): {C.RED if hits>4 else C.GREEN}{hits:2d}/12{C.RESET} {bar(hits, 12, 20, C.RED)}
│  Min Obstacle Dist: {C.RED if min_d<0.8 else C.YELLOW if min_d<1.5 else C.GREEN}{min_d:6.3f}m{C.RESET}  {'⚠ DANGER!' if min_d<0.8 else 'CLEAR' if min_d>1.5 else 'CAUTION'}
{C.BOLD}└───────────────────────────────────────────────────────────────────────┘{C.RESET}

{C.BOLD}┌─ ALGORITHM STATUS ────────────────────────────────────────────────────┐{C.RESET}
│  PRM + Informed RRT*   : Initial optimal path computed
│  D* Lite Replanning    : {C.RED + 'ACTIVE  ← Changing route!' if dsr else C.GREEN + 'Standby'}{C.RESET}
│  RRT* Replan (blocked) : {C.YELLOW + 'TRIGGERED' if repl else C.DIM + 'Inactive'}{C.RESET}
│  Active Waypoints      : {C.CYAN}{awps:3d}{C.RESET}
{C.BOLD}└───────────────────────────────────────────────────────────────────────┘{C.RESET}

{C.BOLD}┌─ PID CONTROLLER (with Anti-Windup) ───────────────────────────────────┐{C.RESET}
│  Integrator X: {C.CYAN}{iwx:+7.3f}{C.RESET}  Y: {C.CYAN}{iwy:+7.3f}{C.RESET}  Z: {C.CYAN}{iwz:+7.3f}{C.RESET}
│  Windup limit: 2.0   Status: {C.GREEN+'OK (anti-windup active)' if windup_ok else C.RED+'CLAMPED'}{C.RESET}
│  {bar(abs(iwx)*50, 100, 10, C.MAGENTA)} X  {bar(abs(iwy)*50, 100, 10, C.MAGENTA)} Y  {bar(abs(iwz)*50, 100, 10, C.MAGENTA)} Z
{C.BOLD}└───────────────────────────────────────────────────────────────────────┘{C.RESET}

{C.BOLD}┌─ POTENTIAL FIELD ──────────────────────────────────────────────────────┐{C.RESET}
│  Total Force  {fmt_vec(pf, 'N')}
│  ||F||: {C.MAGENTA}{pf_mag:6.3f}{C.RESET} N   {bar(pf_mag*15, 100, 25, C.MAGENTA)}
│  Obstacle Inflation Radius: 0.6m applied globally
{C.BOLD}└───────────────────────────────────────────────────────────────────────┘{C.RESET}

{C.BOLD}┌─ PERFORMANCE METRICS MATRIX ───────────────────────────────────────────┐{C.RESET}
│  Path Optimality  : {C.CYAN}{m.get('path_optimality_%', 0):5.1f}%{C.RESET}  {bar(m.get('path_optimality_%', 0), 100, 18)}
│  Safety Score     : {C.GREEN}{m.get('safety_score_%', 0):5.1f}%{C.RESET}  {bar(m.get('safety_score_%', 0), 100, 18, C.GREEN)}
│  Energy Efficiency: {C.YELLOW}{m.get('energy_score_%', 0):5.1f}%{C.RESET}  {bar(m.get('energy_score_%', 0), 100, 18, C.YELLOW)}
│  Replanning Score : {C.BLUE}{m.get('replanning_score_%', 0):5.1f}%{C.RESET}  {bar(m.get('replanning_score_%', 0), 100, 18, C.BLUE)}
│  ─────────────────────────────────
│  TOTAL SCORE      : {C.BOLD}{C.GREEN if m.get('TOTAL_SCORE_%',0)>80 else C.YELLOW if m.get('TOTAL_SCORE_%',0)>60 else C.RED}{m.get('TOTAL_SCORE_%', 0):5.1f}%{C.RESET}  {bar(m.get('TOTAL_SCORE_%',0), 100, 18, C.GREEN)}
│  Replans  (PRM/RRT*): {m.get('prm_rrt_replans', 0):2d}   D* Lite: {m.get('dstar_replans', 0):2d}
│  Close calls        : {m.get('close_calls', 0):2d}   Path: {m.get('path_length_m', 0):.2f}m
{C.BOLD}└───────────────────────────────────────────────────────────────────────┘{C.RESET}
""")

def run_live():
    print(f"{C.GREEN}Connected to live simulation feed ({LIVE_FILE})...{C.RESET}\n")
    last_mod = 0
    while True:
        try:
            if os.path.exists(LIVE_FILE):
                mod_time = os.path.getmtime(LIVE_FILE)
                if mod_time != last_mod:
                    last_mod = mod_time
                    with open(LIVE_FILE, 'r') as f:
                        data = json.load(f)
                    
                    if data.get('__done__'):
                        print(f"\n{C.GREEN}{C.BOLD}═══ SIMULATION COMPLETE ═══{C.RESET}")
                        final = data.get('metrics', {})
                        print(json.dumps(final, indent=2))
                        print(f"\nFinal metrics saved.")
                        break
                    render(data)
            time.sleep(0.1)
        except Exception:
            # ignore transient file read errors during write
            time.sleep(0.1)

def run_from_log():
    print(f"{C.YELLOW}Live mode unavailable. Reading from {LOG_FILE}...{C.RESET}\n")
    if os.path.exists(LOG_FILE):
        with open(LOG_FILE) as f:
            data = {'metrics': json.load(f), 't': 0, 'step': 0,
                    'pos':[0,0,0], 'vel':[0,0,0], 'imu':[0,0,0]}
        render(data)
    else:
        print(f"No log found. Run `python advanced_drone_sim.py` first.")

if __name__ == '__main__':
    # Start live polling. If the file doesn't exist yet, it will wait for it.
    run_live()
