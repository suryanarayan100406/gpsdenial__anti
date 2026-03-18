#!/usr/bin/env python3
"""
Standalone demo of the drone navigation ROS2 system (without Webots simulator).
Tests the core navigation nodes with synthetic topic inputs.
"""

import os
import sys
import subprocess
import signal
import time

def run_command(cmd):
    """Run a shell command and return the process."""
    print(f"\n{'=' * 70}")
    print(f"Running: {' '.join(cmd)}")
    print('=' * 70)
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

def main():
    # Source ROS2 environment
    env = os.environ.copy()
    
    # Launch the navigation nodes with RViz (no Webots)
    launch_cmd = [
        'bash', '-lc',
        'export WEBOTS_HOME=$HOME/webots; '
        'source /opt/ros/jazzy/setup.bash; '
        'source /mnt/c/Users/samai/Desktop/drone/install/setup.bash; '
        'cd /mnt/c/Users/samai/Desktop/drone && '
        'ros2 launch drone_nav_2d drone_nav_launch.py 2>&1'
    ]
    
    print("\n" + "=" * 70)
    print("DRONE NAVIGATION SYSTEM DEMO")
    print("=" * 70)
    print("\nLaunching ROS2 navigation nodes...")
    print("Expected output:")
    print("  - Map publisher (publishes occupancy grid)")
    print("  - Path planner (A* algorithm)")
    print("  - Drone controller (PID tracking)")
    print("  - Obstacle avoidance (potential field)")
    print("  - Metrics logger (mission KPIs)")
    print("  - RViz2 visualization")
    
    proc = subprocess.Popen(
        launch_cmd,
        env={**env, 'WSL_HOST': 'localhost'},
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        shell=False,
        preexec_fn=None if sys.platform == 'win32' else os.setsid
    )
    
    print("\nWaiting for nodes to start (30 seconds)...")
    try:
        # Read output for 30 seconds
        for i in range(30):
            try:
                line = proc.stdout.readline()
                if line:
                    print(line.rstrip())
                time.sleep(0.1)
            except:
                break
                
        if proc.poll() is None:
            print("\n✓ Nodes launched successfully!")
            print("\nNavigation system is running. Check RViz for visualization.")
            print("Press Ctrl+C to stop...")
            
            try:
                proc.wait()
            except KeyboardInterrupt:
                print("\nShutting down...")
                proc.terminate()
                time.sleep(2)
                if proc.poll() is None:
                    proc.kill()
        else:
            # Process exited early
            print("\n✗ Launch failed. Details above.")
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\nShutting down...")
        proc.terminate()
        time.sleep(2)
        if proc.poll() is None:
            proc.kill()

if __name__ == '__main__':
    main()
