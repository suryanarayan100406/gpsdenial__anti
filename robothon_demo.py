#!/usr/bin/env python3
"""
Robothon 2026 - GPS-Denied Drone Navigation Demo Script
Showcases all unique features and evaluation capabilities
"""

import os
import subprocess
import time
from pathlib import Path
from datetime import datetime


class RobothonDemo:
    """Interactive demo of Robothon submission capabilities."""
    
    DEMO_FEATURES = {
        '1': {
            'name': 'Quick Easy-Level Test (30 seconds)',
            'desc': 'Simple open environment - baseline performance',
            'cmd': 'ros2 launch drone_nav_2d drone_nav_launch.py use_hard_world:=false',
            'duration': 35
        },
        '2': {
            'name': 'Medium Difficulty with Dynamic Obstacles',
            'desc': 'Moving obstacles + rough terrain test',
            'cmd': 'ros2 launch drone_nav_2d drone_nav_launch.py difficulty_level:=medium enable_dynamic_obstacles:=true',
            'duration': 65
        },
        '3': {
            'name': 'Automated Test Suite (All Levels)',
            'desc': 'Easy → Medium → Hard with automatic evaluation',
            'cmd': 'python3 robothon_tester.py',
            'duration': 180
        },
        '4': {
            'name': 'Headless Mode (RViz Only, No Webots GUI)',
            'desc': 'Lightweight visualization - good for CI/CD',
            'cmd': 'ros2 launch drone_nav_2d drone_nav_headless.py',
            'duration': 60
        },
        '5': {
            'name': 'Show Evaluation Reports',
            'desc': 'Display latest metrics and performance data',
            'cmd': 'cat results/evaluation_*.json',
            'duration': 0
        }
    }
    
    FEATURES_HIGHLIGHT = """
╔════════════════════════════════════════════════════════════════════╗
║          GPS-DENIED AUTONOMOUS DRONE NAVIGATION SYSTEM             ║
║                   Robothon 2026 Submission                         ║
╚════════════════════════════════════════════════════════════════════╝

🎯 UNIQUE FEATURES (USP):

1. 🔄 DYNAMIC ENVIRONMENT MANAGER
   • Runtime obstacle generation and movement
   • Difficulty scaling (Easy/Medium/Hard with explicit constraints)
   • Battery/energy consumption modeling
   • Environmental constraint enforcement
   • Safety zone definition and monitoring

2. 📊 ADVANCED METRICS EVALUATOR  
   • Path optimality tracking
   • Energy efficiency analysis
   • Computational performance profiling (CPU/memory)
   • Collision detection with safety scoring
   • Automated JSON report generation

3. 🔬 ROBOTHON TESTING SUITE
   • Automated multi-level testing framework
   • Pre-configured test scenarios
   • Quantitative metrics generation
   • Evaluation report automation with scoring

4. 🎛️ ADAPTIVE NAVIGATION
   • Environmental constraint compliance
   • Battery-aware mission planning
   • Dynamic replanning on obstacle detection
   • Safety margin enforcement

5. 📈 PERFORMANCE VISUALIZATION
   • Real-time RViz monitoring
   • Rosbag-based trajectory recording
   • Python visualization tools
   • Energy consumption analysis

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

✅ ROBOTHON EVALUATION CRITERIA COMPLIANCE:

✓ Simulation environment documented (3 difficulty levels)
✓ Environment constraints and assumptions specified
✓ Simulation parameters in JSON format
✓ Quantitative performance metrics tracked (15+ metrics)
✓ Hardware/software constraints profiled
✓ ROS2 package with all dependencies
✓ Public GitHub repository
✓ Detailed README with install/run instructions
✓ Technical presentation materials
✓ Multiple unique/advanced features
✓ Reproducible testing framework

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

🧪 TESTING ENVIRONMENTS:

EASY             MEDIUM              HARD
────────────────────────────────────────────
Obstacles: 15%   Obstacles: 25%      Obstacles: 40%
Static only      2 dynamic           5 dynamic
Flat terrain     Rough (0.3)         Very rough (0.7)
Low noise        Moderate noise      High noise
Fast response    Adaptive planning   Robust handling
Score target:    Score target:       Score target:
  >95              >80                 >70

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

📊 MEASURED METRICS:

• Overall Score (0-100): Aggregated mission performance
• Safety Score: Collision-free operation
• Path Optimality: Efficiency of planned path vs straight-line
• Energy Efficiency: Distance traveled per battery unit
• CPU Usage: Computational load percentage
• Memory Usage: RAM consumption
• Replans: Adaptability to obstacles
• Min Obstacle Distance: Safety margin maintained

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""
    
    def __init__(self):
        self.workspace = Path('/mnt/c/Users/samai/Desktop/drone')
        
    def print_header(self):
        """Print welcome banner."""
        print(self.FEATURES_HIGHLIGHT)
    
    def print_menu(self):
        """Print interactive menu."""
        print("\n📋 DEMO OPTIONS:\n")
        for key, feature in self.DEMO_FEATURES.items():
            print(f"{key}. {feature['name']}")
            print(f"   {feature['desc']}")
            if feature['duration'] > 0:
                print(f"   Estimated duration: {feature['duration']}s")
            print()
        
        print("0. Exit")
        print("\n" + "="*70)
    
    def run_demo(self, choice: str):
        """Execute selected demo."""
        if choice not in self.DEMO_FEATURES and choice != '0':
            print("❌ Invalid choice")
            return
        
        if choice == '0':
            print("\n👋 Thanks for exploring Robothon submission!")
            return False
        
        feature = self.DEMO_FEATURES[choice]
        
        print(f"\n🚀 Running: {feature['name']}")
        print(f"📝 Description: {feature['desc']}")
        print(f"⏱️  Estimated time: {feature['duration']}s\n")
        
        if input("Continue? (y/n): ").lower() != 'y':
            return True
        
        # Build full command for WSL
        if 'robothon_tester' in feature['cmd']:
            # Python script
            full_cmd = f"wsl -d Ubuntu -- bash -lc 'cd /mnt/c/Users/samai/Desktop/drone && {feature['cmd']}'"
        elif 'results' in feature['cmd']:
            # Show results
            full_cmd = f"powershell -Command \"Get-Content (Get-ChildItem results/evaluation_*.json | Sort-Object LastWriteTime -Descending | Select-Object -First 1).FullName | ConvertFrom-Json | ConvertTo-Json -Depth 10 | Write-Host\""
        else:
            # ROS2 launch
            full_cmd = f"wsl -d Ubuntu -- bash -lc 'export WEBOTS_HOME=$HOME/webots && source /opt/ros/jazzy/setup.bash && source /mnt/c/Users/samai/Desktop/drone/install/setup.bash && cd /mnt/c/Users/samai/Desktop/drone && timeout {feature['duration']} {feature['cmd']}'"
        
        print(f"\n▶️ Executing: {feature['cmd']}\n")
        print("="*70 + "\n")
        
        try:
            result = subprocess.run(full_cmd, shell=True)
            
            if result.returncode == 0:
                print("\n✅ Demo completed successfully!")
            elif result.returncode == 124:
                print("\n⏱️ Demo timed out (as expected)")
            else:
                print(f"\n⚠️ Process exited with code {result.returncode}")
        
        except KeyboardInterrupt:
            print("\n⏹️ Demo interrupted by user")
        except Exception as e:
            print(f"\n❌ Error: {e}")
        
        return True
    
    def run_interactive(self):
        """Run interactive menu."""
        self.print_header()
        
        while True:
            self.print_menu()
            choice = input("Select option (0-5): ").strip()
            
            if not self.run_demo(choice):
                break
        
        self.print_goodbye()
    
    def print_goodbye(self):
        """Print closing message."""
        print("""

╔════════════════════════════════════════════════════════════════════╗
║                     THANK YOU FOR EXPLORING!                      ║
║                                                                    ║
║  📌 More Information:                                             ║
║    • Full README: drone_nav_2d/README.md                         ║
║    • Submission Summary: ROBOTHON_SUBMISSION_SUMMARY.md          ║
║    • GitHub: suryanarayan100406/GPS-Denied-Drone-...            ║
║    • Test Results: results/evaluation_*.json                     ║
║                                                                    ║
║  🎯 For Robothon Evaluation:                                      ║
║    1. Run automated test suite: python3 robothon_tester.py       ║
║    2. Check results in results/ directory                         ║
║    3. Review metrics for all difficulty levels                    ║
║    4. Examine code architecture in drone_nav_2d/                 ║
║                                                                    ║
║  ✅ Ready for Shortlisting Round Submission!                      ║
╚════════════════════════════════════════════════════════════════════╝
""")


def main():
    """Entry point."""
    demo = RobothonDemo()
    
    if len(__import__('sys').argv) > 1:
        # Command-line mode
        choice = __import__('sys').argv[1]
        if choice in demo.DEMO_FEATURES:
            demo.run_demo(choice)
        else:
            print("Usage: python3 robothon_demo.py [0-5]")
    else:
        # Interactive mode
        demo.run_interactive()


if __name__ == '__main__':
    main()
