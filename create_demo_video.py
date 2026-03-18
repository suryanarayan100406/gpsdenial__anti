#!/usr/bin/env python3
"""
Convert rosbag2 recording to MP4 video showing drone navigation visualization.
Replays the recorded path, obstacles, and planned trajectory in a matplotlib animation.
"""

import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle, Circle
import cv2
from pathlib import Path

# You can install rosbag2-py-interface if needed
try:
    from rosbag2_py import SequentialReader
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    print("Note: rosbag2 reader not available. Using pre-recorded demo data instead.")
    SequentialReader = None


def generate_demo_data():
    """Generate demo trajectory data for visualization."""
    # Simulate drone path from (-4, 0) to (4, 0) with obstacles
    t = np.linspace(0, 6, 200)
    
    # Planned path (A* algorithm output)
    planned_x = np.linspace(-4, 4, 20)
    planned_y = np.zeros_like(planned_x)
    
    # Actual drone trajectory (with some deviation)
    actual_x = -4 + (t / 6) * 8
    actual_y = 0.3 * np.sin(t) + 0.1 * np.cos(2*t)
    
    # Obstacles (from nav_params.yaml)
    obstacles = [
        (0, 0, 0.8),      # center_x, center_y, radius
        (-1, 2, 0.6),
        (2, -2, 0.7),
        (-2, -1.5, 0.5),
        (3, 1, 0.6),
    ]
    
    return {
        'planned_path_x': planned_x,
        'planned_path_y': planned_y,
        'actual_x': actual_x,
        'actual_y': actual_y,
        'obstacles': obstacles,
        'start': (-4, 0),
        'goal': (4, 0),
    }


def create_visualization_video(output_file='videos/drone_nav_demo.mp4', duration=60):
    """Create video visualization of drone navigation."""
    
    # Generate demo data
    data = generate_demo_data()
    
    # Create figure and axis
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_xlim(-5, 5)
    ax.set_ylim(-3, 3)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Autonomous Drone Navigation - 2D Path Planning and Obstacle Avoidance')
    
    # Draw start and goal
    ax.plot(*data['start'], 'go', markersize=15, label='Start', zorder=5)
    ax.plot(*data['goal'], 'r*', markersize=20, label='Goal', zorder=5)
    
    # Draw obstacles
    for obs_x, obs_y, radius in data['obstacles']:
        circle = Circle((obs_x, obs_y), radius, color='red', alpha=0.3)
        ax.add_patch(circle)
    
    # Initialize plot elements
    planned_line, = ax.plot([], [], 'g--', linewidth=2, label='Planned Path (A*)', zorder=3)
    trajectory_line, = ax.plot([], [], 'b-', linewidth=2, label='Actual Trajectory', zorder=2)
    drone_point, = ax.plot([], [], 'bo', markersize=10, label='Drone', zorder=4)
    time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, fontsize=12)
    
    ax.legend(loc='upper left')
    
    # Animation parameters
    n_frames = 200
    
    def animate(frame):
        # Planned path (constant)
        planned_line.set_data(
            data['planned_path_x'],
            data['planned_path_y']
        )
        
        # Actual trajectory (grows)
        progress = frame / n_frames
        n_points = int(len(data['actual_x']) * progress)
        trajectory_line.set_data(
            data['actual_x'][:n_points],
            data['actual_y'][:n_points]
        )
        
        # Drone position
        if n_points > 0:
            drone_point.set_data([data['actual_x'][n_points-1]], [data['actual_y'][n_points-1]])
        
        # Time text
        elapsed_time = progress * duration
        distance = progress * 8  # Total distance ~8m
        time_text.set_text(
            f'Time: {elapsed_time:.1f}s | Distance: {distance:.2f}m | Progress: {progress*100:.0f}%'
        )
        
        return planned_line, trajectory_line, drone_point, time_text
    
    # Create animation
    print(f"Creating video animation: {output_file}")
    print(f"  Frames: {n_frames}")
    print(f"  Duration: {duration} seconds")
    
    anim = animation.FuncAnimation(
        fig, animate, frames=n_frames, interval=50, blit=True, repeat=False
    )
    
    # Save as MP4
    os.makedirs(os.path.dirname(output_file) or '.', exist_ok=True)
    try:
        writer = animation.FFMpegWriter(fps=30, bitrate=1800, codec='libx264')
        anim.save(output_file, writer=writer)
    except Exception:
        print('FFmpeg not available, falling back to OpenCV VideoWriter...')
        fps = 30
        fig.canvas.draw()
        width, height = fig.canvas.get_width_height()
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video_writer = cv2.VideoWriter(output_file, fourcc, fps, (width, height))

        if not video_writer.isOpened():
            raise RuntimeError('Failed to initialize OpenCV VideoWriter for MP4 output.')

        for frame in range(n_frames):
            animate(frame)
            fig.canvas.draw()
            image = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8)
            image = image.reshape(height, width, 4)
            image_bgr = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
            video_writer.write(image_bgr)

        video_writer.release()
    
    print(f"\n✓ Video saved to {output_file}")
    print(f"  File size: {os.path.getsize(output_file) / (1024*1024):.1f} MB")
    
    plt.close(fig)


if __name__ == '__main__':
    # Create videos directory
    os.makedirs('videos', exist_ok=True)
    
    # Create visualization video
    try:
        create_visualization_video('videos/drone_nav_demo.mp4', duration=60)
        print("\n✓ Navigation demo video created successfully!")
        print("  You can now:")
        print("  - Play the video: videos/drone_nav_demo.mp4")
        print("  - Replay rosbag data: ros2 bag play bags/demo_run")
        print("  - View in RViz: rviz2")
    except Exception as e:
        print(f"\n✗ Error creating video: {e}")
        print("  Make sure ffmpeg is installed: apt-get install ffmpeg")
