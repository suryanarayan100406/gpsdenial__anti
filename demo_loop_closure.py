#!/usr/bin/env python3
"""
demo_loop_closure.py
══════════════════════════════════════════════════════════════
DEMO 4 — LOOP CLOSURE & RELOCALISATION
Features: Loop Closure  +  Lidar Sensor Suite
══════════════════════════════════════════════════════════════
Drone flies an oval loop so it revisits positions.
Lidar beams shown as rays. Keyframe dots grow on the map.
When loop closure fires: GOLD flash + drift correction arrow.
"""

import math, random
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

print("═"*60)
print("  DEMO 4 — LOOP CLOSURE & RELOCALISATION")
print("  Loop Closure  +  Lidar Sensor")
print("  🟢 Green dots  = stored keyframes")
print("  🟡 Gold dot    = matched keyframe (loop closure!)")
print("  🔴 Red arrow  = drift  |  🟢 Green arrow = correction")
print("═"*60)

# ── World ─────────────────────────────────────────
W = 10.0
STATIC_OBS = [
    (-5.0,  3.0, 0.8), ( 4.0,  5.0, 0.9),
    ( 6.0, -3.0, 0.7), (-4.0, -5.0, 0.8),
    ( 0.0,  0.0, 1.0),
]

# ── Loop path: ellipse ────────────────────────────
def make_loop_path(steps=300):
    angles = np.linspace(0, 2*math.pi, steps, endpoint=False)
    xs = 6.5 * np.cos(angles)
    ys = 4.5 * np.sin(angles)
    return np.column_stack([xs, ys])

loop_path = make_loop_path(300)
PATH_IDX = [0]

# ── Simulated Lidar (2D horizontal) ──────────────
def read_lidar_2d(pos, n_beams=12, max_range=5.0):
    hits = []
    for i in range(n_beams):
        angle = 2*math.pi*i/n_beams
        dx = math.cos(angle); dy = math.sin(angle)
        hit = None
        for dist in np.linspace(0.1, max_range, 40):
            p = pos + np.array([dx,dy])*dist
            # check static obstacles
            for (cx,cy,r) in STATIC_OBS:
                if math.sqrt((p[0]-cx)**2+(p[1]-cy)**2) < r:
                    hit = (angle, dist, p.copy()); break
            if hit: break
            if abs(p[0])>W or abs(p[1])>W:
                hit = (angle, dist, p.copy()); break
        if hit is None:
            ex = pos + np.array([dx,dy])*max_range
            hit = (angle, max_range, ex)
        hits.append(hit)
    return hits

# ── Loop Closure ─────────────────────────────────
FINGERPRINT_BINS = 16
MATCH_THRESHOLD  = 0.86
MIN_KF_DIST      = 1.5

keyframes = []   # list of (pos, fingerprint)
closures  = []   # list of (pos, matched_kf_pos, correction)

def fingerprint(lidar_hits, drone_pos):
    ranges = [h[1] for h in lidar_hits]
    if not ranges: return np.zeros(FINGERPRINT_BINS)
    max_r = max(ranges) + 1e-6
    hist = np.zeros(FINGERPRINT_BINS)
    for r in ranges:
        idx = min(int(r/max_r*FINGERPRINT_BINS), FINGERPRINT_BINS-1)
        hist[idx] += 1.0
    n = np.linalg.norm(hist)
    return hist/(n+1e-9)

def cosine_sim(a, b):
    return float(np.dot(a,b)/((np.linalg.norm(a)*np.linalg.norm(b))+1e-9))

def add_keyframe(pos, lidar_hits):
    fp = fingerprint(lidar_hits, pos)
    if np.linalg.norm(fp) < 1e-6: return False
    for (kp,_) in keyframes:
        if np.linalg.norm(pos-kp) < MIN_KF_DIST: return False
    keyframes.append((pos.copy(), fp))
    return True

def check_loop(pos, lidar_hits, step):
    if len(keyframes) < 4: return False, None, None
    fp_cur = fingerprint(lidar_hits, pos)
    if np.linalg.norm(fp_cur) < 1e-6: return False, None, None
    best_sim=-1; best_idx=-1
    for i,(kp,kfp) in enumerate(keyframes):
        if np.linalg.norm(pos-kp) > 4.0: continue
        s = cosine_sim(fp_cur, kfp)
        if s > best_sim: best_sim=s; best_idx=i
    if best_idx<0 or best_sim<MATCH_THRESHOLD: return False, None, None
    matched_pos = keyframes[best_idx][0]
    correction = (matched_pos - pos)*0.30
    closures.append((pos.copy(), matched_pos.copy(), correction.copy()))
    return True, matched_pos, correction

# ── Drift simulation (accumulate small random drift) ──
drift = np.zeros(2)
DRIFT_RATE = 0.008   # metres per step drifts the drone's belief

# ── State ─────────────────────────────────────────
drone_pos = loop_path[0].copy()
flash_frames = [0]     # countdown for loop closure flash
last_closure = [None]  # (matched_kf_pos)

# ── Figure ────────────────────────────────────────
fig, ax = plt.subplots(figsize=(12, 10))
fig.patch.set_facecolor('#0d1117')
ax.set_facecolor('#161b22')
ax.set_xlim(-W-1, W+1); ax.set_ylim(-W-1, W+1)
ax.set_aspect('equal'); ax.grid(True, alpha=0.12, color='white')
ax.tick_params(colors='#888')
ax.set_xlabel('X (m)', color='#888'); ax.set_ylabel('Y (m)', color='#888')
fig.suptitle("DEMO 4 — LOOP CLOSURE & RELOCALISATION\nLidar Sensor + Keyframe DB + Cosine Similarity",
             fontsize=14, fontweight='bold', color='white')
for spine in ax.spines.values(): spine.set_edgecolor('#30363d')

# static obstacles
for (cx,cy,r) in STATIC_OBS:
    ax.add_patch(plt.Circle((cx,cy),r,color='#388bfd',alpha=0.7,zorder=3))
    ax.add_patch(plt.Circle((cx,cy),r+0.5,color='#1f6feb',alpha=0.2,zorder=2))

# loop path (faint)
ax.plot(loop_path[:,0], loop_path[:,1], '--', color='white', alpha=0.10, lw=1, label='Flight path')

drone_dot,  = ax.plot([],[],'o',color='#ffa657',ms=14,zorder=12)
trail_line, = ax.plot([],[],'-',color='#ffa657',lw=1.5,alpha=0.5,zorder=6)
lidar_lines = [ax.plot([],[],'-',color='#58a6ff',lw=0.7,alpha=0.35,zorder=4)[0] for _ in range(12)]
kf_dots,    = ax.plot([],[],'o',color='#3fb950',ms=8,zorder=8,label='Keyframes')
flash_text  = ax.text(0,0,'',ha='center',va='center',fontsize=22,color='#e3b341',
                       fontweight='bold',zorder=20,alpha=0.0)
status_text = ax.text(-W+0.2,W+0.4,'',color='white',fontsize=10,va='top',
                       bbox=dict(facecolor='#21262d',edgecolor='#30363d',alpha=0.9,pad=5))

correction_arrow = [None]
drift_arrow      = [None]

trail_x=[]; trail_y=[]

step_count = [0]

def update(frame):
    global drone_pos, drift
    step = frame
    step_count[0] = step

    # Advance along loop path
    PATH_IDX[0] = (PATH_IDX[0]+1) % len(loop_path)
    true_pos = loop_path[PATH_IDX[0]].copy()

    # Accumulate small drift (simulates position sensor error)
    drift += np.random.randn(2)*DRIFT_RATE
    drone_pos = true_pos + drift   # "believed" position

    # Lidar
    lidar_hits = read_lidar_2d(true_pos, n_beams=12, max_range=5.5)

    # Add keyframe every 15 steps
    if step % 15 == 0:
        add_keyframe(true_pos, lidar_hits)

    # Check loop closure
    lc_detected, matched_kf, correction = check_loop(drone_pos, lidar_hits, step)
    if lc_detected and correction is not None:
        drone_pos = drone_pos + correction
        drift *= 0.7   # reduce drift accumulation
        flash_frames[0] = 18   # flash for 18 frames
        last_closure[0] = matched_kf

    # Update trail
    trail_x.append(drone_pos[0]); trail_y.append(drone_pos[1])
    if len(trail_x)>200: trail_x.pop(0); trail_y.pop(0)

    # ── Draw ──────────────────────────────────────
    drone_dot.set_data([drone_pos[0]],[drone_pos[1]])
    trail_line.set_data(trail_x, trail_y)

    # Lidar beams
    for i,(ang,dist,hit_p) in enumerate(lidar_hits):
        dx=math.cos(ang)*dist; dy=math.sin(ang)*dist
        lidar_lines[i].set_data([true_pos[0], true_pos[0]+dx],
                                 [true_pos[1], true_pos[1]+dy])

    # Keyframe dots
    if keyframes:
        kxs=[k[0][0] for k in keyframes]; kys=[k[0][1] for k in keyframes]
        kf_dots.set_data(kxs, kys)

    # Flash on loop closure
    if flash_frames[0] > 0:
        flash_frames[0] -= 1
        alpha = min(1.0, flash_frames[0]/9)
        flash_text.set_position((drone_pos[0], drone_pos[1]+2.0))
        flash_text.set_text('🔄 LOOP CLOSURE!')
        flash_text.set_alpha(alpha)
        # Drone dot turns gold
        drone_dot.set_color('#e3b341')
        # Draw correction arrow
        if last_closure[0] is not None and correction_arrow[0]:
            try: correction_arrow[0].remove()
            except: pass
        if last_closure[0] is not None:
            corr_len = last_closure[0]-drone_pos
            correction_arrow[0] = ax.annotate('',
                xy=last_closure[0], xytext=drone_pos,
                arrowprops=dict(arrowstyle='->', color='#3fb950', lw=2.5),
                zorder=15)
    else:
        flash_text.set_alpha(0.0)
        drone_dot.set_color('#ffa657')
        if correction_arrow[0]:
            try: correction_arrow[0].remove()
            except: pass
            correction_arrow[0] = None

    # Status
    drift_mag = np.linalg.norm(drift)
    status_text.set_text(
        f"  Step: {step}\n"
        f"  Keyframes stored: {len(keyframes)}\n"
        f"  Loop closures: {len(closures)}\n"
        f"  Current drift: {drift_mag:.3f}m\n"
        f"  Threshold: sim ≥ {MATCH_THRESHOLD}"
    )

    return (drone_dot, trail_line, kf_dots, flash_text, status_text, *lidar_lines)

anim = FuncAnimation(fig, update, frames=600, interval=55, blit=False)

from matplotlib.lines import Line2D
legend_elems=[
    Line2D([0],[0],color='#3fb950',marker='o',ms=8,label=f'Keyframes (every 15 steps)',linestyle='None'),
    Line2D([0],[0],color='#e3b341',marker='o',ms=10,label='Matched keyframe (closure)',linestyle='None'),
    Line2D([0],[0],color='#58a6ff',lw=1.5,label='Lidar beams'),
    Line2D([0],[0],color='#ffa657',lw=2,label='Drone trail'),
]
ax.legend(handles=legend_elems,loc='lower right',facecolor='#21262d',
          edgecolor='#30363d',labelcolor='white',fontsize=9)

print("\n✅ Animation ready! Close the window to exit.")
plt.tight_layout()
plt.show()
