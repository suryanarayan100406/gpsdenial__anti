#!/usr/bin/env python3
"""
demo_reactive_control.py
══════════════════════════════════════════════════════════════
DEMO 5 — REACTIVE CONTROL
Features: Potential Fields  +  Emergency Reflexes  +  PID
══════════════════════════════════════════════════════════════
Force field arrows drawn on a grid. A fast-moving obstacle
approaches; field updates live. When dist < 0.8m: REFLEX FIRES,
screen flashes red. PID P/I/D terms plotted on side panel.
"""

import math
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as mpatches

print("═"*60)
print("  DEMO 5 — REACTIVE CONTROL")
print("  Potential Fields  +  Emergency Reflexes  +  PID")
print("  🟦 Arrow field = attractive + repulsive forces live")
print("  🔴 Red flash   = EMERGENCY REFLEX fired!")
print("  📈 Right panel = PID P/I/D terms over time")
print("═"*60)

# ── World ─────────────────────────────────────────
W = 8.0
START = np.array([-6.5, -5.5])
GOAL  = np.array([ 6.5,  5.5])

STATIC_OBS = [
    (-3.0,  1.5, 0.9),
    ( 2.5, -1.5, 1.0),
    ( 0.0,  3.5, 0.8),
    (-1.5, -3.5, 0.7),
    ( 4.5,  1.0, 0.8),
]

# Fast obstacle heading directly at the drone's route
DYN_OBS = {'pos': np.array([ 5.0, -5.0]), 'vel': np.array([-0.14, 0.17]), 'r': 1.0}

# ── Potential Field ───────────────────────────────
K_ATT = 1.5; K_REP = 8.0; RHO_0 = 1.8

def potential_force(pos, goal, dyn_pos, dyn_r):
    to_goal = goal - pos
    dist_goal = np.linalg.norm(to_goal)
    f_att = K_ATT * to_goal / (dist_goal + 1e-6)
    f_rep = np.zeros(2)
    all_obs = [(np.array([cx,cy]), r) for (cx,cy,r) in STATIC_OBS]
    all_obs.append((dyn_pos, dyn_r))
    for (opos,orad) in all_obs:
        diff = pos - opos
        rho = np.linalg.norm(diff) - orad
        if 0 < rho < RHO_0:
            coeff = K_REP*(1/rho - 1/RHO_0)*(1/rho**2)
            grad = diff/(np.linalg.norm(diff)+1e-9)
            f_rep += coeff*grad
    f_total = f_att + f_rep
    mag = np.linalg.norm(f_total)
    if mag > 4.0: f_total = f_total*4.0/mag
    return f_total

# ── PID Controller ────────────────────────────────
class PID2D:
    def __init__(self, kp, ki, kd, limit=3.5, windup=2.0):
        self.kp=kp; self.ki=ki; self.kd=kd
        self.limit=limit; self.windup=windup
        self.integral=np.zeros(2); self.prev_err=np.zeros(2)
        self.p_hist=[]; self.i_hist=[]; self.d_hist=[]

    def step(self, error, dt=0.1):
        self.integral += error*dt
        mag=np.linalg.norm(self.integral)
        if mag>self.windup: self.integral*=self.windup/mag
        deriv=(error-self.prev_err)/(dt+1e-9)
        self.prev_err=error.copy()
        P=self.kp*error; I=self.ki*self.integral; D=self.kd*deriv
        out=P+I+D
        out_mag=np.linalg.norm(out)
        if out_mag>self.limit: out*=self.limit/out_mag
        self.p_hist.append(np.linalg.norm(P))
        self.i_hist.append(np.linalg.norm(I))
        self.d_hist.append(np.linalg.norm(D))
        if len(self.p_hist)>120:
            self.p_hist.pop(0); self.i_hist.pop(0); self.d_hist.pop(0)
        return out

pid = PID2D(kp=2.2, ki=0.15, kd=0.8, limit=3.0, windup=2.0)

# ── State ─────────────────────────────────────────
drone_pos = START.copy().astype(float)
drone_vel = np.zeros(2)
reflex_active = [False]
reflex_flash  = [0]
trail_x=[]; trail_y=[]

# Pre-compute force field grid
GRID_RES = 1.0
gx = np.arange(-W, W+GRID_RES, GRID_RES)
gy = np.arange(-W, W+GRID_RES, GRID_RES)
GX, GY = np.meshgrid(gx, gy)

# ── Figure ────────────────────────────────────────
fig, (ax, ax_pid) = plt.subplots(1, 2, figsize=(16, 9),
                                   gridspec_kw={'width_ratios':[2.5,1]})
fig.patch.set_facecolor('#0d1117')
fig.suptitle("DEMO 5 — REACTIVE CONTROL\nPotential Fields + Emergency Reflexes + PID",
             fontsize=14, fontweight='bold', color='white')

ax.set_facecolor('#161b22')
ax.set_xlim(-W, W); ax.set_ylim(-W, W)
ax.set_aspect('equal')
ax.tick_params(colors='#888')
ax.set_xlabel('X (m)', color='#888'); ax.set_ylabel('Y (m)', color='#888')
for spine in ax.spines.values(): spine.set_edgecolor('#30363d')

ax_pid.set_facecolor('#161b22')
ax_pid.tick_params(colors='#888')
ax_pid.set_xlabel('Steps', color='#888'); ax_pid.set_ylabel('Term magnitude', color='#888')
ax_pid.set_title('PID Terms over Time', color='white', fontsize=10)
ax_pid.set_xlim(0, 120); ax_pid.set_ylim(0, 5)
for spine in ax_pid.spines.values(): spine.set_edgecolor('#30363d')

# Static obstacles
for (cx,cy,r) in STATIC_OBS:
    ax.add_patch(plt.Circle((cx,cy), r, color='#388bfd', alpha=0.7, zorder=3))
    ax.add_patch(plt.Circle((cx,cy), RHO_0, color='#1f6feb', alpha=0.08, zorder=2))

ax.plot(*START,'*',color='#3fb950',ms=16,zorder=10)
ax.plot(*GOAL, '*',color='#f85149',ms=16,zorder=10)
ax.text(START[0]+0.3,START[1]+0.3,'START',color='#3fb950',fontsize=9)
ax.text(GOAL[0]+0.3, GOAL[1]+0.3, 'GOAL', color='#f85149', fontsize=9)

# Quiver arrows for potential field (will update every frame)
U_init = np.zeros_like(GX)
V_init = np.zeros_like(GY)
quiv = ax.quiver(GX, GY, U_init, V_init,
                  color='#58a6ff', alpha=0.45, scale=25,
                  width=0.003, zorder=4)

dyn_circ = plt.Circle(DYN_OBS['pos'], DYN_OBS['r'], color='#f85149', alpha=0.85, zorder=8)
ax.add_patch(dyn_circ)
dyn_lbl = ax.text(DYN_OBS['pos'][0], DYN_OBS['pos'][1]+DYN_OBS['r']+0.35,
                   '⚠ FAST OBS', ha='center', color='#f85149', fontsize=9, fontweight='bold')

drone_dot,  = ax.plot([],[],'D',color='#ffa657',ms=14,zorder=12)
trail_line, = ax.plot([],[],'-',color='#ffa657',lw=1.5,alpha=0.5,zorder=6)
reflex_arrow = [None]
status_text = ax.text(-W+0.2, W-0.3, '', color='white', fontsize=9, va='top',
                       bbox=dict(facecolor='#21262d',edgecolor='#30363d',alpha=0.9,pad=5))
flash_bg = ax.add_patch(
    plt.Rectangle((-W,-W), 2*W, 2*W, color='#f85149', alpha=0.0, zorder=18))

pid_p, = ax_pid.plot([],[],'-',color='#58a6ff',lw=2,label='P term')
pid_i, = ax_pid.plot([],[],'-',color='#e3b341',lw=2,label='I term')
pid_d, = ax_pid.plot([],[],'-',color='#3fb950',lw=2,label='D term')
ax_pid.legend(facecolor='#21262d',edgecolor='#30363d',labelcolor='white',fontsize=9)
reflex_marker = ax_pid.axvline(x=-1,color='#f85149',lw=2,alpha=0,label='Reflex!')
reflex_lines=[]

legend_patches=[
    mpatches.Patch(color='#58a6ff',alpha=0.6,label='Attractive + Repulsive field'),
    mpatches.Patch(color='#3fb950',alpha=0.8,label='PID velocity command'),
    mpatches.Patch(color='#f85149',alpha=0.8,label='Emergency reflex escape'),
]
ax.legend(handles=legend_patches,loc='lower left',facecolor='#21262d',
          edgecolor='#30363d',labelcolor='white',fontsize=8)

step_num = [0]

def update(frame):
    global drone_pos, drone_vel
    step_num[0] = frame

    # Move dynamic obstacle; bounce off walls
    DYN_OBS['pos'] += DYN_OBS['vel']
    for dim in range(2):
        if abs(DYN_OBS['pos'][dim]) > W-0.5:
            DYN_OBS['vel'][dim] *= -1
    dyn_circ.center = DYN_OBS['pos']
    dyn_lbl.set_position((DYN_OBS['pos'][0], DYN_OBS['pos'][1]+DYN_OBS['r']+0.35))

    # Minimum distance to any obstacle (for reflex check)
    min_dist = min(
        [math.sqrt((drone_pos[0]-cx)**2+(drone_pos[1]-cy)**2)-r for (cx,cy,r) in STATIC_OBS]
        + [np.linalg.norm(drone_pos-DYN_OBS['pos'])-DYN_OBS['r']]
    )

    reflex_fired = False
    if min_dist < 0.8:
        # EMERGENCY REFLEX: escape directly away from nearest obstacle
        all_obs_pos = [(np.array([cx,cy]),r) for (cx,cy,r) in STATIC_OBS]
        all_obs_pos.append((DYN_OBS['pos'].copy(), DYN_OBS['r']))
        nearest_obs_pos = min(all_obs_pos,
            key=lambda op: np.linalg.norm(drone_pos-op[0])-op[1])[0]
        away = drone_pos - nearest_obs_pos
        away_norm = away/(np.linalg.norm(away)+1e-6)
        vel_cmd = away_norm * 2.5
        reflex_active[0] = True; reflex_flash[0]=15; reflex_fired=True
    else:
        # PF + PID
        pf_force = potential_force(drone_pos, GOAL, DYN_OBS['pos'], DYN_OBS['r'])
        pos_err = GOAL - drone_pos
        pid_out = pid.step(pos_err)
        vel_cmd = pid_out + 0.35*pf_force
        spd=np.linalg.norm(vel_cmd)
        if spd>2.5: vel_cmd*=2.5/spd
        reflex_active[0] = False

    drone_vel = vel_cmd
    drone_pos = drone_pos + drone_vel*0.1
    drone_pos = np.clip(drone_pos, -W+0.2, W-0.2)

    # Trail
    trail_x.append(drone_pos[0]); trail_y.append(drone_pos[1])
    if len(trail_x)>150: trail_x.pop(0); trail_y.pop(0)
    drone_dot.set_data([drone_pos[0]],[drone_pos[1]])
    trail_line.set_data(trail_x, trail_y)

    # Update quiver (potential field) every 5 frames
    if frame % 5 == 0:
        U = np.zeros_like(GX); V = np.zeros_like(GY)
        for i in range(GX.shape[0]):
            for j in range(GX.shape[1]):
                pos_ij = np.array([GX[i,j], GY[i,j]])
                # skip if inside obstacle
                skip=any(math.sqrt((pos_ij[0]-cx)**2+(pos_ij[1]-cy)**2)<r+0.1
                         for (cx,cy,r) in STATIC_OBS)
                if not skip:
                    f=potential_force(pos_ij, GOAL, DYN_OBS['pos'], DYN_OBS['r'])
                    U[i,j]=f[0]; V[i,j]=f[1]
        quiv.set_UVC(U.flatten()[:len(quiv.U)], V.flatten()[:len(quiv.U)])

    # Reflex arrow
    if reflex_arrow[0]:
        try: reflex_arrow[0].remove()
        except: pass
        reflex_arrow[0]=None
    if reflex_fired:
        reflex_arrow[0]=ax.annotate('',
            xy=drone_pos+vel_cmd*0.5, xytext=drone_pos,
            arrowprops=dict(arrowstyle='->', color='#f85149', lw=4), zorder=15)
        drone_dot.set_color('#f85149')
    else:
        # PF+PID arrow (green)
        if reflex_arrow[0]: pass
        reflex_arrow[0]=ax.annotate('',
            xy=drone_pos+drone_vel*0.5, xytext=drone_pos,
            arrowprops=dict(arrowstyle='->', color='#3fb950', lw=2.5), zorder=13)
        drone_dot.set_color('#ffa657')

    # Flash background red on reflex
    if reflex_flash[0]>0:
        reflex_flash[0]-=1
        flash_bg.set_alpha(0.08*(reflex_flash[0]/15))
    else:
        flash_bg.set_alpha(0.0)

    # PID time series
    n=len(pid.p_hist)
    xs=list(range(n))
    pid_p.set_data(xs, pid.p_hist)
    pid_i.set_data(xs, pid.i_hist)
    pid_d.set_data(xs, pid.d_hist)
    ax_pid.set_ylim(0, max(max(pid.p_hist+[0.1]),max(pid.i_hist+[0.1]),max(pid.d_hist+[0.1]))*1.2)

    # Reflex marker line on PID plot
    if reflex_fired:
        rl=ax_pid.axvline(x=n-1,color='#f85149',lw=1.5,alpha=0.7)
        reflex_lines.append(rl)
    if len(reflex_lines)>20: reflex_lines.pop(0)

    # Status
    mode='⚡ REFLEX' if reflex_active[0] else '🧠 PID + PF'
    status_text.set_text(
        f"  Mode: {mode}\n"
        f"  Min dist: {min_dist:.2f}m (threshold 0.8m)\n"
        f"  Speed: {np.linalg.norm(drone_vel):.2f} m/s\n"
        f"  Dist to goal: {np.linalg.norm(drone_pos-GOAL):.1f}m"
    )
    status_text.set_bbox({'facecolor':('#4a0000' if reflex_active[0] else '#21262d'),
                           'edgecolor':('#f85149' if reflex_active[0] else '#30363d'),
                           'alpha':0.92,'pad':5})

    # Reset on goal reached
    if np.linalg.norm(drone_pos-GOAL)<0.8:
        drone_pos[:]=START.copy()
        drone_vel[:]=0
        trail_x.clear(); trail_y.clear()
        pid.__init__(kp=2.2,ki=0.15,kd=0.8,limit=3.0,windup=2.0)

    return (drone_dot, trail_line, dyn_circ, dyn_lbl, status_text, flash_bg,
            quiv, pid_p, pid_i, pid_d)

anim = FuncAnimation(fig, update, frames=600, interval=65, blit=False)
print("\n✅ Animation ready! Close the window to exit.")
plt.tight_layout()
plt.show()
