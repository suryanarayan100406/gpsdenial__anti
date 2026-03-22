#!/usr/bin/env python3
"""
demo_predictive_avoidance.py
══════════════════════════════════════════════════════════════
DEMO 3 — PREDICTIVE AVOIDANCE
Features: ObstaclePredictor  +  DWA  +  MPC
══════════════════════════════════════════════════════════════
Ghost trails show where obstacles will be. DWA velocity arrows
show candidate velocities (red=bad, green=best). MPC rollout
traces show 60 sampled future trajectories.
"""

import math, random, time
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
from matplotlib.patches import FancyArrowPatch

print("═"*60)
print("  DEMO 3 — PREDICTIVE AVOIDANCE")
print("  ObstaclePredictor  +  DWA  +  MPC")
print("  Ghost trails = predicted obstacle positions (6 steps)")
print("  Green arrow = DWA best velocity")
print("  Faint lines = MPC rollout trajectories")
print("═"*60)

# ── World (2D top-view) ───────────────────────────
W = 10.0
START = np.array([-8.0, -6.0])
GOAL  = np.array([ 8.0,  6.0])

STATIC_OBS = [(-3.0, 1.0, 0.8), (2.0, -2.0, 0.9), (5.0, 3.0, 0.7)]

# Fast dynamic obstacles heading toward the drone's route
DYN_OBS = [
    {'pos': np.array([ 2.0,  8.0]), 'vel': np.array([-0.05, -0.22]), 'r': 1.0, 'color':'#f85149'},
    {'pos': np.array([-5.0, -8.0]), 'vel': np.array([ 0.08,  0.20]), 'r': 0.9, 'color':'#e3b341'},
]

# ── ObstaclePredictor ─────────────────────────────
class ObstaclePredictor:
    def __init__(self, obs):
        self.obs = obs
        self.history = {i:[] for i in range(len(obs))}

    def update(self):
        for i,o in enumerate(self.obs):
            self.history[i].append(o['pos'].copy())
            if len(self.history[i])>10: self.history[i].pop(0)

    def predict(self, horizon=6, dt=0.1):
        preds=[]
        for i,o in enumerate(self.obs):
            if len(self.history[i])>=2:
                v_est=(self.history[i][-1]-self.history[i][-2])/dt
            else:
                v_est=o['vel'].copy()
            cur=o['pos'].copy()
            future=[(cur+v_est*k*dt, o['r']) for k in range(1,horizon+1)]
            preds.append(future)
        return preds

predictor = ObstaclePredictor(DYN_OBS)

# ── DWA ──────────────────────────────────────────
def dwa_compute(pos, vel, goal, dyn_pred_1step, dt=0.1):
    max_speed=2.5; max_accel=3.0; v_samp=6; t_samp=12; sim_t=1.0; dt_s=0.15
    cur_speed=np.linalg.norm(vel)
    v_min=max(0.0,cur_speed-max_accel*dt_s); v_max=min(max_speed,cur_speed+max_accel*dt_s)
    w_head=2.0; w_clear=3.0; w_speed=0.8

    best_score=-1e9; best_vel=vel.copy()
    candidates=[]
    for vi in range(v_samp):
        v_mag=v_min+(v_max-v_min)*vi/max(1,v_samp-1)
        for ti in range(t_samp):
            angle=2*math.pi*ti/t_samp
            direction=np.array([math.cos(angle),math.sin(angle)])
            vc=direction*v_mag
            # simulate
            p=pos.copy(); min_cl=1e9; ok=True
            for _ in range(int(sim_t/dt_s)):
                p=p+vc*dt_s
                for (cx,cy,r) in STATIC_OBS:
                    d=math.sqrt((p[0]-cx)**2+(p[1]-cy)**2)-r
                    if d<0.4: ok=False; break
                    min_cl=min(min_cl,d)
                if not ok: break
                for (dp,dr) in dyn_pred_1step:
                    d=np.linalg.norm(p-dp)-dr
                    if d<0.4: ok=False; break
                    min_cl=min(min_cl,d)
                if not ok: break
            score=-1e9
            if ok:
                dist=np.linalg.norm(goal-p)
                score=(w_head/(dist+0.01)+w_clear*min(1,min_cl/2)+w_speed*v_mag/max_speed)
            candidates.append((vc,score,ok))
            if score>best_score: best_score=score; best_vel=vc
    return best_vel, candidates

# ── MPC ──────────────────────────────────────────
def mpc_compute(pos, vel, goal, dyn_preds, n_rollouts=40, N=5, dt=0.1):
    max_speed=2.5; max_accel=2.5
    Q_goal=2.0; Q_obs=5.0; Q_ctrl=0.3; Q_smooth=0.4
    best_cost=1e9; best_vel=vel.copy(); rollout_traces=[]

    for _ in range(n_rollouts):
        controls=[]; v_prev=vel.copy()
        for k in range(N):
            dv=np.random.randn(2)*max_accel*dt
            vc=np.clip(v_prev+dv,-max_speed,max_speed)
            spd=np.linalg.norm(vc)
            if spd>max_speed: vc*=max_speed/spd
            controls.append(vc); v_prev=vc

        p=pos.copy(); cost=0.0; ok=True; trace=[pos.copy()]
        for k,vc in enumerate(controls):
            p=p+vc*dt; trace.append(p.copy())
            obs_k=dyn_preds[min(k,len(dyn_preds)-1)]
            for (dp,dr) in obs_k:
                d=np.linalg.norm(p-dp)-dr
                if d<0.25: ok=False; break
                cost+=Q_obs*max(0,1.5-d)
            if not ok: break
            for (cx,cy,r) in STATIC_OBS:
                d=math.sqrt((p[0]-cx)**2+(p[1]-cy)**2)-r
                if d<0.25: ok=False; break
                cost+=Q_obs*max(0,1.2-d)
            if not ok: break
            cost+=Q_ctrl*np.linalg.norm(vc)**2
            if k>0: cost+=Q_smooth*np.linalg.norm(vc-controls[k-1])
        if not ok: rollout_traces.append((trace,False)); continue
        cost+=Q_goal*np.linalg.norm(p-goal)
        rollout_traces.append((trace,True))
        if cost<best_cost: best_cost=cost; best_vel=controls[0]
    return best_vel, rollout_traces

# ── Simulation ────────────────────────────────────
drone_pos = START.copy().astype(float)
drone_vel = np.zeros(2)
dt = 0.1

# ── Figure ────────────────────────────────────────
fig, ax = plt.subplots(figsize=(13, 9))
fig.patch.set_facecolor('#0d1117')
ax.set_facecolor('#161b22')
ax.set_xlim(-W, W); ax.set_ylim(-W, W)
ax.set_aspect('equal')
ax.grid(True, alpha=0.12, color='white')
ax.tick_params(colors='#888')
ax.set_xlabel('X (m)', color='#888'); ax.set_ylabel('Y (m)', color='#888')
fig.suptitle("DEMO 3 — PREDICTIVE AVOIDANCE\nObstaclePredictor + DWA + MPC",
             fontsize=14, fontweight='bold', color='white')
for spine in ax.spines.values(): spine.set_edgecolor('#30363d')

# static obs
for (cx,cy,r) in STATIC_OBS:
    ax.add_patch(plt.Circle((cx,cy),r,color='#388bfd',alpha=0.6,zorder=3))

ax.plot(*START,'*',color='#3fb950',ms=16,zorder=10); ax.text(START[0]+0.3,START[1]+0.3,'START',color='#3fb950',fontsize=9)
ax.plot(*GOAL, '*',color='#f85149',ms=16,zorder=10); ax.text(GOAL[0]+0.3, GOAL[1]+0.3, 'GOAL', color='#f85149',fontsize=9)
ax.plot([START[0],GOAL[0]],[START[1],GOAL[1]],'--',color='white',alpha=0.15,lw=1)

legend_patches=[
    mpatches.Patch(color='#3fb950',alpha=0.8,label='DWA best velocity'),
    mpatches.Patch(color='#f85149',alpha=0.4,label='DWA bad candidates'),
    mpatches.Patch(color='#58a6ff',alpha=0.25,label='MPC rollout traces'),
    mpatches.Patch(color='white',alpha=0.5,label='Ghost (predicted pos)'),
]
ax.legend(handles=legend_patches,loc='lower left',facecolor='#21262d',edgecolor='#30363d',labelcolor='white',fontsize=8)

drone_dot,=ax.plot([],[],'D',color='#ffa657',ms=12,zorder=12,label='Drone')
drone_trail_x=[]; drone_trail_y=[]
trail_line,=ax.plot([],[],'-',color='#ffa657',lw=1,alpha=0.4,zorder=6)
status_text=ax.text(-W+0.3,W-0.5,'',color='white',fontsize=9,va='top',
                    bbox=dict(facecolor='#21262d',edgecolor='#30363d',alpha=0.9,pad=5))

dyn_circles=[plt.Circle(o['pos'],o['r'],color=o['color'],alpha=0.8,zorder=8) for o in DYN_OBS]
dyn_labels=[]
for i,o in enumerate(DYN_OBS):
    ax.add_patch(dyn_circles[i])
    lbl=ax.text(o['pos'][0],o['pos'][1]+o['r']+0.3,f'OBS {i+1}',ha='center',color=o['color'],fontsize=8,fontweight='bold')
    dyn_labels.append(lbl)

ghost_dots=[[ax.plot([],[],'+',color=o['color'],ms=6,alpha=0.0,zorder=5)[0] for _ in range(6)] for o in DYN_OBS]
mpc_lines=[ax.plot([],[],'-',color='#58a6ff',lw=0.8,alpha=0.0,zorder=4)[0] for _ in range(40)]
arrow_lines=[]

def clear_arrows():
    for a in arrow_lines:
        try: a.remove()
        except: pass
    arrow_lines.clear()

def update(frame):
    global drone_pos, drone_vel
    # Move dynamic obstacles
    for o in DYN_OBS:
        o['pos'] += o['vel']
        if abs(o['pos'][0])>W or abs(o['pos'][1])>W:
            o['vel'] *= -1

    predictor.update()
    preds_full=predictor.predict(horizon=6, dt=dt)

    # 1-step ahead for DWA
    dwa_pred1=[(preds_full[i][0][0], preds_full[i][0][1]) for i in range(len(DYN_OBS))]
    mpc_preds=[]
    for k in range(5):
        mpc_preds.append([(preds_full[i][k][0], preds_full[i][k][1]) for i in range(len(DYN_OBS))])

    # DWA
    dwa_vel, cands = dwa_compute(drone_pos, drone_vel, GOAL, dwa_pred1)
    # MPC
    mpc_vel, rollouts = mpc_compute(drone_pos, drone_vel, GOAL, mpc_preds, n_rollouts=40)

    # Blend
    vel_cmd = 0.4*dwa_vel + 0.6*mpc_vel
    spd=np.linalg.norm(vel_cmd)
    if spd>2.5: vel_cmd*=2.5/spd
    drone_vel = vel_cmd
    drone_pos = drone_pos + drone_vel*dt

    # Clamp to world
    drone_pos = np.clip(drone_pos,-W+0.2,W-0.2)

    # Update visuals
    drone_dot.set_data([drone_pos[0]],[drone_pos[1]])
    drone_trail_x.append(drone_pos[0]); drone_trail_y.append(drone_pos[1])
    if len(drone_trail_x)>80: drone_trail_x.pop(0); drone_trail_y.pop(0)
    trail_line.set_data(drone_trail_x, drone_trail_y)

    # Dynamic obstacles
    for i,o in enumerate(DYN_OBS):
        dyn_circles[i].center=o['pos']
        dyn_labels[i].set_position((o['pos'][0],o['pos'][1]+o['r']+0.3))

    # Ghost predictions
    for i in range(len(DYN_OBS)):
        for k in range(6):
            gp,_ = preds_full[i][k]
            alpha=0.5*(1-(k/7))
            ghost_dots[i][k].set_data([gp[0]],[gp[1]])
            ghost_dots[i][k].set_alpha(alpha)

    # DWA arrows
    clear_arrows()
    for vc,score,ok in cands[::3]:
        if not ok: continue
        c_val=max(0,min(1,(score+2)/6))
        col=(1-c_val, c_val, 0.2)
        arr=ax.annotate('',xy=drone_pos+vc*0.5,xytext=drone_pos,
                        arrowprops=dict(arrowstyle='->',color=col,lw=1.2),zorder=7)
        arrow_lines.append(arr)
    # Best DWA arrow (bold green)
    arr=ax.annotate('',xy=drone_pos+dwa_vel*0.8,xytext=drone_pos,
                    arrowprops=dict(arrowstyle='->',color='#3fb950',lw=3),zorder=11)
    arrow_lines.append(arr)

    # MPC rollouts
    for idx,(trace,ok) in enumerate(rollouts[:40]):
        if idx<len(mpc_lines):
            xs=[p[0] for p in trace]; ys=[p[1] for p in trace]
            mpc_lines[idx].set_data(xs,ys)
            mpc_lines[idx].set_alpha(0.18 if ok else 0.04)
            mpc_lines[idx].set_color('#58a6ff' if ok else '#f85149')

    # Status
    dist=np.linalg.norm(drone_pos-GOAL)
    status_text.set_text(f"  Frame: {frame}\n"
                         f"  Drone speed: {np.linalg.norm(drone_vel):.2f} m/s\n"
                         f"  Dist to goal: {dist:.1f}m\n"
                         f"  DWA + MPC blending active" if True else "")
    if dist<0.8:
        global drone_pos
        drone_pos=START.copy().astype(float); drone_vel[:]=0
        drone_trail_x.clear(); drone_trail_y.clear()

    return (drone_dot,trail_line,status_text,*[d for row in ghost_dots for d in row],*mpc_lines)

anim=FuncAnimation(fig,update,frames=500,interval=60,blit=False)
print("\n✅ Animation ready! Close the window to exit.")
plt.tight_layout()
plt.show()
