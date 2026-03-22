#!/usr/bin/env python3
"""
demo_path_planning.py
══════════════════════════════════════════════════════════════
DEMO 1 — PATH PLANNING
Features: PRM  +  Theta*  +  Bidirectional A*
══════════════════════════════════════════════════════════════
Shows all three planners on the SAME map simultaneously.
Each path is drawn in a different colour with metrics annotated.
"""

import heapq, math, random, time
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

print("═"*60)
print("  DEMO 1 — PATH PLANNING SHOWCASE")
print("  PRM  +  Theta*  +  Bidirectional A*")
print("  Watch three planners race to find the best path!")
print("═"*60)

# ── World ─────────────────────────────────────────────────
W, H_MAX = 10.0, 5.0
VRES   = 0.4
NX = NY = int(2*W/VRES)
NZ = int(H_MAX/VRES)

START = np.array([-5.0, -4.0, 1.0])
GOAL  = np.array([ 5.0,  4.0, 2.5])
INFL  = 0.6

OBSTACLES = [
    (-3.5,  2.0, 0.55, 3.2),
    (-2.0, -3.0, 0.45, 2.5),
    ( 3.5,  2.5, 0.55, 3.5),
    ( 1.0,  0.0, 0.50, 3.0),
    (-1.0,  3.0, 0.40, 2.8),
    ( 2.5, -2.0, 0.50, 0.5),
    (-3.0,  0.5, 0.45, 0.4),
    ( 0.0, -1.5, 0.35, 0.35),
    (-2.5,  1.5, 0.40, 0.45),
    ( 3.0, -1.0, 0.38, 0.40),
    (-0.5, -3.5, 0.45, 0.42),
]

# ── Grid ──────────────────────────────────────────────────
GRID = np.zeros((NX, NY, NZ), dtype=bool)

def w2v(p):
    ix = int((p[0]+W)/VRES); iy = int((p[1]+W)/VRES); iz = int(p[2]/VRES)
    return (max(0,min(NX-1,ix)), max(0,min(NY-1,iy)), max(0,min(NZ-1,iz)))

def v2w(v):
    return np.array([v[0]*VRES-W+VRES/2, v[1]*VRES-W+VRES/2, v[2]*VRES+VRES/2])

for (cx,cy,r,h) in OBSTACLES:
    ri = r + INFL
    for ix in range(max(0,int((cx-ri+W)/VRES)), min(NX-1,int((cx+ri+W)/VRES))+1):
        for iy in range(max(0,int((cy-ri+W)/VRES)), min(NY-1,int((cy+ri+W)/VRES))+1):
            xw = ix*VRES - W; yw = iy*VRES - W
            if (xw-cx)**2+(yw-cy)**2 <= (ri)**2:
                iz_hi = min(NZ-1,int((h+INFL*0.5)/VRES))
                GRID[ix, iy, 0:iz_hi+1] = True

def edge_free(p1, p2, steps=10):
    for t in np.linspace(0,1,steps):
        p = p1 + t*(p2-p1)
        if GRID[w2v(p)]: return False
    return True

# ══════════════════════════════════════════
# PLANNER 1 — PRM
# ══════════════════════════════════════════
def build_prm(n=150):
    nodes = [START.copy(), GOAL.copy()]
    for _ in range(n):
        for _ in range(500):
            x = random.uniform(-W+0.3, W-0.3)
            y = random.uniform(-W+0.3, W-0.3)
            z = random.uniform(0.3, H_MAX-0.3)
            p = np.array([x,y,z])
            if not GRID[w2v(p)]:
                nodes.append(p); break
    adj = {i: [] for i in range(len(nodes))}
    for i,ni in enumerate(nodes):
        for j,nj in enumerate(nodes):
            if i==j: continue
            d = np.linalg.norm(ni-nj)
            if d < 2.5 and edge_free(ni, nj):
                adj[i].append((j, d))
    return nodes, adj

def dijkstra(nodes, adj):
    n = len(nodes); dist = [float('inf')]*n; dist[0]=0; prev=[None]*n
    heap = [(0.0, 0)]
    while heap:
        d,u = heapq.heappop(heap)
        if d > dist[u]: continue
        if u == 1:
            path=[]; c=u
            while c is not None: path.append(nodes[c]); c=prev[c]
            return list(reversed(path))
        for v,w in adj[u]:
            if dist[u]+w < dist[v]:
                dist[v]=dist[u]+w; prev[v]=u
                heapq.heappush(heap,(dist[v],v))
    return [START.copy(), GOAL.copy()]

# ══════════════════════════════════════════
# PLANNER 2 — Theta*
# ══════════════════════════════════════════
DIRS18 = [(dx,dy,dz) for dx in(-1,0,1) for dy in(-1,0,1) for dz in(-1,0,1)
          if (dx or dy or dz) and sum(abs(x) for x in(dx,dy,dz))<=2]

def los3d(a, b):
    x0,y0,z0=a; x1,y1,z1=b
    dx,dy,dz=abs(x1-x0),abs(y1-y0),abs(z1-z0)
    sx=1 if x1>x0 else -1; sy=1 if y1>y0 else -1; sz=1 if z1>z0 else -1
    if dx>=dy and dx>=dz:
        p1,p2=2*dy-dx,2*dz-dx
        while x0!=x1:
            x0+=sx
            if p1>0: y0+=sy; p1-=2*dx
            if p2>0: z0+=sz; p2-=2*dx
            p1+=2*dy; p2+=2*dz
            if not(0<=x0<NX and 0<=y0<NY and 0<=z0<NZ): return False
            if GRID[x0,y0,z0]: return False
    elif dy>=dz:
        p1,p2=2*dx-dy,2*dz-dy
        while y0!=y1:
            y0+=sy
            if p1>0: x0+=sx; p1-=2*dy
            if p2>0: z0+=sz; p2-=2*dy
            p1+=2*dx; p2+=2*dz
            if not(0<=x0<NX and 0<=y0<NY and 0<=z0<NZ): return False
            if GRID[x0,y0,z0]: return False
    else:
        p1,p2=2*dx-dz,2*dy-dz
        while z0!=z1:
            z0+=sz
            if p1>0: x0+=sx; p1-=2*dz
            if p2>0: y0+=sy; p2-=2*dz
            p1+=2*dx; p2+=2*dy
            if not(0<=x0<NX and 0<=y0<NY and 0<=z0<NZ): return False
            if GRID[x0,y0,z0]: return False
    return True

def theta_star():
    sv=w2v(START); gv=w2v(GOAL)
    OPEN=[]; heapq.heappush(OPEN,(math.dist(sv,gv),sv))
    g={sv:0.0}; parent={sv:sv}; closed=set()
    cnt=0
    while OPEN and cnt<10000:
        _,s=heapq.heappop(OPEN)
        if s==gv:
            path=[]; c=s
            while c!=parent[c]: path.append(v2w(c)); c=parent[c]
            path.append(v2w(sv)); path.reverse(); return path
        if s in closed: continue
        closed.add(s); cnt+=1
        for d in DIRS18:
            nb=(s[0]+d[0],s[1]+d[1],s[2]+d[2])
            if not(0<=nb[0]<NX and 0<=nb[1]<NY and 0<=nb[2]<NZ): continue
            if GRID[nb]: continue
            gp=parent[s]
            ng = g[gp]+math.dist(gp,nb) if los3d(gp,nb) else g[s]+math.dist(s,nb)
            np2 = gp if los3d(gp,nb) else s
            if nb not in g or ng<g[nb]:
                g[nb]=ng; parent[nb]=np2
                heapq.heappush(OPEN,(ng+math.dist(nb,gv),nb))
    return []

# ══════════════════════════════════════════
# PLANNER 3 — Bidirectional A*
# ══════════════════════════════════════════
DIRS_BD = [(1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1),
           (1,1,0),(1,-1,0),(-1,1,0),(-1,-1,0),
           (0,1,1),(0,1,-1),(0,-1,1),(0,-1,-1),
           (1,0,1),(1,0,-1),(-1,0,1),(-1,0,-1)]

def bidir_astar():
    sv=w2v(START); gv=w2v(GOAL)
    fwd_open=[(0.0,sv)]; fwd_g={sv:0.0}; fwd_par={sv:None}
    bwd_open=[(0.0,gv)]; bwd_g={gv:0.0}; bwd_par={gv:None}
    fwd_cl={}; bwd_cl={}
    best=math.inf; meeting=None; cnt=0
    while (fwd_open or bwd_open) and cnt<8000:
        if fwd_open:
            _,sf=heapq.heappop(fwd_open)
            if sf not in fwd_cl:
                fwd_cl[sf]=True; cnt+=1
                for d in DIRS_BD:
                    nb=(sf[0]+d[0],sf[1]+d[1],sf[2]+d[2])
                    if not(0<=nb[0]<NX and 0<=nb[1]<NY and 0<=nb[2]<NZ): continue
                    if GRID[nb]: continue
                    ng=fwd_g[sf]+math.dist(sf,nb)
                    if nb not in fwd_g or ng<fwd_g[nb]:
                        fwd_g[nb]=ng; fwd_par[nb]=sf
                        heapq.heappush(fwd_open,(ng+math.dist(nb,gv),nb))
                    if nb in bwd_cl and ng+bwd_g.get(nb,math.inf)<best:
                        best=ng+bwd_g.get(nb,math.inf); meeting=nb
        if bwd_open:
            _,sb=heapq.heappop(bwd_open)
            if sb not in bwd_cl:
                bwd_cl[sb]=True; cnt+=1
                for d in DIRS_BD:
                    nb=(sb[0]+d[0],sb[1]+d[1],sb[2]+d[2])
                    if not(0<=nb[0]<NX and 0<=nb[1]<NY and 0<=nb[2]<NZ): continue
                    if GRID[nb]: continue
                    ng=bwd_g[sb]+math.dist(sb,nb)
                    if nb not in bwd_g or ng<bwd_g[nb]:
                        bwd_g[nb]=ng; bwd_par[nb]=sb
                        heapq.heappush(bwd_open,(ng+math.dist(nb,sv),nb))
                    if nb in fwd_cl and ng+fwd_g.get(nb,math.inf)<best:
                        best=ng+fwd_g.get(nb,math.inf); meeting=nb
        if meeting and fwd_open and bwd_open and fwd_open[0][0]+bwd_open[0][0]>=best:
            break
    if meeting is None: return []
    fp=[]; c=meeting
    while c is not None: fp.append(v2w(c)); c=fwd_par.get(c)
    fp.reverse()
    bp=[]; c=bwd_par.get(meeting)
    while c is not None: bp.append(v2w(c)); c=bwd_par.get(c)
    return fp+bp

# ══════════════════════════════════════════
# RUN ALL PLANNERS
# ══════════════════════════════════════════
print("\n[1/3] Building PRM + Dijkstra...", end=" ", flush=True)
t0=time.time(); prm_nodes, prm_adj=build_prm(150); prm_path=dijkstra(prm_nodes,prm_adj); t_prm=(time.time()-t0)*1000
prm_len=sum(np.linalg.norm(prm_path[i+1]-prm_path[i]) for i in range(len(prm_path)-1))
print(f"✓  {len(prm_path)} waypoints | {prm_len:.1f}m | {t_prm:.0f}ms")

print("[2/3] Running Theta*...", end=" ", flush=True)
t0=time.time(); theta_path=theta_star(); t_theta=(time.time()-t0)*1000
theta_len=sum(np.linalg.norm(theta_path[i+1]-theta_path[i]) for i in range(len(theta_path)-1)) if theta_path else 0
print(f"✓  {len(theta_path)} waypoints | {theta_len:.1f}m | {t_theta:.0f}ms")

print("[3/3] Running Bidirectional A*...", end=" ", flush=True)
t0=time.time(); bd_path=bidir_astar(); t_bd=(time.time()-t0)*1000
bd_len=sum(np.linalg.norm(bd_path[i+1]-bd_path[i]) for i in range(len(bd_path)-1)) if bd_path else 0
print(f"✓  {len(bd_path)} waypoints | {bd_len:.1f}m | {t_bd:.0f}ms")

straight=np.linalg.norm(GOAL-START)
print(f"\n  Straight-line distance: {straight:.1f}m")
print(f"  PRM+Dijkstra:    {prm_len:.2f}m  ({prm_len/straight*100:.1f}% of straight)")
print(f"  Theta*:          {theta_len:.2f}m  ({theta_len/straight*100:.1f}% of straight)")
print(f"  Bidirectional A*:{bd_len:.2f}m  ({bd_len/straight*100:.1f}% of straight)")

# ══════════════════════════════════════════
# VISUALISE
# ══════════════════════════════════════════
fig, axes = plt.subplots(1, 2, figsize=(16, 8))
fig.patch.set_facecolor('#0d1117')
fig.suptitle("DEMO 1 — PATH PLANNING COMPARISON\nPRM + Theta* + Bidirectional A*",
             fontsize=15, fontweight='bold', color='white', y=0.98)

def draw_scene(ax, title):
    ax.set_facecolor('#161b22')
    ax.set_title(title, color='white', fontsize=11, pad=6)
    ax.set_xlim(-W, W); ax.set_ylim(-W, W)
    ax.set_aspect('equal'); ax.grid(True, alpha=0.15, color='white')
    ax.tick_params(colors='#888'); ax.set_xlabel('X (m)', color='#888'); ax.set_ylabel('Y (m)', color='#888')
    for spine in ax.spines.values(): spine.set_edgecolor('#30363d')
    # obstacles
    for (cx,cy,r,h) in OBSTACLES:
        ax.add_patch(plt.Circle((cx,cy), r+INFL, color='#1f6feb', alpha=0.35, zorder=2))
        ax.add_patch(plt.Circle((cx,cy), r, color='#388bfd', alpha=0.7, zorder=3))
    # PRM nodes (faint)
    for p in prm_nodes:
        ax.plot(p[0], p[1], '.', color='#484f58', ms=3, zorder=1)
    # PRM edges (faint)
    for i in range(len(prm_nodes)):
        for j,_ in prm_adj[i]:
            if j>i:
                p1,p2=prm_nodes[i],prm_nodes[j]
                ax.plot([p1[0],p2[0]],[p1[1],p2[1]], color='#21262d', lw=0.5, zorder=1)
    # start/goal
    ax.plot(*START[:2], '*', color='#3fb950', ms=16, zorder=10, label='START')
    ax.plot(*GOAL[:2],  '*', color='#f85149', ms=16, zorder=10, label='GOAL')
    ax.annotate('START', START[:2], textcoords='offset points', xytext=(8,4), color='#3fb950', fontsize=9)
    ax.annotate('GOAL',  GOAL[:2],  textcoords='offset points', xytext=(8,4), color='#f85149', fontsize=9)

# Left panel: all 3 paths
ax = axes[0]
draw_scene(ax, "All Three Paths Compared")

# straight line
ax.plot([START[0],GOAL[0]],[START[1],GOAL[1]], '--', color='white', alpha=0.25, lw=1, label=f'Straight ({straight:.1f}m)', zorder=4)

# PRM path
if prm_path:
    xs=[p[0] for p in prm_path]; ys=[p[1] for p in prm_path]
    ax.plot(xs,ys,'o-', color='#e3b341', lw=2, ms=5, label=f'PRM+Dijkstra ({prm_len:.1f}m)', zorder=6)

# Bidirectional A*
if bd_path:
    xs=[p[0] for p in bd_path]; ys=[p[1] for p in bd_path]
    ax.plot(xs,ys,'s-', color='#db61a2', lw=2, ms=5, label=f'BiDir A* ({bd_len:.1f}m)', zorder=7)

# Theta*
if theta_path:
    xs=[p[0] for p in theta_path]; ys=[p[1] for p in theta_path]
    ax.plot(xs,ys,'^-', color='#58a6ff', lw=2.5, ms=6, label=f'Theta* ({theta_len:.1f}m)', zorder=8)

ax.legend(loc='upper left', facecolor='#21262d', edgecolor='#30363d',
          labelcolor='white', fontsize=9)

# Right panel: metrics bar chart
ax2 = axes[1]
ax2.set_facecolor('#161b22')
ax2.set_title("Path Length & Time Comparison", color='white', fontsize=11, pad=6)
ax2.tick_params(colors='#888')
for spine in ax2.spines.values(): spine.set_edgecolor('#30363d')

labels = ['Straight\nLine\n(ideal)', 'PRM +\nDijkstra', 'Bidir\nA*', 'Theta*']
lengths = [straight, prm_len, bd_len if bd_len else 0, theta_len if theta_len else 0]
times_ms = [0, t_prm, t_bd, t_theta]
colors = ['#484f58', '#e3b341', '#db61a2', '#58a6ff']

x = np.arange(len(labels))
w2 = 0.35
bars1 = ax2.bar(x - w2/2, lengths, w2, color=colors, label='Path length (m)', alpha=0.85)
ax2.set_ylabel('Path Length (m)', color='white', fontsize=10)
ax2.set_ylim(0, max(lengths)*1.3)

ax3 = ax2.twinx()
ax3.bar(x + w2/2, times_ms, w2, color=colors, alpha=0.45, hatch='//', label='Compute time (ms)')
ax3.set_ylabel('Compute Time (ms)', color='#888', fontsize=10)
ax3.tick_params(colors='#888')

ax2.set_xticks(x); ax2.set_xticklabels(labels, color='white', fontsize=10)
ax2.set_facecolor('#161b22')

# annotate bars with values
for bar, length in zip(bars1, lengths):
    if length > 0:
        ax2.text(bar.get_x()+bar.get_width()/2, bar.get_height()+0.2,
                 f'{length:.1f}m', ha='center', color='white', fontsize=9, fontweight='bold')

# summary text box
opt_pct = theta_len/straight*100 if theta_len else 0
summary = (f"Straight-line distance:  {straight:.1f} m\n"
           f"PRM+Dijkstra:            {prm_len:.1f} m  ({prm_len/straight*100:.0f}%)\n"
           f"Bidirectional A*:        {bd_len:.1f} m  ({bd_len/straight*100:.0f}%)\n"
           f"Theta* (WINNER):  {theta_len:.1f} m  ({opt_pct:.0f}%)\n\n"
           f"Theta* gives the SHORTEST path!\n"
           f"(Any-angle LOS skips grid staircases)")
ax2.text(0.02, 0.02, summary, transform=ax2.transAxes,
         fontsize=9, color='white', verticalalignment='bottom',
         bbox=dict(facecolor='#21262d', edgecolor='#58a6ff', alpha=0.9, pad=8))

ax2.legend(loc='upper right', facecolor='#21262d', edgecolor='#30363d', labelcolor='white', fontsize=9)

plt.tight_layout()
print("\n✅ Visualization ready! Close the window to exit.")
plt.show()
