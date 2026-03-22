#!/usr/bin/env python3
"""
demo_dynamic_replanning.py
══════════════════════════════════════════════════════════════
DEMO 2 — DYNAMIC REPLANNING
Features: D* Lite  +  Jump Point Search (JPS)
══════════════════════════════════════════════════════════════
Drone flies toward goal. A moving blocker crosses the path.
D* Lite replans incrementally. JPS fires if directly blocked.
"""

import heapq, math, random, time
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation

print("═"*60)
print("  DEMO 2 — DYNAMIC REPLANNING")
print("  D* Lite  +  Jump Point Search (JPS)")
print("  Watch the drone replan in real-time when blocked!")
print("═"*60)

# ── World ────────────────────────────────────────
W = 10.0; VRES = 0.5
NX = NY = int(2*W/VRES); NZ = 6

START = np.array([-7.0, -6.0, 1.0])
GOAL  = np.array([ 7.0,  6.0, 1.0])

OBSTACLES = [
    (-4.0,  1.0, 0.8), (-1.5, -3.0, 0.8),
    ( 2.0,  2.5, 0.9), ( 5.0, -2.0, 0.7),
    (-3.0,  4.0, 0.7), ( 1.0,  0.0, 0.8),
]  # (cx, cy, r)

INFL = 0.5

GRID = np.zeros((NX, NY, NZ), dtype=bool)

def w2v(p):
    ix = int((p[0]+W)/VRES); iy = int((p[1]+W)/VRES)
    iz = max(0, min(NZ-1, int(p[2]/VRES)))
    return (max(0,min(NX-1,ix)), max(0,min(NY-1,iy)), iz)

def v2w(v):
    return np.array([v[0]*VRES-W+VRES/2, v[1]*VRES-W+VRES/2, v[2]*VRES+VRES/2])

for (cx,cy,r) in OBSTACLES:
    ri=r+INFL
    for ix in range(max(0,int((cx-ri+W)/VRES)), min(NX-1,int((cx+ri+W)/VRES))+1):
        for iy in range(max(0,int((cy-ri+W)/VRES)), min(NY-1,int((cy+ri+W)/VRES))+1):
            if ((ix*VRES-W)-cx)**2+((iy*VRES-W)-cy)**2 <= ri**2:
                GRID[ix,iy,:] = True

def get_grid_with_dynamic(dyn_pos, dyn_r):
    g = GRID.copy()
    ri = dyn_r + INFL
    cx,cy = dyn_pos[0], dyn_pos[1]
    for ix in range(max(0,int((cx-ri+W)/VRES)), min(NX-1,int((cx+ri+W)/VRES))+1):
        for iy in range(max(0,int((cy-ri+W)/VRES)), min(NY-1,int((cy+ri+W)/VRES))+1):
            if ((ix*VRES-W)-cx)**2+((iy*VRES-W)-cy)**2 <= ri**2:
                g[ix,iy,:] = True
    return g

# ── D* Lite ──────────────────────────────────────
class DStarLite2D:
    INF = float('inf')
    DIRS = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]

    def __init__(self, grid):
        self.grid = grid
        self._g = {}; self._rhs = {}; self._U = []; self._km = 0
        self.start = None; self.goal = None

    def _v(self,n): return self._g.get(n, self.INF)
    def _r(self,n): return self._rhs.get(n, self.INF)
    def _h(self,a,b): return math.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)
    def _key(self,s):
        k2=min(self._v(s),self._r(s)); return (k2+self._h(s,self.start)+self._km, k2)

    def _succ(self,s):
        out=[]
        for d in self.DIRS:
            nb=(s[0]+d[0],s[1]+d[1])
            if 0<=nb[0]<NX and 0<=nb[1]<NY and not self.grid[nb[0],nb[1],0]:
                out.append((nb, math.sqrt(d[0]**2+d[1]**2)))
        return out

    def initialize(self, sv2d, gv2d):
        self.start=sv2d; self.goal=gv2d
        self._g={}; self._rhs={}; self._U=[]; self._km=0
        self._rhs[gv2d]=0.0
        heapq.heappush(self._U,(self._key(gv2d),gv2d))

    def _update_vertex(self,u):
        if u!=self.goal:
            self._rhs[u]=min((self._v(s)+c for s,c in self._succ(u)),default=self.INF)
        self._U=[(k,v) for k,v in self._U if v!=u]; heapq.heapify(self._U)
        if self._v(u)!=self._r(u): heapq.heappush(self._U,(self._key(u),u))

    def compute(self, limit=50000):
        i=0
        while self._U and i<limit:
            i+=1
            k_old,u=heapq.heappop(self._U)
            if k_old<self._key(u): heapq.heappush(self._U,(self._key(u),u)); continue
            if self._v(u)>self._r(u):
                self._g[u]=self._r(u)
                for s,_ in self._succ(u): self._update_vertex(s)
            else:
                self._g[u]=self.INF
                for s,_ in self._succ(u)+[(u,0)]: self._update_vertex(s)
            if u==self.start: break

    def extract_path(self):
        path=[]; cur=self.start; vis=set(); vis.add(cur)
        for _ in range(2000):
            path.append(cur)
            if cur==self.goal: break
            succs=self._succ(cur)
            if not succs: break
            cur=min(succs,key=lambda sc:self._v(sc[0])+sc[1])[0]
            if cur in vis: break
            vis.add(cur)
        return path

    def update_for_dynamic(self, new_grid, new_start, affected_cells):
        self.grid=new_grid; old_start=self.start
        self._km+=self._h(old_start, new_start); self.start=new_start
        for cell in affected_cells:
            self._update_vertex(cell)

# ── JPS (2D) ─────────────────────────────────────
def jps_2d(grid, start2d, goal2d):
    sx,sy=start2d; gx,gy=goal2d

    def free(x,y): return 0<=x<NX and 0<=y<NY and not grid[x,y,0]

    def neighbors(node, par):
        x,y=node
        if par is None:
            return [(x+dx,y+dy) for dx in(-1,0,1) for dy in(-1,0,1)
                    if (dx or dy) and free(x+dx,y+dy)]
        px,py=par; dx=max(-1,min(1,x-px)); dy=max(-1,min(1,y-py))
        result=[]
        if dx and dy:
            if free(x+dx,y): result.append((x+dx,y))
            if free(x,y+dy): result.append((x,y+dy))
            if free(x+dx,y+dy): result.append((x+dx,y+dy))
        elif dx:
            if free(x+dx,y): result.append((x+dx,y))
            if not free(x,y+1) and free(x+dx,y+1): result.append((x+dx,y+1))
            if not free(x,y-1) and free(x+dx,y-1): result.append((x+dx,y-1))
        else:
            if free(x,y+dy): result.append((x,y+dy))
            if not free(x+1,y) and free(x+1,y+dy): result.append((x+1,y+dy))
            if not free(x-1,y) and free(x-1,y+dy): result.append((x-1,y+dy))
        return result

    def jump(x,y,dx,dy,depth=0):
        if depth>100 or not free(x,y): return None
        if x==gx and y==gy: return (x,y)
        if dx and dy:
            if (free(x-dx,y+dy) and not free(x-dx,y)) or \
               (free(x+dx,y-dy) and not free(x,y-dy)): return (x,y)
            if jump(x+dx,y,dx,0,depth+1) or jump(x,y+dy,0,dy,depth+1): return (x,y)
        elif dx:
            if (free(x+dx,y+1) and not free(x,y+1)) or \
               (free(x+dx,y-1) and not free(x,y-1)): return (x,y)
        else:
            if (free(x+1,y+dy) and not free(x+1,y)) or \
               (free(x-1,y+dy) and not free(x-1,y)): return (x,y)
        return jump(x+dx,y+dy,dx,dy,depth+1)

    OPEN=[]; heapq.heappush(OPEN,(0.0,(sx,sy),None))
    g_vals={(sx,sy):0.0}; parent={}; visited=0

    while OPEN and visited<5000:
        f,node,par=heapq.heappop(OPEN); visited+=1
        if node==(gx,gy):
            path=[]; c=node
            while c in parent: path.append(c); c=parent[c]
            path.append(c); path.reverse()
            return path
        for nb in neighbors(node,par):
            nx2,ny2=nb
            dx=max(-1,min(1,nx2-node[0])); dy=max(-1,min(1,ny2-node[1]))
            jp=jump(nx2,ny2,dx,dy)
            if jp is None: continue
            jx,jy=jp; d=math.sqrt((jx-node[0])**2+(jy-node[1])**2)
            ng=g_vals[node]+d
            if (jx,jy) not in g_vals or ng<g_vals[(jx,jy)]:
                g_vals[(jx,jy)]=ng; parent[(jx,jy)]=node
                heapq.heappush(OPEN,(ng+math.dist((jx,jy),(gx,gy)),(jx,jy),node))
    return []

# ── Simulation State ──────────────────────────────
drone_pos = START[:2].copy()
drone_speed = 0.25   # units per frame

# Blocker obstacle: starts off-path and moves across it
blocker_start = np.array([-8.0, 5.0])
blocker_goal  = np.array([ 8.0, -5.0])
blocker_pos   = blocker_start.copy()
blocker_vel   = (blocker_goal - blocker_start)
blocker_vel   = blocker_vel / np.linalg.norm(blocker_vel) * 0.18
BLOCKER_R = 1.2

# Initial plan
sv2d = w2v(START)[:2]; gv2d = w2v(GOAL)[:2]
init_grid = GRID.copy()
dstar = DStarLite2D(init_grid)
dstar.initialize(sv2d, gv2d)
dstar.compute()
current_path_vox = dstar.extract_path()
current_path_world = [v2w((v[0],v[1],0))[:2] for v in current_path_vox]
wp_idx = 0

replan_events = []   # (frame, type)  type: 'dstar' or 'jps'
frame_count = [0]
replan_count = {'dstar':0, 'jps':0}
path_color = ['#58a6ff']  # mutable so closure can update

# ── Figure ────────────────────────────────────────
fig, (ax_main, ax_log) = plt.subplots(1, 2, figsize=(15, 8),
                                       gridspec_kw={'width_ratios':[3,1]})
fig.patch.set_facecolor('#0d1117')
fig.suptitle("DEMO 2 — DYNAMIC REPLANNING\nD* Lite + Jump Point Search (JPS)",
             fontsize=14, fontweight='bold', color='white')

ax_main.set_facecolor('#161b22')
ax_main.set_xlim(-W, W); ax_main.set_ylim(-W, W)
ax_main.set_aspect('equal'); ax_main.grid(True, alpha=0.15, color='white')
ax_main.tick_params(colors='#888')
ax_main.set_xlabel('X (m)', color='#888'); ax_main.set_ylabel('Y (m)', color='#888')

ax_log.set_facecolor('#161b22')
ax_log.tick_params(colors='#888')
ax_log.set_xlabel('Frame', color='#888'); ax_log.set_ylabel('Replan Count', color='#888')
ax_log.set_title('Replan Events', color='white')

for spine in ax_main.spines.values(): spine.set_edgecolor('#30363d')
for spine in ax_log.spines.values(): spine.set_edgecolor('#30363d')

# Static obstacles
for (cx,cy,r) in OBSTACLES:
    ax_main.add_patch(plt.Circle((cx,cy), r+INFL, color='#1f6feb', alpha=0.2))
    ax_main.add_patch(plt.Circle((cx,cy), r, color='#388bfd', alpha=0.7))

ax_main.plot(*START[:2],'*', color='#3fb950', ms=16, zorder=10)
ax_main.plot(*GOAL[:2], '*', color='#f85149', ms=16, zorder=10)
ax_main.annotate('START', START[:2], xytext=(6,4), textcoords='offset points', color='#3fb950', fontsize=9)
ax_main.annotate('GOAL',  GOAL[:2],  xytext=(6,4), textcoords='offset points', color='#f85149', fontsize=9)

path_line, = ax_main.plot([], [], '-', color='#58a6ff', lw=2, zorder=5, label='D* Lite path')
drone_dot,  = ax_main.plot([], [], 'o', color='#ffa657', ms=12, zorder=9, label='Drone')
blocker_circ = plt.Circle(blocker_start, BLOCKER_R, color='#f85149', alpha=0.8, zorder=8)
ax_main.add_patch(blocker_circ)
blocker_label = ax_main.text(blocker_start[0], blocker_start[1]+BLOCKER_R+0.3,
                              '🔴 BLOCKER', ha='center', color='#f85149', fontsize=9, fontweight='bold')
status_text = ax_main.text(-W+0.3, W-0.5, '', color='white', fontsize=10,
                            fontweight='bold', va='top',
                            bbox=dict(facecolor='#21262d', edgecolor='#30363d', alpha=0.9, pad=5))
legend_patches = [
    mpatches.Patch(color='#58a6ff', label='D* Lite path'),
    mpatches.Patch(color='#f85149', label='JPS emergency path'),
    mpatches.Patch(color='#e3b341', label='JPS active'),
]
ax_main.legend(handles=legend_patches, loc='lower right', facecolor='#21262d',
               edgecolor='#30363d', labelcolor='white', fontsize=8)

dstar_counts = [0]; jps_counts = [0]; frames_list = [0]
log_dstar, = ax_log.plot([], [], color='#58a6ff', lw=2, label='D* Lite replans')
log_jps,   = ax_log.plot([], [], color='#f85149', lw=2, label='JPS replans')
ax_log.legend(facecolor='#21262d', edgecolor='#30363d', labelcolor='white', fontsize=8)
ax_log.set_xlim(0, 200); ax_log.set_ylim(0, 10)

def update(frame):
    global drone_pos, blocker_pos, wp_idx, current_path_world, current_path_vox
    frame_count[0] = frame

    # Move blocker
    blocker_pos = blocker_pos + blocker_vel
    if np.linalg.norm(blocker_pos - blocker_goal) < 0.5:
        blocker_pos[:] = blocker_start.copy()

    blocker_circ.center = blocker_pos
    blocker_label.set_position((blocker_pos[0], blocker_pos[1]+BLOCKER_R+0.3))

    # Check if blocker blocks current path
    new_grid = get_grid_with_dynamic(blocker_pos, BLOCKER_R)
    sv2d_cur = (max(0,min(NX-1,int((drone_pos[0]+W)/VRES))),
                max(0,min(NY-1,int((drone_pos[1]+W)/VRES))))

    # Replan every 8 frames
    replan_type = ''
    if frame % 8 == 0:
        dstar.update_for_dynamic(new_grid, sv2d_cur,
            [(max(0,min(NX-1,int((blocker_pos[0]+W)/VRES))),
              max(0,min(NY-1,int((blocker_pos[1]+W)/VRES))))])
        dstar.compute(limit=20000)
        new_vox_path = dstar.extract_path()
        if len(new_vox_path) > 2:
            current_path_vox = new_vox_path
            current_path_world = [v2w((v[0],v[1],0))[:2] for v in current_path_vox]
            replan_count['dstar'] += 1; replan_type = 'D* Lite'
            path_color[0] = '#58a6ff'
        else:
            # JPS emergency
            jps_path2d = jps_2d(new_grid, sv2d_cur, gv2d)
            if jps_path2d:
                current_path_vox = jps_path2d
                current_path_world = [v2w((v[0],v[1],0))[:2] for v in jps_path2d]
                replan_count['jps'] += 1; replan_type = 'JPS'
                path_color[0] = '#f85149'

    # Advance drone toward next waypoint
    if wp_idx < len(current_path_world):
        target = np.array(current_path_world[min(wp_idx, len(current_path_world)-1)])
        diff = target - drone_pos
        dist = np.linalg.norm(diff)
        if dist < drone_speed:
            wp_idx = min(wp_idx+1, len(current_path_world)-1)
        else:
            drone_pos = drone_pos + diff/dist * drone_speed

    # Draw path
    if current_path_world:
        xs=[p[0] for p in current_path_world]; ys=[p[1] for p in current_path_world]
        path_line.set_data(xs, ys); path_line.set_color(path_color[0])

    drone_dot.set_data([drone_pos[0]], [drone_pos[1]])

    # Status
    dist_to_goal = np.linalg.norm(drone_pos - GOAL[:2])
    algo = 'JPS ⚡' if path_color[0]=='#f85149' else 'D* Lite 🔄'
    status = (f"  Active: {algo}\n"
              f"  D* Lite replans: {replan_count['dstar']}\n"
              f"  JPS replans:     {replan_count['jps']}\n"
              f"  Dist to goal: {dist_to_goal:.1f}m")
    if replan_type: status = f"  ⚡ {replan_type} REPLANNED!\n" + status
    status_text.set_text(status)
    status_text.set_color('#f85149' if replan_type=='JPS' else '#3fb950' if replan_type else 'white')

    # Log panel
    dstar_counts.append(replan_count['dstar'])
    jps_counts.append(replan_count['jps'])
    frames_list.append(frame)
    log_dstar.set_data(frames_list, dstar_counts)
    log_jps.set_data(frames_list, jps_counts)
    ax_log.set_xlim(0, max(frame+1, 10))
    ax_log.set_ylim(0, max(max(dstar_counts)+2, max(jps_counts)+2, 5))

    # Goal check
    if dist_to_goal < 0.8:
        status_text.set_text("  ✅ GOAL REACHED!\n  Mission Complete!")
        status_text.set_color('#3fb950')

    return path_line, drone_dot, blocker_circ, blocker_label, status_text, log_dstar, log_jps

anim = FuncAnimation(fig, update, frames=200, interval=80, blit=False)
print("\n✅ Animation ready! Close the window to exit.")
plt.tight_layout()
plt.show()
