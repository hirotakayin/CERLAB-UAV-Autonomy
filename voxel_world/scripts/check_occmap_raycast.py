#!/usr/bin/env python3
"""Line-by-line Python port of map_manager's occMap::raycastUpdate, for checking the fast map against it.

Usage (from this directory, ROS sourced): NSCANS=117 ALIGN=0 NOBREAK=0 python3 check_occmap_raycast.py
Integrates the first NSCANS scans of the bag below (drone parked) and prints the pit % the monitor would log.
  ALIGN=1   run the ray walk on the map's own grid (map_manager divides by the resolution without removing the map
            origin, so with ground_height -10.5 at 0.2 m its walk is half a voxel off the map in z)
  NOBREAK=1 drop the early stop (a ray stops at the first voxel an earlier ray of the same scan crossed)
  NOSKIP=1  drop the skip of rays ending in a voxel another ray already ended in
Found 2026-11-23 on excavation_long: as is 7.63 % after 117 parked scans (the monitor logged 7.39 %); aligned and
without the early stop it matches fast.OccupancyMap (about 3x more known space).
"""
import sys, math, numpy as np
sys.argv=['x']; sys.path.insert(0,'.')
import replay_flight as rf
from voxel_world.fast import OccupancyMap, PitScore
import rosbag
O=__import__('os').environ.get('BAG', __import__('os').path.expanduser('~/mission_runs/voxel_replay/long_pitch23.5/run.bag'))
with rosbag.Bag(O) as b:
    od=None; sc=[]
    for topic,msg,t in b.read_messages(topics=['/CERLAB/quadcopter/odom','/cloud_registered_body']):
        if topic=='/cloud_registered_body': sc.append((msg.header.stamp.to_sec(), rf.read_cloud(msg)))
        elif od is None: od=msg
        if od is not None and len(sc)>=int(__import__('os').environ.get('NSCANS','3')): break
p,q=od.pose.pose.position,od.pose.pose.orientation
odom=(np.array([0.0,1e9]),np.array([[p.x,p.y,p.z]]*2),np.array([[q.x,q.y,q.z,q.w]]*2))
pit=rf.PITS['excavation_long']
def intbound(s,ds):
    if ds<0: return intbound(-s,-ds)
    s=math.fmod(math.fmod(s,1)+1,1); return (1-s)/ds if ds!=0 else math.inf
def exact(m, pos, pts):
    res=m.res; org=m.origin; shp=m.shape
    def addr(pt):
        i=np.floor((pt-org)/res).astype(int); return int(np.ravel_multi_index(np.clip(i,0,shp-1),shp))
    cnt={}; hitc={}
    fe=set(); ft=set()
    for pt in pts:
        if not np.all(np.isfinite(pt)) or np.linalg.norm(pt-pos)<0.5: continue
        adj=False
        lo,hi=org,org+m.size
        if np.any(pt<lo) or np.any(pt>=hi):
            # occMap::adjustPointInMap: step back along the ray until inside
            d=pt-pos; tt=min([((hi[a]-1e-6 if d[a]>0 else lo[a]+1e-6)-pos[a])/d[a] for a in range(3) if d[a]!=0]); pt=pos+d*min(tt,1); adj=True
        L=np.linalg.norm(pt-pos)
        if L>20: pt=pos+(pt-pos)*20/L; adj=True
        v=addr(pt); cnt[v]=cnt.get(v,0)+1
        if not adj: hitc[v]=hitc.get(v,0)+1
        if v in fe and not NOSKIP: continue
        fe.add(v)
        s=(pt-org)/res if ALIGN else pt/res; e=(pos-org)/res if ALIGN else pos/res
        x,y,z=[int(math.floor(c)) for c in s]; ex,ey,ez=[int(math.floor(c)) for c in e]
        dx,dy,dz=ex-x,ey-y,ez-z
        sx,sy,sz=[(c>0)-(c<0) for c in (dx,dy,dz)]
        tmx,tmy,tmz=intbound(s[0],dx),intbound(s[1],dy),intbound(s[2],dz)
        tdx=sx/dx if dx else math.inf; tdy=sy/dy if dy else math.inf; tdz=sz/dz if dz else math.inf
        if sx==0 and sy==0 and sz==0: continue
        n=0
        while True:
            rp=np.array([x,y,z],float)
            if x==ex and y==ey and z==ez: break
            if tmx<tmy:
                if tmx<tmz: x+=sx; tmx+=tdx
                else: z+=sz; tmz+=tdz
            else:
                if tmy<tmz: y+=sy; tmy+=tdy
                else: z+=sz; tmz+=tdz
            ap=org+(rp+0.5)*res if ALIGN else (rp+0.5)*res; w=addr(ap); cnt[w]=cnt.get(w,0)+1
            n+=1
            if w in ft and not NOBREAK: break
            ft.add(w)
            if n>5000: print('runaway'); break
    flat=m.occ.reshape(-1)
    for v,c in cnt.items():
        h=hitc.get(v,0); miss=c-h
        up=m.hit_log if (h>=miss and h!=0) else m.miss_log
        cur=flat[v]
        if up>=0 and cur>=m.max_log: continue
        if up<=0 and cur==m.min_log: continue
        if up<=0 and cur<m.min_log: flat[v]=m.min_log; continue
        flat[v]=min(max(cur+up,m.min_log),m.max_log)
import os
ALIGN=os.environ.get('ALIGN')=='1'; NOBREAK=os.environ.get('NOBREAK')=='1'; NOSKIP=os.environ.get('NOSKIP')=='1'
for name in ['exact']:
    m=OccupancyMap(pit['origin'],pit['size'],0.2); s=PitScore(m,*pit['region'],floor_z=-10)
    for st,pts in sc:
        o,R=rf.sensor_pose(odom,st,23.5); w=pts@R.T+o; w[np.linalg.norm(pts,axis=1)<1e-6]=np.nan
        (exact(m,o,w) if name=='exact' else m.integrate(o,w))
    print(name, len(sc),'scans pit %.2f known %d'%(s()['pit_pct'], m.known().sum()))
