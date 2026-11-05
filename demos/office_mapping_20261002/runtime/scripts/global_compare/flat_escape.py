"""Bounded reverse-only separation manoeuvres on the Office entrance apron.

This is a local recovery, not a global planner. Normal pedestrian comfort
distances remain unchanged. Never move through a candidate to reach a goal.
"""
import math
import numpy as np
from scipy.spatial import cKDTree


def choose_reverse_escape(pose, clusters, geoms, observed, desired_turn=0.):
    if not clusters or abs(pose['z']-.4)>.12 or max(abs(pose['roll']),abs(pose['pitch']))>.25:
        return None
    origin=np.array([pose['x'],pose['y']],float)
    if not np.isfinite([*origin,pose['yaw']]).all():return None
    seen=np.asarray(observed,float).reshape(-1,3)
    if len(seen)<100 or not np.isfinite(seen).all():return None
    # Robot returns are excluded, floor/ceiling cannot be horizontal barriers.
    barriers=seen[(seen[:,2]>.16)&(seen[:,2]<1.25)&(np.linalg.norm(seen[:,:2]-origin,axis=1)>.48)]
    tree=cKDTree(barriers[:,:2]) if len(barriers) else None
    best=None
    for angular in (0., -.60, .60):
        dt=.05;xy=origin.copy();yaw=pose['yaw'];path=[xy.copy()]
        for _ in range(10):
            yaw+=angular*dt;xy+=-.30*dt*np.array([math.cos(yaw),math.sin(yaw)]);path.append(xy.copy())
        path=np.array(path);times=np.arange(11)*dt
        # Independently verified flat entrance, bounded away from the riser,
        # wall and surrounding furniture. Do not recover on stairs/slopes.
        if not ((path[:,0]>=-6.35)&(path[:,0]<=-3.30)&(path[:,1]>=1.15)&(path[:,1]<=4.20)).all():continue
        safe=True;gain=float('inf');minimum=float('inf')
        for c in clusters:
            center=np.array(c['xy'],float);velocity=np.array(c['velocity'],float)
            if not np.isfinite([*center,*velocity,c.get('radius',.3)]).all():safe=False;break
            distances=np.linalg.norm(path-center-times[:,None]*velocity,axis=1)
            # Include body, margin and a conservative RL drift allowance.
            if distances.min()<c.get('radius',.3)+.75: safe=False;break
            if distances.min()<distances[0]-.015:safe=False;break
            if distances[0]<2.0:gain=min(gain,float(distances[-1]-distances[0]))
            minimum=min(minimum,float(distances.min()))
        if not safe or not math.isfinite(gain) or gain<.08:continue
        if tree is not None and tree.query(path)[0].min()<.45:continue
        for name,kind,center,size,rotation in geoms:
            center=np.asarray(center);size=np.asarray(size)
            if kind=='box':
                height=float(np.abs(rotation[2])@size)
                if center[2]+height<=.12 or center[2]-height>=1.25:continue
                # Conservative world XY bounds include tilted geometry.
                extent=np.abs(rotation[:2])@size
                delta=np.maximum(np.abs(path-center[:2])-extent,0.)
                if np.linalg.norm(delta,axis=1).min()<.45:safe=False;break
            elif kind=='cylinder':
                if center[2]+size[1]<=.12 or center[2]-size[1]>=1.25:continue
                if np.linalg.norm(path-center[:2],axis=1).min()<size[0]+.45:safe=False;break
        if not safe:continue
        score=gain-.015*abs(angular-float(desired_turn))
        if best is None or score>best['score']:
            best=dict(linear_x=-.30,angular_z=angular,score=score,separation_gain_m=gain,
                      predicted_minimum_distance_m=minimum,horizon_sec=.50,endpoint=path[-1].tolist())
    return best
