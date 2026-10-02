"""Offline point-cloud clearance analysis, inspired by flycar SDFMap.
Does not command a robot. All radii below are configurable analysis assumptions.
"""
from dataclasses import dataclass, asdict
from pathlib import Path
import numpy as np
from scipy.ndimage import distance_transform_edt, label as connected_components
from scipy.spatial import cKDTree


def read_pcd(path):
    with Path(path).open('rb') as f:
        h={}
        while True:
            line=f.readline()
            if not line: raise ValueError('Missing DATA header')
            words=line.decode('ascii').strip().split()
            if not words or words[0].startswith('#'):continue
            h[words[0]]=words[1:]
            if words[0]=='DATA':break
        types={('F',4):'<f4',('F',8):'<f8',('U',1):'u1',('U',2):'<u2',('U',4):'<u4',('I',1):'i1',('I',2):'<i2',('I',4):'<i4'}
        counts=list(map(int,h.get('COUNT',['1']*len(h['FIELDS']))))
        fields=[]
        for k,s,t,c in zip(h['FIELDS'],h['SIZE'],h['TYPE'],counts):
            dt=types[(t,int(s))];fields.append((k,dt) if c==1 else (k,dt,(c,)))
        dtype=np.dtype(fields);n=int(h['POINTS'][0])
        if h['DATA']==['binary']:
            a=np.fromfile(f,dtype=dtype,count=n)
            if len(a)!=n:raise ValueError('Truncated binary PCD')
        elif h['DATA']==['ascii']:
            a=np.loadtxt(f,dtype=dtype,ndmin=1)
            if len(a)!=n:raise ValueError('ASCII PCD count mismatch')
        else:raise ValueError('Compressed PCD must first be decoded by PCL')
    p=np.column_stack([a[k] for k in ['x','y','z']]).astype(np.float32)
    lab=a['label'].astype(np.uint8) if 'label' in a.dtype.names else np.zeros(len(p),np.uint8)
    good=np.isfinite(p).all(1)
    return p[good],lab[good],dict(header_points=n,finite_points=int(good.sum()),fields=h['FIELDS'])


def voxel_centroids(p,size=.05):
    _,inv,count=np.unique(np.floor(p/size).astype(np.int32),axis=0,return_inverse=True,return_counts=True)
    return np.column_stack([np.bincount(inv,weights=p[:,i])/count for i in range(3)]).astype(np.float32)


def build_grid(p,labels,floor_z,res=.05):
    # A horizontal body-height slice, NOT a full traversability map over stairs.
    band=(p[:,2]>floor_z+.18)&(p[:,2]<floor_z+1.65)&(labels!=6)
    obs=p[band];lab=labels[band]
    if not len(obs):raise ValueError('No points in obstacle height band')
    origin=np.floor(p[:,:2].min(0)/res)*res-1.5
    shape=np.ceil((p[:,:2].max(0)+1.5-origin)/res).astype(int)+1
    if shape.prod()>2_000_000:raise ValueError('Map extents too large; explicit ROI required')
    ij=np.floor((obs[:,:2]-origin)/res).astype(int)
    occ=np.zeros((shape[1],shape[0]),bool);occ[ij[:,1],ij[:,0]]=True
    # Unknown is not free. Evidence mask records XY columns with actual point returns.
    allij=np.floor((p[:,:2]-origin)/res).astype(int);observed=np.zeros_like(occ);observed[allij[:,1],allij[:,0]]=True
    people=np.zeros_like(occ);people[ij[lab==4,1],ij[lab==4,0]]=True
    distance=np.maximum(0,distance_transform_edt(~occ)*res-res*np.sqrt(2)/2)
    components,n=connected_components(occ,np.ones((3,3),int))
    return dict(occupancy=occ,observed=observed,people=people,distance=distance,origin=origin,res=res,floor=floor_z,obstacle_points=obs,component_count=n)


@dataclass
class Settings:
    robot_radius:float=.35
    minimum_margin:float=.08
    comfort_margin:float=.30
    reaction_time:float=.25
    deceleration:float=1.
    people_extra_margin:float=.45
    hysteresis:float=.02


def radius_policy(width,speed,settings=Settings(),previous=None):
    if speed<0 or not np.isfinite(speed):raise ValueError('Invalid speed')
    if settings.robot_radius<=0 or settings.deceleration<=0:raise ValueError('Invalid footprint/deceleration')
    braking=speed*settings.reaction_time+speed**2/(2*settings.deceleration)
    hard=settings.robot_radius+settings.minimum_margin+braking
    valid=width is not None and np.isfinite(width) and width>0
    # Only discretionary comfort shrinks; body clearance and stopping allowance do not.
    slack=max(0,width/2-.05-hard) if valid else settings.comfort_margin
    radius=hard+min(settings.comfort_margin,slack)
    if previous is not None and hard<=previous<=radius and radius-previous<settings.hysteresis:radius=previous
    blocked=bool(valid and width<2*hard)
    return dict(radius=float(radius),hard_radius=float(hard),braking=float(braking),width_valid=bool(valid),too_narrow=blocked,parameters=asdict(settings))


def corridor_width(g,position,yaw):
    # flycar: three headings, six 25 cm forward samples, 5 cm lateral steps.
    # Here a width is valid ONLY if both raw-obstacle sides are observed.
    p=np.asarray(position,float);xy=g['obstacle_points'][:,:2];tree=cKDTree(xy)
    rays=[];widths=[]
    for offset in [-.15,0,.15]:
        angle=yaw+offset;forward=np.array([np.cos(angle),np.sin(angle)]);side=np.array([-forward[1],forward[0]])
        for t in np.arange(.25,1.51,.25):
            center=p+t*forward
            if tree.query(center)[0]<.10:continue
            hits=[]
            for sign in [-1,1]:
                rs=np.arange(.10,2.001,.05);checks=center+sign*rs[:,None]*side
                dist,_=tree.query(checks);ix=np.flatnonzero(dist<=.08)
                hits.append((rs[ix[0]],checks[ix[0]]) if len(ix) else None)
            if all(h is not None for h in hits):
                width=hits[0][0]+hits[1][0];widths.append(width);rays.append(dict(width=float(width),ends=[hits[0][1].tolist(),hits[1][1].tolist()]))
    # No caching / fabricated default width for missing observations.
    width=float(min(widths)) if len(widths)>=3 else None
    return width,rays


def analyze(g,position,yaw=0,speed=.3,settings=Settings(),previous=None):
    w,rays=corridor_width(g,position,yaw);result=radius_policy(w,speed,settings,previous)
    r=result['radius'];d=g['distance'];occ=g['occupancy']
    inflated=d<=r
    person_d=distance_transform_edt(~g['people'])*g['res'] if g['people'].any() else np.full_like(d,np.inf)
    inflated|=person_d<=r+settings.people_extra_margin
    fixed=d<=settings.robot_radius+settings.minimum_margin+settings.comfort_margin
    cost=np.where(inflated,100,0).astype(np.uint8)
    # Smooth soft cost beyond hard exclusion (derived, not measured points).
    extra=np.maximum(0,d-r);cost=np.maximum(cost,(65*np.exp(-extra/.45)).astype(np.uint8));cost[occ]=100
    clearance=float(cKDTree(g['obstacle_points'][:,:2]).query(position)[0])
    result.update(position=list(map(float,position)),yaw=float(yaw),speed=float(speed),corridor_width=w,valid_sections=len(rays),rays=rays,clearance=clearance,pose_conflict=bool(clearance<result['hard_radius']),inflated_cells=int(inflated.sum()),raw_cells=int(occ.sum()),fixed_cells=int(fixed.sum()))
    return result,dict(inflated=inflated,fixed=fixed,cost=cost)
