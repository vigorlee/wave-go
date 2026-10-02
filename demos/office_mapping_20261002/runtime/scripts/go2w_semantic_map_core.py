"""Bounded semantic voxel fusion and density clustering; no ROS or scene priors."""
from collections import deque
import numpy as np
from scipy.spatial import cKDTree

LABELS=['unknown obstacle','stairs','ramp','cylinder','person','wall','floor','stairs/ramp uncertain']
COLORS=np.array([[110,125,140],[45,170,255],[180,105,250],[255,165,35],[250,65,135],[155,175,195],[65,195,135],[235,205,65]],np.uint8)

def voxel_sample(xyz, size=.12):
    if not len(xyz):return np.empty(0,dtype=int)
    return np.unique(np.floor(xyz/size).astype(np.int32),axis=0,return_index=True)[1]

def dbscan(xyz, eps=.32, min_points=5):
    """KD-tree DBSCAN, queries one neighborhood at a time (bounded memory)."""
    tree=cKDTree(xyz);labels=np.full(len(xyz),-1,dtype=np.int32);seen=np.zeros(len(xyz),bool);cid=0
    for i in range(len(xyz)):
        if seen[i]:continue
        seen[i]=True;neighbors=tree.query_ball_point(xyz[i],eps)
        if len(neighbors)<min_points:continue
        labels[i]=cid;queue=deque(neighbors)
        while queue:
            j=queue.popleft()
            if not seen[j]:
                seen[j]=True;near=tree.query_ball_point(xyz[j],eps)
                if len(near)>=min_points:queue.extend(k for k in near if not seen[k])
            if labels[j]<0:labels[j]=cid
        cid+=1
    return labels

class SemanticVoxels:
    def __init__(self,size=.12,limit=180000):
        self.size=size;self.limit=limit;self.cells={};self.tree=None;self.classes=np.empty(0,np.uint8);self.support=np.empty(0)
        self.people=None;self.people_time=-np.inf;self.last_stamp=-np.inf
    def update(self,xyz,labels,stamp):
        if stamp<=self.last_stamp:return False
        self.last_stamp=stamp
        good=np.isfinite(xyz).all(axis=1)&(labels>0)&(labels<len(LABELS));xyz=xyz[good];labels=labels[good]
        # One vote per voxel per sensor frame, not per densely sampled pixel.
        idx=voxel_sample(xyz,self.size);xyz=xyz[idx];labels=labels[idx]
        self.people=xyz[labels==4];self.people_time=stamp
        for p,label in zip(xyz,labels):
            if label==4:continue  # People never enter the static label archive.
            k=tuple(np.floor(p/self.size).astype(int))
            if k not in self.cells:self.cells[k]=[p,np.zeros(8,np.float32),stamp,0]
            c=self.cells[k];c[1]*=.98;c[1][label]+=1;c[2]=stamp;c[3]+=1
        if len(self.cells)>self.limit:
            for k in sorted(self.cells,key=lambda k:self.cells[k][2])[:len(self.cells)-self.limit]:del self.cells[k]
        stable=[c for c in self.cells.values() if c[3]>=3 and c[1].max()/c[1].sum()>=.6]
        self.tree=cKDTree(np.array([c[0] for c in stable])) if stable else None
        self.classes=np.array([c[1].argmax() for c in stable],np.uint8)
        self.support=np.array([c[1].max()/c[1].sum() for c in stable],np.float32)
        return True
    def assign(self,xyz,stamp,live=True):
        labels=np.zeros(len(xyz),np.uint8);support=np.zeros(len(xyz),np.float32)
        if self.tree is not None and len(xyz):
            dist,idx=self.tree.query(xyz,distance_upper_bound=.18);good=np.isfinite(dist)
            labels[good]=self.classes[idx[good]];support[good]=self.support[idx[good]]
        if live and self.people is not None and len(self.people) and 0<=stamp-self.people_time<=.8 and len(xyz):
            dist,_=cKDTree(self.people).query(xyz,distance_upper_bound=.18);good=np.isfinite(dist)
            labels[good]=4;support[good]=1 # association support, NOT model confidence
        return labels,support

def cluster_obstacles(xyz,labels,support):
    if not len(xyz):return []
    idx=voxel_sample(xyz);xyz=xyz[idx];labels=labels[idx];support=support[idx]
    # Local lower envelope removes horizontal ground without a world-height prior.
    xy=np.floor(xyz[:,:2]/.6).astype(int);_,inv=np.unique(xy,axis=0,return_inverse=True)
    low=np.full(inv.max()+1,np.inf);np.minimum.at(low,inv,xyz[:,2])
    keep=(xyz[:,2]-low[inv]>.18)|(np.isin(labels,[1,2,3,4,7]))
    keep&=labels!=6;xyz=xyz[keep];labels=labels[keep];support=support[keep]
    if len(xyz)>16000:
        idx=np.linspace(0,len(xyz)-1,16000,dtype=int);xyz=xyz[idx];labels=labels[idx];support=support[idx]
    result=[]
    # Separate semantic classes before density clustering, to avoid merging a person with a wall.
    for label in np.unique(labels):
        select=np.where(labels==label)[0];parts=dbscan(xyz[select])
        for cid in np.unique(parts[parts>=0]):
            ix=select[parts==cid]
            if len(ix)<12:continue
            pts=xyz[ix];lo=np.min(pts,axis=0);hi=np.max(pts,axis=0)
            result.append(dict(class_id=int(label),label=LABELS[label],center=((lo+hi)/2).tolist(),size=np.maximum(hi-lo,.1).tolist(),points=len(ix),vote_support=float(support[ix].mean())))
    return sorted(result,key=lambda x:x['points'],reverse=True)[:60]

class Tracks:
    def __init__(self):self.items={};self.next_id=1
    def update(self,objects,stamp):
        available=set(self.items);out=[]
        for obj in objects:
            choices=[(np.linalg.norm(np.array(obj['center'])-self.items[k]['center']),k) for k in available if self.items[k]['class_id']==obj['class_id'] and stamp-self.items[k]['last_seen']<1.5]
            d,k=min(choices,default=(np.inf,None))
            if d>.8:k=self.next_id;self.next_id+=1
            else:available.remove(k)
            hits=self.items[k]['hits']+1 if k in self.items else 1
            item=dict(obj,id=k,last_seen=stamp,hits=hits);self.items[k]=item
            if hits>=3:out.append(item)
        self.items={k:v for k,v in self.items.items() if stamp-v['last_seen']<1.5}
        return out
