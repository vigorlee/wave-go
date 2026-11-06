"""Sensor-only occupancy and frontier selection. No scene or route inputs."""
import math
import numpy as np
from scipy import ndimage as ndi


class FrontierGrid:
    def __init__(self, size=600, resolution=.10):
        self.resolution = resolution
        self.origin = -size * resolution / 2
        self.odds = np.zeros((size, size), np.float32)
        self.seen = np.zeros((size, size), bool)

    def cells(self, xy):
        return np.floor((np.asarray(xy)-self.origin)/self.resolution).astype(int)

    def world(self, cells):
        return self.origin+(np.asarray(cells)+.5)*self.resolution

    def integrate(self, xyz, sensor_xyz, floor_z):
        """Clear only measured ray segments, stopping at actual returns."""
        a = self.cells(np.asarray(sensor_xyz)[:2])
        xyz = np.asarray(xyz)
        # Near-horizontal beams support the planar collision map. Full 3D
        # geometry is kept separately; high beams never clear ground space.
        keep = (xyz[:,2] > floor_z-.1) & (xyz[:,2] < floor_z+.9)
        p = xyz[keep]
        if not len(p): return
        distances = np.linalg.norm(p[:,:2]-np.asarray(sensor_xyz)[:2],axis=1)
        p = p[(distances>.55) & (distances<12.)]
        if not len(p): return
        hit = self.cells(p[:,:2])
        occupied = p[:,2] > floor_z+.10
        # Deduplicate endpoints before ray casting to bound CPU per scan.
        _, ids = np.unique(hit,axis=0,return_index=True)
        hit, occupied = hit[ids],occupied[ids]
        n = self.odds.shape[0]
        free_cells=[]; hit_cells=[]; ground_cells=[]
        for b, obstacle in zip(hit,occupied):
            steps=int(max(abs(b-a)))
            if steps<1:continue
            ray=np.rint(a+np.arange(steps)[:,None]*(b-a)/steps).astype(int)
            valid=(ray[:,0]>=0)&(ray[:,0]<n)&(ray[:,1]>=0)&(ray[:,1]<n)
            free_cells.append(ray[valid])
            if 0<=b[0]<n and 0<=b[1]<n:
                (hit_cells if obstacle else ground_cells).append(b[None,:])
        if free_cells:
            q=np.unique(np.concatenate(free_cells),axis=0)
            # A projected high ray can pass ABOVE a low curb. It must not
            # erase a measured obstacle merely by crossing its XY cell.
            q=q[self.odds[q[:,1],q[:,0]]<=0]
            self.odds[q[:,1],q[:,0]]-=.35;self.seen[q[:,1],q[:,0]]=True
        if ground_cells:
            q=np.unique(np.concatenate(ground_cells),axis=0)
            self.odds[q[:,1],q[:,0]]-=.35;self.seen[q[:,1],q[:,0]]=True
        if hit_cells:
            q=np.unique(np.concatenate(hit_cells),axis=0)
            self.odds[q[:,1],q[:,0]]+=1.2;self.seen[q[:,1],q[:,0]]=True
        np.clip(self.odds,-3,5,out=self.odds)

    def occupancy(self):
        a=np.full(self.odds.shape,-1,np.int8)
        a[self.seen & (self.odds<=0)]=0
        a[self.seen & (self.odds>0)]=100
        return a

    def frontiers(self, position, visited=(), far_origin=None, far_direction=None):
        def progress(xy):
            delta=np.asarray(xy)-far_origin
            return delta@far_direction if far_direction is not None else np.linalg.norm(delta,axis=-1)
        a=self.occupancy();free=a==0
        clearance=ndi.distance_transform_edt(a!=100)*self.resolution
        safe=free & (clearance>.38)
        c=self.cells(position)
        if not (0<=c[0]<a.shape[1] and 0<=c[1]<a.shape[0]):return []
        labels,_=ndi.label(safe)
        neighborhood=labels[max(0,c[1]-2):c[1]+3,max(0,c[0]-2):c[0]+3]
        ids=neighborhood[neighborhood>0]
        if not len(ids):return []
        reachable=labels==int(np.bincount(ids).argmax())
        boundary=reachable & ndi.binary_dilation(a==-1)
        groups,count=ndi.label(ndi.binary_dilation(boundary),structure=np.ones((3,3)))
        candidates=[]
        for label in range(1,count+1):
            yx=np.argwhere((groups==label)&boundary)
            if len(yx)<4:continue
            xy=self.world(yx[:,::-1]);distance=np.linalg.norm(xy-np.asarray(position),axis=1)
            ok=(distance>1.0)&(distance<(45. if far_origin is not None else 8.))
            if not ok.any():continue
            xy=xy[ok]
            # One explored portion must not blacklist an entire connected
            # frontier, which can surround a large newly observed room.
            for v in visited:
                xy=xy[np.linalg.norm(xy-v,axis=1)>.9]
            if not len(xy):continue
            target=xy[np.argmax(progress(xy))] if far_origin is not None else xy[np.argmin(np.linalg.norm(xy-xy.mean(axis=0),axis=1))]
            # Back off into already observed free space, never target unknown.
            direction=(target-position)/np.linalg.norm(target-position)
            backed=target-.35*direction;bc=self.cells(backed)
            if reachable[bc[1],bc[0]]:target=backed
            if any(np.linalg.norm(target-v)<.8 for v in visited):continue
            d=float(np.linalg.norm(target-position))
            score=float(progress(target)) if far_origin is not None else min(len(yx),100)/(d+1)
            candidates.append(dict(x=float(target[0]),y=float(target[1]),
                                   yaw=math.atan2(direction[1],direction[0]),score=score,cells=len(yx)))
        if far_origin is not None:
            # The task is to reach the farthest observed place, including a
            # fully scanned corridor end that no longer borders unknown space.
            current_radius=float(progress(position))
            candidates=[c for c in candidates if c['score']>current_radius+.75]
            yx=np.argwhere(reachable)
            xy=self.world(yx[:,::-1]);radius=progress(xy)
            distance=np.linalg.norm(xy-position,axis=1)
            eligible=(radius>current_radius+.75)&(distance>1.)&(distance<45.)
            for v in visited:eligible&=np.linalg.norm(xy-v,axis=1)>.9
            if eligible.any():
                choices=xy[eligible];target=choices[np.argmax(radius[eligible])]
                direction=(target-position)/np.linalg.norm(target-position)
                backed=target-.35*direction;bc=self.cells(backed)
                if reachable[bc[1],bc[0]]:target=backed
                if progress(target)>current_radius+.5:
                    candidates.append(dict(x=float(target[0]),y=float(target[1]),yaw=math.atan2(direction[1],direction[0]),score=float(progress(target)),cells=int(eligible.sum()),source='farthest_observed_free_space'))
        return sorted(candidates,key=lambda x:x['score'],reverse=True)
