#!/usr/bin/env python3
"""Density-adaptive, non-overlapping occupied octree leaves from measured points."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree

def build(xyz):
    sizes=np.array([0.05,0.10,0.20,0.40])
    distances,_=cKDTree(xyz).query(xyz,k=9,workers=4)
    # For a sampled surface d8 estimates local spacing; no fabricated points.
    target=np.clip(0.65*distances[:,-1],sizes[0],sizes[-1])
    desired=sizes[np.searchsorted(sizes,target)]
    remaining=np.ones(len(xyz),dtype=bool)
    centers=[];widths=[];supports=[]
    for size in sizes[::-1]:
        ids=np.flatnonzero(remaining)
        if not len(ids):continue
        keys=np.floor(xyz[ids]/size+1e-7).astype(np.int32)
        unique,inv=np.unique(keys,axis=0,return_inverse=True)
        wanted=np.bincount(inv,weights=(desired[ids]>=size),minlength=len(unique))>0
        count=np.bincount(inv,minlength=len(unique))
        centers.append((unique[wanted]+0.5)*size)
        widths.append(np.full(wanted.sum(),size))
        supports.append(count[wanted])
        remaining[ids[wanted[inv]]]=False
    return np.concatenate(centers),np.concatenate(widths),np.concatenate(supports),distances[:,1],distances[:,-1]

def verify(xyz,centers,widths,supports):
    assert np.isfinite(centers).all() and supports.sum()==len(xyz)
    owner=np.zeros(len(xyz),np.int8)
    coarse=[]
    for size in sorted(np.unique(widths),reverse=True):
        c=centers[widths==size];keys=np.floor(c/size+1e-7).astype(np.int32)
        keyset=set(map(tuple,keys))
        for large,occupied in coarse:
            assert not any(tuple(k) in occupied for k in np.floor(c/large+1e-7).astype(np.int32))
        owner += np.fromiter((tuple(k) in keyset for k in np.floor(xyz/size+1e-7).astype(np.int32)),np.int8,len(xyz))
        coarse.append((size,keyset))
    assert np.all(owner==1),'Every measured point must have exactly one leaf'

if __name__=='__main__':
    import argparse
    ap=argparse.ArgumentParser();ap.add_argument('pcd');ap.add_argument('out');args=ap.parse_args()
    out=Path(args.out);out.mkdir(parents=True,exist_ok=True)
    raw=Path(args.pcd).read_bytes();header,payload=raw.split(b'DATA binary\n',1)
    assert b'FIELDS x y z\n' in header and b'SIZE 4 4 4\n' in header
    xyz=np.frombuffer(payload,dtype='<f4').reshape(-1,3);xyz=xyz[np.isfinite(xyz).all(axis=1)]
    c,s,n,nearest,d8=build(xyz);verify(xyz,c,s,n)
    # Shared height range and palette across every resolution.
    zlo,zhi=np.percentile(xyz[:,2],[1,99]);t=np.clip((c[:,2]-zlo)/(zhi-zlo),0,1)
    anchors=np.array([[35,65,230],[0,190,245],[30,210,120],[245,225,30],[240,65,30]])
    rgb=np.column_stack([np.interp(t,np.linspace(0,1,5),anchors[:,j]) for j in range(3)]).astype(np.uint8)
    np.savez_compressed(out/'adaptive_voxels.npz',centers=c.astype('f4'),sizes=s.astype('f4'),support=n,rgb=rgb,height_range=[zlo,zhi],frame='world')
    report=dict(input_points=len(xyz),occupied_leaves=len(c),size_counts={str(v):int((s==v).sum()) for v in np.unique(s)},nearest_spacing_percentiles_m=np.percentile(nearest,[10,50,90,99]).tolist(),height_range_m=[zlo,zhi],color='blue low -> cyan -> green -> yellow -> red high; clipped at 1st/99th percentile',validation='all input points covered exactly once; octree leaves do not overlap',method='edge = next power-of-two level >= 0.65 * eighth-neighbor distance, clamped 0.05..0.40 m; coarser leaves absorb descendants',limitation='Geometric occupied surface approximation. No ray-traced free space, no filling unobserved gaps. Coarsening may narrow gaps by up to leaf scale; not validated for navigation.')
    (out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
