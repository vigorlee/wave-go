#!/usr/bin/env python3
"""Re-accumulate measured registered scans with real first-observation times.

Run with the ROS1 Python environment. No fabricated points or online semantic
claims: semantic labels are associated from the existing FINAL offline map.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import rosbag
from scipy.spatial import cKDTree

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--bag',type=Path,required=True)
    parser.add_argument('--voxel',type=float,default=.04)
    parser.add_argument('--output',type=Path,default=ROOT/'outputs/reference_demo')
    args=parser.parse_args()
    out=args.output
    out.mkdir(parents=True,exist_ok=True)
    seen=set()
    point_chunks=[]
    time_chunks=[]
    scan_stats=[]
    input_count=0
    roi_count=0
    with rosbag.Bag(str(args.bag),'r') as bag:
        for seq,(_,msg,stamp) in enumerate(bag.read_messages(topics=['/wavego/slam/registered_points'])):
            assert msg.header.frame_id=='map', msg.header.frame_id
            offsets={f.name:f.offset for f in msg.fields}
            dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,
                            'offsets':[offsets[x] for x in 'xyz'],'itemsize':msg.point_step})
            values=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,
                              strides=(msg.row_step,msg.point_step)).reshape(-1)
            p=np.column_stack([values[x] for x in 'xyz'])
            input_count+=len(p)
            keep=np.isfinite(p).all(axis=1)
            keep&=(p[:,0]>-3)&(p[:,0]<21)&(p[:,1]>-8)&(p[:,1]<13)&(p[:,2]>-1.5)&(p[:,2]<8.2)
            p=p[keep]
            roi_count+=len(p)
            q=np.floor(p/args.voxel).astype(np.int64)+(1<<20)
            keys=(q[:,0]<<42)|(q[:,1]<<21)|q[:,2]
            keys,ids=np.unique(keys,return_index=True)
            new=np.fromiter((int(k) not in seen for k in keys),dtype=bool,count=len(keys))
            seen.update(map(int,keys[new]))
            p=p[ids[new]]
            t=stamp.to_sec()
            point_chunks.append(p)
            time_chunks.append(np.full(len(p),t,np.float64))
            scan_stats.append([t,len(p),len(seen)])
            if seq%250==0:print(f'SCAN {seq}: unique={len(seen):,}',flush=True)
    points=np.concatenate(point_chunks).astype(np.float32)
    first_seen=np.concatenate(time_chunks)
    assert np.all(np.diff(first_seen)>=0)
    old=np.load(ROOT/'outputs/semantic_cloud.npz')
    distance,nearest=cKDTree(old['points']).query(points,workers=4)
    labels=np.where(distance<=.12,old['labels'][nearest],0).astype(np.uint8)
    evidence=np.where(distance<=.12,old['evidence'][nearest],0).astype(np.uint8)
    source_index=np.where(distance<=.12,nearest,-1).astype(np.int32)
    np.savez_compressed(out/'mapping_replay.npz',points=points,first_seen=first_seen,
                        labels=labels,evidence=evidence,source_index=source_index,
                        scan_stats=np.array(scan_stats),voxel=np.array(args.voxel))
    header=('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
            f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n')
    (out/'dense_measured_map.pcd').write_bytes(header.encode()+points.astype('<f4').tobytes())
    report=dict(source_bag=str(args.bag.resolve()),topic='/wavego/slam/registered_points',
                source_frame='map',display_frame='lio_map (documented same LIO coordinates)',
                input_scans=len(scan_stats),input_points=input_count,roi_returns=roi_count,
                unique_measured_points=len(points),voxel_m=args.voxel,
                previous_display_roi_points=int(old['roi'].sum()),
                first_stamp=float(first_seen[0]),last_stamp=float(scan_stats[-1][0]),
                duration_seconds=float(scan_stats[-1][0]-scan_stats[0][0]),
                labelled_points=int((labels>0).sum()),
                semantic_source='nearest association from final offline semantic map; not online predictions',
                semantic_sha256=hashlib.sha256((ROOT/'outputs/semantic_cloud.npz').read_bytes()).hexdigest(),
                coordinate_changes='none; first measured return retained in each occupied voxel',
                roi=[[-3,-8,-1.5],[21,13,8.2]],
                limitations=['not the building from the reference video',
                             'recorded LIO accumulation, not a new navigation run',
                             'semantic appearance time is visibility replay, not inference timestamp'])
    (out/'replay_report.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':main()
