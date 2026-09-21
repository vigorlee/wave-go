#!/usr/bin/env python3
"""Run local pretrained models on new RGB, fuse only depth-supported evidence.

Depth Anything is relative-depth visualization only. Metric projection uses
recorded axial depth and the accepted calibrated LIO trajectory, not simulator
odometry. Votes count distinct frames, not the number of projected pixels.
"""
import argparse, hashlib, json, sys, time
from pathlib import Path
import cv2
import numpy as np
from PIL import Image
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp

ROOT=Path(__file__).resolve().parents[1]
SIM=ROOT.parents[2]/'matrix_go2w_office_20260916'
sys.path.insert(0,str(SIM/'scripts'))
from go2w_anything_models import Anything, panel, LABELS, chinese_label
from academic_palette import SEMANTIC

def write_pcd(path,p,labels):
    a=np.empty(len(p),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4'),('label','u1'),('evidence','u1')])
    for i,k in enumerate('xyz'):a[k]=p[:,i]
    c=SEMANTIC[labels].astype(np.uint32)
    a['rgb']=(c[:,0]<<16)|(c[:,1]<<8)|c[:,2];a['label']=labels;a['evidence']=(labels>0).astype(np.uint8)
    h=f'VERSION .7\nFIELDS x y z rgb label evidence\nSIZE 4 4 4 4 1 1\nTYPE F F F U U U\nCOUNT 1 1 1 1 1 1\nWIDTH {len(p)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(p)}\nDATA binary\n'
    path.write_bytes(h.encode()+a.tobytes())

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--dataset',type=Path,default=ROOT/'outputs/fresh_semantic_20260919');args=ap.parse_args()
    data=args.dataset;ref=data/'outputs/reference_demo';input_dir=data/'data/rgbd'
    frames=json.loads((input_dir/'frames.json').read_text());report=json.loads((ref/'replay_report.json').read_text());run=Path(report['source_run'])
    out=data/'outputs/rgbd_models';out.mkdir(parents=True,exist_ok=True);(out/'frames').mkdir(exist_ok=True);(out/'panels').mkdir(exist_ok=True)
    config=json.loads((run/'sensor_config.json').read_text())['robot']['sensors'];dc=config['depth_sensor'];rc=config['camera']
    assert dc['position']==rc['position'] and dc['rotation']==rc['rotation'],'RGB/depth mounts differ; a calibrated extrinsic projection is required'
    trajectory=np.load(run/'structural_calibrated/calibration_trajectory.npz');stamps=trajectory['stamp'];position=trajectory['corrected_position'];rotations=Slerp(stamps,Rotation.from_quat(trajectory['corrected_quaternion']))
    dr=dc['rotation'];sensor_rotation=Rotation.from_euler('xyz',[dr['roll'],-dr['pitch'],-dr['yaw']],degrees=True)
    dp=dc['position'];sensor_translation=np.array([dp['x'],-dp['y'],dp['z']])*.01
    imu_offset=np.array([-.02557,0,.04232])
    replay=np.load(ref/'mapping_replay.npz');points=replay['points'];tree=cKDTree(points);votes=np.zeros((len(points),8),np.uint16)
    engine=Anything(temporal=False);audit=[];started=time.monotonic()
    for i,frame in enumerate(frames):
        rgb=np.array(Image.open(input_dir/frame['rgb']).convert('RGB'));stored=out/'frames'/f'{i:04d}.npz';meta=stored.with_suffix('.json')
        if stored.exists() and meta.exists():
            cached=np.load(stored);relative=cached['relative_depth'].astype(np.float32);sem=cached['semantic_labels'];entry=json.loads(meta.read_text());objects=entry['objects']
            # Keep the model's raw text for provenance and add only a display translation.
            for obj in objects: obj.setdefault('label_zh', chinese_label(obj.get('class_id'), obj.get('label','')))
            stats=entry['stats']
            assert entry['frame']==frame,'Cached model output belongs to a different RGB-D frame'
        else:
            relative,sem,objects,stats=engine.infer(rgb)
            np.savez_compressed(stored,relative_depth=relative.astype(np.float16),semantic_labels=sem)
        depth=np.load(input_dir/frame['depth'])['depth'];h,w=depth.shape;rh,rw=rgb.shape[:2]
        assert h>0 and w>0 and depth.size>0,'Empty metric depth must not silently suppress semantic projection'
        fd=w/(2*np.tan(np.deg2rad(dc['fov']/2)));fc=rw/(2*np.tan(np.deg2rad(rc['fov']/2)))
        v,u=np.mgrid[0:h:2,0:w:2];uc=np.rint((u-(w-1)/2)*fc/fd+(rw-1)/2).astype(int);vc=np.rint((v-(h-1)/2)*fc/fd+(rh-1)/2).astype(int)
        valid=(uc>=0)&(uc<rw)&(vc>=0)&(vc<rh);dep=depth[v,u];valid&=np.isfinite(dep)&(dep>.15)&(dep<8.)
        inner=np.zeros_like(sem)
        for lab in range(1,8):inner[cv2.erode((sem==lab).astype(np.uint8),np.ones((5,5),np.uint8)).astype(bool)]=lab
        labels=np.zeros(u.shape,np.uint8);labels[valid]=inner[vc[valid],uc[valid]];valid&=labels>0
        dd=dep[valid];q=np.column_stack((dd,-(u[valid]-(w-1)/2)*dd/fd,-(v[valid]-(h-1)/2)*dd/fd))
        stamp=frame['depth_stamp'];pose=np.array([np.interp(stamp,stamps,position[:,k]) for k in range(3)])
        world=rotations(stamp).apply(sensor_rotation.apply(q)+sensor_translation-imu_offset)+pose
        distance,index=tree.query(world,distance_upper_bound=.12,workers=4);ok=np.isfinite(distance);label=labels[valid][ok];index=index[ok]
        for lab in range(1,8):
            ids=np.unique(index[label==lab]);votes[ids,lab]+=1
        entry=dict(frame=frame,objects=objects,stats=stats,projected_pixels=len(world),associated_pixels=int(ok.sum()),unique_map_points=int(len(np.unique(index))),median_association_distance_m=float(np.median(distance[ok])) if ok.any() else None)
        meta.write_text(json.dumps(entry,indent=2));audit.append(entry)
        if i%40==0 or i==len(frames)-1:
            display=panel(rgb,relative,sem,objects,stats);display[:40,:480]=[12,20,32]
            cv2.putText(display,f'Recorded RGB | {stamp-report["first_stamp"]:.1f} s',(12,26),cv2.FONT_HERSHEY_SIMPLEX,.55,(230,240,250),1,cv2.LINE_AA)
            Image.fromarray(display).save(out/'panels'/f'frame_{i:04d}.png')
        if i%10==0:print(f'{i+1}/{len(frames)} associated={len(np.unique(index))} objects={[(o["label"],round(o["detection_score"],2)) for o in objects]} elapsed={time.monotonic()-started:.1f}s',flush=True)
    winner=votes.argmax(1).astype(np.uint8);maximum=votes.max(1);total=votes.sum(1);ratio=maximum/np.maximum(total,1)
    labels=np.where((maximum>=2)&(ratio>=.60),winner,0).astype(np.uint8)
    np.savez_compressed(out/'fusion_votes.npz',votes=votes,winning_class=winner,winning_fraction=ratio.astype(np.float32))
    np.savez_compressed(data/'data/semantic_map.npz',points=points,labels=labels)
    np.savez_compressed(ref/'model_semantics.npz',labels=labels,colors=SEMANTIC[labels],source_index=np.arange(len(points),dtype=np.int32),accepted_map_index=replay['source_index'])
    write_pcd(ref/'dense_model_semantic.pcd',points,labels)
    metadata=dict(source_recording=str(run),model_frames=len(frames),model_source='fresh recorded RGB; DepthAnythingV2-small + GroundingDINO-tiny + SAM2.1-hiera-tiny',metric_depth='synchronized simulator axial depth, not DepthAnything output',pose='accepted calibrated LIO IMU pose; contains documented structural priors',association_radius_m=.12,mask_erosion_px=2,pixel_stride=2,minimum_distinct_frame_votes=2,minimum_winning_fraction=.6,class_counts={name:int((labels==k).sum()) for k,name in enumerate(LABELS)},frames_with_class={name:sum(any(o['class_id']==k for o in a['objects']) for a in audit) for k,name in enumerate(LABELS)},elapsed_seconds=time.monotonic()-started,geometry_source_sha256=report['source_sha256'],geometry_changes='none; labels assigned to original accepted map points',limitations=['Offline inference; timeline shows scan visibility with final labels.','Model predictions are not ground truth; unknowns remain unclassified.','Person points, if detected, are historical observations, not a persistent dynamic track.','No full Hydra or S-Graphs pipeline, no metric accuracy claim.'])
    (out/'report.json').write_text(json.dumps(metadata,indent=2));report['semantic_source']=metadata['model_source'];(ref/'replay_report.json').write_text(json.dumps(report,indent=2));print(json.dumps(metadata,indent=2),flush=True)

if __name__=='__main__':main()
