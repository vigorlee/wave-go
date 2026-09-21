#!/usr/bin/env python3
"""Extract synchronized, unmodified RGB/metric depth from the new ROS 2 capture."""
import argparse,json
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image,CompressedImage
ROOT=Path(__file__).resolve().parents[1];p=argparse.ArgumentParser();p.add_argument('--run',type=Path,default=ROOT/'outputs/level_fix/fresh_20260919_02');p.add_argument('--output',type=Path,default=ROOT/'outputs/fresh_semantic_20260919/data/rgbd');p.add_argument('--period',type=float,default=.39);a=p.parse_args();a.output.mkdir(exist_ok=True,parents=True)
assert not (a.output/'frames.json').exists(),'Do not overwrite an existing extraction'
sensor=json.loads((a.run/'sensor_config.json').read_text())['robot']['sensors']['depth_sensor'];fallback_count=0
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(a.run/'sensors_rosbag2'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=['/image_raw/compressed','/go2w/depth/image_raw']));rgbs=[];depths=[]
while r.has_next():
 topic,data,t=r.read_next();m=deserialize_message(data,CompressedImage if topic=='/image_raw/compressed' else Image);stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
 if topic=='/image_raw/compressed':rgbs.append((stamp,bytes(m.data)))
 else:
  assert m.encoding=='32FC1'
  h,w=m.height or sensor['height'],m.width or sensor['width'];step=m.step or w*4
  assert len(m.data)==h*step and h>0 and w>0,'Depth payload does not match recorded sensor dimensions'
  fallback_count+=int(not m.height or not m.width)
  d=np.ndarray((h,w),buffer=m.data,dtype='>f4' if m.is_bigendian else '<f4',strides=(step,4)).copy();depths.append((stamp,d))
selected=[];last=-1;dt=np.array([x[0] for x in depths]);times=np.load(a.run/'structural_calibrated/calibration_trajectory.npz')['stamp']
for stamp,jpg in rgbs:
 if stamp-last<a.period or stamp<times[0] or stamp>times[-1]:continue
 j=int(np.argmin(abs(dt-stamp)));delta=abs(dt[j]-stamp)
 if delta>.04:continue
 index=len(selected);(a.output/f'{index:04d}.jpg').write_bytes(jpg);np.savez_compressed(a.output/f'{index:04d}_depth.npz',depth=depths[j][1]);selected.append(dict(id=index,stamp=stamp,depth_stamp=float(dt[j]),sync_delta_s=float(delta),rgb=f'{index:04d}.jpg',depth=f'{index:04d}_depth.npz'));last=stamp
(a.output/'frames.json').write_text(json.dumps(selected,indent=2));(a.output/'extraction_report.json').write_text(json.dumps(dict(raw_rgb=len(rgbs),raw_depth=len(depths),selected_pairs=len(selected),max_sync_delta_s=max(x['sync_delta_s'] for x in selected),metric_depth='UE 32FC1 axial metres; no Depth Anything scaling',zero_dimension_headers=fallback_count,dimension_fallback='recorded sensor_config.json; exact payload length verified',source=str(a.run/'sensors_rosbag2')),indent=2));print('extracted',len(selected),'pairs')
