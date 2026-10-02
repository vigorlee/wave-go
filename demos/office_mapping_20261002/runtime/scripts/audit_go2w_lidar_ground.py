#!/usr/bin/python3
"""Measure ground returns from current raw LiDAR at the flat start area."""
import argparse,json,time,sys
from collections import deque
from pathlib import Path
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs_py import point_cloud2
from go2w_mid360_semantics import sensor_world
ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--seconds',type=float,default=8);args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=True)
cfg=json.loads((Path(__file__).resolve().parents[1]/'matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/config/config.json').read_text());rclpy.init();n=rclpy.create_node('lidar_ground_audit');poses=deque(maxlen=80);rows=[];last=None
stamp=lambda m:m.header.stamp.sec+m.header.stamp.nanosec*1e-9
n.create_subscription(Odometry,'/odom/mujoco_odom',lambda m:poses.append(m),qos_profile_sensor_data)
def cloud(m):
 global last
 if not poses:return
 od=min(list(poses),key=lambda o:abs(stamp(o)-stamp(m)))
 if abs(stamp(od)-stamp(m))>.1:return
 p=point_cloud2.read_points(m,field_names=('x','y','z'),skip_nans=True);p=np.column_stack([p[k] for k in ['x','y','z']]);w=sensor_world(p,cfg['robot']['sensors']['lidar'],od.pose.pose);xy=w[:,:2]-[od.pose.pose.position.x,od.pose.pose.position.y];distance=np.linalg.norm(xy,axis=1);ground=(abs(w[:,2])<.08)&(distance>.4)&(distance<10);near=ground&(distance<3)
 rows.append(dict(time=time.monotonic(),stamp=stamp(m),points=len(w),ground_points=int(sum(ground)),near_ground_points=int(sum(near)),ground_fraction=float(np.mean(ground)),z_percentiles=np.percentile(w[:,2],[1,25,50,75,99]).tolist(),raw_elevation_degrees=np.percentile(np.degrees(np.arctan2(p[:,2],np.linalg.norm(p[:,:2],axis=1))),[1,50,99]).tolist()))
 last=(w,ground)
n.create_subscription(PointCloud2,'/livox/lidar_raw',cloud,qos_profile_sensor_data);end=time.monotonic()+args.seconds
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
out=dict(sensor=cfg['robot']['sensors']['lidar'],frames=len(rows),median_ground_points=float(np.median([r['ground_points'] for r in rows])) if rows else 0,median_near_ground_points=float(np.median([r['near_ground_points'] for r in rows])) if rows else 0,median_points=float(np.median([r['points'] for r in rows])) if rows else 0,frames_detail=rows)
(args.output/'ground_audit.json').write_text(json.dumps(out,indent=2)+'\n')
if last:np.savez_compressed(args.output/'raw_world_ground.npz',points=last[0],ground_mask=last[1])
print(json.dumps({k:v for k,v in out.items() if k!='frames_detail'}));n.destroy_node();rclpy.shutdown()
