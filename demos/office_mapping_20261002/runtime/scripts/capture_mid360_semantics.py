#!/usr/bin/python3
"""Record actual labeled MID360 frames and odometry for display verification."""
import argparse,json,time
from pathlib import Path
import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs_py import point_cloud2
ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--seconds',type=float,default=480);args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=True)
rclpy.init();n=rclpy.create_node('capture_mid360_semantics');pose=None;counts={};last=0

def odom(m):
 global pose
 pose=m.pose.pose

def cloud(m):
 global last
 if pose is None or time.monotonic()-last<1:return
 last=time.monotonic();x,y=pose.position.x,pose.position.y;region='other'
 for name,lo,hi in [('stairs',1,4),('stairs_down',6,9),('flat',10,13),('ramp',13,18),('dynamic',19,26)]:
  if abs(x)<2 and lo<y<hi:region=name;break
 count=counts.get(region,0)
 if count>=5:return
 arr=point_cloud2.read_points(m,field_names=('x','y','z','label'),skip_nans=True)
 xyz=np.column_stack([arr[k] for k in ('x','y','z')]);lab=arr['label'];key=f'{region}_{count:02d}'
 np.savez_compressed(args.output/(key+'.npz'),points=xyz,labels=lab)
 (args.output/(key+'.json')).write_text(json.dumps(dict(source='/go2w/anything/mid360_points',geometry_source='/livox/lidar_raw',frame=m.header.frame_id,stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,robot_xy=[x,y],points=len(xyz),labeled=int((lab>0).sum())),indent=2));counts[region]=count+1
n.create_subscription(Odometry,'/odom/mujoco_odom',odom,10);n.create_subscription(PointCloud2,'/go2w/anything/mid360_points',cloud,qos_profile_sensor_data)
end=time.monotonic()+args.seconds
try:
 while rclpy.ok() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.2)
except KeyboardInterrupt:pass
finally:rclpy.shutdown()
