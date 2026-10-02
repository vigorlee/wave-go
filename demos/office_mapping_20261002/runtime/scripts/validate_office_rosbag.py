from pathlib import Path
from collections import Counter,defaultdict
import json,sys,hashlib
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores,get_typestore
run=Path(sys.argv[1]).resolve();reports=[]
for path,store in [(run/'pointcloud_odom_rosbag2',Stores.ROS2_HUMBLE),(run/'go2w_pointcloud_odom_noetic.bag',Stores.ROS1_NOETIC)]:
 counts=Counter();frames=defaultdict(set);poses=defaultdict(list);stamps=defaultdict(list);points=Counter()
 with AnyReader([path],default_typestore=get_typestore(store)) as reader:
  for conn,t,data in reader.messages():
   counts[conn.topic]+=1
   if conn.msgtype.endswith(('PointCloud2','Odometry','Imu')):
    msg=reader.deserialize(data,conn.msgtype);frames[conn.topic].add(msg.header.frame_id);stamps[conn.topic].append(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec)
    if conn.msgtype.endswith('PointCloud2'):
     assert len(msg.data)==msg.row_step*msg.height and msg.width*msg.point_step<=msg.row_step
     assert {'x','y','z'}.issubset({x.name for x in msg.fields});points[conn.topic]+=msg.width*msg.height
    elif conn.msgtype.endswith('Odometry'):
     p=msg.pose.pose.position;poses[conn.topic].append([p.x,p.y,p.z])
  for key in ['/livox/lidar_raw','/wavego/slam/registered_points','/wavego/slam/odom','/odom/mujoco_odom']:assert counts[key]>100,key
  report=dict(path=str(path),message_counts=dict(counts),duration_seconds=reader.duration/1e9,frame_ids={k:sorted(v) for k,v in frames.items()},point_counts=dict(points),payloads_deserialized=True,odom_bounds={})
  for k,v in poses.items():
   a=np.array(v);assert np.isfinite(a).all();report['odom_bounds'][k]=dict(min=a.min(0).tolist(),max=a.max(0).tolist())
  a=np.array(stamps['/wavego/slam/registered_points']);b=np.array(stamps['/wavego/slam/odom']);idx=np.searchsorted(b,a);distance=np.minimum(abs(a-b[np.clip(idx,0,len(b)-1)]),abs(a-b[np.clip(idx-1,0,len(b)-1)]));report['registered_cloud_to_lio_odom_nearest_stamp_ms']=dict(median=float(np.median(distance)/1e6),max=float(distance.max()/1e6))
  reports.append(report)
assert reports[0]['message_counts']==reports[1]['message_counts']
assert reports[0]['point_counts']==reports[1]['point_counts']
(run/'rosbag_validation.json').write_text(json.dumps(dict(bags=reports,conversion_preserves_message_and_point_counts=True),indent=2));print(json.dumps(reports,indent=2))
