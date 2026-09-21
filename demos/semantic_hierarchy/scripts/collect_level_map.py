#!/usr/bin/env python3
"""Record fresh registered returns and estimated poses without changing geometry."""
import argparse,json,signal
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

class Collector(Node):
    def __init__(self,out):
        super().__init__('level_map_collector');self.out=out;out.mkdir(parents=True,exist_ok=True)
        if (out/'registered_xyzi.float32').exists():raise RuntimeError('Refusing to overwrite a capture')
        self.raw=(out/'registered_xyzi.float32').open('wb');self.index=(out/'frames.jsonl').open('w')
        self.poses=(out/'estimated_poses.jsonl').open('w');self.scans=0;self.points=0;self.last=-1.
        self.create_subscription(PointCloud2,'/wavego/level/registered_points',self.cloud,QoSProfile(depth=200,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Odometry,'/wavego/level/odom',self.pose,qos_profile_sensor_data)
    def pose(self,m):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        self.poses.write(json.dumps([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,p.x,p.y,p.z,q.x,q.y,q.z,q.w])+'\n')
    def cloud(self,m):
        stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if stamp<=self.last:return
        fields={f.name:f.offset for f in m.fields};names=['x','y','z','intensity']
        dtype=np.dtype(dict(names=names,formats=['<f4']*4,offsets=[fields[n] for n in names],itemsize=m.point_step))
        a=np.ndarray((m.height,m.width),dtype=dtype,buffer=m.data,strides=(m.row_step,m.point_step)).reshape(-1)
        xyz=np.column_stack([a[n] for n in names]);xyz=xyz[np.isfinite(xyz).all(1)].astype('<f4')
        self.raw.write(xyz.tobytes());self.index.write(json.dumps(dict(stamp=stamp,points=len(xyz),offset=self.points,frame=m.header.frame_id))+'\n')
        self.scans+=1;self.points+=len(xyz);self.last=stamp
        if self.scans%20==0:
            self.raw.flush();self.index.flush();self.poses.flush()
            (self.out/'status.json').write_text(json.dumps(dict(scans=self.scans,points=self.points,last_stamp=stamp)))
    def close(self):
        self.raw.close();self.index.close();self.poses.close()
        (self.out/'capture_complete.json').write_text(json.dumps(dict(scans=self.scans,points=self.points,last_stamp=self.last,source='/wavego/level/registered_points',coordinates='original registered returns, not flattened'),indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    rclpy.init();n=Collector(a.output)
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.close();n.destroy_node();rclpy.try_shutdown()
