"""Read-only live telemetry audit; writes small JSON, never controls the robot."""
import argparse
from collections import Counter, defaultdict, deque
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray

class Audit(Node):
    def __init__(self, out):
        super().__init__('office_live_audit')
        self.out=out;self.started=time.monotonic();self.counts=Counter();self.names=Counter()
        self.ages=defaultdict(list);self.tracks=defaultdict(list);self.radii=[]
        self.maps=[];self.reference=deque(maxlen=1000);self.errors=[]
        self.transform=np.asarray(json.loads((out/'frame_alignment.json').read_text())['matrix'])
        for topic,kind in [('/livox/lidar_raw',PointCloud2),('/odom/mujoco_odom',Odometry),('/wavego/navigation/odom',Odometry)]:
            self.create_subscription(kind,topic,lambda m,k=topic:self.sensor(m,k),qos_profile_sensor_data)
        self.create_subscription(PointCloud2,'/wavego/live_dense/points',lambda m:self.maps.append(m.width),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        for topic in ['/wavego/global/markers','/wavego/live_dense/inflation']:
            self.create_subscription(MarkerArray,topic,lambda m,k=topic:self.markers(m,k),10)
        self.create_timer(5,self.save)

    def sensor(self,m,key):
        stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        self.counts[key]+=1;self.ages[key].append(time.time()-stamp)
        if isinstance(m,Odometry):
            p=m.pose.pose.position;xyz=np.array([p.x,p.y,p.z])
            if key=='/odom/mujoco_odom':self.reference.append((stamp,xyz))
            elif self.reference:
                t,ref=min(self.reference,key=lambda a:abs(a[0]-stamp))
                if abs(t-stamp)<.04:
                    aligned=(self.transform@np.r_[ref,1])[:3]
                    self.errors.append(float(np.linalg.norm(aligned-xyz)))

    def markers(self,m,key):
        self.counts[key]+=1
        for x in m.markers:
            if x.action==3:continue
            self.names[x.ns]+=1
            if x.ns=='measured_dynamic':self.tracks[x.id].append([x.pose.position.x,x.pose.position.y])
            if x.ns=='robot_adaptive_radius' and x.points:
                a=x.points
                self.radii.append(float(np.hypot(a[0].x-a[len(a)//2].x,a[0].y-a[len(a)//2].y)/2))

    def save(self):
        elapsed=time.monotonic()-self.started
        report=dict(duration_sec=elapsed,message_counts=self.counts,
                    receive_rates_hz={k:v/elapsed for k,v in self.counts.items()},
                    sensor_age_sec={k:dict(p95=float(np.percentile(v,95)),maximum=float(max(v))) for k,v in self.ages.items()},
                    namespace_observations=self.names,
                    adaptive_radius_range_m=[min(self.radii),max(self.radii)] if self.radii else [],
                    map_point_range=[min(self.maps),max(self.maps)] if self.maps else [],
                    moving_geometric_candidates={str(k):dict(samples=len(v),displacement_m=float(np.linalg.norm(np.array(v[-1])-v[0]))) for k,v in self.tracks.items()},
                    aligned_lio_reference_error_m=dict(samples=len(self.errors),p95=float(np.percentile(self.errors,95)),maximum=max(self.errors)) if self.errors else None,
                    reference_note='Initial rigid alignment and nearest timestamp within 40 ms; simulation reference only audits LIO and never generates map points')
        target=self.out/'live_overlay_validation.json';temp=target.with_suffix('.tmp')
        temp.write_text(json.dumps(report,indent=2));temp.replace(target)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--duration',type=float,default=240);a=p.parse_args()
    rclpy.init();n=Audit(a.output)
    try:
        while time.monotonic()-n.started<a.duration:rclpy.spin_once(n,timeout_sec=.1)
    finally:
        n.save();n.destroy_node();rclpy.try_shutdown()
