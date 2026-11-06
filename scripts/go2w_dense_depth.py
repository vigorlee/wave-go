#!/usr/bin/python3
"""Project actual UE axial depth pixels; never fills missing depth or samples scene models."""
import json,time
from pathlib import Path
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,PointCloud2,PointField
from nav_msgs.msg import Odometry
from std_msgs.msg import Header,String
from scipy.spatial.transform import Rotation

def project_depth(depth,fov):
    h,w=depth.shape;v,u=np.indices((h,w));f=w/(2*np.tan(np.deg2rad(fov)/2))
    # ROS body coordinates: x forward, y left, z up; UE depth is axial metres.
    return np.stack([depth,-(u-(w-1)/2)*depth/f,-(v-(h-1)/2)*depth/f],axis=-1).reshape(-1,3)

class Dense(Node):
    def __init__(self):
        super().__init__('go2w_dense_depth');self.history=deque(maxlen=500);self.frames=0;self.last=0
        config=Path(__file__).resolve().parents[1]/'matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/config/config.json'
        self.config=config;self.s=json.loads(config.read_text())['robot']['sensors']['depth_sensor']
        self.create_subscription(Odometry,'/odom/mujoco_odom',lambda m:self.history.append(m),30)
        self.create_subscription(Image,self.s['topic'],self.depth,qos_profile_sensor_data)
        self.pub=self.create_publisher(PointCloud2,'/go2w/dense/points_world',qos_profile_sensor_data)
        self.status=self.create_publisher(String,'/go2w/dense/status',10)
    def depth(self,m):
        if not self.history:return
        started=time.time();stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        od=min(self.history,key=lambda o:abs(o.header.stamp.sec+o.header.stamp.nanosec/1e9-stamp))
        age=abs(od.header.stamp.sec+od.header.stamp.nanosec/1e9-stamp)
        if age>.2:return
        self.s=json.loads(self.config.read_text())['robot']['sensors']['depth_sensor']
        w=m.width or self.s['width'];h=m.height or self.s['height']
        if m.encoding!='32FC1' or len(m.data)!=w*h*4:return
        d=np.frombuffer(m.data,dtype='>f4' if m.is_bigendian else '<f4').reshape(h,w)
        p=project_depth(d,self.s.get('fov',90));p=p[np.isfinite(p).all(axis=1)&(p[:,0]>.15)&(p[:,0]<8)]
        r=self.s['rotation'];p=Rotation.from_euler('xyz',[r['roll'],-r['pitch'],-r['yaw']],degrees=True).apply(p)
        pos=self.s['position'];p+=np.array([pos['x'],-pos['y'],pos['z']])*.01
        q=od.pose.pose.orientation;t=od.pose.pose.position
        p=Rotation.from_quat([q.x,q.y,q.z,q.w]).apply(p)+[t.x,t.y,t.z];p=p.astype('<f4')
        self.pub.publish(PointCloud2(header=Header(stamp=m.header.stamp,frame_id='odom'),height=1,width=len(p),fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,k in enumerate(('x','y','z'))],is_bigendian=False,point_step=12,row_step=len(p)*12,data=p.tobytes(),is_dense=True))
        self.frames+=1;self.status.publish(String(data=json.dumps(dict(source='UE_axial_depth_backprojection',width=w,height=h,points=len(p),frame=self.frames,odom_delta_seconds=age,processing_ms=(time.time()-started)*1000,observed_hz=1/(started-self.last) if self.last else 0))))
        self.last=started
if __name__=='__main__':
    rclpy.init();n=Dense()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.shutdown()
