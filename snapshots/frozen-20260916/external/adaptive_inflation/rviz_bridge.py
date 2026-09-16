"""Publishes the offline browser analysis only, in an isolated ROS domain.
Never publishes commands, goals, TF or navigation costmap topics.
"""
from pathlib import Path
import json,numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from sensor_msgs.msg import PointCloud2,PointField
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header,String
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point
R=Path(__file__).resolve().parent
class Replay(Node):
 def __init__(self):
  super().__init__('pcd_adaptive_offline_preview');q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
  self.cloud=self.create_publisher(PointCloud2,'/wavego/adaptive_preview/points',q)
  self.buffer=self.create_publisher(PointCloud2,'/wavego/adaptive_preview/buffer_cells',q)
  self.grid=self.create_publisher(OccupancyGrid,'/wavego/adaptive_preview/inflated',q)
  self.mark=self.create_publisher(MarkerArray,'/wavego/adaptive_preview/markers',q)
  self.state=self.create_publisher(String,'/wavego/adaptive_preview/status',q)
  self.last=0;self.key=None;self.create_timer(.5,self.tick)
 def tick(self):
  path=R/'current_costmap.npz'
  if not path.exists() or path.stat().st_mtime_ns==self.last:return
  modified=path.stat().st_mtime_ns
  a=np.load(path);m=json.loads(str(a['metadata']));h=Header(frame_id='adaptive_preview',stamp=self.get_clock().now().to_msg())
  if self.key!=m['dataset']:
   b=np.load(R/f"{m['dataset']}_prepared.npz");p=b['points'];l=b['labels'];colors=np.array([[145,164,186],[50,185,255],[179,124,255],[255,180,76],[255,86,139],[187,199,213],[91,189,167],[228,209,109]],np.uint32)[l]
   arr=np.zeros(len(p),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')])
   for i,n in enumerate(['x','y','z']):arr[n]=p[:,i]
   arr['rgb']=(colors[:,0]<<16)|(colors[:,1]<<8)|colors[:,2]
   fields=[PointField(name=n,offset=4*i,datatype=7 if i<3 else 6,count=1) for i,n in enumerate(arr.dtype.names)]
   self.cloud.publish(PointCloud2(header=h,height=1,width=len(p),fields=fields,point_step=16,row_step=16*len(p),is_dense=True,data=arr.tobytes()));self.key=m['dataset']
  grid=OccupancyGrid(header=h);grid.info.resolution=float(a['resolution']);grid.info.height=a['inflated'].shape[0];grid.info.width=a['inflated'].shape[1];grid.info.origin.orientation.w=1.;grid.info.origin.position.x=float(a['origin'][0]);grid.info.origin.position.y=float(a['origin'][1]);grid.info.origin.position.z=float(a['floor'])+.04
  risk=np.full(a['inflated'].shape,-1,np.int8);risk[a['inflated']]=100 # No observed-return column is claimed to be free.
  grid.data=risk.ravel().tolist();self.grid.publish(grid)
  rows,cols=np.nonzero(a['inflated']&~a['occupancy']);buf=np.zeros(len(rows),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')]);buf['x']=float(a['origin'][0])+(cols+.5)*float(a['resolution']);buf['y']=float(a['origin'][1])+(rows+.5)*float(a['resolution']);buf['z']=float(a['floor'])+.04;buf['rgb']=(252<<16)|(170<<8)|66
  bf=[PointField(name=n,offset=4*i,datatype=7 if i<3 else 6,count=1) for i,n in enumerate(buf.dtype.names)]
  self.buffer.publish(PointCloud2(header=h,height=1,width=len(buf),fields=bf,point_step=16,row_step=len(buf)*16,is_dense=True,data=buf.tobytes()))
  markers=[Marker(header=h,action=Marker.DELETEALL)]
  for k,radius in enumerate([.35,m['radius']]):
   marker=Marker(header=h,ns='probe',id=k,type=Marker.LINE_STRIP,action=Marker.ADD);marker.pose.orientation.w=1.;marker.scale.x=.025;marker.color.a=1.;marker.color.r=1. if k else .3;marker.color.g=.7;marker.color.b=.2 if k else 1.
   for angle in np.linspace(0,2*np.pi,81):marker.points.append(Point(x=m['position'][0]+radius*np.cos(angle),y=m['position'][1]+radius*np.sin(angle),z=float(a['floor'])+.10))
   markers.append(marker)
  marker=Marker(header=h,ns='readout',id=9,type=Marker.TEXT_VIEW_FACING,action=Marker.ADD);marker.pose.orientation.w=1.;marker.pose.position.x=m['position'][0];marker.pose.position.y=m['position'][1];marker.pose.position.z=float(a['floor'])+1.5;marker.scale.z=.24;marker.color.r=1.;marker.color.g=1.;marker.color.b=1.;marker.color.a=1.;marker.text=f"OFFLINE {m['dataset']} | width {str(round(m['corridor_width'],2)) if m['corridor_width'] is not None else 'unobserved'} m\nradius {m['radius']:.2f} m | speed {m['speed']:.2f} m/s";markers.append(marker)
  self.mark.publish(MarkerArray(markers=markers));self.state.publish(String(data=json.dumps(m)));self.last=modified
  self.get_logger().info(f"Published offline {self.key}, radius {m['radius']:.2f} m")
rclpy.init();n=Replay()
try:rclpy.spin(n)
except KeyboardInterrupt:pass
finally:n.destroy_node();rclpy.try_shutdown()
