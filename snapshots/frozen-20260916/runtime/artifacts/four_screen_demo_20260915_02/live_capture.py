#!/usr/bin/python3
"""Record every received registered LiDAR frame; live mapping and inflation overlay.
Navigation commands are never published here. Raw and display data stay separate.
"""
import argparse,json,time,threading,signal,sys,queue
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,PointField
from sensor_msgs_py import point_cloud2 as pc
from nav_msgs.msg import Odometry
from std_msgs.msg import Header,String
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point
sys.path.insert(0,'/home/unitree/semantic_map_preview/adaptive_inflation')
from adaptive_core import build_grid,analyze

def points_message(xyz,floor,stamp):
 a=np.empty(len(xyz),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')])
 for i,k in enumerate(['x','y','z']):a[k]=xyz[:,i]
 t=np.clip((xyz[:,2]-floor)/3,0,1);a['rgb']=((40+210*t).astype(np.uint32)<<16)|((140+45*t).astype(np.uint32)<<8)|(240-200*t).astype(np.uint32)
 fields=[PointField(name=k,offset=4*i,datatype=7 if i<3 else 6,count=1) for i,k in enumerate(a.dtype.names)]
 return PointCloud2(header=Header(frame_id='lio_map',stamp=stamp),height=1,width=len(a),fields=fields,point_step=16,row_step=len(a)*16,data=a.tobytes(),is_dense=True)

class Recorder(Node):
 def __init__(self,out):
  super().__init__('wavego_live_dense_recorder');self.out=out;out.mkdir(parents=True,exist_ok=True)
  self.raw=(out/'registered_xyzi.float32').open('wb');self.index=(out/'frames.jsonl').open('w');self.trajectory=(out/'lio_trajectory.jsonl').open('w')
  self.scans=0;self.npoints=0;self.rejected=0;self.last_stamp=-1.;self.first=None;self.last_receive=0.;self.pose=None;self.last_xyz=None;self.cells={};self.jobs=queue.Queue(maxsize=40);self.preview_drops=0;self.stop_event=threading.Event();self.status={};self.floor=None;self.radii=[]
  q=QoSProfile(depth=300,reliability=ReliabilityPolicy.BEST_EFFORT)
  self.create_subscription(PointCloud2,'/wavego/slam/registered_points',self.scan,q)
  self.create_subscription(Odometry,'/wavego/navigation/odom',self.odom,qos_profile_sensor_data)
  self.pub=self.create_publisher(PointCloud2,'/wavego/live_dense/points',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
  self.mark=self.create_publisher(MarkerArray,'/wavego/live_dense/inflation',10)
  self.stat=self.create_publisher(String,'/wavego/live_dense/status',10)
  self.worker=threading.Thread(target=self.process,daemon=True);self.worker.start()
 def odom(self,m):
  p=m.pose.pose.position;q=m.pose.pose.orientation;yaw=np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));v=np.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y)
  self.pose=dict(x=p.x,y=p.y,z=p.z,yaw=float(yaw),speed=float(v),received=time.monotonic())
  if self.floor is None:self.floor=p.z-.40
  self.trajectory.write(json.dumps(dict(stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,**self.pose))+'\n')
 def scan(self,m):
  ts=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
  if ts<=self.last_stamp:self.rejected+=1;return
  a=pc.read_points(m,skip_nans=True);xyz=np.column_stack([a[k] for k in ['x','y','z']]).astype(np.float32);ok=np.isfinite(xyz).all(1);xyz=xyz[ok]
  if not len(xyz):return
  vals=np.empty((len(xyz),4),np.float32);vals[:,:3]=xyz;vals[:,3]=a['intensity'][ok] if 'intensity' in a.dtype.names else 0
  self.raw.write(vals.tobytes());self.index.write(json.dumps(dict(stamp=ts,received=time.time(),offset_points=self.npoints,points=len(xyz),input_frame=m.header.frame_id,export_frame='lio_map'))+'\n')
  self.npoints+=len(xyz);self.scans+=1;self.last_stamp=ts;self.last_receive=time.monotonic();self.last_xyz=xyz
  if self.first is None:self.first=ts
  if self.scans%10==0:self.raw.flush();self.index.flush();self.trajectory.flush()
  try:self.jobs.put_nowait(xyz)
  except queue.Full:self.preview_drops+=1
 def process(self):
  last_pub=0
  while not self.stop_event.is_set() or not self.jobs.empty():
   try:xyz=self.jobs.get(timeout=.3)
   except queue.Empty:continue
   # Six-centimetre live display; final export recomputes three centimetres from ALL recorded frames.
   keys=np.floor(xyz/.06).astype(np.int64);packed=((keys[:,0]+1048576)<<42)|((keys[:,1]+1048576)<<21)|(keys[:,2]+1048576)
   _,ix=np.unique(packed,return_index=True);self.cells.update(zip(packed[ix].tolist(),xyz[ix]))
   if time.monotonic()-last_pub<3 or self.stop_event.is_set():continue
   last_pub=time.monotonic();allpts=np.asarray(list(self.cells.values()),np.float32)
   self.pub.publish(points_message(allpts,self.floor or 0,self.get_clock().now().to_msg()))
   pose=self.pose;adaptive=None
   if pose and time.monotonic()-pose['received']<1.5:
    scan=self.last_xyz;delta=scan-[pose['x'],pose['y'],pose['z']];near=scan[(np.linalg.norm(delta[:,:2],axis=1)<6)&(np.abs(delta[:,2])<2)]
    try:
     g=build_grid(near,np.zeros(len(near),np.uint8),pose['z']-.40);adaptive,grid=analyze(g,[pose['x'],pose['y']],pose['yaw'],min(pose['speed'],1.2))
     self.radii.append(adaptive['radius']);self.publish_markers(g,grid,adaptive)
    except ValueError:pass
   self.status=dict(scans=self.scans,received_points=self.npoints,live_voxels=len(self.cells),preview_voxel_m=.06,export_voxel_m=.03,sensor_elapsed_sec=self.last_stamp-(self.first or self.last_stamp),preview_dropped_batches=self.preview_drops,rejected_duplicate_or_old_scans=self.rejected,adaptive=adaptive)
   (self.out/'live_status.json').write_text(json.dumps(self.status,indent=2));self.stat.publish(String(data=json.dumps(self.status)))
   with (self.out/'inflation_events.jsonl').open('a') as f:f.write(json.dumps(dict(time=time.time(),**self.status))+'\n')
 def publish_markers(self,g,grids,m):
  h=Header(frame_id='lio_map',stamp=self.get_clock().now().to_msg());items=[Marker(header=h,action=Marker.DELETEALL)]
  ys,xs=np.nonzero(grids['inflated']&~g['occupancy']);z=g['floor']+.05
  cell=Marker(header=h,ns='live_adaptive_cells',id=1,type=Marker.POINTS,action=Marker.ADD);cell.pose.orientation.w=1.;cell.scale.x=cell.scale.y=.05;cell.color.r=1.;cell.color.g=.6;cell.color.b=.15;cell.color.a=.35
  cell.points=[Point(x=float(x*g['res']+g['origin'][0]),y=float(y*g['res']+g['origin'][1]),z=float(z)) for x,y in zip(xs,ys)];items.append(cell)
  text=Marker(header=h,ns='live_readout',id=2,type=Marker.TEXT_VIEW_FACING,action=Marker.ADD);text.pose.orientation.w=1.;text.pose.position.x=m['position'][0];text.pose.position.y=m['position'][1];text.pose.position.z=z+1.3;text.scale.z=.28;text.color.r=text.color.g=text.color.b=text.color.a=1.;text.text=f"LIVE LiDAR | {self.scans} scans\ninflation {m['radius']:.2f} m | v {m['speed']:.2f} m/s";items.append(text)
  self.mark.publish(MarkerArray(markers=items))
 def close(self):
  self.stop_event.set();self.worker.join(timeout=30);self.raw.close();self.index.close();self.trajectory.close()
  (self.out/'capture_complete.json').write_text(json.dumps(dict(scans=self.scans,points=self.npoints,first_stamp=self.first,last_stamp=self.last_stamp,voxel_m=.03,preview_drops=self.preview_drops,source='/wavego/slam/registered_points',frame_alias='native LIO map published as lio_map by existing mapping sidecar; no simulator ground-truth pose',adaptive_radius_range=[min(self.radii),max(self.radii)] if self.radii else [],contains_dynamic_history=True),indent=2))

if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);args=ap.parse_args();rclpy.init();node=Recorder(args.output)
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:node.close();node.destroy_node();rclpy.try_shutdown()
