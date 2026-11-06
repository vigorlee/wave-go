#!/usr/bin/python3
"""Record every received registered LiDAR frame; live mapping and inflation overlay.
Navigation commands are never published here. Raw and display data stay separate.
"""
import argparse,json,time,threading,signal,sys,queue
from pathlib import Path
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time as RosTime
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,PointField
from sensor_msgs_py import point_cloud2 as pc
from nav_msgs.msg import Odometry
from std_msgs.msg import Header,String,ColorRGBA
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration
from adaptive_core import build_grid,analyze,Settings
from navigation_surface import GroundSurface

DISPLAY_VOXEL_M=.03
MIN_OBSERVATIONS=2
GRAPH_NODE_SPACING_M=6.0
INFLATION_SETTINGS=Settings(comfort_margin=.10)

class MeasuredVoxelMap:
    """Hash voxel keys to contiguous, growable measured-point accumulators.

    The dictionary stores only array indices. Snapshotting a large map therefore
    never builds a Python list of every point/count tuple. Float64 accumulators
    retain the previous fusion arithmetic; published clouds remain float32.
    Only the mapping worker accesses this storage.
    """
    def __init__(self, initial_capacity=65536):
        self.index = {}
        self.size = 0
        self.xyz = np.empty((initial_capacity, 3), dtype=np.float64)
        self.counts = np.empty(initial_capacity, dtype=np.uint8)

    def __len__(self):
        return self.size

    def _reserve(self, needed):
        if needed <= len(self.counts):
            return
        capacity = max(needed, max(1, len(self.counts)) * 2)
        xyz = np.empty((capacity, 3), dtype=np.float64)
        counts = np.empty(capacity, dtype=np.uint8)
        xyz[:self.size] = self.xyz[:self.size]
        counts[:self.size] = self.counts[:self.size]
        self.xyz, self.counts = xyz, counts

    def update(self, xyz):
        # Each received scan contributes at most one real return per voxel.
        keys = np.floor(xyz / DISPLAY_VOXEL_M).astype(np.int64)
        packed = ((keys[:, 0] + 1048576) << 42) | ((keys[:, 1] + 1048576) << 21) | (keys[:, 2] + 1048576)
        unique_keys, first = np.unique(packed, return_index=True)
        key_list = unique_keys.tolist()
        slots = np.fromiter((self.index.get(key, -1) for key in key_list), dtype=np.intp, count=len(key_list))
        new = slots < 0
        added = int(np.count_nonzero(new))
        self._reserve(self.size + added)
        if added:
            new_slots = np.arange(self.size, self.size + added, dtype=np.intp)
            slots[new] = new_slots
            self.index.update(zip(unique_keys[new].tolist(), new_slots.tolist()))
            self.xyz[new_slots] = xyz[first[new]]
            self.counts[new_slots] = 1
            self.size += added
        known = ~new
        if np.any(known):
            old_slots = slots[known]
            old_counts = self.counts[old_slots]
            weight = 1.0 / (np.minimum(old_counts, 100).astype(np.float64) + 1.0)
            # Advanced indices are unique, so all independent observations can
            # be fused together without per-point Python arithmetic.
            old_xyz = self.xyz[old_slots]
            self.xyz[old_slots] = old_xyz + (xyz[first[known]] - old_xyz) * weight[:, None]
            self.counts[old_slots] = np.minimum(old_counts + 1, 101)

    def observed_points(self):
        confirmed = self.counts[:self.size] >= MIN_OBSERVATIONS
        return self.xyz[:self.size][confirmed].astype(np.float32)

def spectral_rgb(z,floor):
 # OfficeWorld LIO palette from ground amber through green/cyan to violet.
 # Colors encode measured height only; they are not semantic classes.
 stops=np.array([0.,.13,.27,.43,.61,.78,1.],np.float32)
 colors=np.array([[255,149,0],[255,224,0],[103,235,26],[0,220,189],
                  [13,132,235],[45,51,178],[123,44,151]],np.float32)
 t=np.clip((np.asarray(z)-floor)/6.5,0,1)
 # Keep identical interpolation/truncation while avoiding three float64
 # channel arrays plus a second full-size float64 RGB stack at map scale.
 rgb=np.empty(t.shape+(3,),dtype=np.uint32)
 for i in range(3):rgb[...,i]=np.interp(t,stops,colors[:,i])
 return rgb

def trajectory_graph(nodes,stamp,floor):
 h=Header(frame_id='lio_map',stamp=stamp);out=[]
 spheres=Marker(header=h,ns='measured_lio_keyframes',id=1,type=Marker.SPHERE_LIST,action=Marker.ADD)
 spheres.pose.orientation.w=1.;spheres.scale.x=spheres.scale.y=spheres.scale.z=.34
 spheres.color=ColorRGBA(r=1.,g=1.,b=1.,a=1.)
 lines=Marker(header=h,ns='measured_lio_keyframe_edges',id=2,type=Marker.LINE_STRIP,action=Marker.ADD)
 lines.pose.orientation.w=1.;lines.scale.x=.035;lines.color=ColorRGBA(r=.78,g=.86,b=.94,a=.82)
 arr=np.asarray(nodes,np.float32).reshape(-1,3)
 if len(arr):
  rgb=spectral_rgb(arr[:,2],floor)/255.
  for xyz,c in zip(arr,rgb):
   point=Point(x=float(xyz[0]),y=float(xyz[1]),z=float(xyz[2]+1.1))
   spheres.points.append(point);lines.points.append(point)
   spheres.colors.append(ColorRGBA(r=float(c[0]),g=float(c[1]),b=float(c[2]),a=.95))
 out.extend([spheres,lines]);return MarkerArray(markers=out)

def points_message(xyz,floor,stamp,mode='height'):
 a=np.empty(len(xyz),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')])
 for i,k in enumerate(['x','y','z']):a[k]=xyz[:,i]
 if mode=='geometry':
  a['rgb']=np.uint32(0xDCE8F2)
 elif mode=='scan':
  a['rgb']=np.uint32(0xEBF5FF)
 else:
  c=spectral_rgb(xyz[:,2],floor);a['rgb']=(c[:,0]<<16)|(c[:,1]<<8)|c[:,2]
 fields=[PointField(name=k,offset=4*i,datatype=7 if i<3 else 6,count=1) for i,k in enumerate(a.dtype.names)]
 return PointCloud2(header=Header(frame_id='lio_map',stamp=stamp),height=1,width=len(a),fields=fields,point_step=16,row_step=len(a)*16,data=a.tobytes(),is_dense=True)

class Recorder(Node):
 def __init__(self,out,alignment=None,record_raw=False,resume_display=None):
  super().__init__('wavego_live_dense_recorder');self.out=out;out.mkdir(parents=True,exist_ok=True)
  self.alignment=alignment;self.display_transform=None
  self.raw=(out/'registered_xyzi.float32').open('wb') if record_raw else None;self.index=(out/'frames.jsonl').open('w');self.trajectory=(out/'lio_trajectory.jsonl').open('w')
  self.scans=0;self.npoints=0;self.rejected=0;self.last_stamp=-1.;self.first=None;self.last_receive=0.;self.pose=None;self.last_xyz=None;self.cells=MeasuredVoxelMap();self.jobs=queue.Queue(maxsize=40);self.preview_drops=0;self.stop_event=threading.Event();self.status={};self.floor=None;self.radii=[];self.graph_nodes=[];self.latest_adaptive=None
  self.pose_history=deque(maxlen=500);self.surface=None
  self.floor_scan_history=deque(maxlen=24)
  self.resume_display=resume_display
  if resume_display:
   saved=np.load(resume_display,allow_pickle=False)
   if alignment is None or str(saved['alignment_path'])!=str(alignment.resolve()):raise ValueError('Display resume requires the same running LIO session')
   xyz=saved['xyz'];self.cells.update(xyz)
   # Published input voxels already had at least two actual observations;
   # this restores that known lower bound, without inventing new returns.
   self.cells.counts[:self.cells.size]=MIN_OBSERVATIONS
   self.floor=float(saved['floor'])
  q=QoSProfile(depth=300,reliability=ReliabilityPolicy.BEST_EFFORT)
  self.create_subscription(PointCloud2,'/wavego/slam/registered_points',self.scan,q)
  self.create_subscription(Odometry,'/wavego/navigation/odom',self.odom,qos_profile_sensor_data)
  self.pub=self.create_publisher(PointCloud2,'/wavego/live_dense/points',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
  self.geometry_pub=self.create_publisher(PointCloud2,'/wavego/live_dense/geometry',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
  self.cutaway_pub=self.create_publisher(PointCloud2,'/wavego/live_dense/cutaway',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
  self.scan_pub=self.create_publisher(PointCloud2,'/wavego/live_dense/current_scan',q)
  self.graph_pub=self.create_publisher(MarkerArray,'/wavego/live_dense/graph',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
  self.mark=self.create_publisher(MarkerArray,'/wavego/live_dense/inflation',10)
  self.stat=self.create_publisher(String,'/wavego/live_dense/status',10)
  self.surface_pub=self.create_publisher(String,'/wavego/live_dense/ground_surface',10)
  self.create_timer(.2,self.update_inflation)
  self.worker=threading.Thread(target=self.process,daemon=True);self.worker.start()
 def odom(self,m):
  p=m.pose.pose.position;q=m.pose.pose.orientation;yaw=np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));v=np.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y)
  self.pose=dict(x=p.x,y=p.y,z=p.z,yaw=float(yaw),speed=float(v),received=time.monotonic(),source_stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
  self.pose_history.append(self.pose)
  if self.floor is None:self.floor=p.z-.40
  xyz=np.array([p.x,p.y,p.z],np.float32)
  if not self.graph_nodes or np.linalg.norm(xyz-np.asarray(self.graph_nodes[-1]))>=GRAPH_NODE_SPACING_M:self.graph_nodes.append(xyz)
  self.trajectory.write(json.dumps(dict(stamp=self.pose['source_stamp'],**self.pose))+'\n')
 def scan(self,m):
  ts=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
  if ts<=self.last_stamp:self.rejected+=1;return
  a=pc.read_points(m,skip_nans=True);xyz=np.column_stack([a[k] for k in ['x','y','z']]).astype(np.float32);ok=np.isfinite(xyz).all(1);xyz=xyz[ok]
  if not len(xyz):return
  vals=np.empty((len(xyz),4),np.float32);vals[:,:3]=xyz;vals[:,3]=a['intensity'][ok] if 'intensity' in a.dtype.names else 0
  if self.raw is not None:self.raw.write(vals.tobytes())
  self.index.write(json.dumps(dict(stamp=ts,received=time.time(),offset_points=self.npoints,points=len(xyz),input_frame=m.header.frame_id,export_frame='lio_map'))+'\n')
  if self.scan_pub.get_subscription_count():self.scan_pub.publish(points_message(xyz,self.floor or 0,m.header.stamp,'scan'))
  self.npoints+=len(xyz);self.scans+=1;self.last_stamp=ts;self.last_receive=time.monotonic();self.last_xyz=xyz
  self.floor_scan_history.append((ts,xyz))
  if self.first is None:self.first=ts
  if self.scans%10==0:
   if self.raw is not None:self.raw.flush()
   self.index.flush();self.trajectory.flush()
  try:self.jobs.put_nowait(xyz)
  except queue.Full:self.preview_drops+=1
 def process(self):
  last_pub=0
  while not self.stop_event.is_set() or not self.jobs.empty():
   try:xyz=self.jobs.get(timeout=.3)
   except queue.Empty:continue
   # One observation per voxel per scan; average only measured returns.
   # Independent repeat observations suppress isolated returns and reduce jitter.
   self.cells.update(xyz)
   if time.monotonic()-last_pub<2 or self.stop_event.is_set():continue
   last_pub=time.monotonic();allpts=self.cells.observed_points()
   # The main view retains all measured heights. Architectural cutaway is a
   # separate optional topic, so it cannot silently hide a floor in RViz.
   cutaway_requested=self.cutaway_pub.get_subscription_count()>0
   cutaway=np.empty((0,3),dtype=np.float32)
   if cutaway_requested:
    # Avoid transforming and masking millions of points when the optional
    # architectural view is disabled. The full map always remains unchanged.
    ceiling = max((self.floor or 0)+2.35, (self.pose or {}).get('z',0)+1.9)
    if self.display_transform is None and self.alignment and self.alignment.exists():
     self.display_transform=np.linalg.inv(np.asarray(json.loads(self.alignment.read_text())['matrix']))
    if self.display_transform is not None:
     # Reveal the lower-floor course while retaining the stairwell and upper
     # destination. This affects only the explicitly requested cutaway topic.
     tr=self.display_transform;world=allpts@tr[:3,:3].T+tr[:3,3]
     region=(world[:,0]>-8.5)&(world[:,0]<11.5)&(world[:,1]>-9)&(world[:,1]<7)&(world[:,2]>-1)&(world[:,2]<8.5)
     levels=(world[:,2]<2.5)|((world[:,0]<-1.8)&(world[:,1]<2.2)&(world[:,2]<7.2))
     cutaway=allpts[region&levels]
    else:
     cutaway=allpts[(allpts[:,2]>(self.floor or 0)-2)&(allpts[:,2]<ceiling)]
   # Only the optional cutaway is cropped. Keep every accumulated measured
   # voxel in the full map, including distant surfaces and upper floors.
   display=allpts
   stamp=self.get_clock().now().to_msg()
   self.pub.publish(points_message(display,self.floor or 0,stamp))
   if self.geometry_pub.get_subscription_count():self.geometry_pub.publish(points_message(display,self.floor or 0,stamp,'geometry'))
   if cutaway_requested:self.cutaway_pub.publish(points_message(cutaway,self.floor or 0,stamp))
   self.graph_pub.publish(trajectory_graph(self.graph_nodes,stamp,self.floor or 0))
   adaptive=self.latest_adaptive
   self.status=dict(scans=self.scans,received_points=self.npoints,live_voxels=len(self.cells),display_points=len(display),cutaway_points=len(cutaway),preview_voxel_m=DISPLAY_VOXEL_M,raw_recording_enabled=self.raw is not None,sensor_elapsed_sec=self.last_stamp-(self.first or self.last_stamp),preview_dropped_batches=self.preview_drops,rejected_duplicate_or_old_scans=self.rejected,adaptive=adaptive)
   self.status['display_cutaway']=False
   self.status['display_region_limited']=False
   self.status['minimum_independent_observations']=MIN_OBSERVATIONS
   self.status['map_update_period_sec']=2
   self.status['inflation_update_period_sec']=.2
   self.status['display_resume_source']=str(self.resume_display) if self.resume_display else None
   (self.out/'live_status.json').write_text(json.dumps(self.status,indent=2));self.stat.publish(String(data=json.dumps(self.status)))
   with (self.out/'inflation_events.jsonl').open('a') as f:f.write(json.dumps(dict(time=time.time(),**self.status))+'\n')
 def update_inflation(self):
  scan=self.last_xyz
  if not self.pose_history or scan is None:return
  pose=min(self.pose_history,key=lambda p:abs(p['source_stamp']-self.last_stamp))
  sync=abs(pose['source_stamp']-self.last_stamp)
  if sync>.08 or time.monotonic()-pose['received']>1.5 or time.monotonic()-self.last_receive>1.5:
   self.clear_inflation('stale_or_unsynchronized',source_delta_sec=sync);return
  base=np.array([pose['x'],pose['y'],pose['z']]);delta=scan-base
  near=scan[(np.linalg.norm(delta[:,:2],axis=1)<3.5)&(np.abs(delta[:,2])<2)]
  # Non-repetitive MID360 returns occasionally leave a single floor strip.
  # Fit against a bounded two-second window of actual registered returns in
  # the same LIO frame; occupancy and pedestrians remain current-scan data.
  floor_samples=[]
  for ts,xyz in self.floor_scan_history:
   if 0<=self.last_stamp-ts<=2.0:
    d=xyz-base;mask=(np.linalg.norm(d[:,:2],axis=1)<3.5)&(np.abs(d[:,2])<2)
    floor_samples.append(xyz[mask])
  surface=GroundSurface.from_scan(np.concatenate(floor_samples) if floor_samples else near,base);self.surface=surface
  self.surface_pub.publish(String(data=json.dumps(surface.payload(self.last_stamp))))
  if not surface.valid:
   self.clear_inflation(surface.reason,ground=surface.metadata(),source_delta_sec=sync);return
  heights,supported=surface.height_at(near[:,:2]);relative=near[:,2]-heights
  # Only the derived clearance grid is flattened. Registered map returns stay
  # unchanged; ceilings/other floors are excluded using measured local ground.
  # An occupied wall column need not contain a ground return underneath it.
  # Use the measured plane there for height classification, then only draw
  # the clearance cells where independent floor support exists.
  selected=(relative>-.08)&(relative<1.65)
  flat=near[selected].copy();flat[:,2]=relative[selected]+surface.floor_z
  try:
   g=build_grid(flat,np.zeros(len(flat),np.uint8),surface.floor_z);adaptive,grid=analyze(g,[pose['x'],pose['y']],pose['yaw'],min(pose['speed'],1.2),settings=INFLATION_SETTINGS)
   adaptive.update(ground=surface.metadata(),ground_measurement_window_sec=2.,source_delta_sec=sync,source_stamp=self.last_stamp,display_frame='lio_map',display_roi_m=3.5)
   self.latest_adaptive=adaptive;self.radii.append(adaptive['radius']);self.publish_markers(g,grid,adaptive,surface)
  except ValueError as exc:self.clear_inflation(str(exc),ground=surface.metadata(),source_delta_sec=sync)
 def clear_inflation(self,reason,**details):
  self.latest_adaptive=dict(display_valid=False,reason=reason,**details)
  self.mark.publish(MarkerArray(markers=[Marker(header=Header(frame_id='lio_map',stamp=self.get_clock().now().to_msg()),action=Marker.DELETEALL)]))
 def publish_markers(self,g,grids,m,surface):
  h=Header(frame_id='lio_map',stamp=RosTime(seconds=m['source_stamp']).to_msg());items=[Marker(header=h,action=Marker.DELETEALL)]
  for ns,mask,alpha in [('live_adaptive_cells',grids['inflated']&~g['occupancy'],.12),('live_hard_cells',(g['distance']<=m['hard_radius'])&~g['occupancy'],.20)]:
   ys,xs=np.nonzero(mask);xy=np.column_stack(((xs+.5)*g['res']+g['origin'][0],(ys+.5)*g['res']+g['origin'][1]));z,supported=surface.height_at(xy)
   supported&=np.linalg.norm(xy-np.asarray(m['position']),axis=1)<3.0
   cell=Marker(header=h,ns=ns,id=1,type=Marker.POINTS,action=Marker.ADD);cell.pose.orientation.w=1.;cell.scale.x=cell.scale.y=g['res'];cell.color=ColorRGBA(r=1.,g=.45,b=.08,a=alpha)
   cell.points=[Point(x=float(x),y=float(y),z=float(zz+.035)) for (x,y),zz in zip(xy[supported],z[supported])];items.append(cell)
  ring=Marker(header=h,ns='robot_adaptive_radius',id=3,type=Marker.LINE_LIST,action=Marker.ADD);ring.pose.orientation.w=1.;ring.scale.x=.025;ring.color=ColorRGBA(r=0.,g=.95,b=1.,a=.85)
  theta=np.linspace(0,2*np.pi,65);xy=np.asarray(m['position'])+m['radius']*np.column_stack((np.cos(theta),np.sin(theta)));z,supported=surface.height_at(xy)
  for i in range(len(xy)-1):
   if supported[i] and supported[i+1]:
    ring.points.extend([Point(x=float(xy[j,0]),y=float(xy[j,1]),z=float(z[j]+.04)) for j in (i,i+1)])
  items.append(ring)
  text=Marker(header=h,ns='live_readout',id=2,type=Marker.TEXT_VIEW_FACING,action=Marker.ADD);text.pose.orientation.w=1.;text.pose.position.x=m['position'][0];text.pose.position.y=m['position'][1];text.pose.position.z=surface.floor_z+1.1;text.scale.z=.14;text.color.r=text.color.g=text.color.b=text.color.a=1.;text.text=f"Measured floor | inflation {m['radius']:.2f} m\nhard {m['hard_radius']:.2f} m | v {m['speed']:.2f} m/s";items.append(text)
  m['display_valid']=True;m['display_cells']=sum(len(item.points) for item in items if item.ns in ('live_adaptive_cells','live_hard_cells'))
  for marker in items:
   if marker.action!=Marker.DELETEALL:marker.lifetime=Duration(sec=1)
  self.mark.publish(MarkerArray(markers=items))
 def close(self):
  self.stop_event.set();self.worker.join(timeout=30)
  if self.raw is not None:self.raw.close()
  self.index.close();self.trajectory.close()
  (self.out/'capture_complete.json').write_text(json.dumps(dict(scans=self.scans,points=self.npoints,first_stamp=self.first,last_stamp=self.last_stamp,voxel_m=DISPLAY_VOXEL_M,preview_drops=self.preview_drops,raw_recording_enabled=self.raw is not None,source='/wavego/slam/registered_points',frame_alias='native LIO map published as lio_map by existing mapping sidecar; no simulator ground-truth pose',adaptive_radius_range=[min(self.radii),max(self.radii)] if self.radii else [],contains_dynamic_history=True),indent=2))

if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--alignment',type=Path);ap.add_argument('--resume-display',type=Path,help='Restore confirmed display voxels from this same live LIO session');ap.add_argument('--record-raw',action='store_true',help='Opt in to large float32 scan recording');args=ap.parse_args();rclpy.init();node=Recorder(args.output,args.alignment,args.record_raw,args.resume_display)
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:node.close();node.destroy_node();rclpy.try_shutdown()
