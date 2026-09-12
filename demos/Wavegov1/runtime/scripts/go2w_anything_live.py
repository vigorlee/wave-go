#!/usr/bin/env python3
"""Live pretrained model inference. Model labels + independently measured depth."""
import argparse,json,time,threading
from pathlib import Path
from collections import deque
import numpy as np,cv2
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage,Image,PointCloud2,PointField
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2
from go2w_mid360_semantics import sensor_world,associate_labels
from std_msgs.msg import String,Header
from go2w_anything_models import Anything,panel,COLORS,LABELS,ROOT
from go2w_dense_depth import project_depth

def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9

def lift_masks(depth,labels,config,pose):
 s=config['robot']['sensors']['depth_sensor'];cam=config['robot']['sensors']['camera']
 if s['position']!=cam['position'] or s['rotation']!=cam['rotation']:raise ValueError('RGB/depth mounts differ; metric semantic cloud suppressed')
 h,w=depth.shape;rh,rw=labels.shape;v,u=np.indices((h,w));fd=w/(2*np.tan(np.deg2rad(s['fov'])/2));fc=rw/(2*np.tan(np.deg2rad(cam['fov'])/2))
 uc=np.rint((u-(w-1)/2)*fc/fd+(rw-1)/2).astype(int);vc=np.rint((v-(h-1)/2)*fc/fd+(rh-1)/2).astype(int)
 good=np.isfinite(depth)&(depth>.15)&(depth<8)&(uc>=0)&(uc<rw)&(vc>=0)&(vc<rh)
 xyz=project_depth(depth,s['fov']).reshape(h,w,3)[good];lab=labels[vc[good],uc[good]]
 r=s['rotation'];xyz=Rotation.from_euler('xyz',[r['roll'],-r['pitch'],-r['yaw']],degrees=True).apply(xyz)
 p=s['position'];xyz+=np.array([p['x'],-p['y'],p['z']])*.01
 q=pose.orientation;t=pose.position;xyz=Rotation.from_quat([q.x,q.y,q.z,q.w]).apply(xyz)+[t.x,t.y,t.z]
 return xyz.astype(np.float32),lab

class Live(Node):
 def __init__(self,out):
  super().__init__('go2w_anything_pretrained');self.latest=None;self.semantic_cache=None;self.midstats={};self.lidars=deque(maxlen=8);self.depths=deque(maxlen=12);self.poses=deque(maxlen=150);self.out=out;out.mkdir(parents=True,exist_ok=True);self.counts={};self.frames=0
  self.create_subscription(CompressedImage,'/image_raw/compressed',lambda m:setattr(self,'latest',(time.time(),m)),qos_profile_sensor_data)
  self.create_subscription(PointCloud2,'/livox/lidar_raw',self.on_lidar,qos_profile_sensor_data)
  self.midpub=self.create_publisher(PointCloud2,'/go2w/anything/mid360_points',qos_profile_sensor_data)
  self.create_subscription(Image,'/go2w/depth/image_raw',lambda m:self.depths.append(m),qos_profile_sensor_data)
  self.create_subscription(Odometry,'/odom/mujoco_odom',lambda m:self.poses.append(m),30)
  self.imgpub=self.create_publisher(Image,'/go2w/anything/annotated_image',qos_profile_sensor_data);self.status=self.create_publisher(String,'/go2w/anything/status',10);self.cloud=self.create_publisher(PointCloud2,'/go2w/anything/semantic_points',qos_profile_sensor_data)
 def on_lidar(self,raw):
  # Geometry follows the sensor clock, independently of model inference.
  self.lidars.append(raw)
  if not self.poses:return
  od=min(list(self.poses),key=lambda x:abs(stamp(x)-stamp(raw)))
  if abs(stamp(od)-stamp(raw))>.1:return
  cfg=json.loads((ROOT/'matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/config/config.json').read_text())
  pts=point_cloud2.read_points(raw,field_names=('x','y','z'),skip_nans=True)
  pts=np.column_stack([pts[k] for k in ('x','y','z')])
  world=sensor_world(pts,cfg['robot']['sensors']['lidar'],od.pose.pose)
  labels=np.zeros(len(world),np.uint8);cache=self.semantic_cache;age=None
  if cache:
   age=stamp(raw)-cache[0]
   if 0<=age<=.6:
    distance,index=cache[1].query(world,distance_upper_bound=.12);good=np.isfinite(distance)
    labels[good]=cache[2][index[good]]
  color=COLORS[labels].astype(np.uint32)
  arr=np.empty(len(world),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4'),('label','u1')])
  for i,k in enumerate(('x','y','z')):arr[k]=world[:,i]
  arr['rgb']=(color[:,0]<<16)|(color[:,1]<<8)|color[:,2];arr['label']=labels
  self.midpub.publish(PointCloud2(header=Header(stamp=raw.header.stamp,frame_id='odom'),height=1,width=len(arr),fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32 if i<3 else PointField.UINT32,count=1) for i,k in enumerate(('x','y','z','rgb'))]+[PointField(name='label',offset=16,datatype=PointField.UINT8,count=1)],point_step=17,row_step=len(arr)*17,data=arr.tobytes(),is_dense=True))
  self.midstats=dict(mid360_points=len(arr),mid360_labeled_points=int((labels>0).sum()),mid360_semantic_age_seconds=age,mid360_geometry_source='/livox/lidar_raw',mid360_association_radius_m=.12,mid360_publish_mode='sensor_callback_independent_of_inference',lidar_sensor_type=cfg['robot']['sensors']['lidar']['sensor_type'],lidar_output_topic='/go2w/anything/mid360_points')
 def process(self,engine,latest):
  received,m=latest;rgb=cv2.imdecode(np.frombuffer(m.data,np.uint8),cv2.IMREAD_COLOR)[:,:,::-1];d,labels,objects,stats=engine.infer(rgb)
  stats.update(frame=self.frames,input_stamp=stamp(m),received_at=received,finished_at=time.time(),input_age_seconds=time.time()-received,objects=objects,labels=LABELS)
  pcloud=None;pose=None;depthmsg=None
  if self.depths and self.poses:
   depthmsg=min(list(self.depths),key=lambda x:abs(stamp(x)-stamp(m)));od=min(list(self.poses),key=lambda x:abs(stamp(x)-stamp(depthmsg)));pose=od.pose.pose
   stats['rgb_depth_delta_seconds']=abs(stamp(depthmsg)-stamp(m));stats['depth_odom_delta_seconds']=abs(stamp(od)-stamp(depthmsg))
   if stats['rgb_depth_delta_seconds']<.15 and stats['depth_odom_delta_seconds']<.1:
    cfg=json.loads((ROOT/'matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/config/config.json').read_text());s=cfg['robot']['sensors']['depth_sensor']
    try:
     z=np.frombuffer(depthmsg.data,dtype='<f4').reshape(depthmsg.height or s['height'],depthmsg.width or s['width']);xyz,lab=lift_masks(z,labels,cfg,pose);pcloud=(xyz,lab)
     color=COLORS[lab].astype(np.uint32);arr=np.empty(len(xyz),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4'),('label','u1')])
     for i,k in enumerate(('x','y','z')):arr[k]=xyz[:,i]
     arr['rgb']=(color[:,0]<<16)|(color[:,1]<<8)|color[:,2];arr['label']=lab
     self.cloud.publish(PointCloud2(header=Header(stamp=depthmsg.header.stamp,frame_id='odom'),height=1,width=len(arr),fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32 if i<3 else PointField.UINT32,count=1) for i,k in enumerate(('x','y','z','rgb'))]+[PointField(name='label',offset=16,datatype=PointField.UINT8,count=1)],point_step=17,row_step=len(arr)*17,data=arr.tobytes(),is_dense=True))
     self.semantic_cache=(stamp(depthmsg),cKDTree(xyz),lab.copy())
     stats.update(self.midstats)
     stats['metric_cloud_points']=len(arr);stats['metric_cloud_depth_source']='UE_depth_sensor_not_DepthAnything';stats['metric_cloud_semantics']='GroundingDINO_plus_SAM2'
    except ValueError as exc:stats['metric_cloud_skip_reason']=str(exc)
  stats.update(finished_at=time.time(),input_age_seconds=time.time()-received)
  vis=panel(rgb,d,labels,objects,stats);self.imgpub.publish(Image(header=m.header,height=vis.shape[0],width=vis.shape[1],encoding='rgb8',step=vis.shape[1]*3,data=vis.tobytes()))
  stats['semantic_pixel_counts']={LABELS[k]:int((labels==k).sum()) for k in range(len(LABELS))};self.status.publish(String(data=json.dumps(stats)));self.frames+=1
  with (self.out/'frames.jsonl').open('a') as f:f.write(json.dumps(stats)+'\n')
  region='stationary'
  if pose:
   x,y=pose.position.x,pose.position.y
   for name,lo,hi in [('stairs',1,4),('stairs_down',6,9),('flat',10,13),('ramp',13,18),('dynamic',19,26)]:
    if abs(x)<2 and lo<y<hi:region=name;break
  n=self.counts.get(region,0)
  if n<8 and self.frames%4==0:
   dest=self.out/f'{region}_{n:02d}';dest.mkdir(exist_ok=True);cv2.imwrite(str(dest/'models.png'),vis[:,:,::-1]);cv2.imwrite(str(dest/'rgb.jpg'),rgb[:,:,::-1]);np.savez_compressed(dest/'predictions.npz',relative_depth=d,semantic_labels=labels)
   if pcloud is not None:np.savez_compressed(dest/'semantic_cloud.npz',points=pcloud[0],labels=pcloud[1])
   (dest/'metadata.json').write_text(json.dumps(stats,indent=2));self.counts[region]=n+1

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);args=ap.parse_args();engine=Anything(temporal=True);rclpy.init();node=Live(args.output);thread=threading.Thread(target=rclpy.spin,args=(node,),daemon=True);thread.start();last=None
 try:
  while rclpy.ok():
   item=node.latest
   if item is None or item is last:time.sleep(.02);continue
   last=item
   if time.time()-item[0]>1:continue
   node.process(engine,item)
 except KeyboardInterrupt:pass
 finally:rclpy.shutdown()
if __name__=='__main__':main()
