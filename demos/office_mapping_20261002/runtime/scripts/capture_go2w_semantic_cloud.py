#!/usr/bin/python3
"""Read-only live raw/fused cloud capture and scene-prior semantic diagnostics."""
import argparse,json,time,signal
from collections import deque,Counter
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2,PointField,CompressedImage
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry,Path as NavPath
from std_msgs.msg import String,Header
from tf2_ros import Buffer,TransformListener,TransformException
from visualization_msgs.msg import Marker,MarkerArray
from go2w_semantic_geometry import SceneSemantics,CLASSES,COLORS
from go2w_world_obstacle_cloud import rotate

class Capture(Node):
    def __init__(self,out,scene):
        super().__init__('go2w_semantic_evidence');self.out=out;out.mkdir(parents=True,exist_ok=True)
        self.scene=SceneSemantics(scene);self.odom_history=deque(maxlen=1000);self.raw=deque(maxlen=12);self.fused=None;self.image=None;self.pose=None;self.plan=None
        self.dense=None;self.dense_status={};self.algorithm=None;self.perception={};self.world={};self.mission={};self.cosmos={};self.route={};self.counts=Counter();self.last_capture={};self.last_raw=0.;self.done=False;self.stats=Counter();self.path=[]
        self.tf=Buffer(cache_time=Duration(seconds=10));self.listener=TransformListener(self.tf,self)
        for topic,fn in [('/livox/lidar_raw',self.on_raw),('/livox/lidar',self.on_fused)]:self.create_subscription(PointCloud2,topic,fn,qos_profile_sensor_data)
        self.create_subscription(CompressedImage,'/image_raw/compressed',lambda m:setattr(self,'image',(time.time(),m)),qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom/mujoco_odom',self.on_odom,20)
        self.create_subscription(PointCloud2,'/go2w/dense/points_world',lambda m:setattr(self,'dense',(time.time(),m)),qos_profile_sensor_data)
        self.create_subscription(String,'/go2w/dense/status',lambda m:self.on_json('dense_status',m),10)
        self.create_subscription(PointCloud2,'/go2w/perception/semantic_points',lambda m:setattr(self,'algorithm',(time.time(),m)),qos_profile_sensor_data)
        self.create_subscription(String,'/go2w/perception/status',lambda m:self.on_json('perception',m),10)
        self.create_subscription(NavPath,'/plan',lambda m:setattr(self,'plan',m),10)
        for topic,name in [('/go2w/world_model/status','world'),('/cosmos_vln/mission_status','mission'),('/cosmos_vln/status','cosmos'),('/cosmos_vln/route_command','route')]:
            self.create_subscription(String,topic,lambda m,n=name:self.on_json(n,m),20)
        self.pub=self.create_publisher(PointCloud2,'/go2w/semantic/points',qos_profile_sensor_data)
        self.mpub=self.create_publisher(MarkerArray,'/go2w/semantic/labels',10)
        self.create_timer(.5,self.capture)
        (out/'schema.json').write_text(json.dumps({'frame':'odom','class_ids':CLASSES,'colors_rgb':COLORS.tolist(),'semantic_source':'scene_geometry_prior_projected_onto_observed_raw_points','dynamic_source':'world_model_status/mujoco_qpos_relay','objects':self.scene.objects},indent=2))
    def on_json(self,n,m):
        try:setattr(self,n,{'received_at':time.time(),'payload':json.loads(m.data)})
        except ValueError:return
        if n in ('perception','dense_status'):
            with (self.out/(n+'.jsonl')).open('a') as f:f.write(json.dumps(getattr(self,n))+'\n')
    def on_odom(self,m):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        self.pose={'received_at':time.time(),'x':p.x,'y':p.y,'z':p.z,'quaternion_xyzw':[q.x,q.y,q.z,q.w],'stamp_ns':m.header.stamp.sec*10**9+m.header.stamp.nanosec,'yaw':float(np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))}
        self.odom_history.append(self.pose.copy())
        if not self.path or time.time()-self.path[-1][0]>.2:self.path.append([time.time(),p.x,p.y,p.z])
    def convert(self,m,physical=False):
        try:
            fallback=False
            try:t=self.tf.lookup_transform('odom',m.header.frame_id,Time.from_msg(m.header.stamp))
            except TransformException:
                fallback=True;t=self.tf.lookup_transform('odom',m.header.frame_id,Time())
            a=point_cloud2.read_points(m,field_names=('x','y','z'),skip_nans=True)
            xyz=np.column_stack([a[k] for k in ('x','y','z')]) if isinstance(a,np.ndarray) and a.dtype.names else np.array(list(a),dtype=np.float32).reshape(-1,3)
            q=t.transform.rotation;p=t.transform.translation
            world=rotate(np.array([q.x,q.y,q.z,q.w]),xyz)+np.array([p.x,p.y,p.z])
            physical_pose=None
            if physical:
                if not self.odom_history:return None
                stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
                physical_pose=min(self.odom_history,key=lambda o:abs(o['stamp_ns']-stamp))
                if abs(physical_pose['stamp_ns']-stamp)>200000000:
                    self.stats['odom_sync_skips']+=1;return None
                ext=self.tf.lookup_transform('base_link',m.header.frame_id,Time())
                eq=ext.transform.rotation;ep=ext.transform.translation
                body=rotate(np.array([eq.x,eq.y,eq.z,eq.w]),xyz)+np.array([ep.x,ep.y,ep.z])
                world=rotate(np.array(physical_pose['quaternion_xyzw']),body)+np.array([physical_pose[k] for k in ('x','y','z')])
            return {'points':world.astype(np.float32),'received_at':time.time(),'stamp_ns':m.header.stamp.sec*10**9+m.header.stamp.nanosec,'frame':m.header.frame_id,'tf_latest_fallback':fallback,'tf_stamp_ns':t.header.stamp.sec*10**9+t.header.stamp.nanosec,'physical_pose':physical_pose,'transform_mode':'full_3d_odometry_plus_static_lidar_extrinsic' if physical else 'navigation_tf'}
        except (TransformException,ValueError) as exc:self.stats['transform_skips']+=1;return None
    def on_fused(self,m):
        self.stats['fused_messages']+=1;self.fused=(time.time(),m)
    def on_raw(self,m):
        self.stats['raw_messages']+=1
        if time.time()-self.last_raw<.2:return
        self.last_raw=time.time();frame=self.convert(m,physical=True)
        if frame is None:return
        self.raw.append(frame)
        actors=self.world.get('payload',{}).get('dynamic_actors',[]) if time.time()-self.world.get('received_at',0)<.5 else []
        labels=self.scene.label(frame['points'],actors)
        rgb=COLORS[labels].astype(np.uint32);packed=(rgb[:,0]<<16)|(rgb[:,1]<<8)|rgb[:,2]
        dtype=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4'),('label','u1')]);arr=np.empty(len(labels),dtype=dtype)
        for k,j in zip(('x','y','z'),range(3)):arr[k]=frame['points'][:,j]
        arr['rgb']=packed;arr['label']=labels
        cloud=PointCloud2(header=Header(stamp=m.header.stamp,frame_id='odom'),height=1,width=len(arr),fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32 if i<3 else PointField.UINT32,count=1) for i,k in enumerate(('x','y','z','rgb'))]+[PointField(name='label',offset=16,datatype=PointField.UINT8,count=1)],is_bigendian=False,point_step=17,row_step=17*len(arr),data=arr.tobytes(),is_dense=True)
        self.pub.publish(cloud);self.stats['semantic_messages']+=1
        markers=[]
        for i,o in enumerate(self.scene.objects):
            if not self.pose or np.linalg.norm(np.array(o['position'][:2])-[self.pose['x'],self.pose['y']])>6:continue
            if o['class_id']==1 and not o['name'].endswith(('OBS5','OBS11')):continue
            mk=Marker();mk.header=cloud.header;mk.ns='scene_prior';mk.id=i;mk.type=Marker.TEXT_VIEW_FACING;mk.action=Marker.ADD;mk.pose.position.x=o['position'][0];mk.pose.position.y=o['position'][1];mk.pose.position.z=o['position'][2]+.7;mk.pose.orientation.w=1.;mk.scale.z=.18;mk.color.r=1.;mk.color.g=1.;mk.color.b=1.;mk.color.a=1.;mk.text=CLASSES[o['class_id']]+' [prior]';mk.lifetime=Duration(seconds=1).to_msg();markers.append(mk)
        self.mpub.publish(MarkerArray(markers=markers))
    def capture(self):
        if not self.pose or not self.raw or not self.fused:return
        now=time.time();p=self.pose;y=p['y'];x=p['x']
        regions=[('stairs_up',1.2,3.7),('stairs_down',7.0,9.0),('flat_slalom',10.7,12.5),('ramp_up',13.1,15.0),('ramp_down',16.2,18.0),('dynamic_crossing',19.0,21.5)]
        region=next((n for n,a,b in regions if a<=y<=b and abs(x)<2),None)
        if not region or now-self.last_capture.get(region,0)<1.5 or self.counts[region]>=10:return
        if now-self.raw[-1]['received_at']>.6 or now-self.fused[0]>.6:return
        fused=self.convert(self.fused[1])
        if fused is None:return
        frames=[f for f in self.raw if now-f['received_at']<1.5];raw=np.concatenate([f['points'] for f in frames]);actors=self.world.get('payload',{}).get('dynamic_actors',[])
        # Temporal accumulation remains raw observation; moving labels use current state and are approximate.
        labels=self.scene.label(raw,actors);idx=self.counts[region];name=f'{region}_{idx:02d}';dest=self.out/name;dest.mkdir()
        np.savez_compressed(dest/'clouds.npz',raw_world=raw,raw_labels=labels,fused_world=fused['points'],robot=np.array([p['x'],p['y'],p['z']]),trajectory=np.array(self.path),plan=np.array([[a.pose.position.x,a.pose.position.y,a.pose.position.z] for a in self.plan.poses]) if self.plan else np.empty((0,3)))
        meta={'region':region,'captured_at':now,'robot':p,'raw_point_count':len(raw),'fused_point_count':len(fused['points']),'raw_frames':[{k:v for k,v in f.items() if k!='points'} for f in frames],'fused':{k:v for k,v in fused.items() if k!='points'},'world':self.world,'mission':self.mission,'cosmos':self.cosmos,'route':self.route,'class_counts':{CLASSES[k]:int((labels==k).sum()) for k in CLASSES},'semantic_source':'scene_prior_geometric_association_not_learned_segmentation','raw_transform':'full_3d_odometry_plus_static_extrinsic','fused_transform':'existing_planar_navigation_tf','accumulation_seconds':1.5}
        if self.image:
            received,img=self.image;meta['camera']={'received_at':received,'age_seconds':now-received,'stamp_ns':img.header.stamp.sec*10**9+img.header.stamp.nanosec,'format':img.format};(dest/'camera.jpg').write_bytes(bytes(img.data))
        if self.algorithm and now-self.algorithm[0]<.8:
            received,algorithm=self.algorithm
            data=point_cloud2.read_points(algorithm,field_names=('x','y','z','label'),skip_nans=True)
            np.savez_compressed(dest/'algorithm.npz',points=np.column_stack([data[k] for k in ('x','y','z')]),labels=data['label'])
            meta['algorithm']={'received_at':received,'age_seconds':now-received,'stamp_ns':algorithm.header.stamp.sec*10**9+algorithm.header.stamp.nanosec,'status':self.perception}
        if self.dense and now-self.dense[0]<.8:
            received,dm=self.dense;da=point_cloud2.read_points(dm,field_names=('x','y','z'),skip_nans=True)
            np.savez_compressed(dest/'dense_depth.npz',points=np.column_stack([da[k] for k in ('x','y','z')]))
            meta['dense_depth']={'source':'UE_axial_depth_pixels','age_seconds':now-received,'status':self.dense_status}
        (dest/'metadata.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2));self.last_capture[region]=now;self.counts[region]+=1;self.get_logger().info(f'saved {name}: raw={len(raw)} fused={len(fused["points"])}')
    def finish(self):
        (self.out/'capture_summary.json').write_text(json.dumps({'snapshots':dict(self.counts),'topic_messages':dict(self.stats),'finished_at':time.time()},indent=2))

def main():
    a=argparse.ArgumentParser();a.add_argument('--output',type=Path,required=True);a.add_argument('--scene',type=Path,required=True);args=a.parse_args();rclpy.init();n=Capture(args.output,args.scene)
    signal.signal(signal.SIGTERM,lambda *_:setattr(n,'done',True));signal.signal(signal.SIGINT,lambda *_:setattr(n,'done',True))
    try:
        while rclpy.ok() and not n.done:rclpy.spin_once(n,timeout_sec=.2)
    finally:n.finish();n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
