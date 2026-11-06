#!/usr/bin/env python3
"""Measured LIO geometry + synchronized GSAM metric labels in RViz. No navigation."""
import argparse,json,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,PointField
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2 as pc
from std_msgs.msg import Header,String
from visualization_msgs.msg import Marker,MarkerArray
from builtin_interfaces.msg import Duration
from go2w_semantic_map_core import SemanticVoxels,Tracks,cluster_obstacles,COLORS,LABELS,voxel_sample

def seconds(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9

def unpack(m,semantic=False):
    a=pc.read_points(m,skip_nans=True);xyz=np.column_stack([a[k] for k in ('x','y','z')]).astype(np.float32)
    valid=np.isfinite(xyz).all(axis=1)
    return (xyz[valid],a['label'][valid].astype(np.uint8)) if semantic else xyz[valid]

def cloud(xyz,labels,header):
    a=np.zeros(len(xyz),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4'),('label','<u4')])
    for i,k in enumerate(('x','y','z')):a[k]=xyz[:,i]
    c=COLORS[labels].astype(np.uint32);a['rgb']=(c[:,0]<<16)|(c[:,1]<<8)|c[:,2];a['label']=labels
    fields=[PointField(name=k,offset=i*4,datatype=7 if i<3 else 6,count=1) for i,k in enumerate(a.dtype.names)]
    return PointCloud2(header=header,height=1,width=len(a),fields=fields,point_step=20,row_step=20*len(a),data=a.tobytes(),is_dense=True),a

class SemanticMap(Node):
    def __init__(self,out):
        super().__init__('wavego_semantic_map');self.out=out;out.mkdir(parents=True,exist_ok=True)
        self.memory=SemanticVoxels();self.tracks=Tracks();self.scan=None;self.map=None;self.pose=None;self.pending=None;self.last=-1;self.map_at=0;self.frames=0;self.rejected=0;self.person_history={};self.max_counts={};self.status={};self.capture_counts={};self.capture_at={}
        latch=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry,'/wavego/navigation/odom',lambda m:setattr(self,'pose',m),qos_profile_sensor_data)
        self.create_subscription(PointCloud2,'/wavego/slam/registered_points',lambda m:setattr(self,'scan',m),qos_profile_sensor_data)
        self.create_subscription(PointCloud2,'/wavego/mapping/points',lambda m:setattr(self,'map',m),latch)
        self.create_subscription(PointCloud2,'/go2w/anything/semantic_points',self.semantic,qos_profile_sensor_data)
        self.map_pub=self.create_publisher(PointCloud2,'/wavego/semantic/map',latch)
        self.live_pub=self.create_publisher(PointCloud2,'/wavego/semantic/live_points',qos_profile_sensor_data)
        self.markers=self.create_publisher(MarkerArray,'/wavego/semantic/objects',10)
        self.status_pub=self.create_publisher(String,'/wavego/semantic/status',10)
        self.create_timer(.5,self.tick)
    def semantic(self,m):
        now=self.get_clock().now().nanoseconds*1e-9
        if m.header.frame_id!='lio_map' or not 0<=now-seconds(m)<=1.0:self.rejected+=1;return
        self.pending=m
    def tick(self):
        started=time.monotonic();now=self.get_clock().now().nanoseconds*1e-9
        if self.pending is not None:
            m=self.pending;self.pending=None
            xyz,lab=unpack(m,True);self.memory.update(xyz,lab,seconds(m))
            for p in xyz[lab==4][::4]:self.person_history[tuple(np.floor(p/.2).astype(int))]=now
            if len(self.person_history)>50000:
                for k in list(self.person_history)[:len(self.person_history)-50000]:del self.person_history[k]
        stale=self.scan is None or not 0<=now-seconds(self.scan)<=1.5
        objects=[];header=Header(stamp=self.get_clock().now().to_msg(),frame_id='lio_map')
        if not stale and seconds(self.scan)!=self.last:
            # Registered cloud uses the native LIO world frame; the mapping sidecar
            # exposes that same coordinate system under lio_map.
            xyz=unpack(self.scan);idx=voxel_sample(xyz);xyz=xyz[idx]
            lab,support=self.memory.assign(xyz,seconds(self.scan))
            roi=np.ones(len(xyz),bool)
            if self.pose is not None:
                p=self.pose.pose.pose.position;delta=xyz-[p.x,p.y,p.z];roi=(np.linalg.norm(delta[:,:2],axis=1)<12)&(delta[:,2]>-1.5)&(delta[:,2]<2.5)
            objects=self.tracks.update(cluster_obstacles(xyz[roi],lab[roi],support[roi]),seconds(self.scan));self.last=seconds(self.scan);self.frames+=1
            self.live_pub.publish(cloud(xyz,lab,header)[0])
        if stale:self.live_pub.publish(cloud(np.empty((0,3)),np.empty(0,np.uint8),header)[0])
        markers=[Marker(header=header,action=Marker.DELETEALL)]
        for obj in sorted(objects,key=lambda o:(o['class_id']==0,-o['points']))[:16]:
            for text in (False,True):
                m=Marker(header=header,ns='labels' if text else 'boxes',id=obj['id'],type=Marker.TEXT_VIEW_FACING if text else Marker.CUBE,action=Marker.ADD)
                m.pose.position.x,m.pose.position.y,m.pose.position.z=map(float,obj['center']);m.pose.orientation.w=1.
                c=COLORS[obj['class_id']]/255.;m.color.r,m.color.g,m.color.b=map(float,c);m.color.a=1. if text else .15
                m.lifetime=Duration(sec=2)
                if text:
                    m.pose.position.z+=obj['size'][2]/2+.35;m.scale.z=.4
                    m.text=f"{obj['label'].upper()} #{obj['id']}\n{obj['points']} pts | "+("vision + LiDAR" if obj['class_id']==4 else f"vote {obj['vote_support']:.0%}")
                else:m.scale.x,m.scale.y,m.scale.z=map(float,obj['size'])
                markers.append(m)
        self.markers.publish(MarkerArray(markers=markers))
        for label in (1,2,3,4):
            if any(o['class_id']==label for o in objects) and self.capture_counts.get(label,0)<3 and now-self.capture_at.get(label,0)>5:
                n=self.capture_counts.get(label,0);dest=self.out/f'{LABELS[label]}_{n:02d}';dest.mkdir(exist_ok=True)
                np.savez_compressed(dest/'live_points.npz',points=xyz,labels=lab)
                (dest/'objects.json').write_text(json.dumps(dict(time=now,frame='lio_map',objects=objects),indent=2)+'\n')
                self.capture_counts[label]=n+1;self.capture_at[label]=now
        counts={name:sum(o['label']==name for o in objects) for name in LABELS}
        for k,v in counts.items():self.max_counts[k]=max(v,self.max_counts.get(k,0))
        self.status=dict(state='sensor_stale' if stale else 'live',frame='lio_map',frames=self.frames,objects=objects,cluster_counts=counts,max_simultaneous_clusters=self.max_counts,semantic_voxels=len(self.memory.cells),rejected_semantic_frames=self.rejected,geometry_source='raw LiDAR + IMU LIO',semantic_source='Grounding DINO + SAM2 + synchronized metric UE depth + LIO pose',ground_truth_pose_used=False,voxel_m=.12,association_m=.18,semantic_accuracy_measured=False,absolute_map_accuracy_measured=False,loop_closure=False,processing_ms=(time.monotonic()-started)*1000)
        self.status_pub.publish(String(data=json.dumps(self.status)))
        with (self.out/'events.jsonl').open('a') as f:f.write(json.dumps(dict(time=now,**self.status))+'\n')
        if self.map is not None and self.map.header.frame_id=='lio_map' and time.monotonic()-self.map_at>3:
            xyz=unpack(self.map);xyz=xyz[voxel_sample(xyz)]
            # Remove places directly observed as people from the static export.
            keep=np.array([tuple(k) not in self.person_history for k in np.floor(xyz/.2).astype(int)])
            xyz=xyz[keep];labels,_=self.memory.assign(xyz,now,live=False)
            msg,a=cloud(xyz,labels,header);self.map_pub.publish(msg)
            np.savez_compressed(self.out/'semantic_map.npz',points=xyz,labels=labels)
            h=f'VERSION .7\nFIELDS x y z rgb label\nSIZE 4 4 4 4 4\nTYPE F F F U U\nCOUNT 1 1 1 1 1\nWIDTH {len(a)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(a)}\nDATA binary\n'
            with (self.out/'semantic_map.pcd').open('wb') as f:f.write(h.encode());f.write(a.tobytes())
            self.status.update(map_points=len(xyz),map_class_counts={n:int((labels==i).sum()) for i,n in enumerate(LABELS)})
            (self.out/'summary.json').write_text(json.dumps(self.status,indent=2)+'\n');self.map_at=time.monotonic()
        (self.out/'objects.json').write_text(json.dumps(dict(time=now,frame='lio_map',objects=objects),indent=2)+'\n')

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);args=ap.parse_args();rclpy.init();node=SemanticMap(args.output)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
