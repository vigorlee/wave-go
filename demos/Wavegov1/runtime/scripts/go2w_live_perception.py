#!/usr/bin/python3
"""Online geometry/temporal semantics. Never reads scene XML or prior labels."""
import time,json,math
import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

LABELS={0:'unclassified',1:'ground/landing',2:'stairs candidate',3:'ramp candidate',4:'wall/large structure',5:'obstacle cluster',6:'moving cluster'}
PALETTE=np.array([[115,135,158],[77,194,134],[55,176,255],[190,137,249],[184,198,209],[255,169,60],[255,67,134]],dtype=np.uint8)

class GeometryTracker:
    def __init__(self):self.tracks={};self.next_id=1
    def process(self,points,robot,now):
        start=time.perf_counter();p=points[np.isfinite(points).all(axis=1)];p=p[(np.linalg.norm(p[:,:2]-robot[:2],axis=1)<8)&(p[:,2]>-.2)&(p[:,2]<robot[2]+2.4)]
        if len(p)<24:return p,np.zeros(len(p),np.uint8),[],{'processing_ms':0.,'point_count':len(p)}
        _,idx=np.unique(np.floor(p/.075).astype(np.int32),axis=0,return_index=True);p=p[np.sort(idx)]
        tree=cKDTree(p);dist,ids=tree.query(p,k=min(18,len(p)),workers=1);neighbors=p[ids];center=neighbors.mean(axis=1);delta=neighbors-center[:,None,:];cov=np.einsum('nki,nkj->nij',delta,delta)/neighbors.shape[1];eig,vec=np.linalg.eigh(cov);nz=np.abs(vec[:,2,0]);curvature=eig[:,0]/np.maximum(eig.sum(axis=1),1e-8);spread=neighbors[:,:,2].max(axis=1)-neighbors[:,:,2].min(axis=1)
        lab=np.zeros(len(p),np.uint8);near_floor=p[:,2]<robot[2]+.25
        lab[(nz>.94)&(curvature<.045)&near_floor]=1
        lab[(nz>.76)&(nz<.99)&(curvature<.035)&near_floor]=3
        # A local height discontinuity next to a roughly horizontal patch is
        # only a stair candidate, not a semantic-network stair prediction.
        lab[(nz>.72)&(spread>.15)&(spread<.55)&near_floor]=2
        lab[(nz<.35)&(spread>.30)]=4
        floor=float(np.percentile(p[near_floor,2],15)) if near_floor.any() else robot[2]-.4
        elevated=(p[:,2]>floor+.16)&~np.isin(lab,[1,2,3]);indices=np.flatnonzero(elevated);objects=[]
        if len(indices)>8:
            q=p[indices];pairs=cKDTree(q[:,:2]).query_pairs(.24,output_type='ndarray');n=len(q)
            graph=coo_matrix((np.ones(len(pairs)*2),(np.r_[pairs[:,0],pairs[:,1]],np.r_[pairs[:,1],pairs[:,0]])),shape=(n,n));_,components=connected_components(graph,directed=False)
            for cid in np.unique(components):
                group=indices[components==cid]
                if len(group)<10:continue
                lo=p[group].min(axis=0);hi=p[group].max(axis=0);extent=hi-lo
                if extent[0]>.95 or extent[1]>.95 or extent[2]<.2 or extent[2]>2.3:continue
                c=(lo+hi)/2;objects.append({'center':c,'low':lo,'high':hi,'indices':group})
        used=set();output=[]
        for obj in objects:
            c=obj['center'];options=[(np.linalg.norm(c[:2]-t['center'][:2]),i) for i,t in self.tracks.items() if i not in used and now-t['time']<1.5];best=min(options,default=(float('inf'),None))
            if best[0]<.6:
                tid=best[1];old=self.tracks[tid];dt=max(.05,now-old['time']);vel=.65*old['velocity']+.35*(c[:2]-old['center'][:2])/dt;age=old['age']+1;origin=old['origin'];moving=age>=4 and np.linalg.norm(c[:2]-origin)>.22 and np.linalg.norm(vel)>.12
            else:tid=self.next_id;self.next_id+=1;vel=np.zeros(2);age=1;origin=c[:2].copy();moving=False
            used.add(tid);self.tracks[tid]={'center':c,'time':now,'velocity':vel,'age':age,'origin':origin};lab[obj['indices']]=6 if moving else 5
            output.append({'id':tid,'label':'moving cluster' if moving else 'obstacle cluster','center':c.tolist(),'low':obj['low'].tolist(),'high':obj['high'].tolist(),'velocity_xy':vel.tolist(),'speed_mps':float(np.linalg.norm(vel)),'track_frames':age,'point_count':len(obj['indices']),'robot_center_distance_m':float(np.linalg.norm(c[:2]-robot[:2]))})
        self.tracks={i:t for i,t in self.tracks.items() if now-t['time']<2.}
        return p,lab,output,{'processing_ms':(time.perf_counter()-start)*1000,'point_count':len(p),'class_counts':{LABELS[k]:int((lab==k).sum()) for k in LABELS}}

def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2,PointField
    from sensor_msgs_py import point_cloud2
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker,MarkerArray
    from rclpy.duration import Duration
    class Live(Node):
        def __init__(self):
            super().__init__('go2w_live_geometry_semantics');self.engine=GeometryTracker();self.robot=np.array([0.,0.,.4]);self.last=0.;self.frame=0
            self.create_subscription(Odometry,'/odom/mujoco_odom',self.odom,10)
            # Input is actual depth-pixel XYZ with full 3D ego-motion compensation.
            # This algorithm never reads scene XML or prior label topics.
            self.create_subscription(PointCloud2,'/go2w/dense/points_world',self.cloud,qos_profile_sensor_data)
            self.pub=self.create_publisher(PointCloud2,'/go2w/perception/semantic_points',qos_profile_sensor_data);self.markers=self.create_publisher(MarkerArray,'/go2w/perception/markers',10);self.status=self.create_publisher(String,'/go2w/perception/status',10)
        def odom(self,m):p=m.pose.pose.position;self.robot=np.array([p.x,p.y,p.z])
        def cloud(self,m):
            received=time.time()
            if received-self.last<.18:return
            dt=received-self.last;self.last=received
            raw=point_cloud2.read_points(m,field_names=('x','y','z'),skip_nans=True);xyz=np.column_stack([raw[k] for k in ('x','y','z')])
            p,l,objects,stats=self.engine.process(xyz,self.robot,received)
            if len(p):
                dense=xyz[np.isfinite(xyz).all(axis=1)&(np.linalg.norm(xyz[:,:2]-self.robot[:2],axis=1)<8)&(xyz[:,2]>-.2)&(xyz[:,2]<self.robot[2]+2.4)]
                _,nearest=cKDTree(p).query(dense,workers=1);l=l[nearest];p=dense
            stats['display_points']=len(p)
            color=PALETTE[l].astype(np.uint32);a=np.empty(len(p),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4'),('label','u1')])
            for i,k in enumerate(('x','y','z')):a[k]=p[:,i]
            a['rgb']=(color[:,0]<<16)|(color[:,1]<<8)|color[:,2];a['label']=l
            fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32 if i<3 else PointField.UINT32,count=1) for i,k in enumerate(('x','y','z','rgb'))]+[PointField(name='label',offset=16,datatype=PointField.UINT8,count=1)]
            self.pub.publish(PointCloud2(header=m.header,height=1,width=len(p),fields=fields,is_bigendian=False,point_step=17,row_step=17*len(p),data=a.tobytes(),is_dense=True))
            markers=[]
            for o in objects:
                for kind in ('box','text','arrow'):
                    mk=Marker();mk.header=m.header;mk.ns=kind;mk.id=o['id'];mk.action=Marker.ADD;mk.pose.orientation.w=1.;mk.lifetime=Duration(seconds=.7).to_msg();mk.color.r=1.;mk.color.g=.25 if o['label'].startswith('moving') else .65;mk.color.b=.4 if o['label'].startswith('moving') else .15;mk.color.a=.15 if kind=='box' else 1.
                    c=o['center'];mk.pose.position.x=c[0];mk.pose.position.y=c[1];mk.pose.position.z=c[2]
                    if kind=='box':mk.type=Marker.CUBE;mk.scale.x=max(.05,o['high'][0]-o['low'][0]);mk.scale.y=max(.05,o['high'][1]-o['low'][1]);mk.scale.z=max(.05,o['high'][2]-o['low'][2])
                    elif kind=='text':mk.type=Marker.TEXT_VIEW_FACING;mk.pose.position.z=o['high'][2]+.2;mk.scale.z=.11;mk.text=f"#{o['id']} {o['label']}\n{o['speed_mps']:.2f}m/s  {o['robot_center_distance_m']:.2f}m"
                    else:
                        if o['speed_mps']<.12:continue
                        mk.type=Marker.ARROW;mk.pose.position.z=o['high'][2]+.4;angle=math.atan2(*o['velocity_xy'][::-1]);mk.pose.orientation.z=math.sin(angle/2);mk.pose.orientation.w=math.cos(angle/2);mk.scale.x=min(1.5,o['speed_mps']*2);mk.scale.y=.05;mk.scale.z=.08
                    markers.append(mk)
            hud=Marker();hud.header=m.header;hud.ns='algorithm_status';hud.id=0;hud.type=Marker.TEXT_VIEW_FACING;hud.pose.position.x=self.robot[0];hud.pose.position.y=self.robot[1]+1.;hud.pose.position.z=self.robot[2]+1.5;hud.pose.orientation.w=1.;hud.scale.z=.105;hud.color.r=.3;hud.color.g=1.;hud.color.b=.8;hud.color.a=1.;hud.text=f"DENSE DEPTH + TRACKING\n{1/dt:.1f} Hz | {len(p)} pts | {stats['processing_ms']:.0f} ms";hud.lifetime=Duration(seconds=.7).to_msg();markers.append(hud)
            self.markers.publish(MarkerArray(markers=markers));self.frame+=1;self.status.publish(String(data=json.dumps({'received_at':received,'frame':self.frame,'observed_hz':1/dt,'algorithm':'voxel_knn_normals_height_discontinuities_xy_components_temporal_tracking','semantic_source':'geometry_heuristics_not_learned_semantic_network','uses_scene_prior':False,'input_source':'UE_dense_depth_pixels','objects':objects,**stats})))
    rclpy.init();n=Live()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
