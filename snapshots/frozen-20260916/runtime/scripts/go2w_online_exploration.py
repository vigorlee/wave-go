#!/usr/bin/python3
"""Online LiDAR map and autonomous frontiers, using only estimated LIO poses."""
import argparse,json,math,time,os
from concurrent.futures import ThreadPoolExecutor
from go2w_cosmos_frontier import choose_frontier
from pathlib import Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,PointField,CompressedImage
from sensor_msgs_py import point_cloud2 as pc
from nav_msgs.msg import OccupancyGrid,Odometry,Path as NavPath
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped,TransformStamped
from std_msgs.msg import Header,String
from visualization_msgs.msg import Marker,MarkerArray
from tf2_ros import TransformBroadcaster
from go2w_frontier_grid import FrontierGrid


def rotation(q):
    x,y,z,w=q.x,q.y,q.z,q.w
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


class OnlineExploration(rclpy.node.Node):
    def __init__(self,output,resume=False):
        super().__init__('wavego_online_exploration');self.frame=os.environ.get('GO2W_MAPPING_FRAME','map');self.base_frame=os.environ.get('GO2W_MAPPING_BASE_FRAME','base_link');self.mapping_only=os.environ.get('GO2W_MAPPING_ONLY','0')=='1';self.out=output;output.mkdir(parents=True,exist_ok=True)
        self.cosmos=os.environ.get("GO2W_COSMOS_FARTHEST", "0")=="1"
        self.pool=ThreadPoolExecutor(max_workers=1);self.reasoning=None;self.camera=None;self.camera_time=0.;self.retry_at=0.;self.origin=None;self.forward=None;self.decisions=0;self.proposals=[];self.candidate_count=0
        self.create_subscription(CompressedImage,"/image_raw/compressed",self.on_camera,qos_profile_sensor_data)
        self.grid=FrontierGrid();self.voxels={};self.pose=None;self.floor=None;self.last_odom=0.;self.last_scan=0.
        self.registered=None;self.trail=[];self.visited=[];self.goal_handle=None;self.pending=False;self.goal_time=0.
        self.goals=0;self.completed=0;self.distance=0.;self.scan_count=0;self.started=time.monotonic();self.last_mapping=0.
        self.pose_stamp=None;self.velocity=np.zeros(3);self.yaw_rate=0.;self.previous_yaw=0.
        self.resumed=resume;self.resume_expected=None
        self.target=None;self.canceling=False;self.credited=None;self.sensor_stamp=0.
        latch=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cloud_pub=self.create_publisher(PointCloud2,'/wavego/mapping/points',latch)
        self.map_pub=self.create_publisher(OccupancyGrid,'/map',latch)
        self.trail_pub=self.create_publisher(NavPath,'/wavego/mapping/trajectory',latch)
        self.markers=self.create_publisher(MarkerArray,'/wavego/exploration/frontiers',latch)
        self.status_pub=self.create_publisher(String,'/wavego/exploration/status',latch)
        self.scan_pub=self.create_publisher(PointCloud2,'/livox/lidar',qos_profile_sensor_data)
        self.odom_pub=self.create_publisher(Odometry,'/wavego/navigation/odom',20)
        self.tf=TransformBroadcaster(self);self.action=ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.create_subscription(PointCloud2,'/livox/lidar_raw',self.raw,qos_profile_sensor_data)
        self.create_subscription(PointCloud2,'/wavego/slam/registered_points',self.register,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/wavego/slam/odom',self.odometry,qos_profile_sensor_data)
        self.create_timer(.5,self.tick);self.create_timer(5.,self.save)
        if resume:self.restore_checkpoint()

    def restore_checkpoint(self):
        data=np.load(self.out/'map.npz');xyz=data['points'];grid=data['occupancy']
        self.voxels={tuple(k):v for k,v in zip(np.floor(xyz/.1).astype(int),xyz)}
        self.grid.seen=grid!=-1;self.grid.odds=np.where(grid==100,3.,np.where(grid==0,-2.,0.)).astype(np.float32)
        old=json.loads((self.out/'summary.json').read_text())
        self.goals=old['goals'];self.completed=old['completed_goals'];self.distance=old['distance_m'];self.scan_count=old['registered_scans'];self.decisions=old['cosmos_decisions']
        trace=json.loads((self.out/'trajectory_checkpoint.json').read_text())
        q=trace[0][3:];self.forward=np.array([1-2*(q[1]*q[1]+q[2]*q[2]),2*(q[0]*q[1]+q[2]*q[3])]);self.forward/=np.linalg.norm(self.forward)
        self.origin=np.array(trace[0][:2]);self.floor=trace[0][2]-.4;self.resume_expected=np.array(trace[-1][:3])
        for row in trace:
            p=PoseStamped();p.header.frame_id=self.frame
            p.pose.position.x,p.pose.position.y,p.pose.position.z=row[:3]
            p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w=row[3:]
            self.trail.append(p)
        for row in (self.out/'events.jsonl').read_text().splitlines():
            e=json.loads(row)
            if e['state']=='frontier_selected':self.visited.append(np.array([e['goal']['x'],e['goal']['y']]))
        self.log('checkpoint_resumed',map_points=len(xyz),completed=self.completed)


    def on_camera(self,m):
        self.camera=bytes(m.data);self.camera_time=time.monotonic()

    def raw(self,m):
        a=pc.read_points(m);xyz=np.column_stack([a[k] for k in ('x','y','z')])
        valid=np.isfinite(xyz).all(axis=1)&(np.linalg.norm(xyz,axis=1)>.55)&(np.linalg.norm(xyz,axis=1)<30.)
        # Preserve all measurement fields, including the simulator's actual
        # zero per-point time offsets; no invented scanning timestamps.
        filtered=PointCloud2(header=m.header,height=1,width=int(valid.sum()),fields=m.fields,
                            is_bigendian=m.is_bigendian,point_step=m.point_step,
                            row_step=int(valid.sum())*m.point_step,data=a[valid].tobytes(),is_dense=True)
        self.scan_pub.publish(filtered)

    def odometry(self,m):
        imu=np.array([m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z]);rot=rotation(m.pose.pose.orientation)
        base=imu-rot@np.array([-.02557,0.,.04232]);stamp=m.header.stamp
        if self.resume_expected is not None:
            if np.linalg.norm(base-self.resume_expected)>.5:raise RuntimeError('Checkpoint frame mismatch: resume requires the same running LIO session')
            self.resume_expected=None
        seconds=stamp.sec+stamp.nanosec*1e-9;yaw=math.atan2(rot[1,0],rot[0,0])
        if self.pose is not None:
            delta=float(np.linalg.norm(base-self.pose))
            if delta<.5:self.distance+=delta
            dt=seconds-self.pose_stamp
            if .02<dt<.5:
                self.velocity=.6*self.velocity+.4*(rot.T@(base-self.pose)/dt)
                rate=math.atan2(math.sin(yaw-self.previous_yaw),math.cos(yaw-self.previous_yaw))/dt
                self.yaw_rate=.6*self.yaw_rate+.4*rate
        self.pose_stamp=seconds;self.previous_yaw=yaw
        if self.origin is None:self.origin=base[:2].copy()
        if self.forward is None:self.forward=rot[:2,0]/np.linalg.norm(rot[:2,0])
        self.pose=base;self.imu=imu;self.rot=rot;self.last_odom=time.monotonic()
        if self.floor is None:self.floor=float(base[2]-.40)
        m.header.frame_id=self.frame;m.child_frame_id=self.base_frame
        m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=map(float,base)
        m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.linear.z=map(float,self.velocity)
        m.twist.twist.angular.z=float(self.yaw_rate)
        self.odom_pub.publish(m)
        transform=TransformStamped();transform.header=m.header;transform.child_frame_id=self.base_frame
        transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z=map(float,base)
        transform.transform.rotation=m.pose.pose.orientation;self.tf.sendTransform(transform)
        if not self.trail or np.linalg.norm(base-np.array([self.trail[-1].pose.position.x,self.trail[-1].pose.position.y,self.trail[-1].pose.position.z]))>.08:
            p=PoseStamped(header=m.header,pose=m.pose.pose);self.trail.append(p)

    def register(self,m):
        self.registered=m;self.last_scan=time.monotonic();self.scan_count+=1
        self.sensor_stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9

    def log(self,state,**values):
        event=dict(time=time.time(),state=state,**values)
        with (self.out/'events.jsonl').open('a') as f:f.write(json.dumps(event)+'\n')
        self.get_logger().info(json.dumps(event))

    def tick(self):
        if self.pose is None or self.registered is None:return
        now=time.monotonic();stale=now-self.last_scan>1.5 or now-self.last_odom>1.5 or self.get_clock().now().nanoseconds*1e-9-self.sensor_stamp>1.5
        if stale:
            if self.goal_handle:self.goal_handle.cancel_goal_async();self.goal_handle=None;self.log('paused_sensor_stale')
            return
        if self.last_mapping!=self.last_scan:
            a=pc.read_points(self.registered,field_names=('x','y','z'),skip_nans=True)
            xyz=np.column_stack([a[k] for k in ('x','y','z')]).astype(np.float32)
            xyz=xyz[np.isfinite(xyz).all(axis=1)]
            if not len(xyz):return
            key=np.floor(xyz/.10).astype(np.int32);_,ix=np.unique(key,axis=0,return_index=True)
            for k,p in zip(key[ix],xyz[ix]):self.voxels[tuple(k)]=p
            if len(self.voxels)>500000:
                for k in list(self.voxels)[:len(self.voxels)-500000]:del self.voxels[k]
            sensor=self.imu+self.rot@np.array([.15568,-.02329,.13366])
            self.grid.integrate(xyz,sensor,self.floor);self.last_mapping=self.last_scan
        header=Header(stamp=self.get_clock().now().to_msg(),frame_id=self.frame)
        xyz=np.asarray(list(self.voxels.values()),dtype=np.float32)
        # Blue -> amber height color; the geometry is measured, not a mesh.
        t=np.clip((xyz[:,2]-self.floor)/3.,0,1)
        rgb=((40+210*t).astype(np.uint32)<<16)|((140+45*t).astype(np.uint32)<<8)|(240-200*t).astype(np.uint32)
        arr=np.empty(len(xyz),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')])
        for i,k in enumerate(('x','y','z')):arr[k]=xyz[:,i]
        arr['rgb']=rgb
        fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32 if i<3 else PointField.UINT32,count=1) for i,k in enumerate(('x','y','z','rgb'))]
        self.cloud_pub.publish(PointCloud2(header=header,height=1,width=len(arr),fields=fields,is_bigendian=False,point_step=16,row_step=16*len(arr),data=arr.tobytes(),is_dense=True))
        grid=OccupancyGrid(header=header);grid.info.resolution=self.grid.resolution;grid.info.width=grid.info.height=self.grid.odds.shape[0]
        grid.info.origin.position.x=grid.info.origin.position.y=self.grid.origin;grid.info.origin.orientation.w=1.
        grid.data=self.grid.occupancy().ravel().tolist();self.map_pub.publish(grid)
        self.trail_pub.publish(NavPath(header=header,poses=self.trail[-10000:]))
        candidates=self.grid.frontiers(self.pose[:2],self.visited,far_origin=self.origin if self.cosmos else None,far_direction=self.forward if self.cosmos else None)
        self.candidate_count=len(candidates)
        markers=[]
        clear=Marker();clear.action=Marker.DELETEALL;markers.append(clear)
        for i,c in enumerate(candidates[:20]):
            m=Marker(header=header,ns='frontiers',id=i,type=Marker.SPHERE,action=Marker.ADD)
            m.pose.position.x=c['x'];m.pose.position.y=c['y'];m.pose.position.z=float(self.pose[2]+.2);m.pose.orientation.w=1.
            m.scale.x=m.scale.y=m.scale.z=.25;m.color.r=1.;m.color.g=.55;m.color.b=.15;m.color.a=1.;markers.append(m)
        self.markers.publish(MarkerArray(markers=markers))
        self.status_pub.publish(String(data=json.dumps(dict(map_points=len(xyz),observed_cells=int(self.grid.seen.sum()),frontiers=len(candidates),goals=self.goals,completed=self.completed,distance_m=self.distance,pose_source='lidar_imu_estimation'))))
        if self.goal_handle and not self.canceling:
            reached=self.target is not None and np.linalg.norm(self.pose[:2]-self.target)<.60
            if reached or now-self.goal_time>75:
                if reached:self.completed+=1;self.credited=self.goal_handle
                self.log('frontier_observed' if reached else 'frontier_timeout',completed=self.completed)
                self.canceling=True
                self.goal_handle.cancel_goal_async()
        if not self.mapping_only and not self.goal_handle and not self.pending and not self.canceling and (candidates or self.reasoning is not None) and self.action.server_is_ready() and now-self.started>8:
            if self.cosmos:self.cosmos_tick(candidates,now)
            elif candidates:self.send_goal(candidates[0])

    def cosmos_tick(self,candidates,now):
        if self.reasoning is not None:
            if not self.reasoning.done():return
            future=self.reasoning;self.reasoning=None;self.retry_at=now+15
            try:choice=future.result()
            except Exception as e:self.log('cosmos_failed',error=str(e));return
            self.decisions+=1;self.log('cosmos_decision',decision=choice)
            if choice['action']!='navigate':return
            selected=self.proposals[choice['candidate_id']]
            # Recompute connectivity against the current map, never execute a
            # model's stale geometry or arbitrary coordinate.
            matches=[c for c in candidates if math.hypot(c['x']-selected['x'],c['y']-selected['y'])<.75]
            if not matches:self.log('cosmos_candidate_changed');return
            c=min(matches,key=lambda c:math.hypot(c['x']-selected['x'],c['y']-selected['y']))
            self.send_goal(c);return
        if not candidates or now<self.retry_at or self.camera is None or now-self.camera_time>2:return
        self.proposals=candidates[:8]
        yaw=math.atan2(self.rot[1,0],self.rot[0,0]);context=[]
        for i,c in enumerate(self.proposals):
            bearing=math.atan2(c['y']-self.pose[1],c['x']-self.pose[0])-yaw
            context.append(dict(candidate_id=i,distance_from_start_m=float(np.linalg.norm(np.array([c['x'],c['y']])-self.origin)),forward_progress_m=float((np.array([c['x'],c['y']])-self.origin)@self.forward),distance_from_robot_m=float(np.linalg.norm(np.array([c['x'],c['y']])-self.pose[:2])),bearing_rad=math.atan2(math.sin(bearing),math.cos(bearing)),observed_free_space=True))
        self.log('cosmos_reasoning',candidates=context)
        self.reasoning=self.pool.submit(choose_frontier,self.camera,context,self.pose.copy(),yaw,self.out)


    def send_goal(self,c):
        p=PoseStamped();p.header.frame_id=self.frame;p.header.stamp=self.get_clock().now().to_msg();p.pose.position.x=c['x'];p.pose.position.y=c['y'];p.pose.orientation.z=math.sin(c['yaw']/2);p.pose.orientation.w=math.cos(c['yaw']/2)
        self.pending=True;self.goals+=1;self.goal_time=time.monotonic();self.visited.append(np.array([c['x'],c['y']]))
        self.target=np.array([c['x'],c['y']])
        self.log('frontier_selected',goal=c,goal_number=self.goals,observed_cells=int(self.grid.seen.sum()),map_points=len(self.voxels))
        f=self.action.send_goal_async(NavigateToPose.Goal(pose=p));f.add_done_callback(self.accepted)

    def accepted(self,f):
        self.pending=False
        try:h=f.result()
        except Exception as e:self.log('goal_error',error=str(e));return
        if not h.accepted:self.log('goal_rejected');return
        self.goal_handle=h;h.get_result_async().add_done_callback(lambda f:self.finished(f,h))

    def finished(self,f,h):
        if self.goal_handle is h:self.goal_handle=None;self.canceling=False
        try:status=f.result().status
        except Exception as e:self.log('goal_result_error',error=str(e));return
        if status==4 and h is not self.credited:self.completed+=1
        self.log('frontier_result',status=status,completed=self.completed,distance_m=self.distance)

    def save(self):
        summary=dict(source='raw LiDAR + IMU, online FAST-LIO-family estimator',prebuilt_map=False,fixed_route=False,ground_truth_pose_used=False,session_resumed=self.resumed,
                     state='mapping_only' if self.mapping_only else 'cosmos_reasoning' if self.reasoning is not None else ('exploring' if self.goal_handle or self.pending else ('farthest_observed_reached' if self.cosmos and self.completed and self.candidate_count==0 else 'waiting_for_frontiers')),
                     map_points=len(self.voxels),observed_cells=int(self.grid.seen.sum()),goals=self.goals,completed_goals=self.completed,distance_m=self.distance,registered_scans=self.scan_count,
                     high_level_planner='Cosmos3-Edge' if self.cosmos else 'frontier_score',cosmos_decisions=self.decisions,forward_progress_m=float((self.pose[:2]-self.origin)@self.forward) if self.pose is not None else 0.,radial_distance_m=float(np.linalg.norm(self.pose[:2]-self.origin)) if self.pose is not None else 0.,loop_closure=False,voxel_resolution_m=.1,occupancy_resolution_m=.1)
        (self.out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        if self.voxels:
            xyz=np.asarray(list(self.voxels.values()),dtype='<f4')
            np.savez_compressed(self.out/'map.npz',points=xyz,occupancy=self.grid.occupancy(),resolution=self.grid.resolution,origin=self.grid.origin)
            header=f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(xyz)}\nDATA binary\n'
            (self.out/'map.pcd').write_bytes(header.encode()+xyz.tobytes())


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--resume',action='store_true');args,ros_args=ap.parse_known_args();rclpy.init(args=ros_args);n=OnlineExploration(args.output,args.resume)
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        if n.goal_handle:n.goal_handle.cancel_goal_async()
        n.save();n.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
