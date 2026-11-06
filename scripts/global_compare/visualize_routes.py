"""Original map TF plus source-time registered display-only navigation overlays.

Simulator reference poses are used only by the new /wavego/visual topics.
No navigation commands, safety radii, LIO points or original TF are modified.
"""
import argparse,copy,json
from collections import deque
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from navigation_surface import GroundSurface
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
    from nav_msgs.msg import Odometry,Path as NavPath
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from geometry_msgs.msg import PoseStamped,TransformStamped,Point
    from visualization_msgs.msg import Marker,MarkerArray
    from std_msgs.msg import String
    from builtin_interfaces.msg import Duration
    from tf2_ros import StaticTransformBroadcaster
except ModuleNotFoundError:
    rclpy=None
    Node=object


def stamp(message):
    return message.header.stamp.sec+message.header.stamp.nanosec*1e-9


def pose_matrix(position,quaternion):
    matrix=np.eye(4)
    matrix[:3,:3]=Rotation.from_quat(quaternion).as_matrix()
    matrix[:3,3]=position
    if not np.isfinite(matrix).all(): raise ValueError('Non-finite pose')
    return matrix


def transform(message):
    p,q=message.pose.pose.position,message.pose.pose.orientation
    return pose_matrix([p.x,p.y,p.z],[q.x,q.y,q.z,q.w])


def rigid_points(xyz,matrix):
    points=np.asarray(xyz,float).reshape(-1,3);matrix=np.asarray(matrix,float)
    if matrix.shape!=(4,4) or not np.isfinite(matrix).all() or not np.isfinite(points).all():
        raise ValueError('Invalid rigid geometry')
    return points@matrix[:3,:3].T+matrix[:3,3]


def bake_marker_points(xyz,position,quaternion,registration):
    """Bake the marker-local pose once; output points require an identity pose."""
    return rigid_points(xyz,registration@pose_matrix(position,quaternion))


def measured_projection(original,floor_candidates,surface,offset=.035):
    """Unsupported points keep their registered height, never a guessed floor."""
    original=np.asarray(original,float).reshape(-1,3)
    if surface is None or not surface.valid: return original.copy(),np.zeros(len(original),bool)
    projected,supported=surface.project(floor_candidates,offset=offset,expected_tolerance=.65)
    output=original.copy();output[supported]=projected[supported]
    return output,supported


class PoseHistory:
    def __init__(self,maxlen=1400): self.samples=deque(maxlen=maxlen)
    def add(self,source_stamp,matrix):
        matrix=np.asarray(matrix,float)
        if not np.isfinite(source_stamp) or matrix.shape!=(4,4) or not np.isfinite(matrix).all() or (self.samples and source_stamp<=self.samples[-1][0]): return False
        self.samples.append((float(source_stamp),matrix.copy()))
        while len(self.samples)>1 and source_stamp-self.samples[0][0]>5: self.samples.popleft()
        return True
    def closest(self,source_stamp):
        return min(self.samples,key=lambda row:abs(row[0]-source_stamp)) if self.samples else None


def synchronized_registration(lio,reference,source_stamp,now,tolerance=.080,max_age=.8):
    """Both poses must agree with source time and each other within tolerance."""
    audit=dict(source_stamp=float(source_stamp),valid=False,ground_truth_pose_used_for_visualization=True,lidar_geometry_modified=False,controller_geometry_modified=False)
    if not np.isfinite([source_stamp,now]).all(): return None,dict(audit,reason='invalid_time')
    age=now-source_stamp;audit['source_age_seconds']=float(age)
    if age<-.08 or age>max_age: return None,dict(audit,reason='source_stale_or_future')
    lp,rp=lio.closest(source_stamp),reference.closest(source_stamp)
    if lp is None or rp is None: return None,dict(audit,reason='pose_missing')
    audit.update(lio_stamp=lp[0],reference_stamp=rp[0],lio_source_delta_seconds=abs(lp[0]-source_stamp),reference_source_delta_seconds=abs(rp[0]-source_stamp),pose_pair_delta_seconds=abs(lp[0]-rp[0]))
    if max(audit['lio_source_delta_seconds'],audit['reference_source_delta_seconds'],audit['pose_pair_delta_seconds'])>tolerance: return None,dict(audit,reason='pose_not_synchronized')
    matrix=lp[1]@np.linalg.inv(rp[1])
    return matrix,dict(audit,valid=True,reason='source_time_visual_registration',matrix=matrix.tolist())


def current_audit_status(audit,now,max_age=.8):
    """Do not advertise an expired successful observation as currently visible."""
    result=dict(audit)
    if 'source_stamp' in result:
        result['source_age_seconds_at_registration']=result.get('source_age_seconds')
        result['source_age_seconds']=float(now-result['source_stamp'])
        result['last_registration_valid']=bool(result.get('valid',False))
        if result['valid'] and not -.08<=result['source_age_seconds']<=max_age:
            result.update(valid=False,reason='last_observation_expired')
    return result


def valid_pose_matrix(matrix):
    matrix=np.asarray(matrix,float)
    return (matrix.shape==(4,4) and np.isfinite(matrix).all()
            and np.allclose(matrix[3],[0.,0.,0.,1.],atol=1e-8)
            and np.allclose(matrix[:3,:3].T@matrix[:3,:3],np.eye(3),atol=1e-6)
            and abs(np.linalg.det(matrix[:3,:3])-1.)<1e-6)


class VisualPoseAnchors:
    """Append-only full source poses, used exclusively for spatial display plans."""
    def __init__(self,path=None):
        self.path=Path(path) if path is not None else None
        self.anchors=[];self.rejected_saved_records=0;self.append_separator=False
        if self.path is not None and self.path.exists():
            text=self.path.read_text();self.append_separator=bool(text and not text.endswith('\n'))
            for line in text.splitlines():
                try:
                    row=json.loads(line);self._validate(row)
                    if self.anchors and row['lio_stamp']<=self.anchors[-1]['lio_stamp']:raise ValueError('Out-of-order anchor')
                    self.anchors.append(row)
                except (ValueError,KeyError,TypeError):self.rejected_saved_records+=1

    @staticmethod
    def _validate(row):
        if not isinstance(row,dict):raise ValueError('Display anchor must be an object')
        if row.get('schema')!='wavego_display_pose_anchor_v1' or row.get('reference_frame')!='map' or row.get('estimated_frame')!='lio_map':raise ValueError('Unknown display anchor format')
        if not np.isfinite([row['lio_stamp'],row['reference_stamp']]).all() or abs(row['lio_stamp']-row['reference_stamp'])>.080:raise ValueError('Unsynchronized saved anchor')
        reference,lio,matrix=(np.asarray(row[key],float) for key in ('reference_pose','lio_pose','registration'))
        if not all(valid_pose_matrix(value) for value in (reference,lio,matrix)) or not np.allclose(lio@np.linalg.inv(reference),matrix,atol=1e-8):raise ValueError('Missing or inconsistent full pose')
        surface=row.get('measured_surface')
        if surface is not None:
            support=np.asarray(surface['support_xyz'],float).reshape(-1,3)
            values=np.r_[support.ravel(),surface['plane'],surface['base_xyz'],surface['floor_z'],surface['source_stamp']]
            if (not np.isfinite(values).all() or (surface['valid'] and len(support)<20) or surface['support_count']!=len(support)
                    or len(surface['plane'])!=3 or len(surface['base_xyz'])!=3
                    or abs(surface['source_stamp']-row['lio_stamp'])>.30):raise ValueError('Invalid measured surface snapshot')
            GroundSurface.from_payload(surface)

    @staticmethod
    def surface(row):
        data=row.get('measured_surface')
        if data is None:return None
        return GroundSurface.from_payload(data)

    def consider(self,lio_stamp,reference_stamp,lio_pose,reference_pose,surface=None,surface_stamp=None,surface_payload=None):
        if not np.isfinite([lio_stamp,reference_stamp]).all() or abs(lio_stamp-reference_stamp)>.080:return False
        if not valid_pose_matrix(lio_pose) or not valid_pose_matrix(reference_pose):return False
        reference_pose,lio_pose=np.asarray(reference_pose,float),np.asarray(lio_pose,float)
        if self.anchors:
            last=self.anchors[-1]
            if lio_stamp<=last['lio_stamp']:return False
            previous=np.asarray(last['reference_pose'])[:3,3];current=reference_pose[:3,3]
            if np.linalg.norm(current[:2]-previous[:2])<.25 and abs(current[2]-previous[2])<.15:
                # The floor message may arrive after a nominal position anchor.
                # Append its actual later observation once, rather than replacing
                # an old pose or rejecting every delayed floor at this location.
                old_surface=self.surface(last)
                upgrade=(surface is not None and surface.valid and surface_stamp is not None
                         and abs(surface_stamp-lio_stamp)<=.080
                         and (old_surface is None or not old_surface.valid))
                if not upgrade:return False
        row=dict(schema='wavego_display_pose_anchor_v1',reference_frame='map',estimated_frame='lio_map',lio_stamp=float(lio_stamp),reference_stamp=float(reference_stamp),reference_pose=reference_pose.tolist(),lio_pose=lio_pose.tolist(),registration=(lio_pose@np.linalg.inv(reference_pose)).tolist(),ground_truth_pose_used_for_visualization=True,lidar_geometry_modified=False,measured_surface=None)
        if surface is not None and surface_stamp is not None and abs(surface_stamp-lio_stamp)<=.30:
            row['measured_surface']=copy.deepcopy(surface_payload) if surface_payload is not None else surface.payload(surface_stamp)
        self._validate(row)
        if self.path is not None:
            self.path.parent.mkdir(parents=True,exist_ok=True)
            with self.path.open('a') as stream:
                if self.append_separator:stream.write('\n')
                stream.write(json.dumps(row,separators=(',',':'))+'\n')
            self.append_separator=False
        self.anchors.append(row);return True


def spatially_registered_path(values,current_matrix,history,max_distance=1.5,max_floor_delta=.7,return_support=False):
    """Register visited plan locations with the poses measured there, not now.

    Existing planners encode floor+.09 m. Query historic reference base positions
    using floor+.40 m; no measured-floor claim is made for unvisited positions.
    """
    values=np.asarray(values,float).reshape(-1,3)
    output=rigid_points(values,current_matrix);indices=np.full(len(values),-1,int);grounded=np.zeros(len(values),bool)
    if history.anchors and len(values):
        body_queries=values.copy();body_queries[:,2]+=.40-.09
        bodies=np.array([np.asarray(row['reference_pose'])[:3,3] for row in history.anchors])
        for start in range(0,len(values),256):
            delta=body_queries[start:start+256,None,:]-bodies[None,:,:]
            distances=np.linalg.norm(delta,axis=2);distances[abs(delta[:,:,2])>=max_floor_delta]=np.inf
            nearest=np.argmin(distances,axis=1);best=distances[np.arange(len(nearest)),nearest]
            indices[start:start+len(nearest)]=np.where(best<=max_distance,nearest,-1)
        for index in np.unique(indices[indices>=0]):
            mask=indices==index;row=history.anchors[index];matrix=np.asarray(row['registration'])
            registered=rigid_points(values[mask],matrix);candidate=values[mask].copy();candidate[:,2]-=.09
            output[mask]=registered
        # A nearest nominal anchor must not veto a nearby real measured floor.
        # Test each eligible actual surface and use the closest supported one.
        best_supported=np.full(len(values),np.inf)
        for index,row in enumerate(history.anchors):
            surface=history.surface(row)
            if surface is None or not surface.valid:continue
            delta=body_queries-bodies[index];distance=np.linalg.norm(delta,axis=1)
            mask=(distance<=max_distance)&(abs(delta[:,2])<max_floor_delta)&(distance<best_supported)
            if not mask.any():continue
            matrix=np.asarray(row['registration']);registered=rigid_points(values[mask],matrix)
            candidate=values[mask].copy();candidate[:,2]-=.09
            projected,supported=measured_projection(registered,rigid_points(candidate,matrix),surface)
            selected=np.flatnonzero(mask)[supported]
            output[selected]=projected[supported];grounded[selected]=True;indices[selected]=index
            best_supported[selected]=distance[selected]
    associated=indices>=0
    audit=dict(display_plan_registration='Spatial history of synchronized full reference/LIO poses; visualization only',historic_registered_points=int(associated.sum()),measured_ground_points=int(grounded.sum()),historic_nominal_height_points=int((associated&~grounded).sum()),nominal_unvisited_points=int((~associated).sum()),anchor_count=len(history.anchors),anchor_match_distance_m=max_distance,anchor_floor_body_height_delta_m=max_floor_delta,measured_anchor_selection='Closest actually supported surface among nearby same-floor source-pose anchors; nearest nominal anchor cannot veto measured support',planned_height_definition='input floor + 0.09 m; matching body floor + 0.40 m',unknown_height='Current rigid registration of nominal planned height; not measured',lidar_geometry_modified=False)
    if return_support:return output,audit,indices,grounded
    return output,audit,indices


def observed_route_segments(xyz,supported,leg_lengths=None):
    """Use original adjacent supported pairs, never join across unknown gaps.

    The returned LINE_LIST describes observed terrain portions of a plan;
    it does not assert that the robot traversed them or completed the plan.
    Input XYZ already carries historical source-pose registration and actual
    measured-floor projection. No replacement heights are generated here.
    """
    xyz=np.asarray(xyz,float).reshape(-1,3);supported=np.asarray(supported,bool).reshape(-1)
    if len(xyz)!=len(supported) or not np.isfinite(xyz).all():raise ValueError('Invalid observed route geometry')
    lengths=[len(xyz)] if leg_lengths is None else list(leg_lengths)
    if any(not isinstance(n,(int,np.integer)) or n<0 for n in lengths) or sum(lengths)!=len(xyz):raise ValueError('Invalid route leg lengths')
    edges=np.ones(max(0,len(xyz)-1),bool)
    boundaries=np.cumsum(np.asarray(lengths,dtype=np.int64))[:-1]-1
    boundaries=boundaries[(boundaries>=0)&(boundaries<len(edges))]
    edges[boundaries]=False
    observed=edges & supported[:-1] & supported[1:]
    starts=np.flatnonzero(observed)
    pairs=np.column_stack((starts,starts+1))
    segments=xyz[pairs].reshape(-1,3)
    audit=dict(display='observed terrain portions of plan',plan_points=len(xyz),measured_supported_points=int(supported.sum()),unknown_height_points=int((~supported).sum()),observed_segments=len(starts),within_leg_segments=int(edges.sum()),excluded_unknown_segments=int((edges & ~observed).sum()),excluded_cross_leg_connections=int((~edges).sum()),segment_rule='Original adjacent points in the same planner leg must both have source-time measured-ground support',association_is_not_proof_of_robot_traversal=True,lidar_geometry_modified=False,safety_guard_radii_modified=False)
    return segments,audit


def mapped_route_marker(name,index,xyz,header):
    """One stable marker per planner; empty geometry also replaces an old route."""
    marker=Marker(header=header,ns='observed_plan_'+name,id=index,type=Marker.LINE_LIST,action=Marker.ADD)
    marker.pose.orientation.w=1.;marker.scale.x=.035
    rgb=(45,185,255) if name=='NavFnDijkstra' else (245,65,195)
    marker.color.r,marker.color.g,marker.color.b=[v/255. for v in rgb];marker.color.a=1.
    marker.points=[Point(x=float(v[0]),y=float(v[1]),z=float(v[2])) for v in xyz]
    return marker


class Display(Node):
    def __init__(self,source,output):
        if rclpy is None: raise RuntimeError('Source OfficeWorld ROS environment before starting display')
        super().__init__('upstream_global_route_display')
        self.routes=json.loads(source.read_text())['planners'];self.output=output
        self.mj=self.lio=None;self.lio_history=PoseHistory();self.reference_history=PoseHistory()
        self.pending=deque(maxlen=20);self.scan=None;self.scan_key=None;self.surface=None;self.surface_stamp=None
        self.canonical_surface_received=False;self.surface_origin='independent_scan_fit';self.surface_payload=None;self.canonical_surface_error=None
        self.anchors=VisualPoseAnchors(output.parent/'visual_pose_anchors.jsonl')
        self.last_visual_status={};self.last_path_status={};self.path_count=self.marker_count=0
        self.last_mapped_status={}
        self.last_marker_source_stamp=-float('inf')
        self.aligned=False;self.initial_matrix=None;self.tf=StaticTransformBroadcaster(self)
        q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pubs={k:self.create_publisher(NavPath,'/wavego/global/'+k,q) for k in self.routes}
        self.visual_paths={k:self.create_publisher(NavPath,'/wavego/visual/paths/'+k,q) for k in self.routes}
        self.mapped_routes=self.create_publisher(MarkerArray,'/wavego/visual/mapped_routes',q)
        self.visual_markers=self.create_publisher(MarkerArray,'/wavego/visual/navigation_markers',10)
        self.status_pub=self.create_publisher(String,'/wavego/visual/registration_status',10)
        self.create_subscription(Odometry,'/odom/mujoco_odom',self.physics,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/wavego/navigation/odom',self.estimated,qos_profile_sensor_data)
        self.create_subscription(PointCloud2,'/wavego/slam/registered_points',lambda m:setattr(self,'scan',m),qos_profile_sensor_data)
        self.create_subscription(String,'/wavego/live_dense/ground_surface',self.ground_surface,10)
        self.create_subscription(MarkerArray,'/wavego/global/markers',self.markers,10)
        # A display hot restart must not overwrite original alignment at a moved robot.
        if output.exists():
            previous=json.loads(output.read_text())
            if previous.get('parent')!='lio_map' or previous.get('child')!='map': raise ValueError('Unexpected existing alignment frames')
            self.send_initial_transform(np.asarray(previous['matrix']));self.aligned=True
        self.create_timer(1.,self.tick);self.create_timer(.2,self.visual_tick)
    def send_initial_transform(self,matrix):
        self.initial_matrix=matrix.copy();t=TransformStamped()
        t.header.frame_id,t.child_frame_id='lio_map','map';t.header.stamp=self.get_clock().now().to_msg()
        t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=matrix[:3,3].tolist()
        q=Rotation.from_matrix(matrix[:3,:3]).as_quat()
        t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=q.tolist();self.tf.sendTransform(t)
    def physics(self,message):
        self.mj=transform(message);self.reference_history.add(stamp(message),self.mj)
    def estimated(self,message):
        if message.header.frame_id!='lio_map': return
        self.lio=transform(message);self.lio_history.add(stamp(message),self.lio)
    def markers(self,message):
        if message.markers and all(m.action in (Marker.DELETE,Marker.DELETEALL) for m in message.markers):
            source_stamp=max(stamp(m) for m in message.markers)
            if source_stamp<=self.last_marker_source_stamp:return
            self.last_marker_source_stamp=source_stamp
            cleaned=copy.deepcopy(message)
            for m in cleaned.markers:m.header.frame_id='lio_map'
            self.visual_markers.publish(cleaned)
        else:self.pending.append(message)
    def ground_surface(self,message):
        self.canonical_surface_received=True;self.surface_origin='canonical_live_dense_measured_surface'
        try:
            payload=json.loads(message.data)
            if not np.isfinite([payload['source_stamp'],payload['floor_z']]).all():raise ValueError('Non-finite canonical ground source')
            surface=GroundSurface.from_payload(payload)
            if payload['support_count']!=surface.support_count:raise ValueError('Canonical ground support count mismatch')
            self.surface=surface;self.surface_stamp=float(payload['source_stamp']);self.surface_payload=payload;self.canonical_surface_error=None
        except (ValueError,KeyError,TypeError) as error:
            self.surface=None;self.surface_stamp=None;self.surface_payload=None;self.canonical_surface_error=str(error)
    def update_surface(self,now):
        # Once canonical ground is available, invalid/stale canonical data must
        # never be replaced by an inconsistent independently fitted old plane.
        if self.canonical_surface_received:return
        if self.scan is None or self.scan_key==stamp(self.scan):return
        source_stamp=stamp(self.scan);lp=self.lio_history.closest(source_stamp)
        if lp is None or abs(lp[0]-source_stamp)>.08 or not 0<=now-source_stamp<=.8:return
        # Dedicated native registered-cloud topic has header "map", but numeric
        # coordinates are the sidecar's lio_map. Never apply static map TF here.
        if self.scan.header.frame_id not in ('map','lio_map'):return
        data=point_cloud2.read_points(self.scan,field_names=('x','y','z'),skip_nans=True)
        xyz=np.column_stack([data[k] for k in ('x','y','z')])
        self.surface=GroundSurface.from_scan(xyz,lp[1][:3,3]);self.surface_stamp=self.scan_key=source_stamp
        self.surface_payload=dict(self.surface.payload(source_stamp),measurement_window_sec=0.,surface_origin='independent_single_scan_fallback')
    def available_surface(self,source_stamp):
        return self.surface if self.surface_stamp is not None and abs(self.surface_stamp-source_stamp)<=.30 else None
    def capture_anchor(self,now):
        if not self.lio_history.samples:return
        # Prefer the actual canonical scan clock. Latest odometry can be ahead
        # of the 2 s measured-floor payload and would save a nominal-only anchor.
        has_fresh_floor=(self.surface is not None and self.surface.valid
                         and self.surface_stamp is not None
                         and -.08<=now-self.surface_stamp<=.8)
        source_stamp=self.surface_stamp if has_fresh_floor else self.lio_history.samples[-1][0]
        matrix,audit=synchronized_registration(self.lio_history,self.reference_history,source_stamp,now)
        if matrix is None:return
        lp=self.lio_history.closest(audit['lio_stamp']);rp=self.reference_history.closest(audit['reference_stamp'])
        self.anchors.consider(lp[0],rp[0],lp[1],rp[1],self.available_surface(source_stamp),self.surface_stamp,self.surface_payload)
    def enrich_audit(self,audit,matrix):
        if matrix is not None and self.initial_matrix is not None:
            audit=dict(audit,local_minus_initial_translation_m=(matrix[:3,3]-self.initial_matrix[:3,3]).tolist(),local_vs_initial_rotation_deg=Rotation.from_matrix(matrix[:3,:3]@self.initial_matrix[:3,:3].T).as_euler('xyz',degrees=True).tolist())
            ref=self.reference_history.closest(audit['reference_stamp'])
            lio=self.lio_history.closest(audit['lio_stamp'])
            delta=lio[1][:3,3]-rigid_points(ref[1][:3,3],self.initial_matrix)[0]
            audit.update(initial_registered_reference_error_xyz_m=delta.tolist(),initial_registered_reference_error_norm_m=float(np.linalg.norm(delta)),registration_is_not_a_map_accuracy_correction=True)
        return audit
    def tick(self):
        if not self.aligned:
            if self.mj is None or self.lio is None:return
            if np.linalg.norm(self.mj[:2,3]-[8.,0.])>.25:raise RuntimeError('Missing original alignment: start at stationary start line')
            matrix=self.lio@np.linalg.inv(self.mj);self.send_initial_transform(matrix)
            self.output.parent.mkdir(parents=True,exist_ok=True)
            self.output.write_text(json.dumps(dict(parent='lio_map',child='map',matrix=matrix.tolist(),source='Initial stationary rigid alignment; LiDAR mapping remains estimated'),indent=2));self.aligned=True
        now=self.get_clock().now().nanoseconds*1e-9;self.update_surface(now);self.capture_anchor(now)
        source_stamp=self.lio_history.samples[-1][0] if self.lio_history.samples else now
        matrix,audit=synchronized_registration(self.lio_history,self.reference_history,source_stamp,now)
        audit=self.enrich_audit(audit,matrix);supported_count=total=0;historic_count=nominal_count=historic_nominal_count=0
        mapped_markers=[];mapped_status={}
        for planner_index,(name,legs) in enumerate(self.routes.items()):
            legacy=NavPath();legacy.header.frame_id='map';legacy.header.stamp=self.get_clock().now().to_msg()
            values=np.array([xyz for leg in legs for xyz in leg['xyz']],dtype=float).reshape(-1,3)
            for xyz in values:
                pose=PoseStamped(header=legacy.header);pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=xyz.tolist();pose.pose.orientation.w=1.;legacy.poses.append(pose)
            self.pubs[name].publish(legacy)
            visual=NavPath();visual.header.frame_id='lio_map';visual.header.stamp=legacy.header.stamp
            mapped_xyz=np.empty((0,3))
            route_status=dict(display='observed terrain portions of plan',registration_valid=matrix is not None,plan_points=len(values),measured_supported_points=0,unknown_height_points=len(values),observed_segments=0,association_is_not_proof_of_robot_traversal=True)
            if matrix is not None:
                xyz,path_audit,indices,grounded=spatially_registered_path(values,matrix,self.anchors,return_support=True)
                supported_count+=path_audit['measured_ground_points'];total+=len(xyz)
                historic_count+=path_audit['historic_registered_points'];nominal_count+=path_audit['nominal_unvisited_points'];historic_nominal_count+=path_audit['historic_nominal_height_points']
                for point in xyz:
                    pose=PoseStamped(header=visual.header);pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=point.tolist();pose.pose.orientation.w=1.;visual.poses.append(pose)
                mapped_xyz,route_status=observed_route_segments(xyz,grounded,[len(leg['xyz']) for leg in legs])
                sources=[self.anchors.anchors[i]['measured_surface']['source_stamp'] for i in np.unique(indices[grounded])]
                route_status.update(registration_valid=True,historic_registered_points=path_audit['historic_registered_points'],historic_nominal_height_points=path_audit['historic_nominal_height_points'],nominal_unvisited_points=path_audit['nominal_unvisited_points'],ground_surface_source_stamp_range=[min(sources),max(sources)] if sources else None)
            mapped_markers.append(mapped_route_marker(name,planner_index,mapped_xyz,visual.header));mapped_status[name]=route_status
            self.visual_paths[name].publish(visual)
        self.mapped_routes.publish(MarkerArray(markers=mapped_markers))
        self.last_mapped_status=dict(frame='lio_map',source_stamp=source_stamp,display='observed terrain portions of plan',full_plan_topics_preserved=True,ground_truth_pose_used_for_visualization=True,lidar_geometry_modified=False,safety_guard_radii_modified=False,association_is_not_proof_of_robot_traversal=True,planners=mapped_status)
        self.path_count+=int(matrix is not None)
        self.last_path_status=dict(audit,supported_ground_points=supported_count,path_points=total,historic_registered_points=historic_count,historic_nominal_height_points=historic_nominal_count,nominal_unvisited_points=nominal_count,anchor_count=len(self.anchors.anchors),display_plan_registration='Spatial history of synchronized full reference/LIO poses; visualization only',association_is_not_proof_of_robot_traversal=True,anchor_match_distance_m=1.5,anchor_floor_body_height_delta_m=.7,unsupported_path_height='Retained nominal planned height; not a measured floor')
    def project_marker(self,source,matrix,source_stamp,surface,line_ends):
        marker=copy.deepcopy(source);marker.header.frame_id='lio_map';marker.frame_locked=False
        if marker.action in (Marker.DELETE,Marker.DELETEALL):return marker,0,0
        if source.header.frame_id not in ('map','world'):raise ValueError('Unexpected controller marker frame')
        p,q=source.pose.position,source.pose.orientation;position=[p.x,p.y,p.z];quat=[q.x,q.y,q.z,q.w]
        original_pose=pose_matrix(position,quat);transformed_pose=matrix@original_pose;count=total=0
        offsets={'navigation_rollout_samples':(.52,.035),'dynamic_guard_radius':(.12,.035),'dynamic_velocity_prediction':(.25,.05)}
        if source.points:
            local_points=np.array([[v.x,v.y,v.z] for v in source.points]);original=bake_marker_points(local_points,position,quat,matrix)
            subtract,offset=offsets.get(source.ns,(0.,.035));candidate=rigid_points(local_points,original_pose);candidate[:,2]-=subtract
            xyz,supported=measured_projection(original,rigid_points(candidate,matrix),surface,offset);count,total=int(supported.sum()),len(xyz)
            marker.pose.position.x=marker.pose.position.y=marker.pose.position.z=0.
            marker.pose.orientation.x=marker.pose.orientation.y=marker.pose.orientation.z=0.;marker.pose.orientation.w=1.
            if source.ns in ('navigation_rollout_samples','dynamic_guard_radius'):
                # Keep original adjacent edges only where both ends have real
                # source-time floor support. Filtering individual vertices and
                # retaining LINE_STRIP would falsely bridge an unobserved gap.
                edges=np.flatnonzero(supported[:-1]&supported[1:])
                selected=np.column_stack((edges,edges+1)).reshape(-1)
                if source.ns=='navigation_rollout_samples':
                    line_ends.pop(source.id,None)
                    if len(xyz)>1 and supported[-2:].all():line_ends[source.id]=xyz[-1].copy()
                xyz=xyz[selected];marker.type=Marker.LINE_LIST
                if len(source.colors)==total:marker.colors=[copy.deepcopy(source.colors[i]) for i in selected]
                else:marker.colors=[]
                if not len(xyz):marker.action=Marker.DELETE
            elif source.ns=='dynamic_velocity_prediction' and not supported.all():
                # An arrow spans both endpoints: one grounded endpoint cannot
                # certify a prediction into an unmeasured part of the floor.
                xyz=np.empty((0,3));marker.colors=[];marker.action=Marker.DELETE
            marker.points=[Point(x=float(v[0]),y=float(v[1]),z=float(v[2])) for v in xyz]
            if source.ns=='dynamic_guard_radius':marker.scale.x=.022;marker.color.a=.30 if source.color.g<.14 else .14
        else:
            xyz=transformed_pose[:3,3].reshape(1,3)
            subtract,offset={'navigation_lookahead':(.55,.035),'navigation_sampling_heads':(.52,.035),'measured_dynamic':(source.scale.z/2,source.scale.z/2+.035),'dynamic_labels':(1.65,1.65)}.get(source.ns,(0.,0.))
            candidate=np.array(position,float).reshape(1,3);candidate[:,2]-=subtract
            if source.ns in ('navigation_lookahead','navigation_sampling_heads','measured_dynamic','dynamic_labels'):
                xyz,supported=measured_projection(xyz,rigid_points(candidate,matrix),surface,offset);count,total=int(supported.sum()),1
            rotation=Rotation.from_matrix(transformed_pose[:3,:3]).as_quat()
            if source.ns=='measured_dynamic' and count:rotation=[0.,0.,0.,1.]
            if source.ns=='navigation_sampling_heads':
                if count and source.id in line_ends:xyz[0]=line_ends[source.id]
                else:marker.action=Marker.DELETE
            if source.ns=='navigation_lookahead' and not count:marker.action=Marker.DELETE
            marker.pose.position.x,marker.pose.position.y,marker.pose.position.z=xyz[0].tolist()
            marker.pose.orientation.x,marker.pose.orientation.y,marker.pose.orientation.z,marker.pose.orientation.w=list(rotation)
            if source.ns=='dynamic_labels':
                marker.scale.z=.14
                marker.text=f"LiDAR candidate envelope #{source.id}"+(' | YIELD' if 'YIELD' in source.text else '')
        marker.lifetime=Duration(sec=0,nanosec=500000000)
        return marker,count,total
    def visual_tick(self):
        now=self.get_clock().now().nanoseconds*1e-9;self.update_surface(now);self.capture_anchor(now);eligible=[];future=deque(maxlen=20)
        for array in self.pending:
            additions=[m for m in array.markers if m.action not in (Marker.DELETE,Marker.DELETEALL)]
            source_stamp=stamp(additions[0]) if additions else now
            # Do not let an older queued DELETEALL erase a newer observation.
            if source_stamp<=self.last_marker_source_stamp:continue
            matrix,audit=synchronized_registration(self.lio_history,self.reference_history,source_stamp,now)
            audit=self.enrich_audit(audit,matrix)
            if matrix is not None:eligible.append((array,matrix,audit,source_stamp))
            elif now-source_stamp<=.8 and audit['reason'] in ('pose_missing','pose_not_synchronized','source_stale_or_future'):
                future.append(array);self.last_visual_status=audit
            else:
                deletes=[copy.deepcopy(m) for m in array.markers if m.action in (Marker.DELETE,Marker.DELETEALL)]
                for m in deletes:m.header.frame_id='lio_map'
                if deletes:self.visual_markers.publish(MarkerArray(markers=deletes))
                self.last_visual_status=audit
        self.pending=future
        if eligible:
            array,matrix,audit,source_stamp=max(eligible,key=lambda item:item[3]);surface=self.available_surface(source_stamp)
            markers=[];projected=total=0;line_ends={};geometry_status={}
            # Preserve clear operations first, then populate actual supported
            # rollout endpoints before processing paired sampling heads.
            ordered=sorted(array.markers,key=lambda m:0 if m.action in (Marker.DELETE,Marker.DELETEALL) else (1 if m.ns=='navigation_rollout_samples' else 2))
            for source in ordered:
                try:
                    marker,count,number=self.project_marker(source,matrix,source_stamp,surface,line_ends);markers.append(marker);projected+=count;total+=number
                    if source.action==Marker.ADD:
                        item=geometry_status.setdefault(source.ns,dict(input_geometry_points=0,ground_supported_input_points=0,rendered_vertices=0,rendered_segments=0,hidden_marker_count=0,unverified_envelope_count=0))
                        item['input_geometry_points']+=number;item['ground_supported_input_points']+=count
                        if marker.action==Marker.DELETE:item['hidden_marker_count']+=1
                        else:
                            item['rendered_vertices']+=len(marker.points) if marker.points else 1
                            if marker.type==Marker.LINE_LIST:item['rendered_segments']+=len(marker.points)//2
                            if source.ns=='measured_dynamic' and not count:item['unverified_envelope_count']+=1
                except ValueError as error:audit['rejected_marker_reason']=str(error)
            self.visual_markers.publish(MarkerArray(markers=markers));self.marker_count+=1
            self.last_marker_source_stamp=source_stamp
            self.pending=deque((a for a in self.pending if any(m.action not in (Marker.DELETE,Marker.DELETEALL) and stamp(m)>source_stamp for m in a.markers)),maxlen=20)
            self.last_visual_status=dict(audit,supported_ground_points=projected,marker_geometry_points=total,rendered_geometry_by_namespace=geometry_status,preview_display_policy='Only original adjacent floor-supported edges; sampling heads require their supported terminal edge. Unsupported rings/arrows/lookahead are hidden, without filling missing ground.',dynamic_display_definition='LiDAR candidate envelope; controller proxy height, not a measured 3D bounding box or verified pedestrian identity. Unsupported envelopes retain registered source height.',original_guard_radii_modified=False,marker_header_is_not_raw_cluster_acquisition_timestamp=True)
        surface_status=self.surface.metadata() if self.surface is not None else dict(valid=False,reason='no_synchronized_registered_scan')
        status=dict(time=now,visual_frame='lio_map',original_static_alignment_preserved=self.aligned,ground_truth_pose_used_for_visualization=True,lidar_geometry_modified=False,controller_geometry_modified=False,safety_guard_radii_modified=False,marker_registration=current_audit_status(self.last_visual_status,now),path_registration=current_audit_status(self.last_path_status,now),mapped_routes=self.last_mapped_status,ground_surface=dict(surface_status,source_stamp=self.surface_stamp,source_age_seconds=now-self.surface_stamp if self.surface_stamp is not None else None,projection_requires_source_delta_seconds=.30,surface_origin=self.surface_origin,canonical_received=self.canonical_surface_received,canonical_payload_error=self.canonical_surface_error),display_pose_history=dict(anchor_count=len(self.anchors.anchors),rejected_saved_records=self.anchors.rejected_saved_records,path=str(self.anchors.path),ground_truth_pose_used_for_visualization=True,not_slam_accuracy_correction=True),published_marker_arrays=self.marker_count,published_path_sets=self.path_count,pending_marker_arrays=len(self.pending),source_note='Header-time matched controller overlays; source cluster acquisition age is not encoded by controller markers')
        self.output.parent.mkdir(parents=True,exist_ok=True);temp=self.output.parent/'visual_registration.json.tmp';temp.write_text(json.dumps(status,indent=2)+'\n');temp.replace(self.output.parent/'visual_registration.json')
        self.status_pub.publish(String(data=json.dumps(status)))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--routes',type=Path,required=True);parser.add_argument('--alignment',type=Path,required=True);args=parser.parse_args()
    if rclpy is None:parser.error('Source OfficeWorld ROS environment before starting display')
    rclpy.init();node=Display(args.routes,args.alignment)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
