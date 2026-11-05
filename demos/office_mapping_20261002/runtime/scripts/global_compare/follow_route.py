"""Follow actual Nav2 path samples with the validated terrain RL controller.

Dynamic crossing candidates come from current measured LiDAR returns, never a
clock-based simulated-person marker. Geometric candidate labels are not an
assertion of neural semantic classification.
"""
import argparse
import json
import math
import os
import subprocess
from pathlib import Path
import sys
import time
import numpy as np
from scipy.ndimage import label, binary_dilation
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Twist, Point
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
from maps import GEOMS, R
sys.path.insert(0, str(R / 'scripts'))
from go2w_office_inline_run import Runner
from go2w_lidar_mount import body_points
from flat_escape import choose_reverse_escape


class NativeStairEntry:
    """Turn on the wide apron, then drive forward along the lower-flight axis.

    The two checkpoints are explicit local entry manoeuvres on the actual
    flat Office floor, not replacements for upstream Nav2 path samples.
    Only measured position, heading and stationary observations authorize
    the existing flight gait; no pose or locomotion model is modified.
    """
    def __init__(self, approach, flight, clearance=.35):
        frame=flight['frame']
        if approach['id']!='native_stair_approach' or flight['id']!='native_lower_landing' or flight['mode']!='up' or frame is None:
            raise ValueError('Lower native stair entry requires its actual approach and flight')
        self.heading=float(frame[2]);self.forward=np.array([math.cos(self.heading),math.sin(self.heading)])
        self.normal=np.array([-self.forward[1],self.forward[0]])
        self.frame_origin=np.array(frame[:2],float)
        original=np.array([approach['x'],approach['y']],float)
        centered=original-self.normal*((original-self.frame_origin)@self.normal)
        self.checkpoint=centered-clearance*self.forward;self.height=float(approach['z'])
        # Leave lateral clearance for the measured RL pivot drift. Turn
        # at actual Office coordinates [-6.20, 4.15], on the
        # independently checked wide flat apron, away from the walkers.
        self.turn_checkpoint=self.checkpoint-2.15*self.forward-.63*self.normal
        self.phase='west_clearance';self.west_checkpoint=None
        self.started=self.improved=None;self.best=float('inf');self.last_stamp=None
        self.approach_origin=None;self.approach_length=None;self.approach_lateral=None
        self.last_step=None
        self.settled_since=None;self.settled_xy=None;self.settled_yaw=None;self.settled_samples=0

    @staticmethod
    def angle(value):return math.atan2(math.sin(value),math.cos(value))

    def step(self, pose, now, source_stamp, yielding=False):
        if self.started is None:self.started=self.improved=now
        elif yielding and self.last_step is not None:
            # Preserve the existing pedestrian guard's stopped-time allowance.
            paused=max(0.,now-self.last_step);self.started+=paused;self.improved+=paused
        self.last_step=now
        xy=np.array([pose['x'],pose['y']],float);yaw=float(pose['yaw'])
        new=np.isfinite(source_stamp) and (self.last_stamp is None or source_stamp>self.last_stamp)
        if new:self.last_stamp=float(source_stamp)
        distance=float(np.linalg.norm(self.checkpoint-xy));lateral=float((xy-self.frame_origin)@self.normal)
        along=float((xy-self.checkpoint)@self.forward)
        height_error=abs(float(pose['z'])-self.height);heading_error=self.angle(self.heading-yaw)
        pre_distance=float(np.linalg.norm(self.turn_checkpoint-xy))
        result=dict(ready=False,linear_x=0.,angular_z=0.,phase=self.phase,checkpoint_xy_distance_m=distance,lateral_center_error_m=lateral,checkpoint_along_error_m=along,pre_turn_distance_m=pre_distance,height_error_m=height_error,heading_error_rad=heading_error,new_observation=bool(new),stationary_observations=self.settled_samples)
        if not np.isfinite([*xy,yaw,pose['z'],now]).all():return dict(result,failure_reason='invalid native entry pose')
        if self.west_checkpoint is None:
            # First move away from the upper walker's x=-4.4 corridor.
            # Preserve the observed Y when already on the wide apron; from
            # the normal y~1.5 handoff add only enough northward clearance
            # to finish above y=1.65, safely before the first riser y=1.05.
            if not -.98<=lateral<=2.10 or not -1.65<=along<=.55:
                return dict(result,failure_reason='native entry starts outside the verified flat apron')
            self.west_checkpoint=xy-max(lateral+.63,0.)*self.normal-max(along-.30,0.)*self.forward
        if now-self.started>90:return dict(result,failure_reason='native entry centering timed out')
        level=height_error<=.12 and abs(pose.get('roll',0.))<.30 and abs(pose.get('pitch',0.))<.30
        # Static MJCF collision geometry supports this broad turning apron;
        # live measured pedestrian guarding still applies to every command.
        pre_region=abs(lateral+.63)<=.35 and -2.55<=along<=-1.60 and level
        final_centered=abs(lateral)<=.12 and -.10<=along<=.15 and level
        previous=self.phase
        if self.phase=='west_clearance' and lateral<=-.53 and along<=.35 and level:
            self.phase='pre_position'
        if self.phase=='pre_heading' and not pre_region:self.phase='pre_position'
        if self.phase=='pre_position' and lateral<=-.53 and -2.55<=along<=-1.95 and level:self.phase='pre_heading'
        if self.phase=='pre_heading' and abs(heading_error)<=.12:
            self.phase='approach'
            self.approach_origin=xy.copy();self.approach_length=-along;self.approach_lateral=lateral
        if self.phase in ('terminal_heading','settling') and not final_centered:
            self.phase='approach';self.settled_since=None;self.settled_samples=0
        if self.phase=='approach' and final_centered and along>=-.015 and abs(heading_error)<=.35:
            self.phase='settling' if abs(heading_error)<=.12 else 'terminal_heading'
        if self.phase=='terminal_heading' and abs(heading_error)<=.12:self.phase='settling'
        if self.phase=='settling' and abs(heading_error)>.12:
            self.phase='terminal_heading';self.settled_since=None;self.settled_samples=0
        if self.phase!=previous:self.best=float('inf');self.improved=now
        target=self.west_checkpoint if self.phase=='west_clearance' else self.turn_checkpoint
        if self.phase in ('west_clearance','pre_position'):
            dx,dy=target-xy;bearing=self.angle(math.atan2(dy,dx)-yaw)
            metric=float(np.linalg.norm(target-xy))+.2*abs(bearing)
        elif self.phase=='approach':
            # Smooth connection to the flight axis: both ends have southward
            # tangent. Extend the target along that axis past the checkpoint
            # so it never turns back at a tiny terminal XY goal.
            progress=float((xy-self.approach_origin)@self.forward)
            ahead=progress+.40
            u=float(np.clip(ahead/self.approach_length,0.,1.))
            blend=u*u*(3.-2.*u)
            target=self.approach_origin+ahead*self.forward-self.approach_lateral*blend*self.normal
            dx,dy=target-xy;bearing=self.angle(math.atan2(dy,dx)-yaw)
            metric=max(-along,0.)+abs(lateral)*2.+abs(heading_error)*.2
        else:
            bearing=heading_error;target=self.checkpoint
            metric=abs(heading_error)
        result['maneuver_target_xy']=target.tolist()
        if new and metric<self.best-.02:self.best=metric;self.improved=now
        if now-self.improved>30:return dict(result,phase=self.phase,failure_reason='native entry alignment made no measured progress')
        if not level:return dict(result,phase=self.phase,reason='waiting_for_level_floor_pose')
        if self.phase in ('west_clearance','pre_position'):
            # Keep the initial clearance motion pointed away from the upper
            # pedestrian corridor. The measured retry yaw ~2.69 otherwise
            # drove north at .13 m/s while aiming for a same-Y west point.
            translation_heading_limit=.12 if self.phase=='west_clearance' else .55
            if abs(bearing)>translation_heading_limit:
                result['angular_z']=math.copysign(float(np.clip(1.5*abs(bearing),.85,.90)),bearing)
            else:
                # The flat policy stalls at the former .13 m/s minimum.
                result['linear_x']=.30
                result['angular_z']=float(np.clip(1.5*bearing,-.65,.65))
        elif self.phase=='pre_heading':
            result['angular_z']=math.copysign(float(np.clip(1.5*abs(heading_error),.85,.90)),heading_error)
        elif self.phase=='approach':
            if along>.20 or abs(lateral)>1.05:
                return dict(result,phase=self.phase,failure_reason='native forward entry left the measured flat approach corridor')
            if abs(heading_error)>.70:
                return dict(result,phase=self.phase,failure_reason='native forward entry lost its pre-aligned heading')
            result['linear_x']=.30
            result['angular_z']=float(np.clip(1.5*bearing,-.65,.65))
        elif self.phase=='terminal_heading':
            # Large turns are confined to the remote apron. Only a small
            # residual correction is allowed at the final entry checkpoint.
            if abs(heading_error)>.35:
                return dict(result,phase=self.phase,failure_reason='native terminal entry needs a new wide-floor pre-turn')
            result['angular_z']=math.copysign(.60,heading_error)
        elif new:
            moved=self.settled_since is None or self.settled_xy is None or np.linalg.norm(xy-self.settled_xy)>.03 or abs(self.angle(yaw-self.settled_yaw))>.04
            if moved:
                self.settled_since=now;self.settled_xy=xy.copy();self.settled_yaw=yaw;self.settled_samples=1
            else:self.settled_samples+=1
            result['ready']=bool(now-self.settled_since>=.75 and self.settled_samples>=5 and final_centered and not yielding)
        result.update(phase=self.phase,stationary_observations=self.settled_samples)
        return result


class PathRunner(Runner):
    def __init__(self, out, source, planner, start_stage=0):
        self.planner = planner
        self.legs = json.loads(source.read_text())['planners'][planner]
        self.index = 0
        self.clusters = []
        self.previous_clusters = []
        self.lidar_received = 0.
        self.lidar_wait_started = None
        self.last_cluster = 0.
        self.yielding = False
        self.escaping = False
        self.yield_started = None
        self.escape_origin = None
        self.escape_started = None
        self.escape_exhausted = False
        self.escape_direction = None
        self.observed_world = np.empty((0,3))
        self.observed_stamp = None
        self.clear_since = None
        self.yield_count = 0
        self.minimum_candidate_distance = float('inf')
        self.tracking = []
        self.next_track_id = 1
        self.preview_command = Twist()
        self.lookahead_target = None
        self.trail_sample = None
        self.door_alignment = None
        self.native_entry = None
        self.pose_stamp = None
        self.start_stage = start_stage
        self.lidar_sensor = json.loads((R/'matrix/config/config.json').read_text())['robot']['sensors']['lidar']
        super().__init__(out, False)
        self.i = start_stage-1
        self.create_subscription(PointCloud2, '/livox/lidar_raw', self.scan, qos_profile_sensor_data)
        self.markers = self.create_publisher(MarkerArray, '/wavego/global/markers', 10)
        self.create_timer(.2, self.visual)

    def event(self, state, **kwargs):
        super().event(state, planner=self.planner, **kwargs)
        p = self.out / 'result.json'
        result = json.loads(p.read_text())
        result.update(controller='Nav2 path sample tracking + existing RL terrain locomotion', planner=self.planner,
                      start_stage_index=self.start_stage,
                      mission_total_stages=len(self.stages),
                      total_stages=len(self.stages)-self.start_stage,
                      completed_stages=sum(e['state']=='stage_completed' for e in self.events),
                      pedestrian_guard='Current LiDAR geometric clusters with stop/yield and prediction; not neural class labels',
                      yield_count=self.yield_count,
                      minimum_candidate_center_distance_m=self.minimum_candidate_distance if math.isfinite(self.minimum_candidate_distance) else None)
        p.write_text(json.dumps(result, indent=2))

    def odom(self, m):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        roll,pitch,yaw=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz')
        self.pose=dict(x=p.x,y=p.y,z=p.z,roll=roll,pitch=pitch,yaw=yaw)
        self.received=time.monotonic()
        self.pose_stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if self.state=='running':
            self.rows.append(dict(time=time.time(),**self.pose))
            xyz=np.array([p.x,p.y,p.z])
            # Publish a smooth measured trace, not every 200 Hz odometry
            # message. An unbounded high-rate Path eventually blocks the
            # executor while serializing and starves safety callbacks.
            if self.trail_sample is None or np.linalg.norm(xyz-self.trail_sample)>.05:
                self.trail.poses.append(PoseStamped(header=m.header,pose=m.pose.pose))
                self.trail_sample=xyz

    def next(self):
        self.index = 0
        self.door_alignment = None
        self.native_entry = None
        self.i += 1
        if self.i >= len(self.stages):
            self.i = len(self.stages)-1
            self.finish('succeeded')
            return
        s = self.stages[self.i]
        frame = s['frame'] or (0, 0, 0, 1, 10)
        self.atomic('.run/go2w/stair_frame', s['id']+' '+' '.join(map(str, frame))+'\n')
        if self.i == 1:
            # Keep the validated stair gait until the rear wheels clear the
            # final riser; the old 1.95 m frame released before full ascent.
            self.atomic('.run/go2w/stair_frame',
                        f'large_stair_crest 7.4 0 {math.pi} 2.40 1.40\n')
        self.pub.publish(Twist())
        self.preview_command = Twist()
        self.lookahead_target = None
        self.atomic('.run/go2w/nav_mode', s['mode']+'\n')
        # Never block the executor while ROS parameter services run: doing so
        # stalls odometry/obstacle callbacks and falsely triggers stale guards.
        self.mode_process = subprocess.Popen(
            ['/usr/bin/python3', str(R/'scripts/set_go2w_nav_mode.py'), 'flat' if s['id']=='native_stair_approach' else s['mode']],
            stdout=subprocess.DEVNULL,
            env=dict(os.environ, STAIR_NAV_MAX_VELOCITY='[0.8,0.0,0.75]',
                     STAIR_NAV_MIN_VELOCITY='[-0.2,0.0,-0.75]',
                     SLOPE_NAV_MAX_VELOCITY='[0.65,0.0,0.55]'))
        self.mode_started = time.monotonic()
        self.state = 'switching'
        self.best = 1e9
        self.stage_start = dict(self.pose)

    def finish_switch(self, now):
        self.pub.publish(Twist())
        code = self.mode_process.poll()
        if now-self.mode_started > 15:
            self.mode_process.kill()
            self.finish('failed', 'terrain parameter switch timed out')
        elif code is not None:
            if code != 0:
                self.finish('failed', f'terrain parameter switch failed: {code}')
            elif now-self.received < .5:
                self.state = 'running'
                self.last_progress = now
                self.event('stage_starting')

    def finish(self, status, reason=''):
        self.atomic('.run/go2w/native_dynamic_yield', f'{time.time()+1:.6f} 1\n')
        super().finish(status, reason)
        (self.out / 'path_tracking.json').write_text(json.dumps(self.tracking))

    def scan(self, m):
        now = time.monotonic()
        self.lidar_received = now
        if self.pose is None or now - self.last_cluster < .16:
            return
        self.last_cluster = now
        p = self.pose
        if p['z'] > 1.0 or p['x'] > -.8:
            self.clusters = []
            return
        a = point_cloud2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True)
        xyz = np.column_stack([a[k] for k in ('x', 'y', 'z')])
        rot = Rotation.from_euler('xyz', [p['roll'], p['pitch'], p['yaw']]).as_matrix()
        xyz = body_points(xyz, self.lidar_sensor) @ rot.T + [p['x'], p['y'], p['z']]
        self.observed_world = xyz[np.isfinite(xyz).all(axis=1)]
        self.observed_stamp = m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        xyz = xyz[(xyz[:, 0] > -5.6) & (xyz[:, 0] < -2.5) & (xyz[:, 1] > -3.3) & (xyz[:, 1] < 4.7) & (xyz[:, 2] > .20) & (xyz[:, 2] < 2.1)]
        # Remove returns on known static surfaces, including the native stairs.
        for name, kind, center, size, rotation in GEOMS:
            if not len(xyz):
                break
            if kind != 'box' or center[0] > -.5 or center[0] < -8:
                continue
            local = (xyz - center) @ rotation
            xyz = xyz[~np.all(np.abs(local) < size + .07, axis=1)]
        clusters = []
        used_ids = set()
        if len(xyz) >= 10:
            ij = np.floor((xyz[:, :2] - [-6., -3.5]) / .12).astype(int)
            grid = np.zeros((70, 32), bool)
            grid[ij[:, 1], ij[:, 0]] = True
            ids, _ = label(binary_dilation(grid), np.ones((3, 3)))
            groups = ids[ij[:, 1], ij[:, 0]]
            for k in np.unique(groups):
                pts = xyz[groups == k]
                extent = np.ptp(pts, axis=0)
                # The low-mounted MID360 often sees only legs at close range.
                # Do not demand a complete standing human silhouette.
                if len(pts) < 18 or extent[2] < .16 or max(extent[:2]) > 1.8:
                    continue
                center = np.median(pts[:, :2], axis=0)
                v = np.zeros(2)
                track_id = None
                if self.previous_clusters:
                    available = [c for c in self.previous_clusters if c.get('track_id') not in used_ids]
                    if available:
                        old = min(available, key=lambda x: np.linalg.norm(center-x['xy']))
                        dt = now-old['time']
                        if 0.05 < dt < .8 and np.linalg.norm(center-old['xy']) < .45:
                            track_id = old.get('track_id')
                            v = .6*np.array(old['velocity'])+.4*(center-old['xy'])/dt
                            if np.linalg.norm(v) > 1.:v /= np.linalg.norm(v)
                if track_id is None:
                    track_id = self.next_track_id; self.next_track_id += 1
                used_ids.add(track_id)
                clusters.append(dict(track_id=track_id,xy=center.tolist(), velocity=v.tolist(), height=float(pts[:, 2].max()), extent=extent.tolist(), radius=max(.30, float(max(extent[:2]))*.5), points=len(pts), time=now))
        # Bridge a brief scan occlusion without inventing observations.
        for old in self.clusters:
            if now-old['time'] < .55 and all(np.linalg.norm(np.array(old['xy'])-c['xy']) > .6 for c in clusters):
                clusters.append(old)
        self.previous_clusters = self.clusters
        self.clusters = clusters

    def guard(self, cmd, now):
        self.escaping = False
        p = self.pose
        # Complete the descending ramp before stopping. A static stand pose on
        # its incline is unstable; pedestrians cross on the floor beyond it.
        active = self.i in (5, 6, 7) and p['x'] < -2.7 and p['z'] < .48
        if not active:
            return cmd
        if now - self.lidar_received > .8:
            self.yielding = True
            self.clear_since = None
            self.last_progress = now
            if self.lidar_wait_started is None:
                self.lidar_wait_started = now
                self.event('lidar_wait')
            elif now-self.lidar_wait_started > 5:
                self.finish('failed', 'LiDAR unavailable for 5 seconds in pedestrian crossing')
            return Twist()
        if self.lidar_wait_started is not None:
            self.lidar_wait_started = None
            self.event('lidar_resumed')
        stop = False
        for c in self.clusters:
            rel = np.array(c['xy'])-[p['x'], p['y']]
            dist = float(np.linalg.norm(rel))
            self.minimum_candidate_distance = min(self.minimum_candidate_distance, dist)
            vel = np.array(c['velocity'])
            # Predict pedestrian motion relative to the robot's intended forward motion.
            relative_v = vel-cmd.linear.x*np.array([math.cos(p['yaw']), math.sin(p['yaw'])])
            t = float(np.clip(-rel@relative_v / max(relative_v@relative_v, .02), 0, 1.4))
            prediction = np.linalg.norm(rel+t*relative_v)
            ahead = rel @ np.array([math.cos(p['yaw']), math.sin(p['yaw'])]) > -.3
            radius = c.get('radius', .30)
            limit = radius+.95
            c['risk'] = bool(ahead and (dist < limit+(.10 if self.yielding else 0) or prediction < radius+.85))
            c['prediction_distance_m'] = float(prediction)
            if c['risk']:
                stop = True
        if stop:
            self.clear_since = None
            if not self.yielding:
                self.yielding = True
                self.yield_count += 1
                self.yield_started = now
                self.escape_origin = None
                self.escape_started = None
                self.escape_exhausted = False
                self.event('pedestrian_yield', clusters=self.clusters)
        elif self.yielding:
            if self.clear_since is None:self.clear_since = now
            if now-self.clear_since > .65:
                self.yielding = False
                self.yield_started = None
                self.event('pedestrian_clear', clusters=self.clusters)
        if self.yielding:
            self.last_progress = now
            recoverable=(self.i==7 and self.native_entry is not None
                         and self.native_entry.phase in ('west_clearance','pre_position','pre_heading')
                         and self.yield_started is not None and now-self.yield_started>=3.
                         and now-self.lidar_received<.30 and self.observed_stamp is not None
                         and self.pose_stamp is not None and abs(self.pose_stamp-self.observed_stamp)<.30)
            if recoverable and not self.escape_exhausted:
                xy=np.array([p['x'],p['y']])
                if self.escape_origin is not None and (np.linalg.norm(xy-self.escape_origin)>.30 or now-self.escape_started>6.
                        or (getattr(self,'escape_direction',None) is not None
                            and ((xy-self.escape_origin)@self.escape_direction < -.075
                                 or (now-self.escape_started>1. and (xy-self.escape_origin)@self.escape_direction < -.035)))):
                    self.escape_exhausted=True
                    self.event('local_escape_bounded_stop',reason='Recovery distance/time limit or measured motion opposing the safe retreat; stop and wait')
                else:
                    choice=choose_reverse_escape(p,self.clusters,GEOMS,self.observed_world,cmd.angular.z)
                    if choice is not None:
                        if self.escape_origin is None:
                            self.escape_origin=xy.copy();self.escape_started=now
                            direction=np.array(choice['endpoint'])-xy
                            self.escape_direction=direction/max(np.linalg.norm(direction),1e-9)
                            self.event('local_escape_started',recovery='Reverse only, checked flat geometry and current LiDAR; pedestrian separation must increase',**choice)
                        self.escaping=True
                        recovery=Twist();recovery.linear.x=choice['linear_x'];recovery.angular.z=choice['angular_z']
                        return recovery
            return Twist()
        return cmd

    def tick(self):
        self.atomic('.run/go2w/native_dynamic_yield', f'{time.time()+1:.6f} {int((self.yielding and not self.escaping) or self.state in ("succeeded", "failed"))}\n')
        if self.state in ('succeeded', 'failed'):
            self.pub.publish(Twist())
            return
        now = time.monotonic()
        if time.time()-self.start > 540:
            self.finish('failed', 'mission timeout')
            return
        if self.pose is None:return
        if self.state == 'switching':
            self.finish_switch(now)
            return
        if now-self.received > 1:
            self.finish('failed', 'odom stale')
            return
        if self.state == 'ready':
            self.next()
            return
        p, s = self.pose, self.stages[self.i]
        dist = math.hypot(s['x']-p['x'], s['y']-p['y'])
        if abs(p['roll']) > .8 or abs(p['pitch']) > .95 or p['z'] < .15:
            self.finish('failed', 'attitude or height safety')
            return
        if self.i == 0 and dist < .18:
            err = math.atan2(math.sin(math.pi-p['yaw']), math.cos(math.pi-p['yaw']))
            if abs(err) > .08:
                cmd = Twist();cmd.angular.z = float(np.clip(1.5*err, -.5, .5))
                self.pub.publish(cmd);self.last_progress = now
                return
        if s['id']=='native_stair_approach':
            # Start this stage's remote pre-turn immediately. Waiting until
            # the old near-riser goal is close would invoke the old path
            # chase first and can prevent the manoeuvre from ever starting.
            if self.native_entry is None:
                self.native_entry=NativeStairEntry(s,self.stages[self.i+1])
                # Explicit stage-7 continuation can retain an already aligned
                # real checkpoint pose. Full runs still execute the wide turn.
                if self.start_stage==7:
                    xy=np.array([p['x'],p['y']]);entry=self.native_entry
                    lateral=float((xy-entry.frame_origin)@entry.normal)
                    along=float((xy-entry.checkpoint)@entry.forward)
                    if (abs(lateral)<=.12 and -.20<=along<=.15 and abs(p['z']-.4)<=.12
                        and abs(entry.angle(entry.heading-p['yaw']))<=.12
                        and max(abs(p['roll']),abs(p['pitch']))<.25):
                        entry.phase='approach';entry.approach_origin=xy.copy()
                        entry.approach_length=max(-along,.40);entry.approach_lateral=lateral
                        self.event('native_entry_aligned_continuation',source='Fresh actual odometry; no pose reset or completed-stage inference')
                self.event('native_entry_centering_started',original_approach_goal=[s['x'],s['y'],s['z']],entry_checkpoint=[*self.native_entry.checkpoint.tolist(),self.native_entry.height],pre_turn_checkpoint=[*self.native_entry.turn_checkpoint.tolist(),self.native_entry.height],flight_heading_rad=self.native_entry.heading,checkpoint_source='Derived wide-floor pre-turn and forward entry manoeuvre, not an upstream Nav2 path sample')
            if self.native_entry is not None:
                self.align_native_entry(now)
                return
        tolerance = .18 if self.i == 0 else (.38 if self.i == 1 else .25)
        if s['id'] == 'second_floor_door' and abs(p['z']-s['z']) < .24:
            if self.door_alignment is not None or dist < .08:
                if self.align_door(now, dist):return
        # A close body center alone does not mean the rear wheels have climbed
        # onto the platform. Require full progress and heading before descent.
        crest_ready = self.i != 1 or (
            p['x'] <= 5.15 and
            abs(math.atan2(math.sin(math.pi-p['yaw']), math.cos(math.pi-p['yaw']))) < .25)
        if s['id'] not in ('second_floor_door','native_stair_approach') and dist < tolerance and abs(p['z']-s['z']) < .24 and crest_ready:
            self.event('stage_completed')
            self.next()
            return
        path = np.array(self.legs[self.i]['raw_xy'])
        distances = np.linalg.norm(path[self.index:] - [p['x'], p['y']], axis=1)
        self.index += int(np.argmin(distances))
        j = self.index
        # A short target near the ramp entrance can request a pure turn.
        # The slope policy turns reliably with forward motion, so preview
        # farther along the same upstream path on ramps.
        lookahead = 1.0 if s['mode'] == 'slope' else (.65 if s['mode'] in ('up', 'down') else .45)
        arc = 0.
        while j < len(path)-1 and arc < lookahead:
            arc += np.linalg.norm(path[j+1]-path[j]);j += 1
        dx, dy = path[j]-[p['x'], p['y']]
        self.lookahead_target = [float(path[j][0]), float(path[j][1]), float(p['z']+.15)]
        err = math.atan2(math.sin(math.atan2(dy, dx)-p['yaw']), math.cos(math.atan2(dy, dx)-p['yaw']))
        metric = dist+abs(s['z']-p['z'])*.3
        if metric < self.best-.07:self.best = metric;self.last_progress = now
        if now-self.last_progress > 35:
            self.finish('failed', 'no progress');return
        cmd = Twist();cmd.angular.z = float(np.clip(1.5*err, -.70, .70))
        speed = .65 if s['mode'] == 'up' else (.30 if s['mode'] == 'down' else (.6 if s['mode'] == 'slope' else .4))
        if s['id'] == 'second_floor_door':speed=.30
        cmd.linear.x = min(speed, max(.13, dist*.7))*max(0, math.cos(err)) if abs(err) < .6 else 0.
        if s['id'] == 'second_floor_door' and abs(err)<.6:
            # Low commands stall this locomotion policy before the door. Move
            # close before requesting stand; stand itself shifts the body back.
            cmd.linear.x=.30*max(0,math.cos(err))
        cmd = self.guard(cmd, now)
        self.atomic('.run/go2w/native_dynamic_yield', f'{time.time()+1:.6f} {int(self.yielding and not self.escaping)}\n')
        self.publish_tracking_command(cmd, now, float(distances.min()))

    def align_native_entry(self,now):
        entry=self.native_entry;previous=entry.phase
        result=entry.step(self.pose,now,self.pose_stamp,yielding=self.yielding)
        if 'failure_reason' in result:
            self.finish('failed',result['failure_reason']);return
        if result['phase']!=previous:
            self.event('native_entry_alignment_phase',entry_phase=result['phase'],entry_checkpoint=[*entry.checkpoint.tolist(),entry.height],lateral_center_error_m=result['lateral_center_error_m'],heading_error_rad=result['heading_error_rad'])
        cmd=Twist();cmd.linear.x=result['linear_x'];cmd.angular.z=result['angular_z']
        self.lookahead_target=[*result.get('maneuver_target_xy',entry.checkpoint.tolist()),float(self.pose['z']+.15)]
        cmd=self.guard(cmd,now)
        self.atomic('.run/go2w/native_dynamic_yield',f'{time.time()+1:.6f} {int(self.yielding and not self.escaping)}\n')
        self.publish_tracking_command(cmd,now,result['checkpoint_xy_distance_m'])
        if result['ready'] and not self.yielding:
            self.event('stage_completed',entry_checkpoint=[*entry.checkpoint.tolist(),entry.height],original_approach_goal=[self.stages[self.i][key] for key in ('x','y','z')],lateral_center_error_m=result['lateral_center_error_m'],heading_error_rad=result['heading_error_rad'],entry_checkpoint_distance_m=result['checkpoint_xy_distance_m'],stationary_confirmation_sec=now-entry.settled_since,distinct_stationary_odometry_samples=result['stationary_observations'])
            self.next()

    def publish_tracking_command(self, cmd, now, cross_track=0.):
        """Record the applied command in both tracking and the live preview."""
        self.preview_command = cmd
        self.pub.publish(cmd)
        if now-self.lastwrite > .3:
            self.lastwrite = now
            self.trail.header.stamp = self.get_clock().now().to_msg()
            self.pathpub.publish(self.trail)
            self.tracking.append(dict(time=time.time(), stage=self.i, path_index=self.index, cross_track_m=cross_track, pose=dict(self.pose), cmd=[cmd.linear.x, cmd.angular.z], yielding=self.yielding, escaping=self.escaping, clusters=self.clusters, door_align=self.door_alignment is not None))

    def align_door(self, now, distance):
        """Latch terminal docking, with bounded measured heading convergence."""
        heading = math.atan2(math.sin(-self.pose['yaw']), math.cos(-self.pose['yaw']))
        if self.door_alignment is None:
            self.door_alignment = dict(started=now, improved=now, best=abs(heading), settled=None, settled_pose=None, stand_started=None)
            self.event('door_heading_start', heading_error_rad=heading)
        alignment = self.door_alignment
        if now-alignment['started'] > 30:
            self.finish('failed', 'door heading alignment timed out')
            return True
        if abs(heading) < alignment['best']-.03:
            alignment['best']=abs(heading);alignment['improved']=now
        if abs(heading) > .12 and now-alignment['improved'] > 12:
            self.finish('failed', 'door heading made no measured progress')
            return True
        if distance > .40:
            self.door_alignment = None
            self.event('door_alignment_reposition', goal_distance_m=distance)
            return False
        cmd = Twist()
        if abs(heading) > .12:
            # The existing RL policy turns poorly at .4 in-place. Route turns
            # already use .7; keep a smaller effective .6-.65 for docking.
            cmd.angular.z=math.copysign(float(np.clip(1.3*abs(heading), .60, .65)),heading)
            alignment['settled']=None
        elif distance <= .38:
            if alignment['stand_started'] is None:alignment['stand_started']=now
            xy=np.array([self.pose['x'],self.pose['y']])
            if alignment['settled'] is None or np.linalg.norm(xy-alignment['settled_pose'])>.03:
                alignment['settled']=now;alignment['settled_pose']=xy
            elif now-alignment['settled'] >= 1. and now-alignment['stand_started'] >= 2.:
                self.publish_tracking_command(cmd, now, distance)
                self.event('stage_completed', heading_error_rad=heading, goal_distance_m=distance, stationary_confirmation_sec=1., stand_transition_observed_sec=now-alignment['stand_started'])
                self.next()
                return True
        else:
            self.door_alignment=None
            return False
        self.publish_tracking_command(cmd, now, distance)
        return True

    def visual(self):
        if self.pose is None:return
        p = self.pose
        h = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
        markers = [Marker(header=h, action=Marker.DELETEALL)]
        if self.state == 'running' and self.lookahead_target is not None:
            target = Marker(header=h, ns='navigation_lookahead', id=1, type=Marker.SPHERE, action=Marker.ADD)
            target.pose.orientation.w=1.
            target.pose.position.x,target.pose.position.y,target.pose.position.z=self.lookahead_target
            target.scale.x=target.scale.y=target.scale.z=.25
            target.color.r,target.color.g,target.color.b,target.color.a=1.,1.,1.,1.
            markers.append(target)
            # Short unicycle rollouts preview steering samples. They are not
            # an MPPI optimizer or a prediction of the RL stair dynamics.
            for index,offset in enumerate(np.linspace(-.45,.45,7)):
                line=Marker(header=h,ns='navigation_rollout_samples',id=index,type=Marker.LINE_STRIP,action=Marker.ADD)
                line.pose.orientation.w=1.;line.scale.x=.07 if index==3 else .025
                x,y,yaw=p['x'],p['y'],p['yaw'];risk=False
                for step in range(17):
                    t=step*.15
                    line.points.append(Point(x=float(x),y=float(y),z=float(p['z']+.12)))
                    for c in self.clusters:
                        xy=np.array(c['xy'])+t*np.array(c['velocity'])
                        if np.linalg.norm(xy-[x,y]) < c.get('radius',.3)+.85:risk=True
                    yaw+=(self.preview_command.angular.z+float(offset))*.15
                    x+=self.preview_command.linear.x*math.cos(yaw)*.15
                    y+=self.preview_command.linear.x*math.sin(yaw)*.15
                line.color.r,line.color.g,line.color.b,line.color.a=(1.,.2,.12,.5) if risk else ((0.,1.,1.,1.) if index==3 else (.55,.72,1.,.45))
                markers.append(line)
                head=Marker(header=h,ns='navigation_sampling_heads',id=index,type=Marker.SPHERE,action=Marker.ADD)
                head.pose.orientation.w=1.;head.pose.position=line.points[-1]
                head.scale.x=head.scale.y=head.scale.z=.13
                head.color=line.color;markers.append(head)
        for c in self.clusters:
            i = c.get('track_id', 0)
            box = Marker(header=h, ns='measured_dynamic', id=i, type=Marker.CYLINDER, action=Marker.ADD)
            box.pose.orientation.w = 1.;box.pose.position.x, box.pose.position.y = c['xy'];box.pose.position.z = .65
            box.scale.x = box.scale.y = c.get('radius', .3)*2;box.scale.z = 1.3
            box.color.r, box.color.g, box.color.b, box.color.a = 1., .2, .1, .28
            markers.append(box)
            danger = c.get('risk', False)
            halo = Marker(header=h, ns='dynamic_guard_radius', id=i, type=Marker.LINE_STRIP, action=Marker.ADD)
            halo.pose.orientation.w = 1.; halo.scale.x = .065
            halo.color.r, halo.color.g, halo.color.b, halo.color.a = (1., .12, .1, .95) if danger else (1., .15, .85, .85)
            radius = c.get('radius', .3)+.95
            halo.points = [Point(x=float(c['xy'][0]+radius*math.cos(a)),y=float(c['xy'][1]+radius*math.sin(a)),z=.12) for a in np.linspace(0,2*math.pi,49)]
            markers.append(halo)
            arrow = Marker(header=h, ns='dynamic_velocity_prediction', id=i, type=Marker.ARROW, action=Marker.ADD)
            arrow.pose.orientation.w = 1.; arrow.scale.x = .065; arrow.scale.y = .16; arrow.scale.z = .18
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1., .3, .8, .95
            arrow.points = [Point(x=float(c['xy'][0]),y=float(c['xy'][1]),z=.25),Point(x=float(c['xy'][0]+1.4*c['velocity'][0]),y=float(c['xy'][1]+1.4*c['velocity'][1]),z=.25)]
            if np.linalg.norm(c['velocity']) > .03:markers.append(arrow)
            label = Marker(header=h, ns='dynamic_labels', id=i, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
            label.pose.orientation.w = 1.; label.pose.position.x, label.pose.position.y = c['xy'];label.pose.position.z=1.65
            label.scale.z = .23;label.color.r=label.color.g=label.color.b=label.color.a=1.
            distance = np.linalg.norm(np.array(c['xy'])-[p['x'],p['y']])
            label.text = f"Pedestrian candidate #{i}\nv={np.linalg.norm(c['velocity']):.2f} m/s | d={distance:.2f} m\n"+('YIELD / collision risk' if danger else 'TRACKING')
            markers.append(label)
        text = Marker(header=h, ns='status', id=100, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
        text.pose.orientation.w = 1.;text.pose.position.x = p['x'];text.pose.position.y = p['y'];text.pose.position.z = p['z']+1.2
        text.scale.z = .20;text.color.r = text.color.g = text.color.b = text.color.a = 1.
        text.text = f'{self.planner} | {min(self.i+1, len(self.stages))}/{len(self.stages)}\n'+('REVERSE ESCAPE - checked clearance' if self.escaping else 'YIELD - measured dynamic obstacle' if self.yielding else self.state.upper())
        markers.append(text)
        for marker in markers:
            if marker.action != Marker.DELETEALL:marker.lifetime=Duration(sec=1)
        self.markers.publish(MarkerArray(markers=markers))

if __name__ == '__main__':
    a = argparse.ArgumentParser()
    a.add_argument('--routes', type=Path, required=True)
    a.add_argument('--planner', choices=['NavFnDijkstra', 'SmacAStar'], required=True)
    a.add_argument('--output', type=Path, required=True)
    a.add_argument('--start-stage', type=int, default=0, choices=range(15),
                   help='Explicit continuation from the current measured pose; does not move/reset the simulator')
    args = a.parse_args()
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    n = PathRunner(args.output, args.routes, args.planner, args.start_stage)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        if n.state not in ('succeeded', 'failed'):n.finish('canceled', 'interrupted')
    finally:
        n.pub.publish(Twist())
        n.destroy_node()
        rclpy.try_shutdown()
