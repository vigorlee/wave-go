"""Follow actual Nav2 path samples with the validated terrain RL controller.

Dynamic crossing candidates come from current measured LiDAR returns, never a
clock-based simulated-person marker. Geometric candidate labels are not an
assertion of neural semantic classification.
"""
import argparse
import json
import math
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
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from maps import GEOMS, R
sys.path.insert(0, str(R / 'scripts'))
from go2w_office_inline_run import Runner

class PathRunner(Runner):
    def __init__(self, out, source, planner):
        self.planner = planner
        self.legs = json.loads(source.read_text())['planners'][planner]
        self.index = 0
        self.clusters = []
        self.previous_clusters = []
        self.lidar_received = 0.
        self.last_cluster = 0.
        self.yielding = False
        self.clear_since = None
        self.yield_count = 0
        self.minimum_candidate_distance = float('inf')
        self.tracking = []
        super().__init__(out, False)
        self.create_subscription(PointCloud2, '/livox/lidar_raw', self.scan, qos_profile_sensor_data)
        self.markers = self.create_publisher(MarkerArray, '/wavego/global/markers', 10)
        self.create_timer(.2, self.visual)

    def event(self, state, **kwargs):
        super().event(state, planner=self.planner, **kwargs)
        p = self.out / 'result.json'
        result = json.loads(p.read_text())
        result.update(controller='Nav2 path sample tracking + existing RL terrain locomotion', planner=self.planner,
                      pedestrian_guard='Current LiDAR geometric clusters with stop/yield and prediction; not neural class labels',
                      yield_count=self.yield_count,
                      minimum_candidate_center_distance_m=self.minimum_candidate_distance if math.isfinite(self.minimum_candidate_distance) else None)
        p.write_text(json.dumps(result, indent=2))

    def next(self):
        self.index = 0
        super().next()

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
        xyz = (xyz + [.13011, -.02329, .17598]) @ rot.T + [p['x'], p['y'], p['z']]
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
                if self.previous_clusters:
                    old = min(self.previous_clusters, key=lambda x: np.linalg.norm(center-x['xy']))
                    dt = now-old['time']
                    if 0.05 < dt < .8 and np.linalg.norm(center-old['xy']) < .45:
                        v = .6*np.array(old['velocity'])+.4*(center-old['xy'])/dt
                        if np.linalg.norm(v) > 1.:v /= np.linalg.norm(v)
                clusters.append(dict(xy=center.tolist(), velocity=v.tolist(), height=float(pts[:, 2].max()), extent=extent.tolist(), radius=max(.30, float(max(extent[:2]))*.5), points=len(pts), time=now))
        # Bridge a brief scan occlusion without inventing observations.
        for old in self.clusters:
            if now-old['time'] < .55 and all(np.linalg.norm(np.array(old['xy'])-c['xy']) > .6 for c in clusters):
                clusters.append(old)
        self.previous_clusters = self.clusters
        self.clusters = clusters

    def guard(self, cmd, now):
        p = self.pose
        # Complete the descending ramp before stopping. A static stand pose on
        # its incline is unstable; pedestrians cross on the floor beyond it.
        active = self.i in (5, 6, 7) and p['x'] < -2.7 and p['z'] < .48
        if not active:
            return cmd
        if now - self.lidar_received > .8:
            self.pub.publish(Twist())
            self.finish('failed', 'LiDAR stale in pedestrian crossing')
            return Twist()
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
            if ahead and (dist < limit+(.10 if self.yielding else 0) or prediction < radius+.85):
                stop = True
        if stop:
            self.clear_since = None
            if not self.yielding:
                self.yielding = True
                self.yield_count += 1
                self.event('pedestrian_yield', clusters=self.clusters)
        elif self.yielding:
            if self.clear_since is None:self.clear_since = now
            if now-self.clear_since > .65:
                self.yielding = False
                self.event('pedestrian_clear', clusters=self.clusters)
        if self.yielding:
            self.last_progress = now
            return Twist()
        return cmd

    def tick(self):
        self.atomic('.run/go2w/native_dynamic_yield', f'{time.time()+1:.6f} {int(self.yielding or self.state in ("succeeded", "failed"))}\n')
        if self.state in ('succeeded', 'failed'):
            self.pub.publish(Twist())
            return
        now = time.monotonic()
        if self.pose is None:return
        if now-self.received > 1:
            self.finish('failed', 'odom stale')
            return
        if self.i < 0:
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
        tolerance = .18 if self.i == 0 else (.38 if self.i == 1 else .25)
        if dist < tolerance and abs(p['z']-s['z']) < .24:
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
        err = math.atan2(math.sin(math.atan2(dy, dx)-p['yaw']), math.cos(math.atan2(dy, dx)-p['yaw']))
        metric = dist+abs(s['z']-p['z'])*.3
        if metric < self.best-.07:self.best = metric;self.last_progress = now
        if now-self.last_progress > 35:
            self.finish('failed', 'no progress');return
        if time.time()-self.start > 540:
            self.finish('failed', 'mission timeout');return
        cmd = Twist();cmd.angular.z = float(np.clip(1.5*err, -.70, .70))
        speed = .65 if s['mode'] == 'up' else (.30 if s['mode'] == 'down' else (.6 if s['mode'] == 'slope' else .4))
        cmd.linear.x = min(speed, max(.13, dist*.7))*max(0, math.cos(err)) if abs(err) < .6 else 0.
        cmd = self.guard(cmd, now)
        self.atomic('.run/go2w/native_dynamic_yield', f'{time.time()+1:.6f} {int(self.yielding)}\n')
        self.pub.publish(cmd)
        if now-self.lastwrite > .3:
            self.lastwrite = now
            self.trail.header.stamp = self.get_clock().now().to_msg()
            self.pathpub.publish(self.trail)
            self.tracking.append(dict(time=time.time(), stage=self.i, path_index=self.index, cross_track_m=float(distances.min()), pose=p, cmd=[cmd.linear.x, cmd.angular.z], yielding=self.yielding, clusters=self.clusters))

    def visual(self):
        if self.pose is None:return
        p = self.pose
        h = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
        markers = [Marker(header=h, action=Marker.DELETEALL)]
        for i, c in enumerate(self.clusters):
            box = Marker(header=h, ns='measured_dynamic', id=i, type=Marker.CYLINDER, action=Marker.ADD)
            box.pose.orientation.w = 1.;box.pose.position.x, box.pose.position.y = c['xy'];box.pose.position.z = .65
            box.scale.x = box.scale.y = c.get('radius', .3)*2;box.scale.z = 1.3
            box.color.r, box.color.g, box.color.b, box.color.a = 1., .2, .1, .28
            markers.append(box)
        text = Marker(header=h, ns='status', id=100, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
        text.pose.orientation.w = 1.;text.pose.position.x = p['x'];text.pose.position.y = p['y'];text.pose.position.z = p['z']+1.2
        text.scale.z = .20;text.color.r = text.color.g = text.color.b = text.color.a = 1.
        text.text = f'{self.planner} | {min(self.i+1, 12)}/12\n'+('YIELD - measured dynamic obstacle' if self.yielding else self.state.upper())
        markers.append(text)
        self.markers.publish(MarkerArray(markers=markers))

if __name__ == '__main__':
    a = argparse.ArgumentParser()
    a.add_argument('--routes', type=Path, required=True)
    a.add_argument('--planner', choices=['NavFnDijkstra', 'SmacAStar'], required=True)
    a.add_argument('--output', type=Path, required=True)
    args = a.parse_args()
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    n = PathRunner(args.output, args.routes, args.planner)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        if n.state not in ('succeeded', 'failed'):n.finish('canceled', 'interrupted')
    finally:
        n.pub.publish(Twist())
        n.destroy_node()
        rclpy.try_shutdown()
