#!/usr/bin/python3
"""Run a fixed original course through the existing continuous Nav2 supervisor.

The route is user-selected, not a Cosmos inference. Reference visualization uses
the known course; the executed trail and terrain checks use measured odometry.
"""
import argparse
import csv
import json
import math
import os
import shutil
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path as NavPath
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from cosmos_vln_mission import CosmosVlnMission
from go2w_semantic_geometry import rotation

ROOT = Path(__file__).resolve().parents[1]


class ContinuousCourse(CosmosVlnMission):
    def __init__(self, output):
        self.output = output
        output.mkdir(parents=True, exist_ok=True)
        configuration = output / 'configuration'
        configuration.mkdir(exist_ok=True)
        for relative in ('config/go2w_wavegov1_route.json',
                         'config/go2w_continuous_nav.xml', 'matrix/config/config.json'):
            source = ROOT / relative
            shutil.copy2(source, configuration / source.name)
        self.events = []
        self.rows = []
        self.state = 'ready'
        self.last_record = 0.0
        self.plan_messages = 0
        super().__init__()
        self.course = self.routes['wavegov1_original_course']
        self.course_stages = self.route_stages(self.course)
        self.terrain = []
        scene = ROOT / 'matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_yard_mid360_course.xml'
        for g in ET.parse(scene).findall('./worldbody/geom'):
            name = g.get('name', '')
            if g.get('type') != 'box' or g.get('contype') == '0':
                continue
            if not name.startswith(('box_OBS', 'box2_OBS', 'extended_ramp_')):
                continue
            self.terrain.append((np.fromstring(g.get('pos', '0 0 0'), sep=' '),
                                 np.fromstring(g.get('size'), sep=' '),
                                 rotation(np.fromstring(g.get('euler', '0 0 0'), sep=' '))))
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.route_pub = self.create_publisher(NavPath, '/go2w/course/route', qos)
        self.trail_pub = self.create_publisher(NavPath, '/go2w/course/trail', qos)
        self.markers_pub = self.create_publisher(MarkerArray, '/go2w/course/markers', qos)
        self.create_subscription(NavPath, '/plan', self.on_plan, 10)
        self.visual_timer = self.create_timer(.5, self.publish_visuals)

    def on_plan(self, message):
        self.plan_messages += 1
        if message.poses:
            (self.output / 'latest_nav2_plan.json').write_text(json.dumps(dict(
                frame=message.header.frame_id, source='/plan',
                points=[[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in message.poses])))

    def on_odom(self, message):
        super().on_odom(message)
        if self.state in ('succeeded', 'failed', 'canceled', 'blocked'):
            return
        now = time.monotonic()
        if now - self.last_record < .045:
            return
        self.last_record = now
        p = self.robot_pose
        self.rows.append([time.time(), self.stage_index, p.x, p.y, p.z, p.roll, p.pitch, p.yaw])

    def publish_status(self, state, **details):
        super().publish_status(state, **details)
        self.state = state
        event = dict(time=time.time(), state=state, stage_index=self.stage_index,
                     stage_id=self.current_stage.stage_id if self.current_stage else '',
                     completed_stages=self.step, **details)
        self.events.append(event)
        with (self.output / 'events.jsonl').open('a') as f:
            f.write(json.dumps(event) + '\n')
        if state in ('stage_completed', 'succeeded', 'failed', 'canceled', 'blocked'):
            print(json.dumps(event), flush=True)
            self.save()

    def save(self):
        with (self.output / 'odom.csv').open('w') as f:
            w = csv.writer(f)
            w.writerow(['wall_time', 'stage_index', 'x_m', 'y_m', 'z_m', 'roll_rad', 'pitch_rad', 'yaw_rad'])
            w.writerows(self.rows)
        result = dict(passed=self.state == 'succeeded', state=self.state,
                      action_goal_count=self.continuous_action_goal_count,
                      completed_stages=self.step, total_stages=len(self.course_stages),
                      route_source='user_requested_fixed_catalog_not_model_inference',
                      nav2_plan_messages=self.plan_messages,
                      last_event=self.events[-1] if self.events else None,
                      ramp_physics=self.ramp_physics_details(), samples=len(self.rows))
        (self.output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')

    def start_course(self):
        if not self.navigate_through_client.wait_for_server(timeout_sec=10):
            raise RuntimeError('Nav2 action server unavailable')
        self.on_mission(String(data='Continuous original stairs, slalom and ramp course'))
        if not self.active:
            raise RuntimeError('Mission startup rejected; see events.jsonl')
        self.next_plan_at = None
        self.active_route = self.course
        self.active_stages = self.course_stages
        self.accepted_plan = {'source': 'user_requested_fixed_catalog', 'future_prediction': {}}
        self.begin_continuous_route()

    def height(self, x, y):
        heights = [0.0]
        for position, size, rot in self.terrain:
            z = position[2] + (size[2] - rot[0, 2] * (x-position[0]) - rot[1, 2] * (y-position[1])) / rot[2, 2]
            local = (np.array([x, y, z]) - position) @ rot
            if abs(local[0]) <= size[0]+.001 and abs(local[1]) <= size[1]+.001:
                heights.append(z)
        return max(heights) + .10

    def pose(self, x, y, z):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y, p.pose.position.z = float(x), float(y), float(z)
        p.pose.orientation.w = 1.
        return p

    def publish_visuals(self):
        route = NavPath()
        route.header.frame_id = 'map'
        route.header.stamp = self.get_clock().now().to_msg()
        waypoints = [(.65, 0.)] + [(s.x, s.y) for s in self.course_stages]
        for a, b in zip(waypoints, waypoints[1:]):
            for t in np.linspace(0, 1, max(2, int(math.dist(a, b)/.08))):
                x, y = np.array(a)*(1-t)+np.array(b)*t
                route.poses.append(self.pose(x, y, self.height(x, y)))
        self.route_pub.publish(route)
        trail = NavPath(header=route.header)
        trail.poses = [self.pose(r[2], r[3], r[4]+.10) for r in self.rows[::2]]
        self.trail_pub.publish(trail)
        markers = []
        names = {'climb_to_upper_landing':'STAIRS UP', 'descend_far_staircase':'STAIRS DOWN',
                 'slalom_exit_before_ramp':'SLALOM', 'ramp_up_crest':'RAMP UP',
                 'ramp_down_exit':'RAMP DOWN', 'course_exit':'FINISH'}
        for i, s in enumerate(self.course_stages):
            if s.stage_id not in names:
                continue
            m = Marker()
            m.header = route.header
            m.ns, m.id, m.type, m.action = 'course_targets', i, Marker.TEXT_VIEW_FACING, Marker.ADD
            m.pose.position = Point(x=s.x+.7, y=s.y, z=self.height(s.x,s.y)+.4)
            m.pose.orientation.w = 1.
            m.scale.z = .42
            m.color.r, m.color.g, m.color.b, m.color.a = (.95,.73,.2,1.) if i < self.step else (.3,.75,1.,1.)
            m.text = f'{i+1:02d} {names[s.stage_id]}'
            markers.append(m)
        m = Marker()
        m.header = route.header
        m.ns, m.id, m.type, m.action = 'course_status', 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        m.pose.position = Point(x=2.3, y=4., z=2.8)
        m.pose.orientation.w = 1.
        m.scale.z = .32
        m.color.r, m.color.g, m.color.b, m.color.a = 1.,.9,.7,1.
        m.text = f'Wavegov1 | {self.state}\n{self.step}/{len(self.course_stages)} stages | {self.continuous_action_goal_count} Nav2 action\nBLUE reference / AMBER measured trail'
        markers.append(m)
        self.markers_pub.publish(MarkerArray(markers=markers))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--preview-only', action='store_true')
    args = parser.parse_args()
    os.environ['COSMOS_VLN_ROUTES_FILE'] = str(ROOT/'config/go2w_wavegov1_route.json')
    os.environ['COSMOS_VLN_CONTINUOUS_ROUTE'] = '1'
    os.environ['COSMOS_VLN_CONTINUOUS_BT_XML'] = str(ROOT/'config/go2w_continuous_nav.xml')
    os.environ.setdefault('COSMOS_VLN_CONTINUOUS_PASS_RADIUS', '.50')
    os.environ.setdefault('COSMOS_VLN_MISSION_TIMEOUT_SEC', '360')
    os.environ.setdefault('COSMOS_VLN_ROUTE_NO_PROGRESS_SEC', '35')
    os.environ.setdefault('COSMOS_VLN_STAIR_MAX_HEADING', '.68')
    rclpy.init()
    n = ContinuousCourse(args.output)
    try:
        deadline = time.monotonic()+10
        while n.robot_pose is None and time.monotonic() < deadline:
            rclpy.spin_once(n, timeout_sec=.1)
        if not args.preview_only:
            n.start_course()
        rclpy.spin(n)  # Keep route and measured trail visible after completion.
    except KeyboardInterrupt:
        pass
    finally:
        n.shutdown()
        n.save()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
