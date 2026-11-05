"""Live room-plate evidence and measured door/goal association.

This component observes auxiliary sensors and publishes visualization only.
It never publishes robot commands or substitutes scene names for OCR text.
"""
from collections import deque
import copy
import json
import os
from pathlib import Path
import shutil
import time

import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Header
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration
from rclpy.qos import qos_profile_sensor_data

from go2w_sign_ocr import AsyncSignOCR, draw_text_evidence
from go2w_door_anchor import DoorAnchorTracker, _pose_matrix, _goal_in_frame
from go2w_detection_filter import DetectionFilter


ROOT = Path(__file__).resolve().parents[1]


def seconds(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


class DoorTargetView:
    def __init__(self, node, output):
        self.node = node
        self.output = output
        output.mkdir(parents=True, exist_ok=True)
        self.ocr = AsyncSignOCR(max_frame_age=4.)
        self.tracker = DoorAnchorTracker(max_visibility_age=4.)
        self.auxiliary_filter = DetectionFilter(temporal=True, max_confirmation_gap=5.)
        self.depths = deque(maxlen=24)
        self.poses = deque(maxlen=150)
        self.references = deque(maxlen=1800)
        self.local_alignment = None
        self.latest_rgb = None
        self.decoded_stamp = None
        self.rgb = None
        self.last_source = None
        self.last_publish = 0.
        self.last_evidence_code = None
        self.frame = os.environ.get('GO2W_SEMANTIC_FRAME', 'lio_map')
        self.desired_code = os.environ.get('GO2W_SIGN_TARGET_CODE') or None
        self.goal = dict(position=[13.35, -2.35, 5.90], frame='map', yaw=0.)
        self.alignment_path = output.parent / 'frame_alignment.json'
        self.result = dict(state='waiting_sign_camera', anchors=[], observations=[],
                           goal_status=dict(state='no_matching_observation', confirmed=False))
        node.create_subscription(CompressedImage, '/go2w/sign/rgb/compressed',
                                 self.receive_rgb, qos_profile_sensor_data)
        node.create_subscription(Image, '/go2w/sign/depth/image_raw',
                                 self.depths.append, qos_profile_sensor_data)
        node.create_subscription(Odometry, '/wavego/navigation/odom',
                                 self.poses.append, qos_profile_sensor_data)
        node.create_subscription(Odometry, '/odom/mujoco_odom',
                                 self.references.append, qos_profile_sensor_data)
        self.status_pub = node.create_publisher(String, '/wavego/door_target/status', 10)
        self.marker_pub = node.create_publisher(MarkerArray, '/wavego/door_target/markers', 10)

    def receive_rgb(self, message):
        self.latest_rgb = message

    def update(self, engine):
        now = self.node.get_clock().now().nanoseconds * 1e-9
        if self.latest_rgb is not None and seconds(self.latest_rgb) != self.decoded_stamp:
            message = self.latest_rgb
            bgr = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
            if bgr is not None:
                self.rgb = bgr[:, :, ::-1]
                self.decoded_stamp = seconds(message)
                self.ocr.submit(self.rgb, self.decoded_stamp)
        ocr = self.ocr.snapshot(self.rgb, self.decoded_stamp) if self.rgb is not None else {
            'state': 'waiting_sign_camera', 'lines': []}
        source = self.ocr.source_result()
        # Door tracking must also run while OCR is still confirming the text.
        # Waiting for accepted text creates gaps that reset temporal detection
        # and can prevent a visible door from ever reaching its second frame.
        if (source and source['stamp'] != self.last_source and self.depths and self.poses):
            source_stamp = source['stamp']
            depth_message = min(list(self.depths), key=lambda m: abs(seconds(m) - source_stamp))
            pose_message = min(list(self.poses), key=lambda m: abs(seconds(m) - seconds(depth_message)))
            if (0 <= now - source_stamp <= 4 and
                    abs(seconds(depth_message) - source_stamp) <= .05 and
                    abs(seconds(pose_message) - seconds(depth_message)) <= .08):
                config = json.loads((ROOT / 'matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/config/config.json').read_text())
                sensors = config['robot']['sensors']
                if 'sign_camera' in sensors and 'sign_depth' in sensors:
                    auxiliary_config = copy.deepcopy(config)
                    auxiliary_config['robot']['sensors']['camera'] = sensors['sign_camera']
                    auxiliary_config['robot']['sensors']['depth_sensor'] = sensors['sign_depth']
                    depth_sensor = sensors['sign_depth']
                    depth = np.frombuffer(depth_message.data, dtype='<f4').reshape(
                        depth_message.height or depth_sensor['height'],
                        depth_message.width or depth_sensor['width'])
                    # Reuse the loaded models serially. Masks and OCR refer to the
                    # same archived source RGB, not a newer moving-camera frame.
                    _, labels, objects, inference = engine.infer(
                        source['rgb'], prompt='wooden door.',
                        detection_filter=self.auxiliary_filter)
                    alignment = json.loads(self.alignment_path.read_text()) if self.alignment_path.exists() else None
                    reference = min(list(self.references), key=lambda m: abs(seconds(m) - seconds(depth_message))) if self.references else None
                    if reference is not None and abs(seconds(reference) - seconds(depth_message)) <= .08:
                        transform = _pose_matrix(pose_message.pose.pose) @ np.linalg.inv(_pose_matrix(reference.pose.pose))
                        self.local_alignment = dict(parent=self.frame, child='map', matrix=transform.tolist(),
                            source='Source-time local LIO/reference body-pose registration for simulation goal overlay only',
                            reference_stamp=seconds(reference), lio_stamp=seconds(pose_message),
                            ground_truth_pose_used_for_goal_overlay=True, lidar_geometry_modified=False)
                    comparison_alignment = self.local_alignment or alignment
                    self.result = self.tracker.observe(
                        source['result'], depth, auxiliary_config, pose_message.pose.pose,
                        rgb_shape=source['rgb'].shape, semantic_labels=labels,
                        door_objects=objects, rgb_stamp=source_stamp,
                        depth_stamp=seconds(depth_message), pose_stamp=seconds(pose_message),
                        frame=self.frame, pose_frame=pose_message.header.frame_id,
                        now=now, navigation_goal=self.goal, frame_alignment=comparison_alignment,
                        desired_code=self.desired_code)
                    self.result.update(ocr=source['result'], auxiliary_model_objects=objects,
                                       source_image_dimensions=list(source['rgb'].shape[:2][::-1]),
                                       auxiliary_inference_ms=inference['total_ms'])
                    self.result.update(goal_alignment=comparison_alignment,
                                       auxiliary_detection_audit=inference['detection_filter_audit'],
                                       ground_truth_pose_used_for_perception_anchor=False)
                    self.last_source = source_stamp
                    with (self.output / 'observations.jsonl').open('a') as stream:
                        stream.write(json.dumps(self.result) + '\n')
                    if self.result['goal_status'].get('confirmed'):
                        code = tuple(self.result['goal_status'].get('observed_room_codes', []))
                        if code != self.last_evidence_code:
                            self.last_evidence_code = code
                            evidence_dir = self.output / 'confirmed' / f"source_{round(source_stamp * 1e6)}"
                            evidence_dir.mkdir(parents=True, exist_ok=False)
                            cv2.imwrite(str(evidence_dir / 'confirmed_source_rgb.png'), source['rgb'][:, :, ::-1])
                            cv2.imwrite(str(evidence_dir / 'confirmed_ocr.png'), draw_text_evidence(
                                source['rgb'], source['result']['lines'])[:, :, ::-1])
                            np.savez_compressed(evidence_dir / 'confirmed_metric_depth.npz', depth=depth,
                                                semantic_labels=labels)
                            (evidence_dir / 'confirmed_source.json').write_text(json.dumps(self.result, indent=2) + '\n')
                            # Source-stamped evidence survives service restarts.
                            # Existing legacy aliases must not change underneath
                            # reports that cite their source timestamp.
                            if not (self.output / 'confirmed_source.json').exists():
                                for path in evidence_dir.iterdir():
                                    shutil.copyfile(path, self.output / path.name)
                            (self.output / 'latest_confirmed_evidence.json').write_text(json.dumps(
                                dict(source_stamp=source_stamp, directory=str(evidence_dir.resolve()),
                                     observed_room_codes=list(code)), indent=2) + '\n')
        data = dict(self.result)
        if data['state'] == 'waiting_sign_camera' and self.rgb is not None:
            data['state'] = 'waiting_confirmed_room_text'
        data.update(ocr=ocr, desired_code=self.desired_code, navigation_goal=self.goal,
                    sensor_rgb_topic='/go2w/sign/rgb/compressed',
                    sensor_depth_topic='/go2w/sign/depth/image_raw',
                    robot_commands_sent=0, visual_association_only=True)
        data['anchors'] = self.tracker.snapshot(now)
        alignment = json.loads(self.alignment_path.read_text()) if self.alignment_path.exists() else None
        data['goal_status'] = self.tracker.goal_status(self.goal, self.frame, self.local_alignment or alignment, self.desired_code, now)
        if alignment:
            data['initial_registration_goal_position'] = _goal_in_frame(self.goal, self.frame, alignment)[0].tolist()
        data['goal_alignment'] = self.local_alignment or alignment
        visible_codes = {code for line in ocr.get('lines', []) for code in line.get('room_codes', [])}
        for anchor in data['anchors']:
            anchor['currently_visible'] = bool(anchor['currently_visible'] and
                                                visible_codes.intersection(anchor['room_codes']))
            if not anchor['currently_visible'] and anchor['identity_confirmed']:
                anchor['state'] = 'persisted_confirmed'
        goal_status = data['goal_status']
        if goal_status.get('confirmed') and not visible_codes.intersection(goal_status.get('observed_room_codes', [])):
            goal_status.update(confirmed=False, current_observation=False, state='confirmed_persisted')
        if time.monotonic() - self.last_publish >= .5:
            self.last_publish = time.monotonic()
            self.status_pub.publish(String(data=json.dumps(data)))
            self.publish_markers(data)
            (self.output / 'status.json').write_text(json.dumps(data, indent=2) + '\n')
        # Keep the room identifier legible without stacking overlapping word
        # and whole-line annotations over the plaque's original lettering.
        room_lines = [line for line in ocr.get('lines', []) if line.get('room_codes')]
        preview = draw_text_evidence(self.rgb, room_lines) if self.rgb is not None else None
        if preview is not None and ocr.get('lines'):
            rooms = [line for line in ocr['lines'] if line.get('room_codes')]
            if rooms:
                regions = {line.get('region_id') for line in rooms}
                same_sign = [line for line in ocr['lines'] if line.get('region_id') in regions]
                boxes = np.array([line['box_xyxy'] for line in same_sign])
                low = np.maximum(0, boxes[:, :2].min(axis=0) - [55, 45])
                high = np.minimum([preview.shape[1], preview.shape[0]], boxes[:, 2:].max(axis=0) + [55, 25])
                if np.all(high > low):
                    preview = preview[low[1]:high[1], low[0]:high[0]]
        return data, preview

    def publish_markers(self, data):
        header = Header(stamp=self.node.get_clock().now().to_msg(), frame_id=self.frame)
        markers = [Marker(header=header, action=Marker.DELETEALL)]
        for anchor in data.get('anchors', []):
            conflict = anchor.get('identity_conflict', False)
            if not anchor.get('identity_confirmed') and not conflict:
                continue
            position = anchor['position']
            fresh = anchor.get('currently_visible')
            for text in (False, True):
                marker = Marker(header=header, ns='room_plate_text' if text else 'room_plate_point',
                                id=anchor['id'], type=Marker.TEXT_VIEW_FACING if text else Marker.SPHERE,
                                action=Marker.ADD, lifetime=Duration(sec=3))
                marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, position)
                marker.pose.orientation.w = 1.
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
                    (1., .2, .2, 1.) if conflict else (1., .88, .2, 1. if fresh else .55))
                if text:
                    marker.pose.position.z += .75
                    marker.scale.z = .6
                    marker.text = ('ROOM CONFLICT ' if conflict else 'ROOM ') + ', '.join(
                        anchor.get('conflicting_room_codes', anchor['room_codes']) if conflict
                        else anchor['room_codes']) + (' | LIVE' if fresh else ' | SAVED')
                else:
                    marker.scale.x = marker.scale.y = marker.scale.z = .14
                markers.append(marker)
        goal = data.get('goal_status', {})
        if 'goal_position' in goal:
            valid = goal.get('confirmed', False)
            marker = Marker(header=header, ns='recognized_navigation_goal', id=0,
                            type=Marker.TEXT_VIEW_FACING, action=Marker.ADD, lifetime=Duration(sec=3))
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, goal['goal_position'])
            # Lift the caption above the observed room so it remains readable
            # in the complete-map overview; the point stays at the real goal.
            marker.pose.position.z += 3.2
            marker.pose.orientation.w = 1.
            marker.scale.z = .85
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (.2, 1., .45, 1.) if valid else (1., .65, .1, 1.)
            codes = ', '.join(goal.get('observed_room_codes', []))
            marker.text = 'TARGET ' + codes + ('\nDOOR MATCHED | LIVE' if valid else '\n' + goal['state'].upper())
            markers.append(marker)
            stop = Marker(header=header, ns='recognized_navigation_stop', id=0,
                          type=Marker.SPHERE, action=Marker.ADD, lifetime=Duration(sec=3))
            stop.pose.position.x, stop.pose.position.y, stop.pose.position.z = map(float, goal['goal_position'])
            stop.pose.orientation.w = 1.
            stop.scale.x = stop.scale.y = stop.scale.z = .3
            stop.color = marker.color
            markers.append(stop)
            leader = Marker(header=header, ns='recognized_navigation_caption', id=0,
                            type=Marker.LINE_STRIP, action=Marker.ADD, lifetime=Duration(sec=3))
            leader.pose.orientation.w = 1.
            leader.scale.x = .025
            leader.color = marker.color
            leader.points = [stop.pose.position, Point(x=marker.pose.position.x,
                y=marker.pose.position.y, z=marker.pose.position.z - .45)]
            markers.append(leader)
            link = Marker(header=header, ns='plate_door_goal_association', id=0,
                          type=Marker.LINE_STRIP, action=Marker.ADD, lifetime=Duration(sec=3))
            link.pose.orientation.w = 1.
            link.scale.x = .035
            link.color = marker.color
            link.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                           for p in (goal['sign_position'], goal['door_position'], goal['goal_position'])]
            markers.append(link)
        self.marker_pub.publish(MarkerArray(markers=markers))

    def close(self):
        self.ocr.close()
