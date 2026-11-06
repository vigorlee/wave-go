#!/usr/bin/env python3
"""Read-only source-aligned auxiliary-camera evidence; never publishes commands.

Run within the Office ROS environment. Missing UE Image dimensions are resolved
from the configured auxiliary sensor only when the payload has exactly that
float32 pixel count. Optional manually specified pixel ROI is validation data,
never an OCR result. Inference panel/status stamps are recorded independently.
"""
from __future__ import annotations

import argparse
from collections import deque
import copy
from datetime import datetime
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from scipy.spatial.transform import Rotation

from go2w_door_anchor import _project, _text_position, _goal_in_frame, _pose_matrix


ROOT = Path(__file__).resolve().parents[1]


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def pose_record(message):
    p, q = message.pose.pose.position, message.pose.pose.orientation
    quaternion = [q.x, q.y, q.z, q.w]
    return dict(stamp=stamp(message), frame=message.header.frame_id,
                child_frame=message.child_frame_id, position=[p.x, p.y, p.z],
                quaternion=quaternion,
                euler_xyz_rad=Rotation.from_quat(quaternion).as_euler('xyz').tolist(),
                linear_velocity=[getattr(message.twist.twist.linear, key) for key in ('x', 'y', 'z')])


def decode_depth(message, sensor):
    width = int(message.width or sensor['width'])
    height = int(message.height or sensor['height'])
    expected = width * height * 4
    if min(width, height) < 1 or len(message.data) != expected:
        raise ValueError(f'Unverified depth shape/payload: {width}x{height}, bytes={len(message.data)}, expected={expected}')
    if message.encoding not in ('', '32FC1'):
        raise ValueError(f'Unsupported depth encoding: {message.encoding}')
    depth = np.frombuffer(message.data, dtype='>f4' if message.is_bigendian else '<f4').reshape(height, width)
    return depth, dict(width=width, height=height, header_width=message.width,
                       header_height=message.height, header_encoding=message.encoding,
                       dimension_source='message_header' if message.width and message.height else 'configured_aux_depth_actor',
                       expected_bytes=expected, actual_bytes=len(message.data),
                       pixel_count_verified=True, axial_units='metres',
                       interpretation_source='configured UE metric depth actor float32 payload')


class Capture(Node):
    def __init__(self):
        super().__init__('go2w_door_evidence_readonly')
        self.rgb = deque(maxlen=40)
        self.depth = deque(maxlen=40)
        self.lio = deque(maxlen=1800)
        self.reference = deque(maxlen=1800)
        self.status = deque(maxlen=80)
        self.panel = deque(maxlen=12)
        self.create_subscription(CompressedImage, '/go2w/sign/rgb/compressed', self.rgb.append, qos_profile_sensor_data)
        self.create_subscription(Image, '/go2w/sign/depth/image_raw', self.depth.append, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/wavego/navigation/odom', self.lio.append, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom/mujoco_odom', self.reference.append, qos_profile_sensor_data)
        self.create_subscription(String, '/wavego/door_target/status', self.receive_status, 10)
        self.create_subscription(Image, '/go2w/anything/annotated_image', self.panel.append, qos_profile_sensor_data)

    def receive_status(self, message):
        try:
            self.status.append((time.time(), json.loads(message.data)))
        except json.JSONDecodeError:
            pass

    def best(self):
        if not all((self.rgb, self.depth, self.lio, self.reference)):
            return None
        choices = []
        for rgb in self.rgb:
            source = stamp(rgb)
            depth = min(self.depth, key=lambda m: abs(stamp(m) - source))
            lio = min(self.lio, key=lambda m: abs(stamp(m) - stamp(depth)))
            reference = min(self.reference, key=lambda m: abs(stamp(m) - stamp(depth)))
            deltas = [abs(stamp(depth) - source), abs(stamp(lio) - stamp(depth)),
                      abs(stamp(reference) - stamp(depth))]
            if deltas[0] > .05 or max(deltas[1:]) > .08:
                continue
            status = min(self.status, key=lambda item: abs(float(item[1].get('source_stamp', 0)) - source)) if self.status else None
            status_delta = abs(float(status[1].get('source_stamp', 0)) - source) if status else None
            confirmed = bool(status and status[1].get('goal_status', {}).get('confirmed'))
            exact = bool(status_delta is not None and status_delta <= .025)
            score = (int(confirmed and exact), int(confirmed), int(exact), source)
            panel = min(self.panel, key=lambda m: abs(stamp(m) - source)) if self.panel else None
            choices.append((score, dict(rgb=rgb, depth=depth, lio=lio, reference=reference,
                                       deltas=deltas, status=status, status_source_delta=status_delta,
                                       panel=panel, confirmed=confirmed, status_exact=exact)))
        return max(choices, key=lambda item: item[0])[1] if choices else None

    def diagnostic(self):
        buffers = dict(aux_rgb=self.rgb, aux_depth=self.depth, lio_pose=self.lio,
                       simulator_reference=self.reference, model_panel=self.panel)
        data = dict(recorded_at=datetime.now().astimezone().isoformat(), robot_commands_sent=0,
                    state='no_synchronized_evidence', status_messages=len(self.status), streams={})
        for name, messages in buffers.items():
            record = dict(received_messages=len(messages))
            if messages:
                record.update(first_stamp=stamp(messages[0]), last_stamp=stamp(messages[-1]),
                              latest_frame=messages[-1].header.frame_id)
            data['streams'][name] = record
        if self.rgb and self.depth:
            reference = stamp(self.rgb[-1])
            depth = min(self.depth, key=lambda message: abs(stamp(message) - reference))
            data['nearest_rgb_depth_delta_seconds'] = abs(stamp(depth) - reference)
            for name, messages in [('lio', self.lio), ('simulator_reference', self.reference)]:
                if messages:
                    data[name + '_depth_delta_seconds'] = min(abs(stamp(m) - stamp(depth)) for m in messages)
        return data


def save(args, chosen, config):
    args.output.mkdir(parents=True, exist_ok=False)
    original = bytes(chosen['rgb'].data)
    rgb = cv2.imdecode(np.frombuffer(original, np.uint8), cv2.IMREAD_COLOR)
    if rgb is None:
        raise ValueError('Invalid source compressed RGB payload')
    rgb_path = args.output / 'source_rgb.jpg'
    rgb_path.write_bytes(original)
    cv2.imwrite(str(args.output / 'source_rgb.png'), rgb)
    sensors = config['robot']['sensors']
    depth, depth_metadata = decode_depth(chosen['depth'], sensors['sign_depth'])
    np.savez_compressed(args.output / 'source_metric_depth.npz', axial_depth_m=depth)
    lio, reference = pose_record(chosen['lio']), pose_record(chosen['reference'])
    evidence = dict(recorded_at=datetime.now().astimezone().isoformat(), robot_commands_sent=0,
                    source_rgb_topic='/go2w/sign/rgb/compressed',
                    source_depth_topic='/go2w/sign/depth/image_raw',
                    rgb_stamp=stamp(chosen['rgb']), rgb_frame=chosen['rgb'].header.frame_id,
                    rgb_format=chosen['rgb'].format, rgb_width=rgb.shape[1], rgb_height=rgb.shape[0],
                    rgb_sha256=hashlib.sha256(original).hexdigest(), original_compressed_bytes_preserved=True,
                    depth_stamp=stamp(chosen['depth']), depth_frame=chosen['depth'].header.frame_id,
                    depth=depth_metadata, rgb_depth_delta_seconds=chosen['deltas'][0],
                    depth_lio_pose_delta_seconds=chosen['deltas'][1],
                    depth_reference_pose_delta_seconds=chosen['deltas'][2],
                    lio_pose=lio, simulator_reference_pose=reference,
                    lidar_mapping_pose_used_for_map=True,
                    simulator_reference_used_only_for_validation=True,
                    sensor_config=sensors, output_directory=str(args.output.resolve()))
    if chosen['status']:
        received_at, status = chosen['status']
        (args.output / 'door_target_status.json').write_text(json.dumps(status, indent=2) + '\n')
        evidence.update(status_received_wall_time=received_at,
                        status_source_stamp=status.get('source_stamp'),
                        status_source_rgb_delta_seconds=chosen['status_source_delta'],
                        status_ocr_source_stamp=status.get('ocr', {}).get('input_stamp'),
                        status_matches_source_rgb=chosen['status_exact'],
                        goal_confirmed_in_status=chosen['confirmed'])
    panel = chosen['panel']
    if panel is not None:
        width, height = int(panel.width), int(panel.height)
        if panel.encoding in ('rgb8', 'bgr8') and len(panel.data) == height * int(panel.step):
            array = np.frombuffer(panel.data, np.uint8).reshape(height, int(panel.step))[:, :width * 3].reshape(height, width, 3)
            cv2.imwrite(str(args.output / 'model_panel.png'), array[:, :, ::-1] if panel.encoding == 'rgb8' else array)
            evidence.update(model_panel_stamp=stamp(panel), model_panel_frame=panel.header.frame_id,
                            model_panel_rgb_delta_seconds=stamp(panel) - stamp(chosen['rgb']),
                            model_panel_same_source_verified=False,
                            model_panel_note='Main camera model panel; not auxiliary-camera inference synchronization proof')
    alias = copy.deepcopy(config)
    alias['robot']['sensors']['camera'] = sensors['sign_camera']
    alias['robot']['sensors']['depth_sensor'] = sensors['sign_depth']
    alignment = json.loads(args.alignment.read_text()) if args.alignment and args.alignment.exists() else None
    if alignment is not None:
        evidence['frame_alignment'] = alignment
    if args.validation_roi:
        line = dict(box_xyxy=args.validation_roi)
        validation = dict(roi_xyxy=args.validation_roi, manual_validation=True,
                          roi_source='manual pixel selection for geometric validation',
                          is_ocr_result=False, text_identity_confirmed=False)
        for name, pose in [('lio', chosen['lio'].pose.pose), ('simulator_reference', chosen['reference'].pose.pose)]:
            try:
                world, valid, u, v, camera = _project(depth, alias, pose, rgb.shape)
                point, quality = _text_position(line, depth, world, valid, u, v, rgb.shape)
                validation[name] = dict(position=point.tolist(), camera_position=camera.tolist(), quality=quality)
            except ValueError as exc:
                validation[name] = dict(reason=str(exc))
        if alignment is not None and 'position' in validation.get('simulator_reference', {}) and 'position' in validation.get('lio', {}):
            matrix = np.asarray(alignment['matrix'])
            predicted = matrix @ [*validation['simulator_reference']['position'], 1.]
            difference = np.asarray(validation['lio']['position']) - predicted[:3]
            validation.update(initial_alignment_prediction_lio=predicted[:3].tolist(),
                              measured_lio_minus_initial_prediction_m=difference.tolist(),
                              drift_norm_m=float(np.linalg.norm(difference)))
        evidence['manual_roi_geometry_validation'] = validation
        x0, y0, x1, y1 = map(int, args.validation_roi)
        cv2.imwrite(str(args.output / 'manual_validation_roi.png'), rgb[y0:y1, x0:x1])
    if chosen['status'] and alignment:
        status = chosen['status'][1]
        goal = status.get('navigation_goal')
        projected = _goal_in_frame(goal, lio['frame'], alignment) if goal else None
        if projected:
            position, direction = projected
            local_registration = _pose_matrix(chosen['lio'].pose.pose) @ np.linalg.inv(
                _pose_matrix(chosen['reference'].pose.pose))
            local_goal = local_registration @ [*goal['position'], 1.]
            difference = local_goal[:3] - position
            evidence['local_registration_goal_validation'] = dict(
                source_depth_stamp=stamp(chosen['depth']),
                parent=lio['frame'], child='map',
                simulator_reference_native_frame=reference['frame'],
                map_and_reference_coordinate_convention='Navigation uses simulator odometry numeric coordinates as map goals',
                matrix=local_registration.tolist(),
                initial_registered_goal_position=position.tolist(),
                locally_registered_goal_position=local_goal[:3].tolist(),
                local_minus_initial_goal_m=difference.tolist(),
                offset_norm_m=float(np.linalg.norm(difference)),
                ground_truth_pose_used_for_this_validation=True,
                lidar_geometry_modified=False,
                note='Diagnostic source-time simulator reference registration, not a SLAM accuracy correction or active overlay')
            direction[2] = 0.
            direction /= np.linalg.norm(direction)
            checks = []
            for door in status.get('measured_doors', []):
                delta = np.asarray(door['position']) - position
                bounds = door['observed_z_bounds_m']
                checks.append(dict(door_id=door['id'], observed_door_position=door['position'],
                                   observed_z_bounds_m=bounds, transformed_robot_goal_position=position.tolist(),
                                   forward_m=float(np.dot(delta[:2], direction[:2])),
                                   lateral_m=abs(float(direction[0] * delta[1] - direction[1] * delta[0])),
                                   normal_agreement=abs(float(np.dot(np.asarray(door['normal'])[:2], direction[:2]))),
                                   original_height_gate=bool(bounds[0] - .5 < position[2] < bounds[1] + .5),
                                   model_status_matches_captured_frame=chosen['status_exact']))
            evidence['existing_door_goal_gate_validation'] = checks
    (args.output / 'evidence.json').write_text(json.dumps(evidence, indent=2) + '\n')
    return evidence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=12.)
    parser.add_argument('--wait-confirmed', action='store_true')
    parser.add_argument('--validation-roi', nargs=4, type=float, metavar=('X0', 'Y0', 'X1', 'Y1'))
    parser.add_argument('--alignment', type=Path)
    parser.add_argument('--config', type=Path, default=ROOT / 'matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/config/config.json')
    args = parser.parse_args()
    if not 0 < args.duration <= 30:
        parser.error('--duration must be between 0 and 30 seconds')
    if args.output.exists():
        parser.error('--output must be a new directory; evidence is not overwritten')
    config = json.loads(args.config.read_text())
    if not all(name in config['robot']['sensors'] for name in ('sign_camera', 'sign_depth')):
        parser.error('Auxiliary sign sensors are not configured')
    rclpy.init()
    node = Capture()
    chosen = None
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            candidate = node.best()
            if candidate is not None:
                chosen = candidate
                if args.wait_confirmed and chosen['confirmed'] and chosen['status_exact']:
                    break
        if chosen is None:
            args.output.mkdir(parents=True, exist_ok=False)
            diagnostic = node.diagnostic()
            (args.output / 'diagnostic.json').write_text(json.dumps(diagnostic, indent=2) + '\n')
            print(json.dumps(diagnostic, indent=2))
            raise RuntimeError('No real RGB/depth/LIO/reference pose set met the synchronization gates')
        evidence = save(args, chosen, config)
        print(json.dumps(evidence, indent=2))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
