#!/usr/bin/python3
"""Read-only, timestamp-gated audit of measured-ground display overlays.

Only subscriptions and TF lookups are used. The audit writes small JSON;
registered clouds, PCD files, simulator state and controls are never changed.
Body trajectories and original body-height reference routes are reported but
are deliberately excluded from the ground-overlay alignment comparison.
"""
import argparse
from collections import Counter, defaultdict, deque
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
from tf2_ros import Buffer, TransformListener

from navigation_surface import GroundSurface


TARGET_FRAME = 'lio_map'
BODY_PATHS = {'/go2w/course/trail', '/go2w/course/route',
              '/wavego/mapping/trajectory'}
INFLATION_CELLS = {'live_adaptive_cells', 'live_hard_cells'}
GROUND_MARKER_NAMES = INFLATION_CELLS | {
    'robot_adaptive_radius', 'dynamic_guard_radius',
    'dynamic_velocity_prediction', 'navigation_rollout_samples',
    'navigation_sampling_heads', 'navigation_preview',
    'navigation_preview_arrow', 'navigation_lookahead',
}


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def statistics(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return {'count': 0, 'min': None, 'p50': None, 'p95': None, 'max': None}
    return dict(count=len(values), min=float(values.min()),
                p50=float(np.percentile(values, 50)),
                p95=float(np.percentile(values, 95)), max=float(values.max()))


def quaternion_matrix(q):
    values = np.array([q.x, q.y, q.z, q.w], dtype=float)
    if np.linalg.norm(values) < 1e-8:
        return np.eye(3)
    return Rotation.from_quat(values).as_matrix()


def runtime_context():
    """Read the run pointer/result for provenance, without changing runtime."""
    pointer = Path(__file__).parents[2] / '.run/go2w/current_output_dir'
    try:
        run = Path(pointer.read_text().strip())
    except OSError:
        return dict(available=False)
    try:
        result = json.loads((run / 'course/result.json').read_text())
        return dict(run=str(run), course_result_available=True, course_state=result.get('state'),
                    completed_stages=result.get('completed_stages'),
                    total_stages=result.get('total_stages'), passed=result.get('passed'))
    except (OSError, ValueError, TypeError):
        return dict(run=str(run), course_result_available=False,
                    note='Run pointer available; course has not produced a readable result yet.')


def partial_scope_context(path):
    """Describe a separately recorded resume without merging mission results."""
    if path is None:
        return None
    try:
        payload = json.loads(path.read_text())
        result_path = path.parent / 'course/result.json'
        result = json.loads(result_path.read_text()) if result_path.exists() else None
        return dict(scope_file=str(path.resolve()), scope=payload,
                    scope_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    local_course_result=result,
                    complete_single_15_stage_run=False,
                    note='Bounded audit of one local resume; prior completed stages are not combined into full-run success.')
    except (OSError, ValueError, TypeError) as exc:
        return dict(scope_file=str(path.resolve()), available=False,
                    error=type(exc).__name__, complete_single_15_stage_run=False)


class Audit(Node):
    def __init__(self, output, scope_path=None):
        super().__init__('read_only_display_alignment_audit')
        self.output = output
        self.runtime_context_start = runtime_context()
        self.scope_path = scope_path
        self.partial_scope_start = partial_scope_context(scope_path)
        self.audit_source_sha256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        self.surface_source_sha256 = hashlib.sha256(
            Path(__file__).with_name('navigation_surface.py').read_bytes()).hexdigest()
        self.started = time.monotonic()
        self.wall_started = time.time()
        self.counts = Counter()
        self.header_frames = defaultdict(Counter)
        self.odom = deque(maxlen=600)
        self.reference = deque(maxlen=600)
        self.initial_alignment = None
        self.initial_alignment_metadata = None
        self.reference_errors = []
        self.scans = deque(maxlen=60)
        self.surfaces = deque(maxlen=40)
        self.canonical = deque(maxlen=180)
        self.canonical_records = []
        self.canonical_payload_counts = Counter()
        self.last_support_verification_stamp = -float('inf')
        self.visual_status = deque(maxlen=180)
        self.pending_overlays = deque(maxlen=1400)
        self.ground_records = []
        self.overlay_records = defaultdict(list)
        self.exclusions = Counter()
        self.namespace_counts = Counter()
        self.aggregate = defaultdict(lambda: dict(
            total_points=0, nearby_points=0, supported_points=0,
            offsets=[], excess=[], absolute_deviations=[], source_deltas=[],
            local_floor_band_offsets=[], local_floor_band_excess=[]))
        self.seen_scan_stamp = None
        self.dynamic_subscriptions = {}
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(Odometry, '/wavego/navigation/odom',
                                 self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom/mujoco_odom',
                                 self.on_reference, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/wavego/slam/registered_points',
                                 self.on_scan, qos_profile_sensor_data)
        self.create_subscription(String, '/wavego/live_dense/ground_surface',
                                 self.on_canonical, 10)
        self.create_subscription(String, '/wavego/visual/registration_status',
                                 self.on_visual_status, 10)
        self.create_timer(.2, self.fit_latest)
        self.create_timer(.1, self.process_pending)
        self.create_timer(1., self.discover)
        self.discover()

    def on_odom(self, message):
        key = '/wavego/navigation/odom'
        self.counts[key] += 1
        self.header_frames[key][message.header.frame_id] += 1
        p = message.pose.pose.position
        self.odom.append(dict(stamp=stamp(message.header),
                              received=time.monotonic(),
                              frame=message.header.frame_id,
                              xyz=np.array([p.x, p.y, p.z]),
                              rotation=quaternion_matrix(message.pose.pose.orientation)))

    def on_reference(self, message):
        key = '/odom/mujoco_odom'
        self.counts[key] += 1
        self.header_frames[key][message.header.frame_id] += 1
        p = message.pose.pose.position
        self.reference.append(dict(stamp=stamp(message.header),
                                   received=time.monotonic(),
                                   frame=message.header.frame_id,
                                   xyz=np.array([p.x, p.y, p.z]),
                                   rotation=quaternion_matrix(message.pose.pose.orientation)))

    def reference_error(self, pose, source_stamp):
        """Audit LIO against reference with the fixed initial TF, never refit it.

        In this project native mujoco_odom calls its simulation world 'world';
        the original planner and initial map alignment use those same numeric
        coordinates as 'map'. This contract is not applied to arbitrary topics.
        """
        if not self.reference:
            self.exclusions['reference_pose_missing'] += 1
            return None
        ref = min(self.reference, key=lambda p: abs(p['stamp'] - source_stamp))
        delta = abs(ref['stamp'] - source_stamp)
        pair_delta = abs(ref['stamp'] - pose['stamp'])
        if max(delta, pair_delta) > .08:
            self.exclusions['reference_pose_delta_over_80ms'] += 1
            return None
        if ref['frame'] not in ('world', 'map'):
            self.exclusions['unexpected_reference_frame'] += 1
            return None
        if self.initial_alignment is None:
            try:
                tf = self.tf.lookup_transform(TARGET_FRAME, 'map', Time())
                matrix = np.eye(4)
                matrix[:3, :3] = quaternion_matrix(tf.transform.rotation)
                t = tf.transform.translation
                matrix[:3, 3] = [t.x, t.y, t.z]
                self.initial_alignment = matrix
                self.initial_alignment_metadata = dict(
                    source='Existing initial lio_map <- map TF; frozen on first observation, no per-frame refit',
                    header_frame=tf.header.frame_id, child_frame=tf.child_frame_id,
                    observed_tf_stamp=stamp(tf.header), matrix=matrix.tolist())
            except Exception as exc:
                self.exclusions['initial_alignment_tf_unavailable:' + type(exc).__name__] += 1
                return None
        matrix = self.initial_alignment
        aligned = ref['xyz'] @ matrix[:3, :3].T + matrix[:3, 3]
        error = pose['xyz'] - aligned
        rotation_error = Rotation.from_matrix(
            pose['rotation'] @ (matrix[:3, :3] @ ref['rotation']).T)
        record = dict(source_stamp=source_stamp,
                      navigation_pose_stamp=pose['stamp'], reference_pose_stamp=ref['stamp'],
                      reference_source_delta_seconds=delta, pose_pair_delta_seconds=pair_delta,
                      reference_native_header_frame=ref['frame'],
                      reference_numeric_frame_contract='Only /odom/mujoco_odom world coordinates also define the planner map',
                      reference_body_xyz=ref['xyz'].tolist(),
                      initially_aligned_reference_body_xyz=aligned.tolist(),
                      navigation_body_xyz=pose['xyz'].tolist(),
                      navigation_rotation_xyzw=Rotation.from_matrix(pose['rotation']).as_quat().tolist(),
                      reference_rotation_xyzw=Rotation.from_matrix(ref['rotation']).as_quat().tolist(),
                      initial_registered_reference_error_xyz_m=error.tolist(),
                      initial_registered_reference_error_norm_m=float(np.linalg.norm(error)),
                      initial_registered_reference_rotation_error_degrees=float(np.degrees(rotation_error.magnitude())))
        self.reference_errors.append(record)
        return record

    def on_scan(self, message):
        key = '/wavego/slam/registered_points'
        self.counts[key] += 1
        self.header_frames[key][message.header.frame_id] += 1
        points = point_cloud2.read_points(message, field_names=('x', 'y', 'z'),
                                         skip_nans=True)
        xyz = np.column_stack([points[k] for k in ('x', 'y', 'z')])
        self.scans.append(dict(stamp=stamp(message.header),
                               received=time.monotonic(), xyz=xyz,
                               native_header_frame=message.header.frame_id))

    def on_canonical(self, message):
        self.counts['/wavego/live_dense/ground_surface'] += 1
        try:
            payload = json.loads(message.data)
            surface = GroundSurface.from_payload(payload)
            ts = float(payload['source_stamp'])
            if not np.isfinite(ts) or int(payload['support_count']) != surface.support_count:
                raise ValueError('Invalid source stamp or support count')
            self.canonical_payload_counts['valid' if surface.valid else 'invalid'] += 1
            if self.canonical and self.canonical[-1]['stamp'] == ts:
                self.canonical_payload_counts['duplicate_source_stamp'] += 1
                return
            self.canonical.append(dict(stamp=ts, received=time.monotonic(),
                                       surface=surface, payload=payload, validation=None))
        except (ValueError, KeyError, TypeError) as exc:
            self.canonical_payload_counts['rejected:' + type(exc).__name__] += 1

    def on_visual_status(self, message):
        self.counts['/wavego/visual/registration_status'] += 1
        try:
            data = json.loads(message.data)
            marker = data.get('marker_registration', {})
            ground = data.get('ground_surface', {})
            if marker.get('source_stamp') is not None and ground.get('source_stamp') is not None:
                self.visual_status.append(dict(marker_stamp=float(marker['source_stamp']),
                    ground_stamp=float(ground['source_stamp']),
                    canonical_received=bool(ground.get('canonical_received')),
                    registration_valid=bool(marker.get('valid')),
                    received=time.monotonic()))
        except (ValueError, KeyError, TypeError):
            self.exclusions['invalid_visual_registration_status'] += 1

    def validate_canonical(self):
        if not self.odom:
            return
        for model in self.canonical:
            if model['validation'] is not None:
                continue
            pose = min(self.odom, key=lambda p: abs(p['stamp'] - model['stamp']))
            delta = abs(pose['stamp'] - model['stamp'])
            if delta > .08 or pose['frame'] != TARGET_FRAME:
                continue
            surface = model['surface']
            record = dict(source_stamp=model['stamp'], pose_stamp=pose['stamp'],
                source_sync_delta_seconds=delta, surface=surface.metadata(),
                navigation_body_xyz=pose['xyz'].tolist(),
                payload_base_xyz=surface.base_xyz.tolist(),
                payload_base_pose_difference_m=float(np.linalg.norm(surface.base_xyz-pose['xyz'])),
                measurement_window_sec=model['payload'].get('measurement_window_sec'),
                support_verification='Not sampled for this packet')
            if surface.valid and self.scans and model['stamp']-self.last_support_verification_stamp >= .8:
                window = [s for s in self.scans if -.000001 <= model['stamp']-s['stamp'] <= 2.000001]
                if window:
                    points = np.concatenate([s['xyz'] for s in window])
                    # KD matching only checks actual captured returns. A partial
                    # startup window is explicitly marked; unseen older returns
                    # are not called fabricated or independently verified.
                    distance, _ = cKDTree(points).query(surface.support_xyz)
                    scan_stamps = sorted(s['stamp'] for s in window)
                    record.update(support_verification='Actual registered-return 3D KD check',
                        support_actual_return_distance_m=statistics(distance),
                        support_exact_return_fraction_0p1mm=float(np.mean(distance < .0001)),
                        captured_actual_window_scans=len(window),
                        captured_window_max_source_gap_seconds=float(np.max(np.diff(scan_stamps)))
                        if len(scan_stamps)>1 else None,
                        captured_actual_window_sec=model['stamp']-min(s['stamp'] for s in window),
                        full_2s_window_captured=model['stamp']-self.scans[0]['stamp'] >= 1.9,
                        window_capture_note='Temporal coverage only; subscriber packet loss may leave uncaptured returns. Unmatched support is not labelled fabricated.')
                    self.last_support_verification_stamp=model['stamp']
            model['validation'] = record
            self.canonical_records.append(record)

    def process_pending(self):
        self.validate_canonical()
        waiting = deque(maxlen=1400)
        for row in self.pending_overlays:
            if self.compare(*row['arguments'], allow_pending=False):
                continue
            if time.monotonic()-row['received'] > 1.2:
                self.exclusions['overlay_canonical_protocol_match_timeout'] += 1
            else:
                waiting.append(row)
        self.pending_overlays = waiting

    def discover(self):
        for topic, kinds in self.get_topic_names_and_types():
            if topic in self.dynamic_subscriptions:
                continue
            path = 'nav_msgs/msg/Path' in kinds and (
                topic.startswith(('/wavego/global/', '/wavego/navigation/', '/wavego/visual/paths/'))
                or topic in BODY_PATHS)
            markers = 'visualization_msgs/msg/MarkerArray' in kinds and (
                topic.startswith(('/wavego/global/', '/wavego/navigation/', '/wavego/visual/'))
                or topic == '/wavego/live_dense/inflation')
            if path:
                path_qos = (QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                            if topic.startswith(('/wavego/visual/paths/', '/wavego/global/'))
                            else qos_profile_sensor_data)
                self.dynamic_subscriptions[topic] = self.create_subscription(
                    NavPath, topic, lambda m, k=topic: self.on_path(m, k),
                    path_qos)
            elif markers:
                self.dynamic_subscriptions[topic] = self.create_subscription(
                    MarkerArray, topic, lambda m, k=topic: self.on_markers(m, k),
                    qos_profile_sensor_data)

    def fit_latest(self):
        if not self.scans or not self.odom:
            return
        scan = self.scans[-1]
        if scan['stamp'] == self.seen_scan_stamp:
            return
        pose = min(self.odom, key=lambda p: abs(p['stamp'] - scan['stamp']))
        delta = abs(pose['stamp'] - scan['stamp'])
        if delta > .08:
            # Keep this scan available for its delayed matching odometry.
            self.exclusions['scan_pose_delta_over_80ms_attempts'] += 1
            return
        self.seen_scan_stamp = scan['stamp']
        if time.monotonic() - scan['received'] > 1.5:
            self.exclusions['stale_scan'] += 1
            return
        if pose['frame'] != TARGET_FRAME:
            self.exclusions['navigation_odom_frame_not_lio_map'] += 1
            return
        # Native registered_points is the existing LIO-coordinate alias "map".
        # This alias applies to this sensor alone; display paths in true map
        # coordinates are transformed through TF instead of being relabelled.
        if scan['native_header_frame'] not in (TARGET_FRAME, 'map'):
            self.exclusions['unexpected_registered_cloud_frame'] += 1
            return
        base = pose['xyz']
        d = scan['xyz'] - base
        near = scan['xyz'][(np.linalg.norm(d[:, :2], axis=1) < 3.5)
                           & (abs(d[:, 2]) < 2.)]
        surface = GroundSurface.from_scan(near, base)
        record = dict(source_scan_stamp=scan['stamp'], pose_stamp=pose['stamp'],
                      source_sync_delta_seconds=delta,
                      navigation_body_xyz=base.tolist(), near_scan_points=len(near),
                      near_scan_z=statistics(near[:, 2]),
                      ground=surface.metadata(),
                      initially_aligned_reference=self.reference_error(pose, scan['stamp']))
        if surface.valid:
            record['body_minus_floor_m'] = float(base[2] - surface.floor_z)
            record['floor_slope_degrees'] = float(np.degrees(
                np.arctan(np.linalg.norm(surface.coefficients[:2]))))
            record['body_xy_ground_supported'] = bool(
                surface.height_at(base[None, :2])[1][0])
        self.ground_records.append(record)
        self.surfaces.append(dict(stamp=scan['stamp'], received=scan['received'],
                                  source_delta=delta, surface=surface))

    def convert_frame(self, xyz, source_frame, source_time=None):
        if source_frame == TARGET_FRAME:
            return xyz, None
        if not source_frame:
            return None, 'missing_header_frame'
        try:
            tr = self.tf.lookup_transform(TARGET_FRAME, source_frame, source_time or Time())
            q, p = tr.transform.rotation, tr.transform.translation
            return xyz @ quaternion_matrix(q).T + [p.x, p.y, p.z], None
        except Exception as exc:
            return None, 'tf_unavailable:' + type(exc).__name__

    def compare(self, topic, namespace, header, xyz, expected_offset=None, allow_pending=True):
        key = topic + '|' + namespace
        if allow_pending:
            self.header_frames[key][header.frame_id] += 1
        if not len(xyz):
            self.exclusions['empty_overlay_points'] += 1
            return True
        source_xyz = xyz
        xyz, reason = self.convert_frame(xyz, header.frame_id, Time.from_msg(header.stamp))
        if reason:
            self.exclusions[key + '|' + reason] += 1
            return True
        marker_stamp = stamp(header)
        ground_stamp = marker_stamp
        registration_source = 'Inflation header equals exact canonical sourceStamp'
        if topic.startswith('/wavego/visual/'):
            status = [v for v in self.visual_status if abs(v['marker_stamp']-marker_stamp) < .000001
                      and v['canonical_received'] and v['registration_valid']]
            ground_stamp = status[-1]['ground_stamp'] if status else None
            registration_source = 'Visual registration_status marker sourceStamp -> exact canonical ground sourceStamp'
        matched = [s for s in self.canonical if ground_stamp is not None
                   and abs(s['stamp']-ground_stamp) < .000001 and s['validation'] is not None]
        if not matched:
            if allow_pending:
                self.pending_overlays.append(dict(received=time.monotonic(),
                    arguments=(topic, namespace, header, source_xyz.copy(), expected_offset)))
            return False
        sample = matched[-1]
        # Invalid canonical models are deliberately not replaced by a valid
        # independent fit or some other nearby model from another timestamp.
        if not sample['surface'].valid:
            self.exclusions['overlay_canonical_ground_invalid_no_fallback'] += 1
            return True
        source_delta = abs(sample['stamp'] - marker_stamp)
        if source_delta > .30 or time.monotonic() - sample['received'] > 1.5:
            self.exclusions['overlay_canonical_source_delta_over_300ms_or_stale'] += 1
            return True
        surface = sample['surface']
        nearby = np.linalg.norm(xyz[:, :2] - surface.base_xyz[:2], axis=1) < 3.5
        local = xyz[nearby]
        heights, supported = surface.height_at(local[:, :2])
        offsets = local[supported, 2] - heights[supported]
        # A whole-building route can overlap current-floor XY on another storey.
        # Keep all deviations observable, and also report a named height band;
        # points outside it may be another floor OR an alignment error. They
        # must not be silently classified as correctly aligned or deleted.
        floor_band = abs(offsets - (expected_offset or 0.)) <= .65
        aggregate = self.aggregate[key]
        aggregate['total_points'] += len(xyz)
        aggregate['nearby_points'] += len(local)
        aggregate['supported_points'] += int(supported.sum())
        aggregate['offsets'].extend(offsets.tolist())
        aggregate['absolute_deviations'].extend(abs(offsets).tolist())
        aggregate['source_deltas'].append(source_delta)
        aggregate['local_floor_band_offsets'].extend(offsets[floor_band].tolist())
        if expected_offset is not None:
            aggregate['excess'].extend((offsets - expected_offset).tolist())
            aggregate['local_floor_band_excess'].extend((offsets[floor_band] - expected_offset).tolist())
        record = dict(header_frame=header.frame_id, header_stamp=marker_stamp,
                      ground_source_stamp=sample['stamp'],
                      ground_model_source='canonical_2s_actual_registered_returns',
                      ground_model_matching=registration_source,
                      ground_pose_source_sync_delta_seconds=sample['validation']['source_sync_delta_seconds'],
                      overlay_ground_source_delta_seconds=source_delta,
                      point_count=len(xyz), near_current_floor_point_count=len(local),
                      nearby_xy_point_count=len(local),
                      supported_count=int(supported.sum()),
                      supported_within_local_floor_height_band_count=int(floor_band.sum()),
                      outside_local_floor_band_or_height_error_count=int((~floor_band).sum()),
                      within_local_floor_height_band_offset_m=statistics(offsets[floor_band]),
                      valid_support_fraction=float(supported.mean()) if len(local) else None,
                      measured_ground_offset_m=statistics(offsets),
                      absolute_ground_deviation_m=statistics(abs(offsets)),
                      expected_display_offset_m=expected_offset,
                      excess_offset_m=statistics(offsets - expected_offset)
                      if expected_offset is not None else None)
        # Keep JSON bounded while retaining every point in the aggregate audit.
        if len(self.overlay_records[key]) < 120:
            self.overlay_records[key].append(record)
        return True

    def on_path(self, message, topic):
        self.counts[topic] += 1
        self.header_frames[topic][message.header.frame_id] += 1
        if topic in BODY_PATHS or topic.startswith('/wavego/global/') or any(word in topic.lower() for word in
                                     ('trajectory', 'trail', 'native_reference', 'body')):
            self.exclusions['body_or_native_reference_path_not_ground_overlay:' + topic] += 1
            return
        if topic.startswith('/wavego/visual/paths/'):
            self.exclusions['historic_anchor_path_not_compared_to_current_floor:' + topic] += 1
            return
        xyz = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z]
                        for p in message.poses], dtype=float).reshape(-1, 3)
        expected = .035 if topic.startswith('/wavego/visual/paths/') else None
        self.compare(topic, 'display_path', message.header, xyz, expected)

    def on_markers(self, message, topic):
        self.counts[topic] += 1
        for marker in message.markers:
            if marker.action in (Marker.DELETE, Marker.DELETEALL):
                continue
            self.namespace_counts[topic + '|' + marker.ns] += 1
            if topic == '/wavego/global/markers':
                self.header_frames[topic + '|' + marker.ns][marker.header.frame_id] += 1
                self.exclusions['original_controller_marker_not_registered_display'] += 1
                continue
            if marker.type in (Marker.LINE_STRIP, Marker.LINE_LIST, Marker.POINTS,
                               Marker.CUBE_LIST, Marker.SPHERE_LIST, Marker.TRIANGLE_LIST) and not marker.points:
                # An empty points marker has no displayed geometry. Its default
                # pose at the frame origin is not a real ring/cell/line sample.
                self.exclusions['empty_points_marker_has_no_displayed_geometry:' + topic + '|' + marker.ns] += 1
                self.header_frames[topic + '|' + marker.ns][marker.header.frame_id] += 1
                continue
            if marker.ns == 'measured_dynamic' and topic.startswith('/wavego/visual/'):
                p = marker.pose.position
                foot = (np.array([[0., 0., -marker.scale.z / 2.]])
                        @ quaternion_matrix(marker.pose.orientation).T + [p.x, p.y, p.z])
                self.compare(topic, 'measured_dynamic_foot', marker.header, foot, .035)
                continue
            if marker.ns not in GROUND_MARKER_NAMES:
                self.header_frames[topic + '|' + marker.ns][marker.header.frame_id] += 1
                continue
            p = marker.pose.position
            if marker.points:
                xyz = np.array([[x.x, x.y, x.z] for x in marker.points], dtype=float)
                xyz = xyz @ quaternion_matrix(marker.pose.orientation).T + [p.x, p.y, p.z]
            else:
                xyz = np.array([[p.x, p.y, p.z]], dtype=float)
            expected_offset = .035 if marker.ns in INFLATION_CELLS or topic.startswith('/wavego/visual/') else None
            if marker.ns == 'robot_adaptive_radius' and topic == '/wavego/live_dense/inflation':
                expected_offset = .04
            if marker.ns == 'dynamic_velocity_prediction' and topic.startswith('/wavego/visual/'):
                expected_offset = .05
            self.compare(topic, marker.ns, marker.header, xyz, expected_offset)

    def save(self):
        self.process_pending()
        elapsed = time.monotonic() - self.started
        aggregates = {}
        for key, a in self.aggregate.items():
            aggregates[key] = dict(total_points=a['total_points'],
                                   nearby_points=a['nearby_points'],
                                   supported_points=a['supported_points'],
                                   valid_support_fraction=a['supported_points'] / a['nearby_points']
                                   if a['nearby_points'] else None,
                                   measured_ground_offset_m=statistics(a['offsets']),
                                   within_local_floor_height_band_offset_m=statistics(a['local_floor_band_offsets']),
                                   within_local_floor_height_band_excess_m=statistics(a['local_floor_band_excess']),
                                   outside_local_floor_band_or_height_error_count=len(a['offsets']) - len(a['local_floor_band_offsets']),
                                   absolute_ground_deviation_m=statistics(a['absolute_deviations']),
                                   excess_offset_m=statistics(a['excess']),
                                   overlay_ground_source_delta_seconds=statistics(a['source_deltas']))
        height_groups = defaultdict(list)
        for row in self.reference_errors:
            low = int(np.floor(row['reference_body_xyz'][2]))
            height_groups[str(low) + '_to_' + str(low+1) + '_m_reference_body_z'].append(row)
        grouped_errors = {key: dict(count=len(rows),
            reference_body_z_range_m=[min(r['reference_body_xyz'][2] for r in rows),
                                      max(r['reference_body_xyz'][2] for r in rows)],
            estimated_z_error_m=statistics([r['initial_registered_reference_error_xyz_m'][2] for r in rows]),
            estimated_norm_error_m=statistics([r['initial_registered_reference_error_norm_m'] for r in rows]))
            for key, rows in height_groups.items()}
        verified = [r for r in self.canonical_records if 'support_exact_return_fraction_0p1mm' in r]
        valid_packets = self.canonical_payload_counts['valid']
        invalid_packets = self.canonical_payload_counts['invalid']
        context_end = runtime_context()
        report = dict(audit_date_utc=datetime.now(timezone.utc).isoformat(),
                      duration_seconds=elapsed, read_only=True, robot_commands_sent=0,
                      audit_wall_started_unix=self.wall_started,
                      audit_wall_finished_unix=time.time(),
                      runtime_context_start=self.runtime_context_start,
                      runtime_context_end=context_end,
                      partial_scope_start=self.partial_scope_start,
                      partial_scope_end=partial_scope_context(self.scope_path),
                      run_pointer_changed=self.runtime_context_start.get('run') != context_end.get('run'),
                      raw_cloud_files_written=0, target_coordinate_frame=TARGET_FRAME,
                      registered_sensor_frame_alias='Only registered_points native map aliases lio_map; all display paths use actual TF.',
                      ground_scan_pose_gate_seconds=.08,
                      overlay_source_time_gate_seconds=.30,
                      canonical_source_stamp_exact_match_tolerance_seconds=.000001,
                      expected_inflation_display_offset_m=.035,
                      body_trajectory_is_not_a_ground_route=True,
                      local_floor_height_band_m=.65,
                      local_floor_band_note='Height band is explicit only; all supported-XY deviations remain reported. Outside-band points may be another storey or an error, not inferred good ground.',
                      message_counts=dict(self.counts),
                      receive_rates_hz={k: v / elapsed for k, v in self.counts.items()},
                      header_frames={k: dict(v) for k, v in self.header_frames.items()},
                      namespace_observations=dict(self.namespace_counts),
                      exclusion_counts=dict(self.exclusions),
                      independent_single_frame_ground_analysis=dict(
                          sample_count=len(self.ground_records),
                          valid_count=sum(r['ground']['valid'] for r in self.ground_records),
                          source='Independent one current registered scan, not the displayed shared2s ground',
                          records_key='ground_records'),
                      canonical_shared_ground_analysis=dict(
                          source_topic='/wavego/live_dense/ground_surface',
                          no_independent_fallback=True,
                          received_packet_counts=dict(self.canonical_payload_counts),
                          decoded_packet_valid_fraction=valid_packets/(valid_packets+invalid_packets)
                          if valid_packets+invalid_packets else None,
                          source_pose_matched_unique_model_count=len(self.canonical_records),
                          valid_source_pose_matched_unique_model_count=sum(r['surface']['valid'] for r in self.canonical_records),
                          actual_return_KD_verifications=len(verified),
                          sampled_support_actual_return_fraction_0p1mm=statistics([
                              r['support_exact_return_fraction_0p1mm'] for r in verified]),
                          full_window_time_coverage_support_actual_return_fraction_0p1mm=statistics([
                              r['support_exact_return_fraction_0p1mm'] for r in verified if r['full_2s_window_captured']]),
                          availability_note='Received sampled models, not a promise of continuous wall-time availability. Partial initial2s raw-return windows are explicit.',
                          records=self.canonical_records),
                      measured_ground_samples=len(self.ground_records),
                      valid_measured_ground_samples=sum(r['ground']['valid'] for r in self.ground_records),
                      measured_ground_slope_degrees=statistics([
                          r['floor_slope_degrees'] for r in self.ground_records if r['ground']['valid']]),
                      initial_alignment=self.initial_alignment_metadata,
                      initially_aligned_reference_errors=dict(
                          count=len(self.reference_errors),
                          norm_m=statistics([r['initial_registered_reference_error_norm_m'] for r in self.reference_errors]),
                          z_m=statistics([r['initial_registered_reference_error_xyz_m'][2] for r in self.reference_errors]),
                          rotation_degrees=statistics([r['initial_registered_reference_rotation_error_degrees'] for r in self.reference_errors]),
                          records=self.reference_errors,
                          note='Simulation reference audits estimated mapping only. No TF refit, point-cloud warp, or motion command.'),
                      reference_height_grouped_errors=grouped_errors,
                      reference_height_group_note='Actual reference body-z 1m bands, not inferred floor labels or fabricated floor positions.',
                      historic_visual_path_audit='Excluded from current-floor comparison; requires each stored past ref/LIO anchor and its past measured ground. Subscription counts only.',
                      geometry_provenance=dict(
                          map='Actual LiDAR/IMU nativeLIO estimate, unchanged by this audit',
                          floor='Shared2s measured registered returns; support KD-checked against actually received raw registered scans',
                          inflation='Current scan geometric clearance; colours/radii are display and planner metadata',
                          dynamic='LiDAR geometric clustering, not verified neural human identity',
                          display_registration='Simulation reference used for visualization only, not nativeSLAM accuracy correction',
                          colour='Height colours are not semantic class labels; semantic correctness not assessed'),
                      ground_records=self.ground_records,
                      aggregate_overlay_alignment=aggregates,
                      overlay_records=dict(self.overlay_records),
                      audit_source_sha256=self.audit_source_sha256,
                      ground_surface_source_sha256=self.surface_source_sha256,
                      limitations='Local measured support only; no global map accuracy or navigation-safety claim. Historical maps with LIO drift cannot be repaired by this audit. Paths outside current support are excluded rather than fabricated.')
        self.output.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.output.with_suffix(self.output.suffix + '.tmp')
        temporary.write_text(json.dumps(report, indent=2))
        temporary.replace(self.output)
        return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True,
                        help='JSON file, or directory to receive display_alignment_audit.json')
    parser.add_argument('--duration', type=float, default=10.)
    parser.add_argument('--scope-json', type=Path,
                        help='Optional local-resume scope JSON; results remain separate from the original full course.')
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error('--duration must be positive')
    output = args.output if args.output.suffix.lower() == '.json' else args.output / 'display_alignment_audit.json'
    rclpy.init()
    node = Audit(output, args.scope_json)
    try:
        while time.monotonic() - node.started < args.duration:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        report = node.save()
        node.destroy_node()
        rclpy.try_shutdown()
        print(json.dumps(dict(output=str(output.resolve()),
                              measured_ground_samples=report['measured_ground_samples'],
                              valid_measured_ground_samples=report['valid_measured_ground_samples'],
                              aggregate_overlay_alignment=report['aggregate_overlay_alignment']), indent=2))


if __name__ == '__main__':
    main()
