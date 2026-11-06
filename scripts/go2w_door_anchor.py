"""Associate measured room text with measured doors, without room/scene priors.

This module has no ROS or navigation side effects. Call ``observe`` with the
*source* OCR image and source-frame masks, metric depth and pose. An asynchronous
OCR box must never be projected using the most recent moving-camera frame.

``frame_alignment`` has the same parent/child/matrix convention as the live
route display: matrix maps child coordinates into parent coordinates. Room text
and directional signs remain distinct; a directory sign does not name a nearby
door. Persistent anchors are explicitly different from current visibility.

Coordinates have three different meanings:
* ``sign_position``: median real depth within the OCR text polygon;
* ``door_position``: center of observed door-plane samples, not the center of
  an inferred or known mesh (partial visibility can shift this center);
* ``goal_position``: supplied robot stand-off/navigation goal transformed into
  the output frame, never replaced by the higher sign position.

Mounts use the existing UE config's centimetres and Euler sign conversion;
output distances are metres. FOV is horizontal; square-pixel intrinsics align
different image resolutions and aspect ratios. An initial rigid alignment
preserves its provenance. Consistency is not surveyed absolute map accuracy.
"""
from __future__ import annotations

from collections import deque
import copy
import math

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from go2w_sign_ocr import ROOM_CODE


def _pose_matrix(pose):
    if isinstance(pose, np.ndarray):
        transform = np.asarray(pose, float)
    elif isinstance(pose, dict):
        transform = np.eye(4)
        position = pose.get('position', pose.get('translation'))
        quaternion = pose.get('quaternion', pose.get('orientation'))
        if isinstance(position, dict):
            position = [position[key] for key in ('x', 'y', 'z')]
        if isinstance(quaternion, dict):
            quaternion = [quaternion[key] for key in ('x', 'y', 'z', 'w')]
        transform[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
        transform[:3, 3] = position
    else:
        transform = np.eye(4)
        p, q = pose.position, pose.orientation
        transform[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        transform[:3, 3] = [p.x, p.y, p.z]
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError('invalid_pose')
    return transform


def _sensor_transform(sensor):
    r, p = sensor['rotation'], sensor['position']
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler(
        'xyz', [r['roll'], -r['pitch'], -r['yaw']], degrees=True).as_matrix()
    transform[:3, 3] = np.array([p['x'], -p['y'], p['z']], float) * .01
    return transform


def _project(depth, config, pose, rgb_shape):
    sensors = config['robot']['sensors']
    sensor, camera = sensors['depth_sensor'], sensors['camera']
    if (sensor['position'] != camera['position'] or
            sensor['rotation'] != camera['rotation']):
        raise ValueError('rgb_depth_mounts_differ')
    if np.asarray(depth).ndim != 2:
        raise ValueError('depth_not_single_channel_metric_image')
    height, width = depth.shape
    rgb_height, rgb_width = rgb_shape[:2]
    if min(height, width, rgb_height, rgb_width) < 2:
        raise ValueError('invalid_image_shape')
    depth_focal = width / (2 * np.tan(np.deg2rad(sensor['fov']) / 2))
    rgb_focal = rgb_width / (2 * np.tan(np.deg2rad(camera['fov']) / 2))
    if not np.isfinite([depth_focal, rgb_focal]).all() or min(depth_focal, rgb_focal) <= 0:
        raise ValueError('invalid_camera_intrinsics')
    v, u = np.indices(depth.shape)
    xyz = np.stack([depth, -(u - (width - 1) / 2) * depth / depth_focal,
                    -(v - (height - 1) / 2) * depth / depth_focal], axis=-1)
    world_from_camera = _pose_matrix(pose) @ _sensor_transform(sensor)
    world = xyz @ world_from_camera[:3, :3].T + world_from_camera[:3, 3]
    rgb_u = (u - (width - 1) / 2) * rgb_focal / depth_focal + (rgb_width - 1) / 2
    rgb_v = (v - (height - 1) / 2) * rgb_focal / depth_focal + (rgb_height - 1) / 2
    valid = np.isfinite(depth) & (depth > .15) & (depth < 8.)
    valid &= ((rgb_u >= 0) & (rgb_u < rgb_width) &
              (rgb_v >= 0) & (rgb_v < rgb_height))
    return world, valid, rgb_u, rgb_v, world_from_camera[:3, 3]


def _polygon_mask(line, rgb_shape):
    height, width = rgb_shape[:2]
    polygon = line.get('polygon_xy')
    if polygon is None:
        x0, y0, x1, y1 = line['box_xyxy']
        polygon = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]
    polygon = np.asarray(polygon, float)
    if polygon.ndim != 2 or polygon.shape[1] != 2 or not np.isfinite(polygon).all():
        raise ValueError('invalid_text_polygon')
    mask = np.zeros((height, width), np.uint8)
    cv2.fillPoly(mask, [np.rint(polygon).astype(np.int32)], 1)
    return mask


def _sample_rgb_mask(mask, rgb_u, rgb_v):
    u = np.clip(np.rint(rgb_u).astype(int), 0, mask.shape[1] - 1)
    v = np.clip(np.rint(rgb_v).astype(int), 0, mask.shape[0] - 1)
    return mask[v, u].astype(bool)


def _text_position(line, depth, world, valid, rgb_u, rgb_v, rgb_shape):
    selected = _sample_rgb_mask(_polygon_mask(line, rgb_shape), rgb_u, rgb_v)
    total = int(selected.sum())
    good = selected & valid
    count = int(good.sum())
    if total < 6 or count < 6 or count / total < .65:
        raise ValueError('insufficient_measured_text_depth')
    values = depth[good]
    median = np.median(values)
    deviations = np.abs(values - median)
    mad = np.median(deviations)
    accepted = deviations <= max(.035, 3.5 * mad)
    if int(accepted.sum()) < 6 or accepted.mean() < .65 or mad > .08:
        raise ValueError('text_depth_not_consistent_surface')
    points = world[good][accepted]
    position = np.median(points, axis=0)
    return position, dict(valid_depth_pixels=count, selected_depth_pixels=total,
                          retained_depth_pixels=len(points), axial_depth_m=float(median),
                          axial_depth_mad_m=float(mad),
                          spatial_spread_p90_m=float(np.quantile(
                              np.linalg.norm(points - position, axis=1), .9)))


def _fit_door(points, camera_position):
    if len(points) < 40:
        raise ValueError('insufficient_measured_door_depth')
    if len(points) > 2400:
        points = points[np.linspace(0, len(points) - 1, 2400, dtype=int)]
    # Glass pixels can measure a wall behind the door. Initial PCA over all
    # samples can rotate toward those deeper points and reject the real face.
    # Search measured vertical planes first, then refine the dominant face.
    # This never creates a point or supplies a scene/door location prior.
    random = np.random.default_rng(0)
    retained = np.zeros(len(points), bool)
    best_count = 0
    for _ in range(180):
        sample = points[random.choice(len(points), size=3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        length = np.linalg.norm(normal)
        if length < 1e-7:
            continue
        normal /= length
        if abs(normal[2]) > .45:
            continue
        candidate = np.abs((points - sample[0]) @ normal) <= .04
        count = int(candidate.sum())
        if count > best_count:
            best_count, retained = count, candidate
    if best_count < 40 or best_count / len(points) < .45:
        raise ValueError('door_mask_not_consistent_measured_plane')
    for _ in range(4):
        current = points[retained]
        center = np.median(current, axis=0)
        _, _, vectors = np.linalg.svd(current - center, full_matrices=False)
        normal = vectors[-1]
        residuals = np.abs((points - center) @ normal)
        median = np.median(residuals[retained])
        threshold = min(.10, max(.025, 3.5 * median))
        retained = residuals <= threshold
        if retained.sum() < 40 or retained.mean() < .45:
            raise ValueError('door_mask_not_consistent_measured_plane')
    current = points[retained]
    center = np.median(current, axis=0)
    if abs(normal[2]) > .45:
        raise ValueError('door_plane_not_vertical')
    if np.dot(normal, camera_position - center) < 0:
        normal *= -1
    tangent = np.cross([0., 0., 1.], normal)
    tangent /= np.linalg.norm(tangent)
    lateral = current @ tangent
    low, high = np.quantile(lateral, [.02, .98])
    z_low, z_high = np.quantile(current[:, 2], [.02, .98])
    # These are extents of observed samples, not an inferred complete door mesh.
    return dict(position=center.tolist(), normal=normal.tolist(), tangent=tangent.tolist(),
                lateral_bounds_m=[float(low), float(high)],
                observed_z_bounds_m=[float(z_low), float(z_high)],
                measured_depth_points=int(len(points)), plane_inlier_points=int(retained.sum()),
                plane_inlier_ratio=float(retained.mean()),
                plane_residual_p90_m=float(np.quantile(residuals[retained], .9)),
                plane_fit_method='fixed_seed_vertical_plane_RANSAC_then_SVD',
                initial_ransac_inlier_ratio=float(best_count / len(points)))


def _directional(line, lines):
    if line.get('sign_type') in ('directional', 'directory', 'directional_sign'):
        return True
    if line.get('direction') or line.get('arrow') or line.get('arrow_detected'):
        return True
    region = line.get('region_id', line.get('sign_id'))
    neighbours = [item for item in lines if region is None or
                  item.get('region_id', item.get('sign_id')) == region]
    codes = {code for item in neighbours for code in item.get('room_codes', [])}
    # A multi-room plaque is conservatively a directory even without arrow OCR.
    return len(codes) > 1


def _door_distance(position, door):
    center, normal, tangent = (np.asarray(door[key]) for key in ('position', 'normal', 'tangent'))
    plane = abs(float(np.dot(position - center, normal)))
    coordinate = float(np.dot(position, tangent))
    lo, hi = door['lateral_bounds_m']
    lateral = max(lo - coordinate, coordinate - hi, 0.)
    lo, hi = door['observed_z_bounds_m']
    vertical = max(lo - position[2], position[2] - hi, 0.)
    return math.sqrt(plane ** 2 + lateral ** 2 + vertical ** 2), plane


def _goal_in_frame(goal, frame, alignment):
    if not goal:
        return None
    position = np.asarray(goal['position'], float)
    yaw = float(goal.get('yaw', 0.))
    direction = np.array([math.cos(yaw), math.sin(yaw), 0.])
    source_frame = goal.get('frame', frame)
    if source_frame == frame:
        return position, direction
    if not alignment:
        raise ValueError('missing_goal_frame_alignment')
    transform = np.asarray(alignment['matrix'], float)
    if alignment.get('parent') == frame and alignment.get('child') == source_frame:
        pass
    elif alignment.get('child') == frame and alignment.get('parent') == source_frame:
        transform = np.linalg.inv(transform)
    else:
        raise ValueError('unrelated_goal_coordinate_frames')
    return (transform[:3, :3] @ position + transform[:3, 3],
            transform[:3, :3] @ direction)


class DoorAnchorTracker:
    """Three independent source frames confirm identity and door association."""

    def __init__(self, confirmation_frames=3, max_anchor_count=64, max_visibility_age=2.5):
        self.confirmation_frames = max(3, int(confirmation_frames))
        self.max_anchor_count = max_anchor_count
        self.max_visibility_age = float(max_visibility_age)
        self.anchors = {}
        self.doors = {}
        self.next_anchor = 1
        self.next_door = 1
        self.last_source_stamp = -math.inf

    def _track_doors(self, measured, frame, stamp):
        available = {key for key, value in self.doors.items()
                     if value['frame'] == frame and stamp - value['last_seen'] < 30.}
        output = []
        for door in measured:
            choices = [(np.linalg.norm(np.asarray(door['position']) - self.doors[key]['position']), key)
                       for key in available if abs(np.dot(door['normal'], self.doors[key]['normal'])) > .85]
            distance, key = min(choices, default=(math.inf, None))
            if distance > .65:
                key = self.next_door
                self.next_door += 1
            else:
                available.remove(key)
            current = dict(door, id=key, frame=frame, last_seen=float(stamp))
            self.doors[key] = current
            output.append(current)
        self.doors = {key: value for key, value in self.doors.items() if stamp - value['last_seen'] < 120.}
        return output

    def _track_anchor(self, observation, stamp):
        position = np.asarray(observation['position'])
        choices = [(np.linalg.norm(position - value['position']), key)
                   for key, value in self.anchors.items()
                   if value['frame'] == observation['frame'] and
                   value['room_codes'] == observation['room_codes'] and
                   value['sign_type'] == observation['sign_type']]
        distance, key = min(choices, default=(math.inf, None))
        if distance > .40:
            key = self.next_anchor
            self.next_anchor += 1
            self.anchors[key] = dict(observation, id=key, first_seen=float(stamp),
                                     history=deque(maxlen=8), associated_history=deque(maxlen=8))
        item = self.anchors[key]
        # Repeated display of one asynchronous result is a single observation.
        if not item['history'] or stamp > item['history'][-1][0] + .05:
            item['history'].append((float(stamp), position.copy(), observation['confidence']))
            item['associated_history'].append(observation.get('door_id'))
        history = item['history']
        points = np.array([entry[1] for entry in history])
        median = np.median(points, axis=0)
        spread = float(np.max(np.linalg.norm(points - median, axis=1)))
        confirmed = len(history) >= self.confirmation_frames and spread <= .18
        association_history = list(item['associated_history'])
        door_id = observation.get('door_id')
        consistent_door = (door_id is not None and len(association_history) >= self.confirmation_frames and
                           all(candidate == door_id for candidate in association_history[-self.confirmation_frames:]))
        for field in ('text', 'confidence', 'box_xyxy', 'polygon_xy', 'depth_quality', 'source_stamp',
                      'door_id', 'door_distance_m', 'association_reason', 'camera_position'):
            if field in observation:
                item[field] = observation[field]
            else:
                item.pop(field, None)
        item.update(position=median.tolist(), last_seen=float(stamp), observation_count=len(history),
                    spatial_consistency_m=spread, identity_confirmed=confirmed,
                    door_association_confirmed=bool(confirmed and consistent_door),
                    confirmation_method='independent OCR source timestamps and measured 3D consistency')
        if len(self.anchors) > self.max_anchor_count:
            oldest = min(self.anchors, key=lambda candidate: self.anchors[candidate]['last_seen'])
            del self.anchors[oldest]
        return key

    def snapshot(self, now):
        output = []
        for item in self.anchors.values():
            visible = 0 <= now - item['last_seen'] <= self.max_visibility_age
            anchor = {key: copy.deepcopy(value) for key, value in item.items()
                      if key not in ('history', 'associated_history')}
            anchor.update(currently_visible=visible, age_seconds=max(0., now - item['last_seen']),
                          state=('observed_confirmed' if visible else 'persisted_confirmed')
                          if item['identity_confirmed'] else 'tentative')
            output.append(anchor)
        # The desired room name must never choose between contradictory reads
        # at one measured physical plate. Independent directory rows are not
        # plate identities. Keep conflicting observations for inspection,
        # while withdrawing their identity/door confirmation in the public
        # snapshot. A runtime reset starts a new observation history; no code
        # is silently rewritten to resolve a disagreement.
        for anchor in output:
            conflicts = [other for other in output if
                         other['id'] != anchor['id'] and
                         anchor['identity_confirmed'] and
                         other.get('observation_identity_confirmed', other['identity_confirmed']) and
                         anchor['frame'] == other['frame'] and
                         anchor['sign_type'] == other['sign_type'] == 'room_plate_candidate' and
                         set(anchor['room_codes']) != set(other['room_codes']) and
                         np.linalg.norm(np.asarray(anchor['position']) - other['position']) <= .25]
            anchor['observation_identity_confirmed'] = anchor['identity_confirmed']
            anchor['geometry_association_confirmed'] = anchor['door_association_confirmed']
            anchor['identity_conflict'] = bool(conflicts)
            if conflicts:
                anchor.update(identity_confirmed=False, door_association_confirmed=False,
                              state='identity_conflict',
                              conflicting_anchor_ids=sorted([anchor['id'], *[other['id'] for other in conflicts]]),
                              conflicting_room_codes=sorted({code for other in [anchor, *conflicts]
                                                             for code in other['room_codes']}))
        return output

    def goal_status(self, goal, frame, alignment, desired_code, now):
        base = dict(state='no_matching_observation', confirmed=False, desired_code=desired_code,
                    identity_requested=desired_code is not None, frame=frame,
                    uses_preset_room_identity=False)
        try:
            transformed = _goal_in_frame(goal, frame, alignment)
        except (ValueError, KeyError, np.linalg.LinAlgError) as exc:
            return dict(base, state='coordinate_transform_unavailable', reason=str(exc))
        if transformed is None:
            return dict(base, state='navigation_goal_unavailable')
        position, direction = transformed
        direction[2] = 0
        if np.linalg.norm(direction) < .5:
            return dict(base, state='invalid_goal_direction')
        direction /= np.linalg.norm(direction)
        candidates = []
        conflicts = []
        for anchor in self.snapshot(now):
            door = self.doors.get(anchor.get('door_id'))
            association = anchor['door_association_confirmed'] or (
                anchor.get('identity_conflict') and anchor.get('geometry_association_confirmed'))
            if not door or door['frame'] != frame or not association:
                continue
            if anchor['sign_type'] == 'directional_sign':
                continue
            delta = np.asarray(door['position']) - position
            forward = float(np.dot(delta[:2], direction[:2]))
            lateral = abs(float(direction[0] * delta[1] - direction[1] * delta[0]))
            normal_agreement = abs(float(np.dot(np.asarray(door['normal'])[:2], direction[:2])))
            # The upward camera can see only the upper part of a door. Visible
            # sample extents are not the full door height or a floor estimate.
            camera_z = anchor['camera_position'][2]
            z_low, z_high = door['observed_z_bounds_m']
            height_ok = (abs(position[2] - camera_z) <= .8 and
                         position[2] - .5 <= z_high <= position[2] + 3.2 and
                         z_low <= position[2] + 2.4)
            spatial = .2 <= forward <= 2.8 and lateral <= .65 and normal_agreement >= .75 and height_ok
            if not spatial:
                continue
            if anchor.get('identity_conflict'):
                conflicts.append((anchor, door))
                continue
            matches_code = desired_code is None or desired_code in anchor['room_codes']
            candidates.append(dict(anchor=anchor, door=door, spatial_match=True,
                                   requested_identity_match=matches_code, goal_to_door_forward_m=forward,
                                   goal_to_door_lateral_m=lateral, door_normal_agreement=normal_agreement,
                                   goal_camera_height_delta_m=abs(float(position[2] - camera_z)),
                                   observed_door_z_bounds_m=door['observed_z_bounds_m']))
        if conflicts:
            anchor, door = conflicts[0]
            return dict(base, state='identity_conflict', persisted_match=False,
                        observed_room_codes=sorted({code for item, _ in conflicts
                                                    for code in item['conflicting_room_codes']}),
                        conflicting_anchor_ids=sorted({key for item, _ in conflicts
                                                       for key in item['conflicting_anchor_ids']}),
                        anchor_id=anchor['id'], door_id=door['id'], sign_position=anchor['position'],
                        door_position=door['position'], goal_position=position.tolist(),
                        reason='Different independently confirmed room codes at the same measured physical plate')
        if not candidates:
            return base
        candidates.sort(key=lambda candidate: (not candidate['requested_identity_match'],
                                               candidate['goal_to_door_lateral_m']))
        matches = [candidate for candidate in candidates if candidate['requested_identity_match']]
        if len(matches) > 1 and abs(matches[0]['goal_to_door_lateral_m'] -
                                    matches[1]['goal_to_door_lateral_m']) < .25:
            return dict(base, state='ambiguous_observed_target_doors',
                        candidate_anchor_ids=[candidate['anchor']['id'] for candidate in matches])
        selected = candidates[0]
        anchor, door = selected.pop('anchor'), selected.pop('door')
        fresh = anchor['currently_visible'] and 0 <= now - door['last_seen'] <= self.max_visibility_age
        identified = bool(selected['requested_identity_match'])
        return dict(base, **selected, state=('confirmed_visible' if fresh else 'confirmed_persisted')
                    if identified else 'observed_identity_mismatch', confirmed=bool(identified and fresh),
                    persisted_match=identified, current_observation=fresh, anchor_id=anchor['id'],
                    door_id=door['id'], observed_text=anchor['text'], observed_room_codes=anchor['room_codes'],
                    sign_position=anchor['position'], door_position=door['position'],
                    goal_position=position.tolist(), last_observed_stamp=anchor['last_seen'],
                    warning='Spatial association of measured plate and door; not a surveyed absolute map accuracy claim')

    def observe(self, ocr_result, depth, config, pose, *, rgb_shape,
                semantic_labels=None, door_objects=(), rgb_stamp, depth_stamp,
                pose_stamp, frame='lio_map', pose_frame=None, now=None,
                navigation_goal=None, frame_alignment=None, desired_code=None):
        now = float(rgb_stamp if now is None else now)
        output = dict(state='observed', observations=[], measured_doors=[], rejected=[],
                      frame=frame, source_stamp=float(rgb_stamp),
                      geometry_source='synchronized metric UE axial depth + source-frame pose',
                      uses_scene_room_prior=False)
        try:
            source_stamp = float(ocr_result['input_stamp'])
            if not np.isfinite([source_stamp, rgb_stamp, depth_stamp, pose_stamp, now]).all():
                raise ValueError('invalid_observation_timestamp')
            if pose_frame is not None and pose_frame != frame:
                raise ValueError('pose_coordinate_frame_mismatch')
            if abs(source_stamp - float(rgb_stamp)) > .025:
                raise ValueError('ocr_source_does_not_match_projection_rgb')
            if abs(float(depth_stamp) - source_stamp) > .05:
                raise ValueError('ocr_depth_not_synchronized')
            if abs(float(pose_stamp) - float(depth_stamp)) > .08:
                raise ValueError('depth_pose_not_synchronized')
            if not 0 <= now - source_stamp <= 4.:
                raise ValueError('source_observation_expired')
            if source_stamp <= self.last_source_stamp:
                raise ValueError('source_frame_already_processed')
            world, valid, u, v, camera = _project(np.asarray(depth), config, pose, rgb_shape)
            self.last_source_stamp = source_stamp
            measured = []
            if semantic_labels is not None:
                if np.asarray(semantic_labels).shape != tuple(rgb_shape[:2]):
                    raise ValueError('source_semantic_mask_shape_mismatch')
                for index, obj in enumerate(door_objects):
                    if obj.get('class_id') != 8 and obj.get('label') != 'door':
                        continue
                    if obj.get('detection_score', 0) < .40 or obj.get('sam_iou_prediction', 0) < .65:
                        continue
                    mask = (np.asarray(semantic_labels) == 8).astype(np.uint8)
                    x0, y0, x1, y1 = map(int, obj['box_xyxy'])
                    roi = np.zeros_like(mask)
                    roi[max(0, y0):max(0, min(mask.shape[0], y1)),
                        max(0, x0):max(0, min(mask.shape[1], x1))] = 1
                    mask = cv2.erode(mask & roi, np.ones((3, 3), np.uint8))
                    try:
                        door = _fit_door(world[valid & _sample_rgb_mask(mask, u, v)], camera)
                    except ValueError as exc:
                        output['rejected'].append(dict(kind='door', index=index, reason=str(exc)))
                        continue
                    door.update(detection_score=float(obj['detection_score']),
                                sam_iou_prediction=float(obj['sam_iou_prediction']),
                                box_xyxy=obj['box_xyxy'], source_stamp=source_stamp)
                    measured.append(door)
            doors = self._track_doors(measured, frame, source_stamp)
            output['measured_doors'] = doors
            lines = ocr_result.get('lines', []) if ocr_result.get('state') not in (
                'stale', 'image_changed', 'unavailable', 'pending') else []
            for line in lines:
                codes = sorted({code for code in line.get('room_codes', []) if ROOM_CODE.fullmatch(code)})
                if not codes or line.get('confidence', 0) < .70:
                    continue
                if line.get('confirmed') is False:
                    output['rejected'].append(dict(kind='text', text=line.get('text'),
                                                   reason='ocr_temporal_confirmation_pending'))
                    continue
                try:
                    position, quality = _text_position(line, depth, world, valid, u, v, rgb_shape)
                except (ValueError, KeyError) as exc:
                    output['rejected'].append(dict(kind='text', text=line.get('text'), reason=str(exc)))
                    continue
                directional = _directional(line, lines)
                observation = dict(text=line['text'], room_codes=codes, confidence=float(line['confidence']),
                                   box_xyxy=line['box_xyxy'], polygon_xy=line.get('polygon_xy'),
                                   position=position.tolist(), frame=frame, depth_quality=quality,
                                   source_stamp=source_stamp, camera_position=camera.tolist(),
                                   sign_type='directional_sign' if directional else 'room_plate_candidate',
                                   association_reason='directional_sign_does_not_name_door' if directional
                                   else 'no_nearby_measured_door')
                candidates = []
                if not directional:
                    for door in doors:
                        distance, plane = _door_distance(position, door)
                        if plane <= .25 and distance <= 1.0:
                            candidates.append((distance, door))
                candidates.sort(key=lambda candidate: candidate[0])
                if candidates and (len(candidates) == 1 or candidates[1][0] - candidates[0][0] >= .25):
                    distance, door = candidates[0]
                    observation.update(door_id=door['id'], door_distance_m=float(distance),
                                       association_reason='measured_coplanar_plate_near_unique_door')
                elif candidates:
                    observation['association_reason'] = 'ambiguous_nearby_doors'
                observation['anchor_id'] = self._track_anchor(observation, source_stamp)
                output['observations'].append(observation)
        except (ValueError, KeyError, TypeError, np.linalg.LinAlgError) as exc:
            output.update(state='projection_rejected', reason=str(exc))
        output['anchors'] = self.snapshot(now)
        output['goal_status'] = self.goal_status(navigation_goal, frame, frame_alignment, desired_code, now)
        return output
