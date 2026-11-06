"""Geometry/identity failure cases: these tests do not claim live OCR accuracy."""
import copy
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from go2w_door_anchor import DoorAnchorTracker, _project, _fit_door


class DoorAnchorGeometryTests(unittest.TestCase):
    def setUp(self):
        sensor = dict(position=dict(x=0, y=0, z=0),
                      rotation=dict(roll=0, pitch=0, yaw=0), fov=90)
        self.config = dict(robot=dict(sensors=dict(camera=copy.deepcopy(sensor),
                                                   depth_sensor=copy.deepcopy(sensor))))
        self.pose = np.eye(4)
        self.pose[2, 3] = 1.
        self.depth = np.full((200, 200), 3.)
        self.labels = np.zeros((200, 200), np.uint8)
        self.labels[40:180, 80:130] = 8
        self.objects = [dict(label='door', class_id=8, detection_score=.8,
                             sam_iou_prediction=.9, box_xyxy=[80, 40, 130, 180])]
        self.line = dict(text='Office B205', room_codes=['B205'], confidence=.95,
                         box_xyxy=[60, 80, 76, 96], region_id=1, confirmed=True,
                         confirmation_frames=3)
        self.goal = dict(position=[1.8, 0, 1.], yaw=0., frame='map')
        self.tracker = DoorAnchorTracker()

    def observe(self, stamp, **changes):
        args = dict(rgb_shape=(200, 200), semantic_labels=self.labels,
                    door_objects=self.objects, rgb_stamp=stamp, depth_stamp=stamp,
                    pose_stamp=stamp, frame='map', pose_frame='map', now=stamp,
                    navigation_goal=self.goal, desired_code='B205')
        args.update(changes)
        ocr = args.pop('ocr', dict(state='readable_text', input_stamp=stamp,
                                  lines=[copy.deepcopy(self.line)]))
        depth = args.pop('depth', self.depth)
        return self.tracker.observe(ocr, depth, self.config, self.pose, **args)

    def test_source_geometry_and_three_independent_observations(self):
        first = self.observe(10.)
        self.assertFalse(first['anchors'][0]['identity_confirmed'])
        self.assertEqual(first['observations'][0]['association_reason'],
                         'measured_coplanar_plate_near_unique_door')
        point = first['observations'][0]['position']
        np.testing.assert_allclose(point, [3., .945, 1.345], atol=.016)
        self.observe(11.)
        final = self.observe(12.)
        self.assertTrue(final['anchors'][0]['door_association_confirmed'])
        self.assertTrue(final['goal_status']['confirmed'])
        self.assertEqual(final['goal_status']['observed_room_codes'], ['B205'])
        self.assertAlmostEqual(final['goal_status']['goal_to_door_forward_m'], 1.2)
        self.assertNotEqual(final['goal_status']['sign_position'], self.goal['position'])

    def test_repeated_async_result_does_not_confirm(self):
        self.observe(10.)
        for _ in range(8):
            repeated = self.observe(10.)
        self.assertEqual(repeated['state'], 'projection_rejected')
        self.assertEqual(repeated['anchors'][0]['observation_count'], 1)
        self.assertFalse(repeated['goal_status']['confirmed'])

    def test_old_ocr_cannot_project_on_new_rgb_depth(self):
        result = self.observe(11., ocr=dict(state='readable_text', input_stamp=10., lines=[self.line]))
        self.assertEqual(result['reason'], 'ocr_source_does_not_match_projection_rgb')
        self.assertFalse(result['anchors'])
        self.assertFalse(result['goal_status']['confirmed'])

    def test_depth_and_pose_clock_gates(self):
        self.assertEqual(self.observe(10., depth_stamp=10.1)['reason'], 'ocr_depth_not_synchronized')
        self.assertEqual(self.observe(10., pose_stamp=10.1)['reason'], 'depth_pose_not_synchronized')

    def test_invalid_metric_depth_and_mount_suppress_anchors(self):
        result = self.observe(10., depth=np.full((200, 200), np.nan))
        self.assertFalse(result['anchors'])
        self.assertEqual(result['rejected'][-1]['reason'], 'insufficient_measured_text_depth')
        self.config['robot']['sensors']['camera']['position']['x'] = 29
        result = self.observe(11.)
        self.assertEqual(result['reason'], 'rgb_depth_mounts_differ')

    def test_directional_directory_does_not_name_door(self):
        other = copy.deepcopy(self.line)
        other.update(text='Storage B203', room_codes=['B203'], box_xyxy=[60, 100, 76, 116])
        for stamp in (10., 11., 12.):
            result = self.observe(stamp, ocr=dict(state='readable_text', input_stamp=stamp,
                                               lines=[self.line, other]))
        self.assertTrue(all(item['identity_confirmed'] for item in result['anchors']))
        self.assertTrue(all(item['sign_type'] == 'directional_sign' for item in result['anchors']))
        self.assertFalse(any(item['door_association_confirmed'] for item in result['anchors']))
        self.assertFalse(result['goal_status']['confirmed'])

    def test_room_identity_mismatch_survives_spatial_match(self):
        for stamp in (10., 11., 12.):
            result = self.observe(stamp, desired_code='B203')
        self.assertEqual(result['goal_status']['state'], 'observed_identity_mismatch')
        self.assertTrue(result['goal_status']['spatial_match'])
        self.assertFalse(result['goal_status']['confirmed'])

    def test_anchors_persist_without_claiming_current_visibility(self):
        for stamp in (10., 11., 12.):
            self.observe(stamp)
        status = self.tracker.goal_status(self.goal, 'map', None, 'B205', 17.)
        self.assertEqual(status['state'], 'confirmed_persisted')
        self.assertFalse(status['confirmed'])
        self.assertTrue(status['persisted_match'])
        self.assertFalse(self.tracker.snapshot(17.)[0]['currently_visible'])

    def test_wrong_frame_is_rejected(self):
        result = self.observe(10., pose_frame='odom')
        self.assertEqual(result['reason'], 'pose_coordinate_frame_mismatch')
        self.assertFalse(result['anchors'])

    def test_pending_ocr_candidates_are_not_anchors(self):
        line = copy.deepcopy(self.line)
        line['confirmed'] = False
        result = self.observe(10., ocr=dict(state='temporal_confirmation_pending', input_stamp=10.,
                                         lines=[line]))
        self.assertFalse(result['anchors'])
        self.assertEqual(result['rejected'][-1]['reason'], 'ocr_temporal_confirmation_pending')

    def test_number_only_and_dotted_room_codes_are_preserved(self):
        for code in ('203', '01.06'):
            self.tracker = DoorAnchorTracker()
            line = copy.deepcopy(self.line)
            line.update(text=code, room_codes=[code])
            for stamp in (10., 11., 12.):
                result = self.observe(stamp, desired_code=code, ocr=dict(
                    state='readable_text', input_stamp=stamp, lines=[line]))
            self.assertEqual(result['goal_status']['observed_room_codes'], [code])
            self.assertTrue(result['goal_status']['confirmed'])

    def test_initial_rigid_alignment_is_used_for_goal_only(self):
        alignment = np.eye(4)
        alignment[:3, :3] = np.diag([-1., -1., 1.])
        alignment[:3, 3] = [8., 0, -.4]
        goal_source = np.linalg.inv(alignment) @ [*self.goal['position'], 1.]
        goal = dict(position=goal_source[:3].tolist(), yaw=np.pi, frame='sim')
        registration = dict(parent='map', child='sim', matrix=alignment.tolist())
        for stamp in (10., 11., 12.):
            result = self.observe(stamp, navigation_goal=goal, frame_alignment=registration)
        self.assertTrue(result['goal_status']['confirmed'])
        np.testing.assert_allclose(result['goal_status']['goal_position'], self.goal['position'])
        np.testing.assert_allclose(result['observations'][0]['position'][0], 3.)

    def test_rgb_depth_different_resolutions_use_intrinsics(self):
        world, valid, rgb_u, rgb_v, camera = _project(
            np.full((5, 5), 2.), self.config, np.eye(4), (15, 15))
        np.testing.assert_allclose(world[2, 2], [2., 0, 0])
        self.assertTrue(valid[2, 2])
        self.assertEqual(rgb_u[2, 2], 7)
        self.assertEqual(rgb_v[2, 2], 7)
        np.testing.assert_allclose(camera, [0, 0, 0])

    def test_visible_upper_door_does_not_require_observed_floor(self):
        self.labels[:] = 0
        self.labels[10:75, 80:130] = 8
        self.objects[0]['box_xyxy'] = [80, 10, 130, 75]
        self.line['box_xyxy'] = [60, 20, 76, 35]
        for stamp in (10., 11., 12.):
            result = self.observe(stamp)
        door = result['measured_doors'][0]
        self.assertGreater(door['observed_z_bounds_m'][0] - .5, self.goal['position'][2])
        self.assertTrue(result['goal_status']['confirmed'])
        self.assertAlmostEqual(result['goal_status']['goal_camera_height_delta_m'], 0.)

    def test_camera_on_other_floor_cannot_confirm_upper_door(self):
        self.labels[:] = 0
        self.labels[10:75, 80:130] = 8
        self.objects[0]['box_xyxy'] = [80, 10, 130, 75]
        self.line['box_xyxy'] = [60, 20, 76, 35]
        wrong_floor = dict(self.goal, position=[1.8, 0, 4.])
        for stamp in (10., 11., 12.):
            result = self.observe(stamp, navigation_goal=wrong_floor)
        self.assertTrue(result['anchors'][0]['door_association_confirmed'])
        self.assertFalse(result['goal_status']['confirmed'])

    def test_confirmed_conflicting_codes_at_one_plate_block_both_targets(self):
        for stamp in (10., 11., 12.):
            self.observe(stamp)
        different = copy.deepcopy(self.line)
        different.update(text='Office B203', room_codes=['B203'])
        for stamp in (13., 14., 15.):
            result = self.observe(stamp, desired_code='B203', ocr=dict(
                state='readable_text', input_stamp=stamp, lines=[different]))
        self.assertEqual(len(result['anchors']), 2)
        self.assertTrue(all(item['identity_conflict'] for item in result['anchors']))
        self.assertFalse(any(item['identity_confirmed'] for item in result['anchors']))
        self.assertTrue(all(item['observation_identity_confirmed'] for item in result['anchors']))
        for desired in (None, 'B205', 'B203'):
            status = self.tracker.goal_status(self.goal, 'map', None, desired, 15.)
            self.assertEqual(status['state'], 'identity_conflict')
            self.assertFalse(status['confirmed'])
            self.assertEqual(status['observed_room_codes'], ['B203', 'B205'])

    def test_one_unconfirmed_different_read_does_not_invent_identity_conflict(self):
        for stamp in (10., 11., 12.):
            self.observe(stamp)
        different = copy.deepcopy(self.line)
        different.update(text='Office B203', room_codes=['B203'])
        result = self.observe(13., ocr=dict(state='readable_text', input_stamp=13., lines=[different]))
        self.assertFalse(any(item['identity_conflict'] for item in result['anchors']))
        self.assertTrue(result['goal_status']['confirmed'])

    def test_different_physical_plates_are_not_an_identity_conflict(self):
        for stamp in (10., 11., 12.):
            self.observe(stamp)
        different = copy.deepcopy(self.line)
        different.update(text='Office B203', room_codes=['B203'], box_xyxy=[130, 80, 146, 96])
        for stamp in (13., 14., 15.):
            result = self.observe(stamp, ocr=dict(state='readable_text', input_stamp=stamp, lines=[different]))
        self.assertEqual(len(result['anchors']), 2)
        self.assertFalse(any(item['identity_conflict'] for item in result['anchors']))
        self.assertTrue(all(item['identity_confirmed'] for item in result['anchors']))

    def test_measured_door_face_survives_glass_background_outliers(self):
        random = np.random.default_rng(7)
        face = np.column_stack([3 + random.normal(0, .005, 700),
                                random.uniform(-1, 1, 700), random.uniform(.6, 2.6, 700)])
        background = np.column_stack([6 + random.normal(0, .005, 300),
                                      random.uniform(-1, 1, 300), random.uniform(.6, 2.6, 300)])
        points = np.vstack([face, background])
        original = points.copy()
        door = _fit_door(points, np.array([0., 0, 1.]))
        self.assertAlmostEqual(door['position'][0], 3., delta=.015)
        self.assertGreater(door['plane_inlier_ratio'], .65)
        self.assertLess(door['plane_inlier_ratio'], .75)
        self.assertLess(door['plane_residual_p90_m'], .015)
        self.assertLess(door['normal'][0], -.99)
        np.testing.assert_array_equal(points, original)

    def test_no_dominant_measured_plane_does_not_lower_support_requirement(self):
        random = np.random.default_rng(7)
        points = np.concatenate([np.column_stack([np.full(300, x), random.uniform(-1, 1, 300),
                                                  random.uniform(.6, 2.6, 300)]) for x in (2., 4., 6.)])
        with self.assertRaisesRegex(ValueError, 'door_mask_not_consistent_measured_plane'):
            _fit_door(points, np.array([0., 0, 1.]))


if __name__ == '__main__':
    unittest.main()
