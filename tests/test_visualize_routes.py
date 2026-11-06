"""Display registration must preserve measured geometry and safety bounds."""
import sys
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts/global_compare'))
import visualize_routes as visual
from navigation_surface import GroundSurface


def plane_surface(z=0., slope=(0., 0.)):
    x, y = np.meshgrid(np.linspace(-2., 2., 51), np.linspace(-2., 2., 51))
    cloud = np.column_stack((x.ravel(), y.ravel(), (z+slope[0]*x+slope[1]*y).ravel()))
    surface = GroundSurface.from_scan(cloud, [0., 0., z+.4])
    assert surface.valid
    return surface


def histories(reference_pose, lio_pose, ref_stamp=10., lio_stamp=10.):
    lio, reference = visual.PoseHistory(), visual.PoseHistory()
    lio.add(lio_stamp, lio_pose); reference.add(ref_stamp, reference_pose)
    return lio, reference


class RegistrationTests(unittest.TestCase):
    def test_source_registration_reconciles_nontrivial_pose_without_distortion(self):
        reference = visual.pose_matrix([8., -2., 3.], Rotation.from_euler('xyz', [.2, -.1, 1.]).as_quat())
        lio = visual.pose_matrix([-4., 1., 6.], Rotation.from_euler('xyz', [-.1, .05, -.5]).as_quat())
        matrix, audit = visual.synchronized_registration(*histories(reference, lio), 10., 10.1)
        self.assertTrue(audit['valid']); self.assertTrue(audit['ground_truth_pose_used_for_visualization'])
        np.testing.assert_allclose(matrix@reference, lio, atol=1e-12)
        points = np.array([[1., 0., .3], [2., 3., 1.]])
        transformed = visual.rigid_points(points, matrix)
        self.assertAlmostEqual(np.linalg.norm(points[0]-points[1]), np.linalg.norm(transformed[0]-transformed[1]))
        np.testing.assert_array_equal(points, [[1., 0., .3], [2., 3., 1.]])

    def test_two_individually_near_poses_still_need_mutual_synchronization(self):
        matrix, audit = visual.synchronized_registration(*histories(np.eye(4), np.eye(4), 9.93, 10.07), 10., 10.1)
        self.assertIsNone(matrix); self.assertEqual(audit['reason'], 'pose_not_synchronized')
        self.assertAlmostEqual(audit['pose_pair_delta_seconds'], .14)

    def test_stale_and_future_source_do_not_refresh_overlay(self):
        values = histories(np.eye(4), np.eye(4))
        for source, now in [(10., 10.81), (10., 9.90)]:
            matrix, audit = visual.synchronized_registration(*values, source, now)
            self.assertIsNone(matrix); self.assertEqual(audit['reason'], 'source_stale_or_future')
        matrix, audit = visual.synchronized_registration(visual.PoseHistory(), values[1], 10., 10.1)
        self.assertIsNone(matrix); self.assertEqual(audit['reason'], 'pose_missing')

    def test_pose_cache_copies_inputs_and_rejects_reordering(self):
        history = visual.PoseHistory(); pose = np.eye(4)
        self.assertTrue(history.add(10., pose)); pose[0, 3] = 100.
        self.assertFalse(history.add(9., pose)); self.assertFalse(history.add(10., pose))
        self.assertEqual(history.closest(10.)[1][0, 3], 0.)
        history.add(16., np.eye(4)); self.assertEqual(len(history.samples), 1)

    def test_successful_old_measurement_is_expired_in_current_status(self):
        saved = dict(valid=True, source_stamp=10., source_age_seconds=.03, reason='source_time_visual_registration')
        current = visual.current_audit_status(saved, 11.)
        self.assertFalse(current['valid']); self.assertTrue(current['last_registration_valid'])
        self.assertEqual(current['reason'], 'last_observation_expired')
        self.assertEqual(current['source_age_seconds'], 1.)
        self.assertEqual(current['source_age_seconds_at_registration'], .03)
        self.assertTrue(saved['valid']); self.assertEqual(saved['source_age_seconds'], .03)

    def test_marker_local_pose_is_applied_exactly_once(self):
        registration = visual.pose_matrix([4., -1., 2.], Rotation.from_euler('z', np.pi/2).as_quat())
        marker_quaternion = Rotation.from_euler('y', np.pi/2).as_quat()
        points = [[1., 0., 0.], [0., 1., 0.]]
        result = visual.bake_marker_points(points, [2., 3., 1.], marker_quaternion, registration)
        np.testing.assert_allclose(result, [[1., 1., 2.], [0., 1., 3.]], atol=1e-12)

    def test_arrow_vector_is_independent_of_registration_translation(self):
        rotation = Rotation.from_euler('xyz', [.2, -.1, .4]).as_quat()
        first = visual.pose_matrix([0., 0., 0.], rotation)
        second = visual.pose_matrix([100., -200., 30.], rotation)
        arrow = np.array([[1., 2., .25], [1.7, 2.3, .25]])
        a = visual.rigid_points(arrow, first); b = visual.rigid_points(arrow, second)
        np.testing.assert_allclose(a[1]-a[0], b[1]-b[0], atol=1e-12)

    def test_missing_measured_floor_never_creates_a_zero_height_floor(self):
        original = np.array([[1., 1., 4.5]])
        for surface in [None, GroundSurface.from_scan([], [0., 0., 4.4])]:
            result, supported = visual.measured_projection(original, [[1., 1., 0.]], surface)
            self.assertFalse(supported.any()); np.testing.assert_array_equal(result, original)

    def test_cross_floor_and_unobserved_xy_keep_original_registered_heights(self):
        surface = plane_surface(3.2)
        original = np.array([[1., 1., .4], [1., 1., 3.8], [8., 8., 3.4]])
        candidates = [[1., 1., 0.], [1., 1., 3.3], [8., 8., 3.3]]
        result, supported = visual.measured_projection(original, candidates, surface)
        np.testing.assert_array_equal(supported, [False, True, False])
        np.testing.assert_allclose(result[:, 2], [.4, 3.235, 3.4])


class SpatialPlanHistoryTests(unittest.TestCase):
    def test_delayed_actual_surface_appends_once_after_nominal_same_position(self):
        history=visual.VisualPoseAnchors();pose=visual.pose_matrix([0.,0.,.4],[0.,0.,0.,1.])
        self.assertTrue(history.consider(10.,10.,pose,pose))
        original=json.loads(json.dumps(history.anchors[0]))
        surface=plane_surface()
        self.assertTrue(history.consider(10.2,10.2,pose,pose,surface,10.2))
        self.assertEqual(history.anchors[0],original)
        self.assertEqual(len(history.anchors),2)
        self.assertFalse(history.consider(10.3,10.3,pose,pose,surface,10.3))
        _,audit,_,supported=visual.spatially_registered_path([[.8,0.,.09],[1.,0.,.09]],np.eye(4),history,return_support=True)
        self.assertTrue(supported.all());self.assertEqual(audit['measured_ground_points'],2)

    def test_bad_floor_time_cannot_upgrade_nominal_anchor(self):
        history=visual.VisualPoseAnchors();pose=visual.pose_matrix([0.,0.,.4],[0.,0.,0.,1.])
        history.consider(10.,10.,pose,pose)
        surface=plane_surface()
        self.assertFalse(history.consider(10.2,10.2,pose,pose,surface,10.1))
        self.assertEqual(len(history.anchors),1)
        _,audit,_,supported=visual.spatially_registered_path([[.8,0.,.09],[1.,0.,.09]],np.eye(4),history,return_support=True)
        self.assertFalse(supported.any());self.assertEqual(audit['measured_ground_points'],0)

    def test_closest_nominal_anchor_does_not_hide_nearby_supported_surface(self):
        history=visual.VisualPoseAnchors();pose=visual.pose_matrix([0.,0.,.4],[0.,0.,0.,1.])
        history.consider(10.,10.,pose,pose)
        ref=visual.pose_matrix([.4,0.,.4],[0.,0.,0.,1.]);lio=visual.pose_matrix([.4,0.,2.4],[0.,0.,0.,1.])
        history.consider(10.2,10.2,lio,ref,plane_surface(2.),10.2)
        xyz,audit,indices,supported=visual.spatially_registered_path([[.1,0.,.09],[.2,0.,.09]],np.eye(4),history,return_support=True)
        self.assertTrue(supported.all());np.testing.assert_array_equal(indices,[1,1])
        np.testing.assert_allclose(xyz[:,2],2.035)
        self.assertEqual(audit['measured_ground_points'],2)

    def test_supported_candidate_on_other_floor_cannot_upgrade_lower_nominal(self):
        history=visual.VisualPoseAnchors();lower=visual.pose_matrix([0.,0.,.4],[0.,0.,0.,1.])
        history.consider(10.,10.,lower,lower)
        ref=visual.pose_matrix([0.,0.,3.6],[0.,0.,0.,1.]);lio=visual.pose_matrix([0.,0.,2.],[0.,0.,0.,1.])
        history.consider(10.2,10.2,lio,ref,plane_surface(1.6),10.2)
        _,audit,indices,supported=visual.spatially_registered_path([[.1,0.,.09],[.2,0.,.09]],np.eye(4),history,return_support=True)
        self.assertFalse(supported.any());np.testing.assert_array_equal(indices,[0,0])
        self.assertEqual(audit['historic_nominal_height_points'],2)

    def test_capture_uses_canonical_source_pose_instead_of_latest_odometry(self):
        pose=visual.pose_matrix([0.,0.,.4],[0.,0.,0.,1.]);lio,reference=histories(pose,pose,10.1,10.1)
        later=visual.pose_matrix([.4,0.,.4],[0.,0.,0.,1.]);lio.add(10.5,later);reference.add(10.5,later)
        history=visual.VisualPoseAnchors();history.consider(10.,10.,pose,pose)
        surface=plane_surface();state=SimpleNamespace(lio_history=lio,reference_history=reference,anchors=history,surface=surface,surface_stamp=10.1,surface_payload=surface.payload(10.1))
        state.available_surface=lambda source:visual.Display.available_surface(state,source)
        visual.Display.capture_anchor(state,10.6)
        self.assertEqual(len(history.anchors),2)
        row=history.anchors[-1];self.assertAlmostEqual(row['lio_stamp'],10.1)
        self.assertAlmostEqual(row['reference_stamp'],10.1)
        self.assertAlmostEqual(row['measured_surface']['source_stamp'],10.1)
        np.testing.assert_array_equal(row['reference_pose'],pose)

    def test_canonical_source_pose_mismatch_does_not_fall_back_to_latest_pose(self):
        pose=visual.pose_matrix([0.,0.,.4],[0.,0.,0.,1.]);lio,reference=histories(pose,pose,10.19,10.1)
        lio.add(10.5,pose);reference.add(10.5,pose)
        history=visual.VisualPoseAnchors();history.consider(10.,10.,pose,pose)
        surface=plane_surface();state=SimpleNamespace(lio_history=lio,reference_history=reference,anchors=history,surface=surface,surface_stamp=10.1,surface_payload=surface.payload(10.1))
        state.available_surface=lambda source:visual.Display.available_surface(state,source)
        visual.Display.capture_anchor(state,10.6)
        self.assertEqual(len(history.anchors),1)
        xyz,_,_,support=visual.spatially_registered_path([[.8,0.,.09],[1.,0.,.09]],np.eye(4),history,return_support=True)
        self.assertEqual(len(visual.observed_route_segments(xyz,support)[0]),0)

    def test_wrong_frame_canonical_cannot_supply_an_upgrade_or_fake_line(self):
        pose=visual.pose_matrix([0.,0.,.4],[0.,0.,0.,1.]);lio,reference=histories(pose,pose,10.2,10.2)
        history=visual.VisualPoseAnchors();history.consider(10.,10.,pose,pose)
        state=SimpleNamespace(lio_history=lio,reference_history=reference,anchors=history,surface=None,surface_stamp=None,surface_payload=None)
        payload=plane_surface().payload(10.2);payload['frame']='map'
        visual.Display.ground_surface(state,SimpleNamespace(data=json.dumps(payload)))
        self.assertIsNone(state.surface)
        state.available_surface=lambda source:visual.Display.available_surface(state,source)
        visual.Display.capture_anchor(state,10.3)
        self.assertEqual(len(history.anchors),1)
        xyz,_,_,support=visual.spatially_registered_path([[.8,0.,.09],[1.,0.,.09]],np.eye(4),history,return_support=True)
        self.assertEqual(len(visual.observed_route_segments(xyz,support)[0]),0)

    def test_stale_actual_surface_does_not_upgrade_same_position(self):
        pose=visual.pose_matrix([0.,0.,.4],[0.,0.,0.,1.]);lio,reference=histories(pose,pose,10.,10.)
        lio.add(10.9,pose);reference.add(10.9,pose)
        history=visual.VisualPoseAnchors();history.consider(10.,10.,pose,pose)
        surface=plane_surface();state=SimpleNamespace(lio_history=lio,reference_history=reference,anchors=history,surface=surface,surface_stamp=10.,surface_payload=surface.payload(10.))
        state.available_surface=lambda source:visual.Display.available_surface(state,source)
        visual.Display.capture_anchor(state,11.)
        self.assertEqual(len(history.anchors),1)

    def test_anchor_spacing_height_and_strict_pose_pair_gate(self):
        history = visual.VisualPoseAnchors()
        first = visual.pose_matrix([0., 0., .4], [0., 0., 0., 1.])
        self.assertTrue(history.consider(10., 10.02, first, first))
        near = visual.pose_matrix([.24, 0., .4], [0., 0., 0., 1.])
        self.assertFalse(history.consider(10.1, 10.1, near, near))
        raised = visual.pose_matrix([.24, 0., .56], [0., 0., 0., 1.])
        self.assertTrue(history.consider(10.2, 10.2, raised, raised))
        distant = visual.pose_matrix([1., 0., .56], [0., 0., 0., 1.])
        self.assertFalse(history.consider(10.3, 10.381, distant, distant))
        self.assertFalse(history.consider(10.4, 10.4, np.zeros((4, 4)), distant))
        self.assertEqual(len(history.anchors), 2)

    def test_lower_floor_uses_its_historical_pose_despite_upper_floor_drift(self):
        history = visual.VisualPoseAnchors()
        lower = visual.pose_matrix([0., 0., .4], [0., 0., 0., 1.])
        self.assertTrue(history.consider(10., 10., lower, lower, plane_surface(), 10.))
        upper_ref = visual.pose_matrix([4., 0., 3.6], [0., 0., 0., 1.])
        upper_lio = visual.pose_matrix([4., 0., 2.], [0., 0., 0., 1.])
        upper_surface = plane_surface(1.6)
        upper_surface.support_xyz[:, 0] += 4.; upper_surface.base_xyz[0] += 4.
        history.consider(11., 11., upper_lio, upper_ref, upper_surface, 11.)
        current = upper_lio@np.linalg.inv(upper_ref)
        points = [[1., 1., .09], [4.1, 0., 3.29], [12., 0., .09]]
        output, audit, indices = visual.spatially_registered_path(points, current, history)
        np.testing.assert_array_equal(indices, [0, 1, -1])
        np.testing.assert_allclose(output[:, 2], [.035, 1.635, -1.51], atol=1e-12)
        self.assertEqual(audit['measured_ground_points'], 2)
        self.assertEqual(audit['nominal_unvisited_points'], 1)
        self.assertFalse(audit['lidar_geometry_modified'])

    def test_close_xy_different_floor_cannot_use_wrong_historic_anchor(self):
        history = visual.VisualPoseAnchors()
        pose = visual.pose_matrix([0., 0., .4], [0., 0., 0., 1.])
        history.consider(10., 10., pose, pose, plane_surface(), 10.)
        points = [[.1, 0., 3.29], [.1, 0., .80]]
        result, audit, indices = visual.spatially_registered_path(points, np.eye(4), history)
        np.testing.assert_array_equal(indices, [-1, -1])
        np.testing.assert_array_equal(result, points)
        self.assertEqual(audit['measured_ground_points'], 0)

    def test_pose_association_without_observed_ground_retains_nominal_height(self):
        history = visual.VisualPoseAnchors()
        ref = visual.pose_matrix([0., 0., .4], [0., 0., 0., 1.])
        lio = visual.pose_matrix([0., 0., 2.4], [0., 0., 0., 1.])
        history.consider(10., 10., lio, ref)
        result, audit, indices = visual.spatially_registered_path([[1., 0., .09]], np.eye(4), history)
        self.assertEqual(indices[0], 0); self.assertAlmostEqual(result[0, 2], 2.09)
        self.assertEqual(audit['historic_nominal_height_points'], 1)
        self.assertEqual(audit['measured_ground_points'], 0)

    def test_hot_restart_preserves_full_rotations_and_exact_measured_support(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'anchors.jsonl'; history = visual.VisualPoseAnchors(path)
            reference = visual.pose_matrix([0., 0., .4], Rotation.from_euler('xyz', [.1, -.2, .3]).as_quat())
            lio = visual.pose_matrix([0., 0., 1.4], Rotation.from_euler('xyz', [-.1, .1, -.2]).as_quat())
            surface = plane_surface(1.)
            self.assertTrue(history.consider(10., 10.01, lio, reference, surface, 10.02))
            expected = history.anchors[0]['reference_pose']
            reference[0, 3] = 100.; surface.support_xyz[0] = 100.
            reloaded = visual.VisualPoseAnchors(path)
            self.assertEqual(reloaded.rejected_saved_records, 0)
            np.testing.assert_allclose(reloaded.anchors[0]['reference_pose'], expected)
            np.testing.assert_array_equal(visual.VisualPoseAnchors.surface(reloaded.anchors[0]).support_xyz, history.anchors[0]['measured_surface']['support_xyz'])
            self.assertEqual(len(visual.VisualPoseAnchors.surface(reloaded.anchors[0]).support_xyz), history.anchors[0]['measured_surface']['support_count'])

    def test_yaw_only_or_broken_records_rejected_and_append_remains_readable(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'anchors.jsonl'; path.write_text('{"yaw":0}\n{"truncated":')
            history = visual.VisualPoseAnchors(path)
            self.assertEqual(history.rejected_saved_records, 2); self.assertFalse(history.anchors)
            pose = visual.pose_matrix([0., 0., .4], [0., 0., 0., 1.])
            history.consider(10., 10., pose, pose)
            reloaded = visual.VisualPoseAnchors(path)
            self.assertEqual(reloaded.rejected_saved_records, 2); self.assertEqual(len(reloaded.anchors), 1)

    def test_valid_json_wrong_types_do_not_crash_anchor_reload(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'anchors.jsonl'; path.write_text('[]\nnull\n123\n')
            history = visual.VisualPoseAnchors(path)
            self.assertEqual(history.rejected_saved_records, 3); self.assertFalse(history.anchors)

    def test_fresh_invalid_canonical_surface_blocks_independent_fallback(self):
        valid = plane_surface().payload(10.)
        state = SimpleNamespace(canonical_surface_received=False, surface=None, surface_stamp=None)
        visual.Display.ground_surface(state, SimpleNamespace(data=json.dumps(valid)))
        self.assertTrue(state.surface.valid)
        invalid = GroundSurface.from_scan([], [0., 0., .4]).payload(10.1)
        visual.Display.ground_surface(state, SimpleNamespace(data=json.dumps(invalid)))
        self.assertFalse(state.surface.valid)
        # The fallback would read a scan; canonical presence must return first.
        visual.Display.update_surface(state, 10.2)
        self.assertEqual(state.surface.reason, 'no_returns')
        self.assertEqual(state.surface_origin, 'canonical_live_dense_measured_surface')


@unittest.skipIf(visual.rclpy is None, 'ROS messages unavailable; run in office ROS environment')
class MarkerProjectionTests(unittest.TestCase):
    def marker(self, namespace, kind=None, position=(0., 0., 0.)):
        marker = visual.Marker(); marker.ns = namespace; marker.id = 3
        marker.header.frame_id = 'map'; marker.type = visual.Marker.SPHERE if kind is None else kind
        marker.action = visual.Marker.ADD; marker.pose.orientation.w = 1.
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = list(position)
        return marker

    def project(self, marker, matrix=None, surface=None, heads=None):
        return visual.Display.project_marker(None, marker, np.eye(4) if matrix is None else matrix, 10., surface, {} if heads is None else heads)

    def test_sampling_head_follows_projected_endpoint_on_measured_slope(self):
        surface = plane_surface(slope=(.15, -.06)); heads = {}
        line = self.marker('navigation_rollout_samples', visual.Marker.LINE_STRIP)
        line.points = [visual.Point(x=.8, y=.8, z=.52), visual.Point(x=1., y=1., z=.52)]
        projected, count, _ = self.project(line, surface=surface, heads=heads)
        head = self.marker('navigation_sampling_heads', position=(1., 1., .52))
        sphere, _, _ = self.project(head, surface=surface, heads=heads)
        self.assertEqual(count, 2)
        endpoint = projected.points[-1]
        np.testing.assert_allclose([sphere.pose.position.x, sphere.pose.position.y, sphere.pose.position.z], [endpoint.x, endpoint.y, endpoint.z])
        self.assertAlmostEqual(endpoint.z, .125, delta=.02)
        self.assertEqual(line.points[-1].z, .52)

    def test_grounded_cylinder_upright_and_foot_on_measured_floor_after_tilt(self):
        surface = plane_surface(2.)
        matrix = visual.pose_matrix([0., 0., 2.], Rotation.from_euler('y', .1).as_quat())
        cylinder = self.marker('measured_dynamic', visual.Marker.CYLINDER, (1., 1., .65))
        cylinder.scale.x = cylinder.scale.y = .7; cylinder.scale.z = 1.3
        projected, count, _ = self.project(cylinder, matrix, surface)
        self.assertEqual(count, 1); self.assertAlmostEqual(projected.pose.position.z-projected.scale.z/2., 2.035)
        self.assertEqual(projected.pose.orientation.w, 1.); self.assertEqual(projected.pose.orientation.y, 0.)
        self.assertEqual(projected.scale.x, .7); self.assertEqual(cylinder.pose.position.z, .65)

    def test_guard_radius_preserved_and_arrow_near_same_measured_floor(self):
        surface = plane_surface(); guard = self.marker('dynamic_guard_radius', visual.Marker.LINE_STRIP)
        angles = np.linspace(0., 2*np.pi, 49)
        guard.points = [visual.Point(x=float(1.4*np.cos(a)), y=float(1.4*np.sin(a)), z=.12) for a in angles]
        projected, count, _ = self.project(guard, surface=surface)
        self.assertEqual(count, 49)
        self.assertAlmostEqual(max(np.hypot(p.x, p.y) for p in projected.points), 1.4)
        np.testing.assert_allclose([p.z for p in projected.points], .035)
        self.assertLess(projected.scale.x, .03)
        arrow = self.marker('dynamic_velocity_prediction', visual.Marker.ARROW)
        arrow.points = [visual.Point(x=1., y=1., z=.25), visual.Point(x=1.4, y=1., z=.25)]
        arrow_projected, count, _ = self.project(arrow, surface=surface)
        self.assertEqual(count, 2); np.testing.assert_allclose([p.z for p in arrow_projected.points], .05)

    def test_marker_pose_points_baked_once_then_identity_pose(self):
        # A genuine measured wireframe keeps its registered source geometry;
        # unsupported prediction arrows are intentionally hidden separately.
        marker = self.marker('measured_cluster_wireframe', visual.Marker.LINE_LIST, (1., 2., 3.))
        marker.pose.orientation.z, marker.pose.orientation.w = Rotation.from_euler('z', np.pi/2).as_quat()[2:]
        marker.points = [visual.Point(x=1., y=0., z=0.), visual.Point(x=2., y=0., z=0.)]
        result, count, _ = self.project(marker)
        self.assertEqual(count, 0)
        np.testing.assert_allclose([[p.x, p.y, p.z] for p in result.points], [[1., 3., 3.], [1., 4., 3.]], atol=1e-12)
        self.assertEqual(result.pose.position.z, 0.); self.assertEqual(result.pose.orientation.w, 1.)

    def test_unsupported_preview_is_deleted_and_cannot_supply_sampling_head(self):
        surface=plane_surface();heads={}
        line=self.marker('navigation_rollout_samples',visual.Marker.LINE_STRIP)
        line.points=[visual.Point(x=6.,y=6.,z=.52),visual.Point(x=6.2,y=6.,z=.52)]
        projected,count,total=self.project(line,surface=surface,heads=heads)
        self.assertEqual((count,total),(0,2));self.assertEqual(projected.action,visual.Marker.DELETE)
        self.assertFalse(projected.points);self.assertNotIn(line.id,heads)
        head=self.marker('navigation_sampling_heads',position=(1.,1.,.52))
        sphere,count,_=self.project(head,surface=surface,heads=heads)
        self.assertEqual(count,1);self.assertEqual(sphere.action,visual.Marker.DELETE)

    def test_preview_never_connects_across_real_ground_gap(self):
        surface=plane_surface();heads={}
        line=self.marker('navigation_rollout_samples',visual.Marker.LINE_STRIP)
        line.points=[visual.Point(x=.5,y=.5,z=.52),visual.Point(x=.7,y=.5,z=.52),
                     visual.Point(x=6.,y=6.,z=.52),visual.Point(x=1.,y=1.,z=.52),visual.Point(x=1.2,y=1.,z=.52)]
        projected,count,total=self.project(line,surface=surface,heads=heads)
        self.assertEqual((count,total),(4,5));self.assertEqual(projected.type,visual.Marker.LINE_LIST)
        xyz=np.array([[p.x,p.y,p.z] for p in projected.points]);self.assertEqual(len(xyz),4)
        np.testing.assert_allclose(xyz[:,:2],[[.5,.5],[.7,.5],[1.,1.],[1.2,1.]])
        np.testing.assert_allclose(xyz[:,2],.035);self.assertIn(line.id,heads)

    def test_supported_head_without_supported_terminal_edge_is_hidden(self):
        surface=plane_surface();heads={}
        line=self.marker('navigation_rollout_samples',visual.Marker.LINE_STRIP)
        line.points=[visual.Point(x=.5,y=.5,z=.52),visual.Point(x=.7,y=.5,z=.52),
                     visual.Point(x=6.,y=6.,z=.52),visual.Point(x=1.,y=1.,z=.52)]
        projected,count,_=self.project(line,surface=surface,heads=heads)
        self.assertEqual(count,3);self.assertEqual(len(projected.points),2);self.assertNotIn(line.id,heads)
        head=self.marker('navigation_sampling_heads',position=(1.,1.,.52))
        sphere,_,_=self.project(head,surface=surface,heads=heads);self.assertEqual(sphere.action,visual.Marker.DELETE)

    def test_unknown_ring_and_partly_supported_prediction_arrow_are_hidden(self):
        surface=plane_surface()
        ring=self.marker('dynamic_guard_radius',visual.Marker.LINE_STRIP)
        ring.points=[visual.Point(x=6.,y=6.,z=.12),visual.Point(x=6.2,y=6.,z=.12)]
        result,count,_=self.project(ring,surface=surface)
        self.assertEqual(count,0);self.assertEqual(result.action,visual.Marker.DELETE);self.assertFalse(result.points)
        arrow=self.marker('dynamic_velocity_prediction',visual.Marker.ARROW)
        arrow.points=[visual.Point(x=1.,y=1.,z=.25),visual.Point(x=6.,y=6.,z=.25)]
        result,count,total=self.project(arrow,surface=surface)
        self.assertEqual((count,total),(1,2));self.assertEqual(result.action,visual.Marker.DELETE)

    def test_unknown_dynamic_envelope_retains_registered_height_and_honest_label(self):
        cylinder=self.marker('measured_dynamic',visual.Marker.CYLINDER,(6.,6.,.65))
        cylinder.scale.x=cylinder.scale.y=.7;cylinder.scale.z=1.3
        result,count,_=self.project(cylinder,surface=plane_surface())
        self.assertEqual(count,0);self.assertEqual(result.action,visual.Marker.ADD)
        self.assertEqual(result.pose.position.z,.65);self.assertEqual(result.scale.z,1.3)
        label=self.marker('dynamic_labels',visual.Marker.TEXT_VIEW_FACING,(6.,6.,1.65));label.text='PERSON | YIELD'
        result,_,_=self.project(label,surface=plane_surface())
        self.assertEqual(result.text,'LiDAR candidate envelope #3 | YIELD')

    def test_preview_vertex_colours_follow_original_supported_edges(self):
        from std_msgs.msg import ColorRGBA
        line=self.marker('navigation_rollout_samples',visual.Marker.LINE_STRIP)
        line.points=[visual.Point(x=.5,y=.5,z=.52),visual.Point(x=.7,y=.5,z=.52),visual.Point(x=.9,y=.5,z=.52)]
        line.colors=[ColorRGBA(r=.1,g=0.,b=0.,a=1.),ColorRGBA(r=.2,g=0.,b=0.,a=1.),ColorRGBA(r=.3,g=0.,b=0.,a=1.)]
        result,_,_=self.project(line,surface=plane_surface())
        self.assertEqual(len(result.points),4);np.testing.assert_allclose([c.r for c in result.colors],[.1,.2,.2,.3])

    def test_delete_bypasses_invalid_pose_and_invalid_frame_add_is_rejected(self):
        marker = visual.Marker(); marker.action = visual.Marker.DELETEALL
        result, count, total = self.project(marker)
        self.assertEqual(result.action, visual.Marker.DELETEALL); self.assertEqual(result.header.frame_id, 'lio_map')
        self.assertEqual(count+total, 0)
        marker = self.marker('measured_dynamic'); marker.header.frame_id = 'unknown'
        with self.assertRaises(ValueError): self.project(marker)

    def test_delayed_delete_only_array_cannot_clear_newer_markers(self):
        published=[]
        state=SimpleNamespace(last_marker_source_stamp=10.,visual_markers=SimpleNamespace(publish=published.append))
        old = visual.Marker(); old.action = visual.Marker.DELETEALL; old.header.stamp.sec = 9
        visual.Display.markers(state, visual.MarkerArray(markers=[old])); self.assertFalse(published)
        old.header.stamp.sec = 11
        visual.Display.markers(state, visual.MarkerArray(markers=[old]))
        self.assertEqual(len(published), 1); self.assertEqual(state.last_marker_source_stamp, 11.)
        self.assertEqual(published[0].markers[0].header.frame_id, 'lio_map')


class ObservedRouteTests(unittest.TestCase):
    def test_original_adjacency_never_bridges_unobserved_gap(self):
        xyz=np.column_stack((np.arange(5.),np.zeros(5),np.full(5,.035)))
        supported=np.array([True,True,False,True,True])
        segments,audit=visual.observed_route_segments(xyz,supported)
        np.testing.assert_array_equal(segments,xyz[[0,1,3,4]])
        self.assertEqual(audit['observed_segments'],2)
        self.assertEqual(audit['excluded_unknown_segments'],2)
        self.assertEqual(audit['unknown_height_points'],1)
        self.assertTrue(audit['association_is_not_proof_of_robot_traversal'])
        np.testing.assert_array_equal(supported,[True,True,False,True,True])

    def test_independent_planner_legs_never_connect_even_if_both_supported(self):
        xyz=np.array([[0.,0.,.035],[.1,0.,.035],[10.,0.,.035],[10.1,0.,.035]])
        segments,audit=visual.observed_route_segments(xyz,np.ones(4,bool),[2,0,2])
        np.testing.assert_array_equal(segments,xyz)
        self.assertEqual(audit['observed_segments'],2)
        self.assertEqual(audit['within_leg_segments'],2)
        self.assertEqual(audit['excluded_cross_leg_connections'],1)

    def test_two_observed_floors_use_their_own_actual_historic_ground(self):
        history=visual.VisualPoseAnchors()
        lower=visual.pose_matrix([0.,0.,.4],[0.,0.,0.,1.])
        history.consider(10.,10.,lower,lower,plane_surface(),10.)
        upper_ref=visual.pose_matrix([4.,0.,3.6],[0.,0.,0.,1.])
        upper_lio=visual.pose_matrix([4.,0.,2.],[0.,0.,0.,1.])
        surface=plane_surface(1.6);surface.support_xyz[:,0]+=4.;surface.base_xyz[0]+=4.
        history.consider(11.,11.,upper_lio,upper_ref,surface,11.)
        values=np.array([[.8,0.,.09],[1.,0.,.09],[2.,0.,1.59],[4.,0.,3.29],[4.1,0.,3.29]])
        matrix=upper_lio@np.linalg.inv(upper_ref)
        xyz,audit,indices,supported=visual.spatially_registered_path(values,matrix,history,return_support=True)
        np.testing.assert_array_equal(supported,[True,True,False,True,True])
        segments,display=visual.observed_route_segments(xyz,supported)
        np.testing.assert_allclose(segments[:,2],[.035,.035,1.635,1.635],atol=1e-12)
        np.testing.assert_array_equal(segments,xyz[[0,1,3,4]])
        self.assertEqual(display['observed_segments'],2)
        self.assertEqual(audit['nominal_unvisited_points'],1)
        # Existing Path callers retain their three-value contract and all points.
        legacy=visual.spatially_registered_path(values,matrix,history)
        self.assertEqual(len(legacy),3);np.testing.assert_array_equal(legacy[0],xyz)

    def test_future_upper_floor_does_not_render_from_lower_floor_xy_support(self):
        history=visual.VisualPoseAnchors();lower=visual.pose_matrix([0.,0.,.4],[0.,0.,0.,1.])
        history.consider(10.,10.,lower,lower,plane_surface(),10.)
        values=[[.8,0.,.09],[1.,0.,.09],[.8,0.,3.29],[1.,0.,3.29]]
        xyz,audit,_,supported=visual.spatially_registered_path(values,np.eye(4),history,return_support=True)
        segments,display=visual.observed_route_segments(xyz,supported,[2,2])
        np.testing.assert_array_equal(supported,[True,True,False,False])
        self.assertEqual(len(segments),2);self.assertEqual(display['unknown_height_points'],2)
        self.assertEqual(audit['nominal_unvisited_points'],2)
        self.assertTrue(np.isfinite(segments).all())

    def test_pose_anchor_without_actual_surface_cannot_render_nominal_height(self):
        history=visual.VisualPoseAnchors();lower=visual.pose_matrix([0.,0.,.4],[0.,0.,0.,1.])
        history.consider(10.,10.,lower,lower)
        xyz,audit,_,supported=visual.spatially_registered_path([[.8,0.,.09],[1.,0.,.09]],np.eye(4),history,return_support=True)
        segments,display=visual.observed_route_segments(xyz,supported)
        self.assertEqual(len(segments),0);self.assertEqual(audit['historic_nominal_height_points'],2)
        self.assertEqual(display['measured_supported_points'],0)

    def test_empty_and_invalid_route_geometry_never_publish_nan_placeholders(self):
        segments,audit=visual.observed_route_segments([],[],[])
        self.assertEqual(segments.shape,(0,3));self.assertEqual(audit['observed_segments'],0)
        for xyz,support,lengths in [([[0.,0.,np.nan]],[True],[1]),([[0.,0.,0.]],[],[1]),([[0.,0.,0.]],[True],[2])]:
            with self.assertRaises(ValueError):visual.observed_route_segments(xyz,support,lengths)

    @unittest.skipIf(visual.rclpy is None,'ROS messages unavailable')
    def test_line_list_marker_preserves_xyz_colors_and_has_identity_pose(self):
        header=visual.NavPath().header;header.frame_id='lio_map';header.stamp.sec=10
        xyz=np.array([[0.,0.,.035],[.1,0.,.035],[3.,0.,1.635],[3.1,0.,1.635]])
        for index,name in enumerate(('NavFnDijkstra','SmacAStar')):
            marker=visual.mapped_route_marker(name,index,xyz,header)
            self.assertEqual(marker.type,visual.Marker.LINE_LIST);self.assertEqual(marker.id,index)
            self.assertEqual(marker.header.frame_id,'lio_map');self.assertEqual(marker.pose.orientation.w,1.)
            np.testing.assert_array_equal([[p.x,p.y,p.z] for p in marker.points],xyz)
            self.assertAlmostEqual(marker.scale.x,.035)
            empty=visual.mapped_route_marker(name,index,np.empty((0,3)),header)
            self.assertEqual(empty.ns,marker.ns);self.assertEqual(empty.id,marker.id);self.assertFalse(empty.points)

    @unittest.skipIf(visual.rclpy is None,'ROS messages unavailable')
    def test_publisher_keeps_full_legacy_paths_and_clears_expired_observed_lines(self):
        pose=visual.pose_matrix([0.,0.,.4],[0.,0.,0.,1.]);lio,reference=histories(pose,pose)
        history=visual.VisualPoseAnchors();history.consider(10.,10.,pose,pose,plane_surface(),10.)
        values=[[.8,0.,.09],[1.,0.,.09],[.8,0.,3.29],[1.,0.,3.29]]
        planners=('NavFnDijkstra','SmacAStar');legacy={name:[] for name in planners};full_visual={name:[] for name in planners};mapped=[]
        now=SimpleNamespace(nanoseconds=10100000000)
        def to_msg():
            header=visual.NavPath().header;header.stamp.sec=now.nanoseconds//1000000000;header.stamp.nanosec=now.nanoseconds%1000000000
            return header.stamp
        now.to_msg=to_msg
        state=SimpleNamespace(aligned=True,get_clock=lambda:SimpleNamespace(now=lambda:now),update_surface=lambda value:None,capture_anchor=lambda value:None,lio_history=lio,reference_history=reference,enrich_audit=lambda audit,matrix:audit,anchors=history,path_count=0,routes={name:[{'xyz':values[:2]},{'xyz':values[2:]}] for name in planners},pubs={name:SimpleNamespace(publish=legacy[name].append) for name in planners},visual_paths={name:SimpleNamespace(publish=full_visual[name].append) for name in planners},mapped_routes=SimpleNamespace(publish=mapped.append))
        visual.Display.tick(state)
        for name in planners:
            self.assertEqual(len(legacy[name][-1].poses),4)
            np.testing.assert_array_equal([[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in legacy[name][-1].poses],values)
            self.assertEqual(len(full_visual[name][-1].poses),4)
            self.assertEqual(state.last_mapped_status['planners'][name]['unknown_height_points'],2)
        self.assertEqual(len(mapped[-1].markers),2)
        for marker in mapped[-1].markers:
            self.assertEqual(len(marker.points),2)
            np.testing.assert_allclose([p.z for p in marker.points],.035)
        self.assertEqual(state.last_path_status['nominal_unvisited_points'],4)
        now.nanoseconds=11000000000
        visual.Display.tick(state)
        self.assertTrue(all(not marker.points for marker in mapped[-1].markers))
        self.assertTrue(all(not state.last_mapped_status['planners'][name]['registration_valid'] for name in planners))
        self.assertTrue(all(len(legacy[name][-1].poses)==4 for name in planners))


if __name__ == '__main__': unittest.main()
