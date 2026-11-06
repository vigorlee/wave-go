"""The measured sensor mount must be applied once, before LIO registration."""
import ast
import json
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from go2w_lidar_mount import mount


def rotation(rpy):
    roll, pitch, yaw = rpy
    cr, cp, cy = np.cos(rpy)
    sr, sp, sy = np.sin(rpy)
    rx = np.array([[1., 0., 0.], [0., cr, -sr], [0., sr, cr]])
    ry = np.array([[cp, 0., sp], [0., 1., 0.], [-sp, 0., cp]])
    rz = np.array([[cy, -sy, 0.], [sy, cy, 0.], [0., 0., 1.]])
    return rz @ ry @ rx


class OfficeLioExtrinsicTest(unittest.TestCase):
    def setUp(self):
        sensor = json.loads((ROOT / 'matrix/src/UeSim/Linux/zsibot_mujoco_ue/'
                                  'Content/model/config/config.json').read_text())['robot']['sensors']['lidar']
        self.mount = np.asarray(mount(sensor))
        self.lio = yaml.safe_load((ROOT / 'config/office_lio_dense.yaml').read_text())['laser_mapping']['ros__parameters']['mapping']
        self.r = np.asarray(self.lio['extrinsic_R']).reshape(3, 3)
        self.t = np.asarray(self.lio['extrinsic_T'])

    def test_lidar_to_imu_rotation_matches_active_sensor_mount(self):
        np.testing.assert_allclose(self.r, rotation(self.mount[3:]), atol=1e-12)
        np.testing.assert_allclose(self.r @ self.r.T, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(self.r), 1., places=12)
        self.assertFalse(self.lio['extrinsic_est_en'])

    def test_translation_preserves_existing_imu_lever_arm(self):
        imu_in_body = np.array([-.02557, 0., .04232])
        np.testing.assert_allclose(self.t, self.mount[:3] - imu_in_body, atol=1e-12)

    def test_tilted_sensor_reconstructs_floor_once_with_body_attitude(self):
        # Construct independent world-plane returns, then express them in the
        # actual sensor frame. The C++ LIO equation must recover that plane.
        x, y = np.meshgrid(np.linspace(.5, 4., 30), np.linspace(-1., 1., 15))
        world = np.column_stack([x.ravel(), y.ravel(), np.zeros(x.size)])
        body_rotation = rotation([.04, -.025, .31])
        body_position = np.array([.2, -.1, .4])
        sensor_rotation = rotation(self.mount[3:])
        raw = ((world - body_position) @ body_rotation - self.mount[:3]) @ sensor_rotation
        imu_position = body_position + body_rotation @ np.array([-.02557, 0., .04232])

        correct = (raw @ self.r.T + self.t) @ body_rotation.T + imu_position
        omitted = (raw + self.t) @ body_rotation.T + imu_position
        reversed_rotation = (raw @ self.r + self.t) @ body_rotation.T + imu_position
        applied_twice = (raw @ self.r.T @ self.r.T + self.t) @ body_rotation.T + imu_position
        np.testing.assert_allclose(correct, world, atol=1e-12)
        for wrong in (omitted, reversed_rotation, applied_twice):
            self.assertGreater(float(np.percentile(np.abs(wrong[:, 2]), 90)), .6)

    def test_raw_relay_filters_without_rotating_sensor_coordinates(self):
        # Execute the real callback without starting a ROS node or publisher.
        source = ast.parse((ROOT / 'scripts/office_live/lidar_relay.py').read_text())
        relay = next(n for n in source.body if isinstance(n, ast.ClassDef) and n.name == 'Relay')
        callback = next(n for n in relay.body if isinstance(n, ast.FunctionDef) and n.name == 'scan')
        code = ast.fix_missing_locations(ast.Module(body=[callback], type_ignores=[]))
        scope = {'np': np, 'pc': SimpleNamespace(read_points=lambda m: m.source)}
        exec(compile(code, 'lidar_relay.scan', 'exec'), scope)
        raw = np.array([(.1, 0., 0., 1.), (1., .2, -.3, 2.),
                        (2., -.7, .4, 3.), (31., 0., 0., 4.)],
                       dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4')])
        msg = SimpleNamespace(source=raw, point_step=raw.dtype.itemsize,
                              header=SimpleNamespace(frame_id='lidar'))
        published = []
        scope['scan'](SimpleNamespace(pub=SimpleNamespace(publish=published.append)), msg)
        self.assertIs(published[0], msg)
        self.assertEqual(msg.header.frame_id, 'lidar')
        self.assertEqual(msg.width, 2)
        self.assertEqual(msg.data, raw[1:3].tobytes())


if __name__ == '__main__':
    unittest.main()
