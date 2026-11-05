"""Local measured floors must not turn several floors into one flat overlay."""
import sys
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts/global_compare'))
from navigation_surface import GroundSurface


def floor_cloud(z=0., slope=(0., 0.)):
    x, y = np.meshgrid(np.linspace(-2, 2, 51), np.linspace(-2, 2, 51))
    return np.column_stack((x.ravel(), y.ravel(), (z + slope[0]*x + slope[1]*y).ravel()))


class GroundSurfaceTest(unittest.TestCase):
    def test_flat_floor_and_small_render_offset(self):
        surface = GroundSurface.from_scan(floor_cloud(), [0, 0, .4])
        self.assertTrue(surface.valid)
        points, supported = surface.project([[.7, .3, .2], [1., 1., .4]])
        self.assertTrue(supported.all())
        np.testing.assert_allclose(points[:, 2], .035, atol=1e-6)

    def test_ramp_follows_measured_height(self):
        surface = GroundSurface.from_scan(floor_cloud(slope=(.15, -.06)), [0, 0, .4])
        self.assertTrue(surface.valid)
        z, supported = surface.height_at([[1., 1.], [-1., -1.]])
        self.assertTrue(supported.all())
        np.testing.assert_allclose(z, [.09, -.09], atol=.025)

    def test_upper_floor_does_not_project_to_floor_below(self):
        cloud = np.vstack((floor_cloud(), floor_cloud(3.2)))
        surface = GroundSurface.from_scan(cloud, [0, 0, 3.6])
        self.assertTrue(surface.valid)
        self.assertAlmostEqual(surface.floor_z, 3.2, places=4)
        points, supported = surface.project([[1., 1., .1], [1., 1., 3.3]])
        self.assertFalse(supported[0])
        self.assertTrue(supported[1])
        self.assertEqual(points[0, 2], .1)

    def test_missing_floor_is_not_body_height_measurement(self):
        surface = GroundSurface.from_scan(floor_cloud(2.5), [0, 0, .4])
        self.assertFalse(surface.valid)
        _, supported = surface.height_at([[0., 0.]])
        self.assertFalse(supported.any())

    def test_no_extrapolation_across_unknown_space(self):
        surface = GroundSurface.from_scan(floor_cloud(), [0, 0, .4])
        points, supported = surface.project([[8., 8., .2]])
        self.assertFalse(supported[0])
        self.assertEqual(points[0, 2], .2)

    def test_shared_navigation_and_inflation_ground_is_identical(self):
        surface = GroundSurface.from_scan(floor_cloud(slope=(.12, -.03)), [0, 0, .4])
        shared = GroundSurface.from_payload(surface.payload(123.4))
        xy = [[1., .4], [-.8, 1.], [10., 10.]]
        for a, b in zip(surface.height_at(xy), shared.height_at(xy)):
            np.testing.assert_array_equal(a, b)
        with self.assertRaises(ValueError):
            GroundSurface.from_payload(dict(surface.payload(123.4), frame='map'))


if __name__ == '__main__':
    unittest.main()
