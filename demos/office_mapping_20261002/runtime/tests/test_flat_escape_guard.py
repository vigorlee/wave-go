"""Recovery must pass freshness, bounded motion and the bridge's stop latch."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from geometry_msgs.msg import Twist
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts/global_compare'))
from follow_route import PathRunner


class EscapeGuardTests(unittest.TestCase):
    def runner(self):
        x,y=np.meshgrid(np.linspace(-6.5,-3.,20),np.linspace(1.3,3.8,20))
        self.events=[]
        return SimpleNamespace(pose=dict(x=-5.74,y=3.57,z=.4,roll=0.,pitch=0.,yaw=-.70),i=7,
            yielding=True,escaping=False,yield_started=5.,escape_origin=None,escape_started=None,
            escape_exhausted=False,clear_since=None,lidar_wait_started=None,lidar_received=10.,
            last_progress=0.,minimum_candidate_distance=float('inf'),pose_stamp=100.,observed_stamp=100.,
            native_entry=SimpleNamespace(phase='pre_heading'),
            observed_world=np.column_stack((x.ravel(),y.ravel(),np.zeros(x.size))),
            clusters=[dict(xy=[-4.60,3.16],velocity=[0.,0.],radius=.3)],
            event=lambda name,**data:self.events.append((name,data)),finish=lambda *args:None)

    def test_checked_reverse_is_not_blocked_by_original_yield_latch(self):
        r=self.runner();cmd=PathRunner.guard(r,Twist(),10.)
        self.assertLess(cmd.linear.x,0.);self.assertTrue(r.escaping)
        self.assertTrue(r.yielding)  # Original forward/waypoint remains blocked.
        self.assertFalse(r.yielding and not r.escaping)  # Bridge can execute recovery.
        self.assertEqual(self.events[0][0],'local_escape_started')

    def test_stale_lidar_never_releases_motion(self):
        r=self.runner();r.lidar_received=8.
        cmd=PathRunner.guard(r,Twist(),10.)
        self.assertEqual(cmd.linear.x,0.);self.assertFalse(r.escaping)

    def test_unsynchronized_cloud_or_near_riser_phase_cannot_recover(self):
        for mismatch in ('stamp','phase','budget'):
            r=self.runner()
            if mismatch=='stamp':r.observed_stamp=99.
            elif mismatch=='phase':r.native_entry.phase='approach'
            else:r.escape_origin=np.array([-6.1,3.57]);r.escape_started=9.
            cmd=PathRunner.guard(r,Twist(),10.)
            self.assertEqual(cmd.linear.x,0.);self.assertFalse(r.escaping)

    def test_policy_moving_opposite_to_checked_retreat_aborts_recovery(self):
        r=self.runner();r.escape_origin=np.array([-5.80,3.57]);r.escape_started=8.
        r.escape_direction=np.array([-1.,0.])
        cmd=PathRunner.guard(r,Twist(),10.)
        self.assertEqual(cmd.linear.x,0.);self.assertFalse(r.escaping)
        self.assertTrue(r.escape_exhausted)
        self.assertEqual(self.events[0][0],'local_escape_bounded_stop')


if __name__=='__main__':unittest.main()
