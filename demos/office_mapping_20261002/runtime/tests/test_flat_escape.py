import math
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts/global_compare'))
from flat_escape import choose_reverse_escape


class ReverseEscapeTests(unittest.TestCase):
    def setUp(self):
        self.pose=dict(x=-5.74,y=3.58,z=.4,roll=0.,pitch=0.,yaw=-.75)
        self.person=dict(xy=[-4.63,3.18],velocity=[0.,0.],radius=.30)
        x,y=np.meshgrid(np.linspace(-6.5,-3.,20),np.linspace(1.3,3.8,20))
        self.floor=np.column_stack((x.ravel(),y.ravel(),np.zeros(x.size)))

    def choose(self,**kwargs):
        return choose_reverse_escape(kwargs.get('pose',self.pose),kwargs.get('clusters',[self.person]),
                                     kwargs.get('geoms',[]),kwargs.get('observed',self.floor))

    def test_stationary_candidate_recovery_is_reverse_and_increases_separation(self):
        action=self.choose();self.assertIsNotNone(action)
        self.assertLess(action['linear_x'],0.)
        self.assertGreaterEqual(action['separation_gain_m'],.08)
        self.assertGreaterEqual(action['predicted_minimum_distance_m'],1.05)

    def test_close_person_cannot_be_bypassed(self):
        c=dict(self.person,xy=[-5.05,3.45])
        self.assertIsNone(self.choose(clusters=[c]))

    def test_closing_person_prediction_prevents_escape(self):
        c=dict(self.person,velocity=[-1.,0.])
        self.assertIsNone(self.choose(clusters=[c]))

    def test_rear_obstacle_blocks_all_reverse_samples(self):
        wall=('rear', 'box',np.array([-6.24,3.7,.5]),np.array([.10,.5,.5]),np.eye(3))
        self.assertIsNone(self.choose(geoms=[wall]))

    def test_observed_rear_return_blocks_even_when_not_in_static_map(self):
        seen=np.vstack((self.floor,[[-6.24,3.7,.5]]))
        self.assertIsNone(self.choose(observed=seen))

    def test_unknown_observation_and_nonflat_pose_stop(self):
        self.assertIsNone(self.choose(observed=[]))
        self.assertIsNone(self.choose(pose=dict(self.pose,z=1.4)))
        self.assertIsNone(self.choose(pose=dict(self.pose,roll=.4)))

    def test_reverse_cannot_enter_unverified_floor_or_riser(self):
        self.assertIsNone(self.choose(pose=dict(self.pose,y=1.10,yaw=math.pi/2)))

    def test_second_candidate_on_rear_blocks_recovery(self):
        c=dict(self.person,xy=[-6.6,3.6])
        self.assertIsNone(self.choose(clusters=[self.person,c]))


if __name__=='__main__':unittest.main()
