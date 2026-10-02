import unittest,sys,numpy as np
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from go2w_mid360_semantics import associate_labels
class Mid360SemanticsTests(unittest.TestCase):
 def test_unobserved_points_stay_unknown_and_geometry_unchanged(self):
  raw=np.array([[0.,0,0],[1,0,0],[2,0,0]]);original=raw.copy()
  labels=associate_labels(raw,np.array([[.01,0,0],[1.2,0,0]]),np.array([4,3],np.uint8))
  np.testing.assert_array_equal(labels,[4,0,0]);np.testing.assert_array_equal(raw,original)
 def test_empty_semantics(self):
  np.testing.assert_array_equal(associate_labels(np.zeros((3,3)),np.empty((0,3)),np.array([],np.uint8)),[0,0,0])
