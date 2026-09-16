import unittest,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from go2w_dense_depth import project_depth
class ProjectionTest(unittest.TestCase):
 def test_forward_plane_and_axes(self):
  p=project_depth(np.full((3,3),2.),90).reshape(3,3,3)
  np.testing.assert_allclose(p[:,:,0],2.)
  np.testing.assert_allclose(p[1,1],[2,0,0])
  self.assertGreater(p[0,0,1],0);self.assertGreater(p[0,0,2],0)
  self.assertLess(p[2,2,1],0);self.assertLess(p[2,2,2],0)
 def test_no_filled_missing_pixels(self):
  d=np.array([[np.nan,2.,0.]])
  p=project_depth(d,90)
  self.assertTrue(np.isnan(p[0]).all());self.assertEqual(p[2,0],0)
if __name__=='__main__':unittest.main()
