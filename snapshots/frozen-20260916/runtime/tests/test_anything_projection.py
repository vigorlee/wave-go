import unittest,sys,copy
from pathlib import Path
import numpy as np
from geometry_msgs.msg import Pose
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from go2w_anything_live import lift_masks
class AlignmentTest(unittest.TestCase):
 def setUp(self):
  s={'position':dict(x=0,y=0,z=0),'rotation':dict(roll=0,pitch=0,yaw=0),'fov':90}
  self.cfg={'robot':{'sensors':{'camera':copy.deepcopy(s),'depth_sensor':copy.deepcopy(s)}}};self.pose=Pose();self.pose.orientation.w=1.0
 def test_metric_distance_and_label_identity(self):
  p,l=lift_masks(np.full((3,3),2.),np.arange(9,dtype=np.uint8).reshape(3,3),self.cfg,self.pose)
  np.testing.assert_allclose(p[:,0],2);np.testing.assert_array_equal(l,np.arange(9))
 def test_unaligned_mount_suppresses_projection(self):
  self.cfg['robot']['sensors']['camera']['position']['x']=29
  with self.assertRaises(ValueError):lift_masks(np.full((3,3),2.),np.ones((3,3),np.uint8),self.cfg,self.pose)
 def test_invalid_depth_does_not_create_points(self):
  p,l=lift_masks(np.array([[np.nan,0.,2.]]),np.array([[1,2,3]],np.uint8),self.cfg,self.pose)
  self.assertEqual(len(p),1);self.assertEqual(l[0],3)
if __name__=='__main__':unittest.main()
