import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import numpy as np
from go2w_semantic_map_core import dbscan,SemanticVoxels,Tracks
class SemanticMapTests(unittest.TestCase):
 def test_density_separates_objects_and_noise(self):
  rng=np.random.default_rng(7);a=rng.normal(0,.03,(30,3));b=rng.normal(2,.03,(30,3));lab=dbscan(np.vstack([a,b,[[8,8,8]]]),.2,5)
  self.assertEqual(len(set(lab[:60])),2);self.assertEqual(lab[-1],-1)
 def test_repeated_same_stamp_not_confirmation(self):
  m=SemanticVoxels();xyz=np.array([[1.,2.,3.]]);lab=np.array([1]);m.update(xyz,lab,1);m.update(xyz,lab,1);m.update(xyz,lab,1)
  self.assertEqual(m.assign(xyz,1)[0][0],0)
  m.update(xyz,lab,2);m.update(xyz,lab,3);self.assertEqual(m.assign(xyz,3)[0][0],1)
 def test_people_expire_and_never_become_static(self):
  m=SemanticVoxels();xyz=np.array([[1.,2.,3.]]);lab=np.array([4]);m.update(xyz,lab,1)
  self.assertEqual(m.assign(xyz,1.1)[0][0],4);self.assertEqual(m.assign(xyz,2)[0][0],0);self.assertEqual(m.assign(xyz,1.1,False)[0][0],0)
 def test_nearby_unobserved_points_stay_unknown(self):
  m=SemanticVoxels();p=np.array([[0.,0.,0.]])
  for t in (1,2,3):m.update(p,np.array([5]),t)
  self.assertEqual(m.assign(np.array([[1.,0.,0.]]),3)[0][0],0)
 def test_tracks_confirm_then_expire(self):
  t=Tracks();o=dict(center=[1,2,3],class_id=4)
  self.assertFalse(t.update([o],0));self.assertFalse(t.update([o],.5));self.assertEqual(t.update([o],1)[0]['id'],1)
  t.update([],3);self.assertFalse(t.items)
if __name__=='__main__':unittest.main()
