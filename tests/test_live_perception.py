import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from go2w_live_perception import GeometryTracker
class LivePerceptionTests(unittest.TestCase):
    def floor(self):
        x,y=np.meshgrid(np.linspace(-2,2,45),np.linspace(-2,2,45));return np.c_[x.ravel(),y.ravel(),np.zeros(x.size)]
    def test_flat_floor_not_moving_or_stair(self):
        p,l,o,s=GeometryTracker().process(self.floor(),np.array([0,0,.4]),1.)
        self.assertGreater((l==1).mean(),.9);self.assertEqual(len(o),0)
    def test_slope_without_scene_information(self):
        q=self.floor();q[:,2]=q[:,1]*np.tan(np.deg2rad(10))+.4
        p,l,o,s=GeometryTracker().process(q,np.array([0,0,.8]),1.)
        self.assertGreater((l==3).mean(),.85)
    def test_temporal_motion_and_static_rejection(self):
        angle,z=np.meshgrid(np.linspace(0,2*np.pi,40),np.linspace(.2,1.6,24));c=np.c_[.22*np.cos(angle.ravel()),.22*np.sin(angle.ravel()),z.ravel()]
        for speed in [0.,.22]:
            tracker=GeometryTracker();last=[]
            for i in range(12):
                cloud=np.r_[self.floor(),c+np.array([.8+speed*i*.2,.8,0])]
                _,_,last,_=tracker.process(cloud,np.array([0,0,.4]),1+i*.2)
            self.assertTrue(last)
            self.assertEqual(any(o['label']=='moving cluster' for o in last),speed>0)
if __name__=='__main__':unittest.main()
