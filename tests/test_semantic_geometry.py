import sys,unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from go2w_semantic_geometry import SceneSemantics,rotation
class SemanticGeometryTests(unittest.TestCase):
    def setUp(self):self.s=SceneSemantics(ROOT/'matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_yard_extended_nav.xml')
    def test_known_surface_labels_and_unknown(self):
        p=np.array([[.7,1.0,.125],[-.02,3.15,1.8],[30.,30.,4.],[30.,30.,0.]])
        self.assertEqual(self.s.label(p).tolist(),[1,3,0,5])
    def test_tilted_ramp_surface(self):
        o=next(o for o in self.s.objects if o['name']=='extended_ramp_up')
        p=np.array([[.7,-.7,o['size'][2]]])@rotation(o['euler']).T+o['position']
        self.assertEqual(self.s.label(p).tolist(),[2])
    def test_dynamic_is_scene_association(self):
        self.assertEqual(self.s.label(np.array([[.22,20.8,1.]]),[{'x':0.,'y':20.8}]).tolist(),[4])
    def test_noncolliding_twins_excluded(self):
        names=[o['name'] for o in self.s.objects]
        self.assertNotIn('box_OBS15',names);self.assertIn('box_OBS5',names)
if __name__=='__main__':unittest.main()
