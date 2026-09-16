import unittest,sys,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from go2w_lidar_mount import mount
class MountTest(unittest.TestCase):
 def test_cm_to_m_and_handedness(self):
  values=mount(dict(position=dict(x=13,y=2,z=18),rotation=dict(roll=180,pitch=-25,yaw=90)))
  for a,b in zip(values,[.13,-.02,.18,math.pi,math.radians(25),-math.pi/2]):self.assertAlmostEqual(a,b)
