import unittest,sys,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from go2w_lidar_mount import mount, body_points
class MountTest(unittest.TestCase):
 def test_cm_to_m_and_handedness(self):
  values=mount(dict(position=dict(x=13,y=2,z=18),rotation=dict(roll=180,pitch=-25,yaw=90)))
  for a,b in zip(values,[.13,-.02,.18,math.pi,math.radians(25),-math.pi/2]):self.assertAlmostEqual(a,b)

 def test_downward_sensor_returns_are_on_body_floor(self):
  import numpy as np
  from scipy.spatial.transform import Rotation
  sensor=dict(position=dict(x=13,y=2,z=18),rotation=dict(roll=0,pitch=-20,yaw=0))
  body=np.array([[1.,0.,-.4],[3.,.8,-.4],[-1.,-.8,-.4]])
  # Actual ground in body coordinates is transformed into the sensor frame.
  r=Rotation.from_euler('y',20,degrees=True).as_matrix()
  raw=(body-np.array([.13,-.02,.18]))@r
  np.testing.assert_allclose(body_points(raw,sensor),body,atol=1e-12)
  # Omitting the fixed rotation was the regression that made the floor
  # become a raised obstacle and corrupted LIO's height during travel.
  self.assertGreater(abs((raw+np.array([.13,-.02,.18]))[1,2]-.0),.3)
