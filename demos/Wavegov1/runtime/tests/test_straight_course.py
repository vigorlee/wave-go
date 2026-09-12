import unittest,xml.etree.ElementTree as ET
from pathlib import Path
R=Path(__file__).resolve().parents[1]
class OriginalCourse(unittest.TestCase):
 def test_no_added_terrain(self):
  a=ET.parse(R/'matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_yard_extended_nav.xml').getroot()
  b=ET.parse(R/'matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_yard_mid360_course.xml').getroot()
  self.assertEqual([g.attrib for g in a.findall('./worldbody/geom')],[g.attrib for g in b.findall('./worldbody/geom')])
  self.assertFalse(b.findall(".//joint[@type='slide']"))
