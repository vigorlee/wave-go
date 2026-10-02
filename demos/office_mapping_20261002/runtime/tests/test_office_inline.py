from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
R=Path(__file__).resolve().parents[1]
def test_native_building_and_second_floor_are_preserved():
 a=E.parse(R/'matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_office.xml');b=E.parse(R/'matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_office_inline.xml')
 for g in a.findall('./worldbody/geom'):assert g.attrib==b.find(f"./worldbody/geom[@name='{g.get('name')}']").attrib
 assert b.find("./worldbody/geom[@name='phs_Cube144']") is not None
 assert b.find("./worldbody/geom[@name='phs_Cube92']") is not None

def test_inline_terrain_axis_size_and_contacts():
 a=E.parse(R/'matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_office_inline.xml');g=[x for x in a.findall('./worldbody/geom') if x.get('name','').startswith('inline_')];assert len(g)==25
 for x in g:
  if x.get('type')=='box':
   p=np.fromstring(x.get('pos'),sep=' ');s=np.fromstring(x.get('size'),sep=' ');assert p[1]==.7;assert s[1]==1.15;assert p[2]+s[2]<=1.080001
  assert x.get('contype')!='0'
 assert not a.findall(".//joint[@type='slide']")
