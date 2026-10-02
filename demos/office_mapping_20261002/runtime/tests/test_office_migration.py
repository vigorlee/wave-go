from pathlib import Path
import json,xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
ROOT=Path(__file__).resolve().parents[1]
def test_terrain_dimensions_and_contact_properties_preserved():
 m=json.loads((ROOT/'config/go2w_office_course_manifest.json').read_text());source=ET.parse(m['source_scene']);dest=ET.parse(m['physics'])
 for name,delta in m['translations'].items():
  a=dict(source.find(f"./worldbody/geom[@name='{name}']").attrib);b=dict(dest.find(f"./worldbody/geom[@name='{name}']").attrib)
  assert np.allclose(np.fromstring(a.pop('pos'),sep=' ')+delta,np.fromstring(b.pop('pos'),sep=' '));assert a==b
 assert len(m['translations'])==41
 assert len([n for n in m['translations'] if n.startswith('box2_OBS')])==11

def test_native_office_collision_geometry_is_unchanged():
 old=ET.parse(ROOT/'matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_office.xml');new=ET.parse(ROOT/'matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_office_course.xml')
 for geom in old.findall('./worldbody/geom'):
  assert geom.attrib==new.find(f"./worldbody/geom[@name='{geom.get('name')}']").attrib

def test_waypoints_have_ground_map_clearance():
 m=json.loads((ROOT/'config/go2w_office_course_manifest.json').read_text());a=np.asarray(Image.open(ROOT/'config/go2w_office_course.pgm'));d=distance_transform_edt(a>200)*m['resolution_m'];route=json.loads((ROOT/'config/go2w_office_continuous_route.json').read_text())['routes'][0]
 for stage in route['stages']:
  g=stage['goal'];x=int((g['x']-m['origin_m'][0])/.05);y=a.shape[0]-1-int((g['y']-m['origin_m'][1])/.05)
  assert d[y,x]>=.30,(stage['stage_id'],float(d[y,x]))
 assert route['stages'][0]['profile']=='stair_up';assert route['stages'][1]['profile']=='stair_down'

def test_ue_actors_match_physics_in_right_handed_coordinates():
 actors=json.loads((ROOT/'matrix/scene/scene_go2w_office_course.json').read_text());m=json.loads((ROOT/'config/go2w_office_course_manifest.json').read_text());scene=ET.parse(m['physics'])
 assert len(actors)==31
 for actor in actors.values():
  name=actor['name'].removeprefix('office_visual_');g=scene.find(f"./worldbody/geom[@name='{name}']");p=actor['position'];assert np.allclose([(p['x']+760)/100,-(p['y']-2420)/100,p['z']/100],np.fromstring(g.get('pos'),sep=' '))
  size=np.fromstring(g.get('size'),sep=' ');s=actor['scale'];expected=[2*size[0],2*size[0],2*size[1]] if g.get('type')=='cylinder' else size*2
  assert np.allclose([s['x'],s['y'],s['z']],expected)
  assert np.isclose(np.deg2rad(actor['rotation']['roll']),float(g.get('euler','0 0 0').split()[0]))

def test_ground_map_blocks_stair_side_shortcuts_but_keeps_corridor():
 m=json.loads((ROOT/'config/go2w_office_course_manifest.json').read_text());a=np.asarray(Image.open(ROOT/'config/go2w_office_course.pgm'));b=np.asarray(Image.open(ROOT/'config/go2w_office_avoid.pgm'))
 assert a.shape==b.shape
 def at(im,x,y):return im[im.shape[0]-1-int((y-m['origin_m'][1])/.05),int((x-m['origin_m'][0])/.05)]
 assert at(a,-2.35,1.0)>200 and at(b,-2.35,1.0)==0
 for y in [3.5,3.,2.,1.]:assert at(b,-.95,y)>200

def test_route_profiles_are_accepted_by_runtime():
 import sys
 sys.path.insert(0,str(ROOT/"scripts"))
 from cosmos_vln_protocol import load_route_catalog
 assert load_route_catalog(ROOT/"config/go2w_office_continuous_route.json")
