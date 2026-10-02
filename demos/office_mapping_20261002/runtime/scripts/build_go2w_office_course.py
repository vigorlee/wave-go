#!/usr/bin/python3
"""Translate intact Yard terrain primitives into verified free OfficeWorld rooms."""
from pathlib import Path
import copy,json,xml.etree.ElementTree as ET,math
from build_nezha2_warehouse_navigation_assets import write_map
from sync_go2w_extended_ue_scene import ue_scene_root
ROOT=Path(__file__).resolve().parents[1];P=ROOT/'matrix/src/robot_mujoco/zsibot_robots/go2w';U=ROOT/'matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/go2w'
def build():
 r=ET.parse(P/'scene_terrain_office.xml').getroot();src=ET.parse(P/'scene_terrain_yard_extended_nav.xml').getroot();w=r.find('worldbody');r.set('model','Go2-W OfficeWorld continuous migrated course');r.find('statistic').set('center','6 -1 1');r.find('statistic').set('extent','20');r.find('./visual/global').set('azimuth','-100');r.find('./visual/global').set('elevation','-65')
 for mat in src.findall('./asset/material'):
  if mat.get('name','').startswith('extended_'):r.find('asset').append(copy.deepcopy(mat))
 added=[];traversable=[];transforms={}
 def add(name,dx,dy,walk=False):
  old=src.find(f"./worldbody/geom[@name='{name}']");assert old is not None,name;g=copy.deepcopy(old);p=list(map(float,g.get('pos').split()));p[0]+=dx;p[1]+=dy;g.set('pos',' '.join(map(str,p)));w.append(g);added.append(name);transforms[name]=[dx,dy,0]
  if walk:traversable.append(name)
 for i in range(1,11):add(f'box_OBS{i}',-3.5,-6.0,True);add(f'box2_OBS{i}',-3.5,-6.0,True)
 add('box2_OBS11',-3.5,-6.0,True)
 for n in ('Cylinder37','box_OBS11'):add(n,-3.95,-6.0)
 for i in range(1,5):add(f'extended_slalom_cylinder_{i}',7.2,-12.57);add(f'box_OBS{19+i}',7.2,-12.57)
 for n in ('extended_ramp_up','extended_ramp_crest','extended_ramp_down','box_OBS15','box_OBS16','box_OBS24'):add(n,17.2,-18.5,True)
 for n in ('Cylinder39','Cylinder41','box_OBS13','box_OBS14'):add(n,17.2,-18.5)
 physics=P/'scene_terrain_office_course.xml';ET.indent(r,space='  ');physics.write_text(ET.tostring(r,encoding='unicode')+'\n');(U/physics.name).write_text(ET.tostring(ue_scene_root(r),encoding='unicode')+'\n')
 actors={}
 for name in added:
  g=w.find(f"geom[@name='{name}']")
  if g.get('contype')=='0':continue
  pos=list(map(float,g.get('pos').split()));size=list(map(float,g.get('size').split()));angles=list(map(float,g.get('euler','0 0 0').split()))
  scale=[2*size[0],2*size[0],2*size[1]] if g.get('type')=='cylinder' else [2*v for v in size]
  actors[f'Element{len(actors)+1:02d}']=dict(name='office_visual_'+name,type='static',model='cylinder1' if g.get('type')=='cylinder' else 'cube1',position=dict(zip(('x','y','z'),[100*pos[0]-760,-100*pos[1]+2420,100*pos[2]])),rotation=dict(pitch=-math.degrees(angles[1]),yaw=math.degrees(angles[2]),roll=math.degrees(angles[0])),scale=dict(zip(('x','y','z'),scale)))
 (ROOT/'matrix/scene/scene_go2w_office_course.json').write_text(json.dumps(actors,indent=2)+'\n')
 nav=copy.deepcopy(r);nw=nav.find('worldbody')
 for g in list(nw.findall('geom')):
  p=list(map(float,g.get('pos','0 0 0').split()));s=list(map(float,g.get('size','0 0 0').split()))
  if g.get('name') in traversable or (g.get('type')=='box' and p[2]-s[2]>2.2):nw.remove(g)
 temp=ROOT/'config/office_course_map_geometry.xml';temp.write_text(ET.tostring(nav,encoding='unicode'));manifest=write_map(temp,ROOT/'config',map_stem='go2w_office_course');manifest.update(physics=str(physics),added_geoms=added,traversable_geoms=traversable,translations=transforms,source_scene=str(P/'scene_terrain_yard_extended_nav.xml'),stair_translation=[-3.5,-6.0],ramp_translation=[17.2,-18.5],dimensions_unchanged=True)
 avoid=copy.deepcopy(nav)
 for name in added:
  if name.startswith('box2_OBS') or (name.startswith('box_OBS') and name.removeprefix('box_OBS').isdigit() and int(name.removeprefix('box_OBS'))<=10):
   avoid.find('worldbody').append(copy.deepcopy(w.find(f"geom[@name='{name}']")))
 avoid_xml=ROOT/'config/office_course_avoid_geometry.xml';avoid_xml.write_text(ET.tostring(avoid,encoding='unicode'));write_map(avoid_xml,ROOT/'config',map_stem='go2w_office_avoid')
 (ROOT/'config/go2w_office_course_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
 stages=[]
 def goal(n,x,y,yaw=math.pi/2,profile='generic',mode='avoid'):
  stages.append(dict(stage_id=n,profile=profile,navigation_mode=mode,goal=dict(frame_id='map',x=x,y=y,yaw_rad=yaw)))
 goal('climb_to_upper_landing',-3.2,-1.2,profile='stair_up',mode='up');goal('descend_far_staircase',-3.4,4.2,profile='stair_down',mode='down')
 for n,x,y,a in [('stair_exit_turn',-1.25,3.65,-math.pi/2),('west_door_entry',-.95,1.0,0),('office_room_entry',1.,1.,0),('slalom_west_approach',5.3,1.,-math.pi/2),('slalom_entry_turn',5.3,-2.7,0),('slalom_entry',7.2,-2.7,math.pi/2),('slalom_pair_1',7.2,-1.5,math.pi/2),('slalom_pair_2',7.2,-.8,math.pi/2),('slalom_exit',7.2,.9,0),('office_corridor_turn',8.85,.9,-math.pi/2),('office_corridor_entry',8.85,-3.9,0),('office_corridor_center',12.,-4.4,0),('ramp_approach_outer',15.3,-5.95,0)]:goal(n,x,y,a)
 goal('ramp_approach_turn',17.2,-5.95,profile='slope_up',mode='slope')
 for n,x,y,pr in [('ramp_up_entry',17.2,-5.25,'slope_up'),('ramp_align_right',17.9,-4.7,'slope_up'),('ramp_up_crest',17.98,-3.1,'slope_up'),('ramp_down_entry',17.98,-2.35,'slope_down'),('ramp_down_exit',17.2,-.25,'slope_down')]:goal(n,x,y,profile=pr,mode='slope')
 goal('office_far_room_exit',17.2,1.0,0);goal('course_exit',19.,1.,0)
 route=dict(version=1,routes=[dict(route_id='officeworld_migrated_course',description='Intact Yard stair pair, cylinders and ramp pair translated into OfficeWorld rooms; one continuous Nav2 goal.',task_examples=['Continuous original stairs, slalom and ramp course'],stages=stages)])
 (ROOT/'config/go2w_office_continuous_route.json').write_text(json.dumps(route,indent=2)+'\n');print(json.dumps(dict(added=len(added),stages=len(stages),physics=str(physics))))
if __name__=='__main__':build()
