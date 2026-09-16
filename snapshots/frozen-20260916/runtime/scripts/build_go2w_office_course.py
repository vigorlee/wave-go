#!/usr/bin/python3
"""Migrate the validated Yard staircase/ramp primitives into OfficeWorld."""
from pathlib import Path
import copy,json,xml.etree.ElementTree as ET
from build_nezha2_warehouse_navigation_assets import write_map
ROOT=Path(__file__).resolve().parents[1]
P=ROOT/'matrix/src/robot_mujoco/zsibot_robots/go2w'
U=ROOT/'matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/go2w'
def build():
 r=ET.parse(P/'scene_terrain_office.xml').getroot();src=ET.parse(P/'scene_terrain_yard_extended_nav.xml').getroot();w=r.find('worldbody');a=r.find('asset')
 r.set('model','Go2-W OfficeWorld stairs ramp obstacles');r.find('statistic').set('center','3 -1 1');r.find('statistic').set('extent','10')
 for mat in src.findall('./asset/material'):
  if mat.get('name','').startswith('extended_'):a.append(copy.deepcopy(mat))
 added=[];traversable=[]
 def add(name,dx,dy,newname=None):
  g=copy.deepcopy(src.find(f"./worldbody/geom[@name='{name}']"));pos=list(map(float,g.get('pos').split()));pos[0]+=dx;pos[1]+=dy;g.set('pos',' '.join(map(str,pos)))
  if newname:g.set('name',newname)
  w.append(g);added.append(g.get('name'));return g
 for i in range(1,11):
  g=add(f'box_OBS{i}',3,-3);traversable.append(g.get('name'))
 # Retain all ten 15 cm risers; shorten only the old 3.5 m upper landing.
 g=add('box2_OBS11',3,-3,'box_OBS28');g.set('pos','3 1.1 1.55');g.set('size','1.5 .85 .075');traversable.append(g.get('name'))
 for name in ['extended_ramp_up','extended_ramp_crest','extended_ramp_down','box_OBS15','box_OBS16','box_OBS24']:
  g=add(name,17.2,-18.5);traversable.append(g.get('name'))
 for old,new,pos in [('Cylinder37','Cylinder37','2.15 .65 1.925'),('Cylinder39','Cylinder39','17.15 -4.35 .555610'),('Cylinder41','Cylinder41','17.25 -1.45 .520350')]:
  g=add(old,0,0,new);g.set('pos',pos)
  skin=copy.deepcopy(g);skin.set('name',{'Cylinder37':'box_OBS11','Cylinder39':'box_OBS13','Cylinder41':'box_OBS14'}[old]);skin.set('contype','0');skin.set('conaffinity','0');w.append(skin);added.append(skin.get('name'))
 for i,(x,y) in enumerate([(6.5,-2),(8.2,-1.1),(6.5,.2),(8.2,1.15)],1):
  g=add(f'extended_slalom_cylinder_{i}',0,0);g.set('pos',f'{x} {y} .34');skin=copy.deepcopy(g);skin.set('name',f'box_OBS{19+i}');skin.set('contype','0');skin.set('conaffinity','0');w.append(skin);added.append(skin.get('name'))
 physics=P/'scene_terrain_office_course.xml';ET.indent(r,space='  ');physics.write_text(ET.tostring(r,encoding='unicode')+'\n')
 ue=copy.deepcopy(r)
 for name in ['box_OBS15','box_OBS16']:
  g=ue.find(f"./worldbody/geom[@name='{name}']");ang=list(map(float,g.get('euler').split()));g.set('euler',f'{-ang[0]} 0 0')
 (U/physics.name).write_text(ET.tostring(ue,encoding='unicode')+'\n')
 # Ground-level planning map: the migrated contact surfaces are traversable,
 # while walls/columns and obstacle cylinders remain occupied.
 nav=copy.deepcopy(r);nw=nav.find('worldbody')
 for g in list(nw.findall('geom')):
  pos=list(map(float,g.get('pos','0 0 0').split()));size=list(map(float,g.get('size','0 0 0').split()))
  if g.get('name') in traversable or (g.get('type')=='box' and pos[2]-size[2]>2.2):nw.remove(g)
 temp=ROOT/'config/office_course_map_geometry.xml';temp.write_text(ET.tostring(nav,encoding='unicode'))
 manifest=write_map(temp,ROOT/'config',map_stem='go2w_office_course');manifest.update(physics=str(physics),added_geoms=added,traversable_geoms=traversable,route={'start':[3.2,-3.1,90],'upper_landing':[3.2,1.1,1.625]},limitations='Ground-level course map, not a multi-floor navigation map. Original Office staircase remains unchanged.')
 (ROOT/'config/go2w_office_course_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print(json.dumps({'added':len(added),'physics':str(physics),'map':manifest['map_yaml']}))
if __name__=='__main__':build()
