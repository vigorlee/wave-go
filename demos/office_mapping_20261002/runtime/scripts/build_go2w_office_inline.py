"""A single ground-floor obstacle axis leading into the native second-floor stairs."""
from pathlib import Path
import copy,json,math,xml.etree.ElementTree as E
from build_nezha2_warehouse_navigation_assets import write_map
R=Path(__file__).resolve().parents[1];P=R/'matrix/src/robot_mujoco/zsibot_robots/go2w';U=R/'matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/go2w'
def build():
 r=E.parse(P/'scene_terrain_office.xml').getroot();w=r.find('worldbody');r.set('model','Office inline course to native second floor');r.find('statistic').set('center','0 -1 2');r.find('statistic').set('extent','15');added=[]
 def box(n,p,s,c,e=(0,0,0)):
  g=E.SubElement(w,'geom',dict(name=n,type='box',pos=' '.join(map(str,p)),size=' '.join(map(str,s)),euler=' '.join(map(str,e)),rgba=c,friction='0.98 0.02 0.01'));added.append(n);return g
 # Larger inline course, centered on the same y=0.7 axis.
 for i in range(9):
  h=.12*(i+1);box('inline_stair_up_'+str(i),(7.11-.18*i,.7,h/2),(.09,1.15,h/2),'0.95 0.26 0.16 1')
 box('inline_stair_top',(5.13,.7,.54),(.45,1.15,.54),'0.95 0.26 0.16 1')
 for i in range(9):
  h=.12*(9-i);box('inline_stair_down_'+str(i),(4.59-.18*i,.7,h/2),(.09,1.15,h/2),'0.95 0.26 0.16 1')
 angle=math.radians(10);L=2.2;hh=.025;z=L/2*math.sin(angle)+hh*math.cos(angle)
 box('inline_ramp_up',(1.5,.7,z),(1.1,1.15,hh),'0.12 0.65 0.32 1',(0,angle,0))
 top=2*z;box('inline_ramp_top',(0.,.7,top/2),(.417,1.15,top/2),'0.12 0.65 0.32 1')
 box('inline_ramp_down',(-1.5,.7,z),(1.1,1.15,hh),'0.12 0.65 0.32 1',(0,-angle,0))
 for i,(x,y,bottom) in enumerate([(5.13,.7,1.08),(1.5,.7,z+hh/math.cos(angle)),(-1.5,.7,z+hh/math.cos(angle))]):
  n='inline_cylinder_'+str(i);E.SubElement(w,'geom',dict(name=n,type='cylinder',pos=f'{x} {y} {bottom+.35}',size='.15 .35',rgba='1 .6 .05 1',friction='.95 .02 .01'));added.append(n)
 # Smaller treads form a gentler overlay over the native lower flight.
 # Native floor/walls remain untouched; every added tread is visible in UE.
 for i in range(34):
  h=3.215*(i+1)/34
  box('native_transition_'+str(i),(-5.57,1.05-.17*i-.085,h/2),(.645,.085,h/2),'0.72 0.65 0.50 1')
 box('native_transition_landing',(-5.57,-5.39,3.215/2),(.645,.66,3.215/2),'0.72 0.65 0.50 1')
 out=P/'scene_terrain_office_inline.xml';E.indent(r,space='  ');out.write_text(E.tostring(r,encoding='unicode'));(U/out.name).write_text(E.tostring(r,encoding='unicode'))
 actors={}
 for n in added:
  g=w.find(f"geom[@name='{n}']");p=list(map(float,g.get('pos').split()));s=list(map(float,g.get('size').split()));e=list(map(float,g.get('euler','0 0 0').split()));sc=[2*s[0],2*s[0],2*s[1]] if g.get('type')=='cylinder' else [2*v for v in s]
  actors[f'Element{len(actors)+1:02d}']=dict(name='office_visual_'+n,type='static',model='cylinder1' if g.get('type')=='cylinder' else 'cube1',position=dict(zip(('x','y','z'),[p[0]*100-760,-p[1]*100+2420,p[2]*100])),rotation=dict(pitch=-math.degrees(e[1]),yaw=math.degrees(e[2]),roll=math.degrees(e[0])),scale=dict(zip(('x','y','z'),sc)))
 # Native walking-human actors cross the ground-level approach after the ramp.
 for i,(x,ya,yb) in enumerate([(-3.4,-1.7,1.7)]):
  def ue(x,y,z=0):return dict(x=x*100-760,y=-y*100+2420,z=z*100)
  actors[f'Element{len(actors)+1:02d}']=dict(name=f'office_walker_{i}',type='dynamic',model='human1',avoid=False,position=ue(x,ya,.9),rotation=dict(pitch=0,yaw=90,roll=0),scale=dict(x=1,y=1,z=1),velocity=.45,trajectory=dict(point1=ue(x,ya),point2=ue(x,yb)))
 (R/'matrix/scene/scene_go2w_office_inline.json').write_text(json.dumps(actors,indent=2))
 manifest=dict(physics=str(out),added_geoms=added,obstacle_axis_y=.7,stair_height=1.08,ramp_height=top,terrain_width=2.3,native_pedestrian_count=1,native_second_floor_height=5.51,original_scene=str(P/'scene_terrain_office.xml'))
 (R/'config/go2w_office_inline_manifest.json').write_text(json.dumps(manifest,indent=2));print(json.dumps(manifest))
if __name__=='__main__':build()
