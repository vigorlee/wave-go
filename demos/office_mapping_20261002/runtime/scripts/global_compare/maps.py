"""Layered traversability for the existing Office terrain (metres, map frame).

The level-transition checkpoints are task constraints. Nav2 searches each
walkable surface; a 2D occupancy grid cannot represent overlapping floors.
"""
from pathlib import Path
import math, sys, json, xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
R=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(R/'scripts'))
from go2w_office_inline_run import STAGES
RES=.05; ORIGIN=(-8.,-8.); WIDTH=360; HEIGHT=240
X,Y=np.meshgrid(ORIGIN[0]+(np.arange(WIDTH)+.5)*RES,ORIGIN[1]+(np.arange(HEIGHT)+.5)*RES)
GEOMS=[]
for g in ET.parse(R/'matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_office_inline.xml').findall('./worldbody/geom'):
 if g.get('type') not in ('box','cylinder'):continue
 p=np.fromstring(g.get('pos'),sep=' ');s=np.fromstring(g.get('size'),sep=' ')
 q=np.fromstring(g.get('quat','1 0 0 0'),sep=' ')
 rot=Rotation.from_euler('xyz',np.fromstring(g.get('euler'),sep=' ')).as_matrix() if g.get('euler') else Rotation.from_quat(q[[1,2,3,0]]).as_matrix()
 GEOMS.append((g.get('name',''),g.get('type'),p,s,rot))
def rect(x0,x1,y0,y1):return (X>=x0)&(X<=x1)&(Y>=y0)&(Y<=y1)
def footprint(t,p,s,rot):
 if t=='cylinder':return (X-p[0])**2+(Y-p[1])**2<=s[0]**2
 u=rot[0,0]*(X-p[0])+rot[1,0]*(Y-p[1]);v=rot[0,1]*(X-p[0])+rot[1,1]*(Y-p[1])
 return (abs(u)<=s[0])&(abs(v)<=s[1])
def layer(index):
 a=np.zeros((HEIGHT,WIDTH),np.int8)
 if index in (0,1,2):free=rect(2.4,8.8,-.45,1.85)
 elif index in (3,4):free=rect(-3.35,3.4,-.45,1.85)
 elif index==8:free=rect(-6.19,-4.94,-6.25,2.08)
 elif index==9:free=rect(-6.24,-3.44,-6.74,-5.04)
 elif index==10:free=rect(-4.74,-3.48,-6.6,.25)
 elif index==11:free=rect(-4.75,-1.5,-.82,1.85)
 else:free=rect(-6.25,8.8,-2.,2.1)
 a[~free]=100
 for name,t,p,s,rot in GEOMS:
  if name.startswith('inline_cylinder') and index<=7:a[footprint(t,p,s,rot)]=100
  if index in (5,6,7):
   top=p[2]+float(abs(rot[2])@s) if t=='box' else p[2]+s[1]
   bottom=p[2]-float(abs(rot[2])@s) if t=='box' else p[2]-s[1]
   if (name.startswith('phs_') or name.startswith('native_transition')) and bottom<1.5 and top>.12:
    a[footprint(t,p,s,rot)]=100
  if index==11 and name.startswith('phs_'):
   extent=float(abs(rot[2])@s)
   if p[2]-extent<6.5 and p[2]+extent>5.7:a[footprint(t,p,s,rot)]=100
 return a
def surface_z(x,y,index):
 cap=2. if index<=2 else (1. if index<=7 else (3.22 if index in (8,9) else 5.54));best=0.
 for name,t,p,s,rot in GEOMS:
  if t!='box':continue
  # Office floor slabs can have local Y as their vertical axis (90-degree
  # rotation). Intersect the upward face, not always the box's local Z face.
  axis=int(np.argmax(np.abs(rot[2])))
  if abs(rot[2,axis])<.7:continue
  z=p[2]+s[axis]/abs(rot[2,axis])-(rot[0,axis]*(x-p[0])+rot[1,axis]*(y-p[1]))/rot[2,axis]
  if not -.01<=z<=cap:continue
  local=rot.T@(np.array([x,y,z])-p)
  if np.all(np.abs(local)<=s+.01):best=max(best,float(z))
 return best
if __name__=='__main__':
 from PIL import Image
 out=R/'config/global_compare/maps';out.mkdir(exist_ok=True)
 for i,s in enumerate(STAGES):
  a=layer(i);np.save(out/f'{i:02d}_{s["id"]}.npy',a);Image.fromarray(np.where(a[::-1]>0,0,255).astype('uint8')).save(out/f'{i:02d}_{s["id"]}.png')
 (out/'contract.json').write_text(json.dumps(dict(resolution=RES,origin=ORIGIN,width=WIDTH,height=HEIGHT,source='Existing MJCF geometry and explicit walkable terrain surfaces; not online SLAM',stages=STAGES),indent=2))
