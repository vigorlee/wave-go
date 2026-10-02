"""Read-only ground-floor view of the running simulator's unmodified UDP state."""
import socket,struct,time,threading
from pathlib import Path
import numpy as np
import mujoco,mujoco.viewer
ROOT=Path(__file__).resolve().parents[1]
m=mujoco.MjModel.from_xml_path(str(ROOT/'matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_office_inline.xml'));d=mujoco.MjData(m)
for i in range(m.ngeom):
 name=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,i) or ''
 if name.startswith('phs_'):
  rot=np.empty(9);mujoco.mju_quat2Mat(rot,m.geom_quat[i]);vertical_extent=float(np.abs(rot.reshape(3,3)[2])@m.geom_size[i])
  if m.geom_pos[i,2]-vertical_extent>2.2 and name not in ('phs_Cube91','phs_Cube92'):m.geom_group[i]=5
  elif name in ('phs_Cube91','phs_Cube92'):m.geom_rgba[i]=[.5,.65,.8,.14]
  else:m.geom_rgba[i]=[.50,.59,.68,.65]
visual_rgba=m.geom_rgba.copy();visual_groups=m.geom_group.copy()
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.bind(('127.0.0.1',25011));s.settimeout(.1);latest=None

def receive():
 global latest
 while True:
  try:
   b,_=s.recvfrom(65535);nq=struct.unpack_from('<i',b,8)[0];nv=struct.unpack_from('<i',b,12+8*nq)[0]
   if nq==m.nq and nv==m.nv:latest=(np.frombuffer(b,dtype='<f8',count=nq,offset=12).copy(),np.frombuffer(b,dtype='<f8',count=nv,offset=16+8*nq).copy())
  except socket.timeout:pass
threading.Thread(target=receive,daemon=True).start()
with mujoco.viewer.launch_passive(m,d,show_left_ui=False,show_right_ui=False) as v:
 v.sync();time.sleep(.5)
 v.cam.lookat[:]=[-.5,-1.6,1.8];v.cam.distance=23;v.cam.azimuth=120;v.cam.elevation=-45;v.opt.geomgroup[5]=0
 while v.is_running():
  if latest is not None:
   with v.lock():
    d.qpos[:]=latest[0];d.qvel[:]=latest[1];mujoco.mj_forward(m,d)
  v.sync()
  m.geom_rgba[:]=visual_rgba;m.geom_group[:]=visual_groups
  v.cam.lookat[:]=[-.5,-1.6,1.8];v.cam.distance=23;v.cam.azimuth=120;v.cam.elevation=-45;v.opt.geomgroup[5]=0
  time.sleep(.025)
