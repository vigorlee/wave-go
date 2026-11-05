"""Read-only floor-aware view of the simulator's unmodified UDP state.

Only the local mirror's display groups, colors and camera change. Robot poses
are copied from UDP without edits; no controls are sent to the simulator.
"""
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
  if m.geom_pos[i,2]-vertical_extent>2.2 and name not in ('phs_Cube91','phs_Cube92'):
   m.geom_group[i]=5;m.geom_rgba[i]=[.80,.18,.16,.78]
  elif name in ('phs_Cube91','phs_Cube92'):m.geom_rgba[i]=[.5,.65,.8,.14]
  else:m.geom_rgba[i]=[.80,.18,.16,.78]
for i in range(m.ngeom):
 name=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,i) or ''
 if name.startswith(('inline_stair_', 'inline_ramp_')):m.geom_rgba[i]=[.95,.16,.12,1.]
visual_rgba=m.geom_rgba.copy();visual_groups=m.geom_group.copy()
upper_slabs=[i for i in range(m.ngeom) if mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,i) in ('phs_Cube91','phs_Cube92')]
GROUND_LOOKAT=np.array([-.5,-1.6,1.8])

def camera_settings(robot_position,upper_floor):
 """Return display-only settings, with height hysteresis to prevent flicker."""
 position=np.asarray(robot_position,dtype=float) if robot_position is not None else None
 if position is not None and position.shape==(3,) and np.isfinite(position).all():
  if position[2]>4.8:upper_floor=True
  elif position[2]<4.6:upper_floor=False
 else:position=None
 if upper_floor and position is not None:
  # Follow the second-floor corridor all the way to its distant door. Centering
  # on the measured robot also keeps it visible during subsequent manual driving.
  return np.array([position[0],position[1],max(5.5,position[2]-.35)]),18.,True
 return GROUND_LOOKAT.copy(),23.,False

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
 upper_floor=False;lookat=GROUND_LOOKAT.copy()
 v.cam.lookat[:]=lookat;v.cam.distance=23;v.cam.azimuth=120;v.cam.elevation=-45;v.opt.geomgroup[5]=0
 while v.is_running():
  state=latest
  target,distance,upper_floor=camera_settings(state[0][:3] if state is not None else None,upper_floor)
  lookat+=.12*(target-lookat)
  with v.lock():
   if state is not None:
    d.qpos[:]=state[0];d.qvel[:]=state[1];mujoco.mj_forward(m,d)
   m.geom_rgba[:]=visual_rgba;m.geom_group[:]=visual_groups
   if upper_floor:
    for i in upper_slabs:m.geom_rgba[i]=[.50,.59,.68,1.]
    m.geom_rgba[visual_groups==5,3]=1.
   v.cam.lookat[:]=lookat;v.cam.distance=distance;v.cam.azimuth=120;v.cam.elevation=-45;v.opt.geomgroup[5]=int(upper_floor)
  v.sync()
  time.sleep(.025)
