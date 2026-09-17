#!/usr/bin/env python3
"""Run only mapping/display processes; supervise and clean up owned children."""
import argparse,os,signal,socket,subprocess,time
from pathlib import Path
root=Path(__file__).resolve().parent.parent
ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['fastlio','grid']);ap.add_argument('--bag',default=os.environ.get('WAVEGO_BAG'));ap.add_argument('--port',type=int);ap.add_argument('--rate',type=float,default=.7);ap.add_argument('--no-rviz',action='store_true');args=ap.parse_args()
port=args.port or (11321 if args.mode=='fastlio' else 11322)
if args.mode=='fastlio' and (not args.bag or not Path(args.bag).is_file()):ap.error('Use --bag /absolute/path/to/recording.bag or set WAVEGO_BAG')
if args.rate<=0:ap.error('--rate must be positive')
with socket.socket() as s:
 if s.connect_ex(('127.0.0.1',port))==0:ap.error(f'Port {port} already in use; stop its demo or select --port {port+1000}')
env=os.environ.copy();env.update(ROS_MASTER_URI=f'http://127.0.0.1:{port}',ROS_IP='127.0.0.1',OMP_NUM_THREADS='4')
logs=root/'.run'/args.mode;logs.mkdir(parents=True,exist_ok=True);children=[];handles=[]
def start(cmd,name):
 f=open(logs/(name+'.log'),'w');handles.append(f);p=subprocess.Popen(cmd,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);children.append((p,name));return p
def stop(signum,frame):raise KeyboardInterrupt
signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
try:
 start(['roscore','-p',str(port)],'master')
 for _ in range(60):
  result=subprocess.run(['rosparam','list'],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
  if result.returncode==0:break
  time.sleep(.25)
 else:raise RuntimeError('ROS master did not start')
 if args.mode=='fastlio':
  subprocess.run(['rosparam','set','use_sim_time','true'],env=env,check=True)
  subprocess.run(['rosparam','load',str(root/'config/fastlio_office.yaml')],env=env,check=True)
  start(['python',str(root/'scripts/office_cloud_adapter.py')],'adapter')
  start(['rosrun','fast_lio','fastlio_mapping'],'fastlio')
  if not args.no_rviz:start(['rviz','-d',str(root/'config/fastlio.rviz')],'rviz')
  time.sleep(3)
  start(['rosbag','play','--clock','--keep-alive','--rate',str(args.rate),str(Path(args.bag).resolve()),'--topics','/livox/lidar_raw','/imu/data_raw'],'bag')
 else:
  start(['rosrun','tf2_ros','static_transform_publisher','0','0','0','0','0','0','world','pcd_origin'],'tf')
  start(['python',str(root/'scripts/publish_adaptive_voxels.py'),'_map_file:='+str(logs/'adaptive_voxels.npz')],'voxels')
  if not args.no_rviz:start(['rviz','-d',str(root/'config/adaptive_grid.rviz')],'rviz')
 print(f'{args.mode} started; ROS_MASTER_URI={env["ROS_MASTER_URI"]}; logs={logs}',flush=True)
 while True:
  for p,name in children:
   if p.poll() is not None:raise RuntimeError(f'{name} exited with {p.returncode}; see {logs}')
  time.sleep(.5)
except KeyboardInterrupt:pass
finally:
 for p,_ in reversed(children):
  if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
 for p,_ in reversed(children):
  try:p.wait(timeout=8)
  except subprocess.TimeoutExpired:
   os.killpg(p.pid,signal.SIGKILL);p.wait()
 for f in handles:f.close()
