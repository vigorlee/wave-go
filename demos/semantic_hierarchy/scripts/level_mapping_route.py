"""Continuous measured-odometry 3D route follower; commands the existing RL policy.
No pose teleportation, no joint playback, and no claim of learned route planning.
"""
import argparse,csv,json,math,time,subprocess,os
from pathlib import Path
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry,Path as NavPath
from geometry_msgs.msg import Twist,PoseStamped
from scipy.spatial.transform import Rotation
R=Path('/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916')
def stage(n,x,y,z,mode='avoid',frame=None):return dict(id=n,x=x,y=y,z=z,mode=mode,frame=frame)
STAGES=[stage('stair_alignment',8.,.0,.40),stage('large_stair_crest',5.13,.0,1.48,'up',(7.4,.0,math.pi,1.95,1.34)),stage('large_stair_exit',2.9,-.05,.40,'down'),stage('ramp_crest',0,-.05,.83,'slope'),stage('ramp_exit',-2.85,-.05,.40,'slope'),stage('pedestrian_crossing',-4.0,.7,.40),stage('native_entry_clearance',-3.8,1.65,.40),stage('native_stair_approach',-5.57,1.65,.40),stage('native_lower_landing',-5.57,-5.75,3.60,'up',(-5.57,1.4,-math.pi/2,6.4,3.4)),stage('native_half_landing_turn',-4.12,-6.20,3.60),stage('native_upper_exit',-4.12,-.35,5.90,'up',(-4.12,-5.9,math.pi/2,5.4,5.7)),stage('second_floor_goal',-2.7,.65,5.90)]
class Runner(Node):
 def __init__(self,out,native,start_stage=None):
  super().__init__('office_inline_follower');
  if (R/'.run/go2w/manual_control').exists():raise RuntimeError('Manual control owns the robot; use the explicit automatic launcher to transfer ownership.')
  self.out=out;out.mkdir(parents=True,exist_ok=True);self.stages=STAGES[start_stage:] if start_stage is not None else (STAGES[8:] if native else STAGES);self.i=-1;self.pose=None;self.received=0;self.state='ready';self.start=time.time();self.last_progress=time.monotonic();self.best=1e9;self.trail=NavPath();self.trail.header.frame_id='map';self.pub=self.create_publisher(Twist,'/cmd_vel_nav',10);self.pathpub=self.create_publisher(NavPath,'/go2w/course/trail',10);self.refpub=self.create_publisher(NavPath,'/go2w/course/route',10);self.create_subscription(Odometry,'/odom/mujoco_odom',self.odom,20);self.timer=self.create_timer(.05,self.tick);self.events=[];self.rows=[];self.lastwrite=0
 def atomic(self,path,text):
  p=R/path;p.parent.mkdir(parents=True,exist_ok=True);tmp=p.with_suffix('.tmp');tmp.write_text(text);tmp.replace(p)
 def event(self,state,**kw):
  e=dict(time=time.time(),state=state,stage_index=self.i,stage_id=self.stages[self.i]['id'] if self.i>=0 else '',completed_stages=(self.i+1 if state in ('stage_completed','succeeded') else max(0,self.i)),robot_pose=self.pose,**kw);self.events.append(e)
  with (self.out/'events.jsonl').open('a') as f:f.write(json.dumps(e)+'\n')
  (self.out/'result.json').write_text(json.dumps(dict(passed=state=='succeeded',state=state,completed_stages=len(self.stages) if state=='succeeded' else max(0,self.i),total_stages=len(self.stages),last_event=e,controller='continuous 3D waypoint tracking with measured MuJoCo odometry and RL locomotion; not Nav2 or Cosmos inference'),indent=2));print(json.dumps(e),flush=True)
 def odom(self,m):
  p=m.pose.pose.position;q=m.pose.pose.orientation;roll,pitch,yaw=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz');self.pose=dict(x=p.x,y=p.y,z=p.z,roll=roll,pitch=pitch,yaw=yaw);self.received=time.monotonic()
  if self.state=='running':self.rows.append(dict(time=time.time(),**self.pose));self.trail.poses.append(PoseStamped(header=m.header,pose=m.pose.pose))
 def next(self):
  self.i+=1
  if self.i>=len(self.stages):self.i=len(self.stages)-1;self.finish('succeeded');return
  s=self.stages[self.i];f=s['frame'] or (0,0,0,1,10);self.atomic('.run/go2w/stair_frame',s['id']+' '+' '.join(map(str,f))+'\n')
  self.pub.publish(Twist());subprocess.run(['/usr/bin/python3',str(R/'scripts/set_go2w_nav_mode.py'),s['mode']],timeout=15,check=True,stdout=subprocess.DEVNULL,env=dict(os.environ,STAIR_NAV_MAX_VELOCITY='[0.8,0.0,0.75]',STAIR_NAV_MIN_VELOCITY='[-0.2,0.0,-0.75]',SLOPE_NAV_MAX_VELOCITY='[0.65,0.0,0.55]'));self.atomic('.run/go2w/nav_mode',s['mode']+'\n');self.state='running';self.best=1e9;self.last_progress=time.monotonic();self.event('stage_starting');self.stage_start=dict(self.pose)
 def finish(self,status,reason=''):
  self.state=status;self.atomic('.run/go2w/nav_mode','avoid\n');self.atomic('.run/go2w/stair_frame','stopped 0 0 0 1 10\n');self.pub.publish(Twist());self.event(status,reason=reason)
  if self.rows:
   with (self.out/'odom.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=self.rows[0]);w.writeheader();w.writerows(self.rows)
 def tick(self):
  if self.state in ('succeeded','failed'):self.pub.publish(Twist());return
  now=time.monotonic()
  if self.pose is None:return
  if now-self.received>1:self.finish('failed','odom stale');return
  if self.i<0:self.next();return
  p=self.pose;s=self.stages[self.i];dx=s['x']-p['x'];dy=s['y']-p['y'];dist=math.hypot(dx,dy);err=math.atan2(math.sin(math.atan2(dy,dx)-p['yaw']),math.cos(math.atan2(dy,dx)-p['yaw']))
  if abs(p['roll'])>.8 or abs(p['pitch'])>.95 or p['z']<.15:self.finish('failed','attitude or height safety');return
  if s['id']=='stair_alignment' and dist<.18:
   yawerr=math.atan2(math.sin(math.pi-p['yaw']),math.cos(math.pi-p['yaw']))
   if abs(yawerr)>.08:
    cmd=Twist();cmd.angular.z=max(-.5,min(.5,1.5*yawerr));self.pub.publish(cmd);self.last_progress=now;return
  if dist<(.18 if s['id']=='stair_alignment' else (.38 if s['id']=='large_stair_crest' else .25)) and abs(p['z']-s['z'])<.24:
   self.event('stage_completed');self.next();return
  metric=dist+abs(s['z']-p['z'])*.3
  if metric<self.best-.07:self.best=metric;self.last_progress=now
  if now-self.last_progress>30:self.finish('failed','no progress');return
  if time.time()-self.start>480:self.finish('failed','mission timeout');return
  cmd=Twist();cmd.angular.z=max(-.70,min(.70,1.5*err));speed=.65 if s['mode']=='up' else (.3 if s['mode']=='down' else (.6 if s['mode']=='slope' else .4));cmd.linear.x=min(speed,max(.13,dist*.7))*max(0,math.cos(err)) if abs(err)<.6 else 0.;self.pub.publish(cmd)
  if now-self.lastwrite>.5:
   self.lastwrite=now;self.trail.header.stamp=self.get_clock().now().to_msg();self.pathpub.publish(self.trail)
   ref=NavPath();ref.header=self.trail.header
   for t in self.stages:
    a=PoseStamped();a.header=ref.header;a.pose.position.x=float(t['x']);a.pose.position.y=float(t['y']);a.pose.position.z=float(t['z']);a.pose.orientation.w=1.;ref.poses.append(a)
   self.refpub.publish(ref)
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('--output',type=Path,required=True);a.add_argument('--native-only',action='store_true');a.add_argument('--start-stage',type=int);args=a.parse_args();rclpy.init();n=Runner(args.output,args.native_only,args.start_stage)
 try:rclpy.spin(n)
 finally:
  n.atomic('.run/go2w/nav_mode','avoid\n');n.atomic('.run/go2w/stair_frame','stopped 0 0 0 1 10\n');n.pub.publish(Twist());n.destroy_node();rclpy.shutdown()
