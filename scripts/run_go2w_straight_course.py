#!/usr/bin/python3
"""Execute ordered course stages through Nav2; record actual pose and height."""
import argparse,csv,json,math,time,subprocess,os
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Odometry,Path as NavPath
from geometry_msgs.msg import PoseStamped
ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--start-stage',default='stairs_up');args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=True)
rclpy.init();n=rclpy.create_node('go2w_straight_course');rows=[];stages=[];stage='ready';plan_count=0
c=ActionClient(n,NavigateThroughPoses,'/navigate_through_poses')
def odom(m):
 p=m.pose.pose.position;q=m.pose.pose.orientation
 rows.append([time.time(),stage,p.x,p.y,p.z,math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y)),math.asin(max(-1,min(1,2*(q.w*q.y-q.z*q.x))))])
def plan(m):
 global plan_count
 plan_count+=1
n.create_subscription(Odometry,'/odom/mujoco_odom',odom,30);n.create_subscription(NavPath,'/plan',plan,10)
if not c.wait_for_server(timeout_sec=30):raise RuntimeError('Nav2 unavailable')
env=dict(os.environ,GO2W_NAV_MODE_FILE=str(Path('.run/go2w/nav_mode').resolve()))
def mode(m):subprocess.run(['/usr/bin/python3','scripts/set_go2w_nav_mode.py',m],env=env,check=True,stdout=subprocess.DEVNULL)
def save():
 with (args.output/'odom.csv').open('w') as out:
  w=csv.writer(out);w.writerow(['wall_time','stage','x_m','y_m','z_m','roll_rad','pitch_rad']);w.writerows(rows)
 (args.output/'result.json').write_text(json.dumps(dict(passed=len(stages)==5 and all(s['passed'] for s in stages),start_stage=args.start_stage,executed_stages_passed=bool(stages) and all(s['passed'] for s in stages),stages=stages,plan_messages=plan_count,last_pose=rows[-1] if rows else None,scope='Original course ascent/descent/ramp. Native pedestrian avoidance not included.'),indent=2)+'\n')
started=False
try:
 for stage,m,targets,minz,maxz in [('stairs_up','up',[(.65,3.8),(.65,4.8)],1.8,2.3),('stairs_down','down',[(.65,6.0),(.65,9.8),(.65,10.2)],.25,.65),('flat_obstacles','avoid',[(0.,11.2),(0.,12.5)],.25,.7),('ramp_up_down','slope',[(.70,13.35),(.78,15.4),(.78,16.3),(.92,18.8)],.25,.65),('course_exit','avoid',[(.75,19.0)],.25,.7)]:
  if stage==args.start_stage:started=True
  if not started:continue
  mode(m);start=len(rows);g=NavigateThroughPoses.Goal()
  for x,y in targets:
   p=PoseStamped();p.header.frame_id='map';p.header.stamp=n.get_clock().now().to_msg();p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.z=math.sin(math.pi/4);p.pose.orientation.w=math.cos(math.pi/4);g.poses.append(p)
  f=c.send_goal_async(g);rclpy.spin_until_future_complete(n,f,timeout_sec=10);h=f.result()
  if not h or not h.accepted:raise RuntimeError('goal rejected: '+stage)
  f=h.get_result_async();end=time.monotonic()+120;reason=''
  while not f.done() and time.monotonic()<end:
   rclpy.spin_once(n,timeout_sec=.1)
   if rows and time.time()-rows[-1][0]>2:reason='odometry_lost';break
   if rows and max(abs(rows[-1][5]),abs(rows[-1][6]))>1:reason='excessive_tilt';break
  status=f.result().status if f.done() else None
  if not f.done():
   reason=reason or 'timeout';f=h.cancel_goal_async();rclpy.spin_until_future_complete(n,f,timeout_sec=5)
  sample=rows[start:];last=rows[-1] if rows else None
  passed=bool(status==4 and last and minz<last[4]<maxz)
  stages.append(dict(stage=stage,passed=passed,status=status,reason=reason,end=last,max_z=max((v[4] for v in sample),default=0),min_z=min((v[4] for v in sample),default=0),samples=len(sample)))
  save();print(json.dumps(stages[-1]),flush=True)
  if not passed:break
finally:
 mode('avoid');end=time.monotonic()+3
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
 save();n.destroy_node();rclpy.shutdown()
