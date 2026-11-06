#!/usr/bin/python3
"""Run one Nav2 climb action and verify measured height plus stable landing."""
import argparse,csv,json,math,time,subprocess,os
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
ap=argparse.ArgumentParser();ap.add_argument('--x',type=float,default=.65);ap.add_argument('--y',type=float,default=4.8);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=True)
rclpy.init();n=rclpy.create_node('go2w_climb_verified');rows=[]
def odom(m):
 p=m.pose.pose.position;q=m.pose.pose.orientation
 rows.append([time.time(),p.x,p.y,p.z,math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y)),math.asin(max(-1,min(1,2*(q.w*q.y-q.z*q.x))))])
n.create_subscription(Odometry,'/odom/mujoco_odom',odom,30)
c=ActionClient(n,NavigateThroughPoses,'/navigate_through_poses')
if not c.wait_for_server(timeout_sec=20):raise RuntimeError('Nav2 unavailable')
deadline=time.monotonic()+10
while not rows and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.1)
if not rows:raise RuntimeError('No fresh odometry before climb')
env=dict(os.environ,GO2W_NAV_MODE_FILE=str(Path('.run/go2w/nav_mode').resolve()))
subprocess.run(['/usr/bin/python3','scripts/set_go2w_nav_mode.py','up'],env=env,check=True)
goal=NavigateThroughPoses.Goal()
for y in [min(args.y-1,3.8),args.y]:
 p=PoseStamped();p.header.frame_id='map';p.header.stamp=n.get_clock().now().to_msg();p.pose.position.x=args.x;p.pose.position.y=y;p.pose.orientation.z=math.sin(math.pi/4);p.pose.orientation.w=math.cos(math.pi/4);goal.poses.append(p)
f=c.send_goal_async(goal);rclpy.spin_until_future_complete(n,f,timeout_sec=10);h=f.result()
if not h or not h.accepted:raise RuntimeError('climb goal rejected')
f=h.get_result_async();deadline=time.monotonic()+90;reason=''
while not f.done() and time.monotonic()<deadline:
 rclpy.spin_once(n,timeout_sec=.1)
 if rows and time.time()-rows[-1][0]>2:reason='odometry_lost';break
 if rows and max(abs(rows[-1][4]),abs(rows[-1][5]))>1:reason='excessive_tilt';break
status=f.result().status if f.done() else None
if not f.done():
 reason=reason or 'timeout';f=h.cancel_goal_async();rclpy.spin_until_future_complete(n,f,timeout_sec=5)
env=dict(os.environ,GO2W_NAV_MODE_FILE=str(Path('.run/go2w/nav_mode').resolve()))
subprocess.run(['/usr/bin/python3','scripts/set_go2w_nav_mode.py','avoid'],env=env,check=True)
end=time.monotonic()+3
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
with (args.output/'climb_odom.csv').open('w') as out:
 w=csv.writer(out);w.writerow(['wall_time','x_m','y_m','z_m','roll_rad','pitch_rad']);w.writerows(rows)
last=rows[-1] if rows else None;passed=bool(status==4 and last and last[3]>1.8 and abs(last[2]-args.y)<.5 and max(abs(last[4]),abs(last[5]))<.25)
summary=dict(passed=passed,action_status=status,reason=reason,samples=len(rows),start=rows[0] if rows else None,end=last,max_z=max((r[3] for r in rows),default=0),goal=[args.x,args.y],scope='Ascending original ten-step course and stopping on 1.625 m landing; ramp and obstacle traversal not tested.')
(args.output/'climb_result.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary));n.destroy_node();rclpy.shutdown()
