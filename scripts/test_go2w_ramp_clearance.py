#!/usr/bin/python3
"""Isolated slope traversal from the explicitly configured ramp starting pose."""
import json,time,math,csv,argparse
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=True)
rclpy.init();n=rclpy.create_node('ramp_clearance_probe');rows=[]
def odom(m):
 p=m.pose.pose.position;q=m.pose.pose.orientation
 yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
 roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y));pitch=math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))
 rows.append([time.time(),p.x,p.y,p.z,roll,pitch,yaw,m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z])
n.create_subscription(Odometry,'/odom/mujoco_odom',odom,20);client=ActionClient(n,NavigateThroughPoses,'/navigate_through_poses')
if not client.wait_for_server(timeout_sec=20):raise RuntimeError('action unavailable')
goal=NavigateThroughPoses.Goal()
for x,y in [(.70,13.35),(.78,15.40),(.78,16.30),(.92,18.25)]:
 p=PoseStamped();p.header.frame_id='map';p.header.stamp=n.get_clock().now().to_msg();p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.z=math.sin(math.pi/4);p.pose.orientation.w=math.cos(math.pi/4);goal.poses.append(p)
f=client.send_goal_async(goal);rclpy.spin_until_future_complete(n,f,timeout_sec=15);handle=f.result()
if not handle or not handle.accepted:raise RuntimeError('goal rejected')
result=handle.get_result_async();deadline=time.time()+150
while not result.done() and time.time()<deadline:rclpy.spin_once(n,timeout_sec=.1)
status=result.result().status if result.done() else None
if not result.done():
 cancel=handle.cancel_goal_async();rclpy.spin_until_future_complete(n,cancel,timeout_sec=5)
with (args.output/'odom_physics.csv').open('w') as f:
 w=csv.writer(f);w.writerow('wall_time,x_m,y_m,z_m,roll_rad,pitch_rad,yaw_rad,vx_mps,vy_mps,wz_rps'.split(','));w.writerows(rows)
summary=dict(action_status=status,passed=status==4,samples=len(rows),start=rows[0][1:4] if rows else None,end=rows[-1][1:4] if rows else None,limitation='Isolated ramp traversal; approach from the preceding slalom is not covered.')
(args.output/'probe_result.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary));rclpy.shutdown()
