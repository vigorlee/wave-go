"""Call the installed upstream Nav2 plugins; preserve raw action results."""
import argparse,json,time,math,subprocess
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy
from lifecycle_msgs.srv import ChangeState,GetState
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid,Path as NavPath
from geometry_msgs.msg import PoseStamped
from maps import R,STAGES,RES,ORIGIN,WIDTH,HEIGHT,layer,surface_z
class Planner(Node):
 def __init__(self):
  super().__init__('office_global_plan_compare');q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
  self.map=self.create_publisher(OccupancyGrid,'/wavego/planning_layer',q)
  self.action=ActionClient(self,ComputePathToPose,'/global_compare/compute_path_to_pose')
 def wait(self,f,timeout=20):
  rclpy.spin_until_future_complete(self,f,timeout_sec=timeout)
  if not f.done():raise TimeoutError('Nav2 request timed out')
  return f.result()
 def pause(self,t):
  end=time.monotonic()+t
  while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.05)
 def publish_layer(self,i):
  a=layer(i);m=OccupancyGrid();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg();m.info.map_load_time=m.header.stamp;m.info.resolution=RES;m.info.width=WIDTH;m.info.height=HEIGHT;m.info.origin.position.x=ORIGIN[0];m.info.origin.position.y=ORIGIN[1];m.info.origin.orientation.w=1.;m.data=a.ravel().tolist();self.map.publish(m);self.pause(1.0)
 def activate(self):
  c=self.create_client(ChangeState,'/global_compare/planner_server/change_state')
  if not c.wait_for_service(timeout_sec=30):raise RuntimeError('Planner server missing')
  self.publish_layer(0)
  state=self.create_client(GetState,"/global_compare/planner_server/get_state");state.wait_for_service(timeout_sec=5)
  if self.wait(state.call_async(GetState.Request())).current_state.id==3:return
  for transition in (1,3):
   req=ChangeState.Request();req.transition.id=transition
   if not self.wait(c.call_async(req)).success:raise RuntimeError('Planner lifecycle transition failed')
  if not self.action.wait_for_server(timeout_sec=20):raise RuntimeError('Planner action missing')
 def pose(self,x,y):
  p=PoseStamped();p.header.frame_id='map';p.header.stamp=self.get_clock().now().to_msg();p.pose.position.x=float(x);p.pose.position.y=float(y);p.pose.orientation.w=1.;return p
 def plan(self,planner,start,goal,i):
  req=ComputePathToPose.Goal();req.planner_id=planner;req.use_start=True;req.start=self.pose(*start);req.goal=self.pose(*goal)
  handle=self.wait(self.action.send_goal_async(req))
  if not handle.accepted:raise RuntimeError('Planner rejected goal')
  result=self.wait(handle.get_result_async());path=result.result.path
  if result.status!=4 or not path.poses:raise RuntimeError(f'{planner} failed at stage {i}: status {result.status}')
  xy=[[p.pose.position.x,p.pose.position.y] for p in path.poses]
  if np.linalg.norm(np.array(xy[-1])-goal)>.13:raise RuntimeError('Planner endpoint outside requested tolerance')
  return dict(stage=STAGES[i],stage_index=i,raw_xy=xy,xyz=[[x,y,surface_z(x,y,i)+.09] for x,y in xy],length_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),planning_time_s=result.result.planning_time.sec+result.result.planning_time.nanosec*1e-9)
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('--output',type=Path,required=True);args=a.parse_args();args.output.mkdir(parents=True,exist_ok=True)
 rclpy.init();n=Planner();n.activate();data={p:[] for p in ('NavFnDijkstra','SmacAStar')};start=(8.,0.)
 try:
  for i,s in enumerate(STAGES):
   n.publish_layer(i);goal=(s['x'],s['y'])
   for p in data:
    route=n.plan(p,start,goal,i);data[p].append(route);print(p,s['id'],len(route['raw_xy']),round(route['length_m'],3),flush=True)
   (args.output/'routes_partial.json').write_text(json.dumps(data,indent=2))
   start=goal
  out=dict(source='Actual ComputePathToPose results from installed upstream Nav2 plugins',frame='map',map_source='Layered prior traversability from existing simulation geometry',planners=data,versions=subprocess.check_output(['dpkg-query','-W','ros-humble-nav2-navfn-planner','ros-humble-nav2-smac-planner'],text=True))
  (args.output/'routes.json').write_text(json.dumps(out,indent=2))
 finally:n.destroy_node();rclpy.shutdown()
