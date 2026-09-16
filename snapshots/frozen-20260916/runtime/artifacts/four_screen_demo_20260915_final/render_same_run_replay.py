from pathlib import Path
import json,time,sys,subprocess,re
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import Path as NavPath
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Header
from PyQt5.QtWidgets import QApplication
R=Path(__file__).resolve().parent
M=json.loads((R/'source_manifest.json').read_text());SRC=Path(M['source_run']);sys.path.insert(0,str(SRC))
from live_capture import points_message,Recorder,build_grid,analyze
app=QApplication([]);rclpy.init();node=Node('same_run_pointcloud_replay');q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.pub=node.create_publisher(type(points_message(np.empty((0,3)),0,node.get_clock().now().to_msg())),'/wavego/replay/points',q)
node.path=node.create_publisher(NavPath,'/wavego/replay/path',q);markerpub=node.create_publisher(MarkerArray,'/wavego/replay/inflation',10)
class MarkerPublisher:
 def publish(self,m):
  for item in m.markers:
   if item.type==item.TEXT_VIEW_FACING:item.text=item.text.replace('LIVE LiDAR','RECORDED LiDAR');item.scale.z=.18
  markerpub.publish(m)
node.mark=MarkerPublisher()
connection_deadline=time.monotonic()+20
while node.pub.get_subscription_count()==0 and time.monotonic()<connection_deadline:rclpy.spin_once(node,timeout_sec=.2)
assert node.pub.get_subscription_count()>0,'RViz subscriber unavailable'
rows=subprocess.check_output(['wmctrl','-l'],text=True).splitlines();wid=next(row.split()[0] for row in rows if '/replay.rviz' in row)
subprocess.run(['wmctrl','-ir',wid,'-b','remove,maximized_vert,maximized_horz']);subprocess.run(['wmctrl','-ir',wid,'-e','0,100,80,1700,850']);subprocess.run(['wmctrl','-ir',wid,'-b','add,above']);subprocess.run(['wmctrl','-ia',wid]);time.sleep(2)
frames=[json.loads(x) for x in (SRC/'capture/frames.jsonl').read_text().splitlines()];ts=np.array([x['stamp'] for x in frames]);odom=[json.loads(x) for x in (SRC/'capture/lio_trajectory.jsonl').read_text().splitlines()];ots=np.array([x['stamp'] for x in odom]);raw=np.memmap(SRC/'capture/registered_xyzi.float32',dtype='<f4',mode='r').reshape(-1,4);floor=odom[0]['z']-.4
cells={};last=0;offset=0;output=R/'rviz_frames';output.mkdir(exist_ok=True);log=[];count=int(np.ceil(M['segment_duration']))+1
for i in range(count):
 target=M['segment_wall_start']+min(i,M['segment_duration']);j=int(np.searchsorted(ts,target,side='right'));j=max(1,j);f=frames[j-1];stop=f['offset_points']+f['points'];xyz=np.asarray(raw[offset:stop,:3]);offset=stop
 if len(xyz):
  keys=np.floor(xyz/.04).astype(np.int64);packed=((keys[:,0]+1048576)<<42)|((keys[:,1]+1048576)<<21)|(keys[:,2]+1048576);_,ix=np.unique(packed,return_index=True);cells.update(zip(packed[ix].tolist(),xyz[ix]))
 allpts=np.asarray(list(cells.values()),np.float32);stamp=node.get_clock().now().to_msg();node.pub.publish(points_message(allpts,floor,stamp));node.scans=j
 k=max(1,int(np.searchsorted(ots,target,side='right')));p=odom[k-1];path=NavPath(header=Header(frame_id='lio_map',stamp=stamp))
 for a in odom[:k:3]:
  pose=PoseStamped(header=path.header);pose.pose.position.x=a['x'];pose.pose.position.y=a['y'];pose.pose.position.z=a['z'];pose.pose.orientation.w=1.;path.poses.append(pose)
 node.path.publish(path)
 scan=np.asarray(raw[f['offset_points']:stop,:3]);delta=scan-[p['x'],p['y'],p['z']];near=scan[(np.linalg.norm(delta[:,:2],axis=1)<6)&(np.abs(delta[:,2])<2)];radius=None
 try:
  grid=build_grid(near,np.zeros(len(near),np.uint8),p['z']-.4);adaptive,grids=analyze(grid,[p['x'],p['y']],p['yaw'],min(p['speed'],1.2));Recorder.publish_markers(node,grid,grids,adaptive);radius=adaptive['radius']
 except ValueError:pass
 deadline=time.monotonic()+.55
 while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.04);app.processEvents()
 image_path=output/f'{i:04d}.png';assert app.primaryScreen().grabWindow(int(wid,16)).save(str(image_path))
 log.append(dict(video_second=i,source_stamp=target,latest_scan_stamp=f['stamp'],accumulated_voxels=len(cells),voxel_m=.04,inflation_radius=radius,png=str(image_path)))
 if i%10==0:print(f'Rendered {i}/{count}: {len(cells):,} measured voxels',flush=True)
(R/'replay_frames.json').write_text(json.dumps(log,indent=2));print('RViz replay rendered',len(log),flush=True)
node.destroy_node();rclpy.shutdown()
