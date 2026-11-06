"""Record real ROS 2 messages through one measured navigation demonstration."""
from pathlib import Path
import os,sys,subprocess,signal,time,json
root=Path(__file__).resolve().parents[1];run=Path(sys.argv[1]).resolve();bag=run/'pointcloud_odom_rosbag2'
if bag.exists():raise SystemExit('Refusing to overwrite existing bag')
topics=['/livox/lidar_raw','/wavego/slam/registered_points','/wavego/slam/odom','/wavego/navigation/odom','/odom/mujoco_odom','/livox/imu','/imu/data_raw','/tf','/tf_static','/wavego/slam_tf','/cmd_vel_nav']
qos=run/'bag_qos.yaml';qos.write_text(''.join(f'{t}:\n  reliability: best_effort\n  durability: volatile\n  history: keep_last\n  depth: 200\n' for t in topics if t not in ['/tf_static'])+'/tf_static:\n  reliability: reliable\n  durability: transient_local\n  history: keep_last\n  depth: 100\n')
cmd=['ros2','bag','record','-s','sqlite3','--max-cache-size','268435456','--qos-profile-overrides-path',str(qos),'-o',str(bag),*topics]
with (run/'rosbag_record.log').open('w') as log:
 p=subprocess.Popen(cmd,cwd=root,stdout=log,stderr=subprocess.STDOUT)
 try:
  time.sleep(4)
  if p.poll() is not None:raise RuntimeError('Recorder exited before navigation')
  subprocess.run(['bash','scripts/record_office_inline_take.sh',str(run)],cwd=root,check=True)
 finally:
  if p.poll() is None:
   p.send_signal(signal.SIGINT)
   try:p.wait(timeout=30)
   except subprocess.TimeoutExpired:p.terminate();p.wait(timeout=10)
(run/'rosbag_record_manifest.json').write_text(json.dumps(dict(topics=topics,bag=str(bag),recorder_return_code=p.returncode,source='live ROS 2 subscriptions; raw simulator LiDAR and LIO odometry; MuJoCo ground truth is a separate topic'),indent=2))
subprocess.run(['ros2','bag','info',str(bag)],check=True)
