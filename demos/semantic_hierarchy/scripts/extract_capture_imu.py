#!/usr/bin/env python3
"""Read IMU signals plus a separate held-out validation trajectory from a ROS2 bag."""
import argparse,json
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args();r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(a.run/'sensors_rosbag2'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=['/imu/data_raw','/odom/mujoco_odom']));rows=[];refs=[]
while r.has_next():
 topic,raw,t=r.read_next()
 if topic=='/imu/data_raw':
  m=deserialize_message(raw,Imu);v=m.linear_acceleration;w=m.angular_velocity;rows.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,v.x,v.y,v.z,w.x,w.y,w.z])
 else:
  m=deserialize_message(raw,Odometry);v=m.pose.pose.position;q=m.pose.pose.orientation;refs.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,v.x,v.y,v.z,q.x,q.y,q.z,q.w])
np.save(a.run/'imu_signals.npy',np.array(rows));np.save(a.run/'validation_only_sim_poses.npy',np.array(refs));print(len(rows),len(refs))
