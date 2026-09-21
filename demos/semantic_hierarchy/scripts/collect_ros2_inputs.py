#!/usr/bin/env python3
"""Read-only ROS2 snapshot adapter for the existing live OfficeWorld stack.

Requires fresh LiDAR map + semantic map + LIO pose in the same frame. Does not
substitute simulation ground truth or silently relabel coordinate frames.
"""
import argparse
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2


def xyz(msg):
    values=point_cloud2.read_points(msg,skip_nans=True)
    p=np.column_stack([values[k] for k in ('x','y','z')]).astype(np.float32)
    mask=np.isfinite(p).all(axis=1)
    return p[mask],values[mask]


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--seconds',type=float,default=180)
    ap.add_argument('--frame',default='lio_map')
    ap.add_argument('--map-topic',default='/wavego/mapping/points')
    ap.add_argument('--semantic-topic',default='/wavego/semantic/map')
    ap.add_argument('--odom-topic',default='/wavego/navigation/odom')
    args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    rclpy.init()
    node=Node('wavego_scene_input_collector')
    state={}
    poses=[]
    def receive(key,msg):
        if msg.header.frame_id!=args.frame:
            node.get_logger().warning(f'Reject {key}: frame {msg.header.frame_id} != {args.frame}')
            return
        state[key]=(msg,time.monotonic())
    def odom(msg):
        if msg.header.frame_id!=args.frame: return
        p=msg.pose.pose.position
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        poses.append([stamp,p.x,p.y,p.z])
        state['odom']=(msg,time.monotonic())
    latch=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(PointCloud2,args.map_topic,lambda m:receive('map',m),latch)
    node.create_subscription(PointCloud2,args.semantic_topic,lambda m:receive('semantic',m),latch)
    node.create_subscription(Odometry,args.odom_topic,odom,qos_profile_sensor_data)
    start=time.monotonic()
    try:
        while time.monotonic()-start<args.seconds:
            rclpy.spin_once(node,timeout_sec=.2)
        assert len(poses)>=80, 'No valid fresh LIO trajectory; refusing stale snapshot'
        for key in ['map','semantic','odom']:
            assert key in state and time.monotonic()-state[key][1]<6, f'{key} stream stale/missing'
            msg=state[key][0]
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            now=node.get_clock().now().nanoseconds*1e-9
            assert -0.1<=now-stamp<6, f'{key} header stamp stale'
        geometry,_=xyz(state['map'][0])
        semantic,values=xyz(state['semantic'][0])
        assert 'label' in values.dtype.names, 'Semantic cloud needs a label field'
        np.savez_compressed(args.output/'semantic_map.npz',points=semantic,labels=values['label'].astype(np.uint8))
        np.save(args.output/'lio_trajectory.npy',np.array(poses))
        header=('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                f'WIDTH {len(geometry)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
                f'POINTS {len(geometry)}\nDATA binary\n')
        (args.output/'geometry.pcd').write_bytes(header.encode()+geometry.astype('<f4').tobytes())
        (args.output/'provenance.json').write_text(json.dumps(vars(args),default=str,indent=2))
        print('INPUT_SNAPSHOT_COMPLETE',args.output)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':main()
