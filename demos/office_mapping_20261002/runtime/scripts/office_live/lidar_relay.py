"""Keep LiDAR preprocessing independent of map rendering and disk compression."""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc

class Relay(Node):
    def __init__(self):
        super().__init__('office_fast_lidar_relay')
        self.pub = self.create_publisher(PointCloud2, '/wavego/slam/raw_lidar', qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/livox/lidar_raw', self.scan, qos_profile_sensor_data)

    def scan(self, m):
        a = pc.read_points(m)
        xyz = np.column_stack([a[k] for k in ('x', 'y', 'z')])
        radius = np.linalg.norm(xyz, axis=1)
        valid = np.isfinite(xyz).all(axis=1) & (radius > .55) & (radius < 30.)
        m.data = a[valid].tobytes()
        m.height = 1
        m.width = int(valid.sum())
        m.row_step = m.width*m.point_step
        m.is_dense = True
        self.pub.publish(m)

if __name__ == '__main__':
    rclpy.init()
    node = Relay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
