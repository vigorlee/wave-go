"""Display upstream paths in the independently estimated LIO map.

Only the initial rigid frame registration uses simulator odometry. LiDAR map
points are still registered by the existing LiDAR/IMU estimator.
"""
import argparse
import json
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, Path as NavPath
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation

def transform(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    a = np.eye(4)
    a[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    a[:3, 3] = [p.x, p.y, p.z]
    return a

class Display(Node):
    def __init__(self, source, output):
        super().__init__('upstream_global_route_display')
        self.routes = json.loads(source.read_text())['planners']
        self.output = output
        self.mj = self.lio = None
        self.aligned = False
        self.tf = StaticTransformBroadcaster(self)
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pubs = {k: self.create_publisher(NavPath, '/wavego/global/' + k, q) for k in self.routes}
        self.create_subscription(Odometry, '/odom/mujoco_odom', self.physics, 10)
        self.create_subscription(Odometry, '/wavego/navigation/odom', self.estimated, 10)
        self.create_timer(1.0, self.tick)

    def physics(self, m):
        self.mj = transform(m)

    def estimated(self, m):
        self.lio = transform(m)

    def tick(self):
        if not self.aligned:
            if self.mj is None or self.lio is None:
                return
            # This node must start with the stationary robot at the start line.
            if np.linalg.norm(self.mj[:2, 3] - [8., 0.]) > .25:
                raise RuntimeError('Start the route display before navigation, at the start line')
            a = self.lio @ np.linalg.inv(self.mj)
            t = TransformStamped()
            t.header.frame_id, t.child_frame_id = 'lio_map', 'map'
            t.header.stamp = self.get_clock().now().to_msg()
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = a[:3, 3].tolist()
            q = Rotation.from_matrix(a[:3, :3]).as_quat()
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q.tolist()
            self.tf.sendTransform(t)
            self.output.parent.mkdir(parents=True, exist_ok=True)
            self.output.write_text(json.dumps({'parent': 'lio_map', 'child': 'map', 'matrix': a.tolist(), 'source': 'Initial stationary rigid alignment; LiDAR mapping remains estimated'}, indent=2))
            self.aligned = True
        for name, legs in self.routes.items():
            m = NavPath()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            for leg in legs:
                for x, y, z in leg['xyz']:
                    p = PoseStamped(header=m.header)
                    p.pose.position.x, p.pose.position.y, p.pose.position.z = float(x), float(y), float(z)
                    p.pose.orientation.w = 1.
                    m.poses.append(p)
            self.pubs[name].publish(m)

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--routes', type=Path, required=True)
    p.add_argument('--alignment', type=Path, required=True)
    args = p.parse_args()
    rclpy.init()
    node = Display(args.routes, args.alignment)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
