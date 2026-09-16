#!/usr/bin/env python3
"""
ROS2 Python node to publish a test ElectronicMap message for the ElectronicMapLayer.

This node publishes, at 1 Hz, a robots_dog_msgs/ElectronicMap containing:
  1. A single POINTS obstacle
  2. A LINE_SEGMENTS obstacle
  3. A POLYGON obstacle

Run this alongside your costmap launcher to verify that the layer draws
points, lines, and polygon boundaries correctly.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Header
from geometry_msgs.msg import Point
from robots_dog_msgs.msg import ElectronicMap, ElectronicObstacle


class ElectronicMapPublisher(Node):
    def __init__(self):
        super().__init__('electronic_map_publisher')
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        self.pub = self.create_publisher(ElectronicMap, '/electronic_map', qos_profile)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('ElectronicMapPublisher started, publishing to /electronic_map')

    def timer_callback(self):
        msg = ElectronicMap()
        # Fill header
        now = self.get_clock().now().to_msg()
        msg.header = Header(stamp=now, frame_id='map')

        # 1) POINTS obstacle
        obs_points = ElectronicObstacle()
        obs_points.label = ElectronicObstacle.LABEL_PEDESTRIAN
        obs_points.points_type = ElectronicObstacle.POINTS_TYPE_POINTS
        obs_points.points = [
            Point(x=1.0, y=1.0, z=0.0),
            Point(x=2.0, y=1.5, z=0.0),
        ]
        msg.obstacles.append(obs_points)

        # 2) LINE_SEGMENTS obstacle
        obs_line = ElectronicObstacle()
        obs_line.label = ElectronicObstacle.LABEL_VEHICLE
        obs_line.points_type = ElectronicObstacle.POINTS_TYPE_LINE_SEGMENTS
        obs_line.points = [
            Point(x=3.0, y=3.0, z=0.0),
            Point(x=4.0, y=3.0, z=0.0),
            Point(x=4.0, y=4.0, z=0.0),
        ]
        msg.obstacles.append(obs_line)

        # 3) POLYGON obstacle
        obs_poly = ElectronicObstacle()
        obs_poly.label = ElectronicObstacle.LABEL_UNKNOWN
        obs_poly.points_type = ElectronicObstacle.POINTS_TYPE_POLYGON
        obs_poly.points = [
            Point(x=-50.0, y=5.0, z=0.0),
            Point(x=6.0, y=5.0, z=0.0),
            Point(x=6.0, y=-50.0, z=0.0),
            Point(x=-50.0, y=-50.0, z=0.0),
        ]
        msg.obstacles.append(obs_poly)

        # Publish
        self.pub.publish(msg)
        self.get_logger().info(f'Published ElectronicMap with '
                               f'{len(msg.obstacles)} obstacles at {now.sec}.{now.nanosec:09d}')


def main(args=None):
    rclpy.init(args=args)
    node = ElectronicMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
