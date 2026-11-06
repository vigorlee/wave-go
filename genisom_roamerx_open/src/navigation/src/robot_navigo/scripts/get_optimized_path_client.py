#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robots_dog_msgs.srv import GetOptimizedPath
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import sys

def parse_txt_file(txt_path):
    with open(txt_path, 'r') as f:
        lines = f.readlines()

    key_poses = []
    paths = []
    current = None

    for line in lines:
        line = line.strip()
        if not line or line.startswith('#'):
            if 'key_poses' in line:
                current = 'key'
            elif 'path' in line:
                current = 'path'
            continue

        parts = list(map(float, line.split()))
        pose = Pose()
        pose.position.x = parts[0]
        pose.position.y = parts[1]
        pose.position.z = 0.0
        # You may choose to set orientation based on parts[2] (theta) if needed
        if current == 'key':
            key_poses.append(pose)
        elif current == 'path':
            stamped = PoseStamped()
            stamped.pose = pose
            paths.append(stamped)

    return key_poses, paths

class PathClient(Node):
    def __init__(self, txt_path):
        super().__init__('path_client')
        self.cli = self.create_client(GetOptimizedPath, 'get_optimized_path')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.req = GetOptimizedPath.Request()

        key_poses, raw_path = parse_txt_file(txt_path)
        path_msg = Path()
        path_msg.poses = raw_path

        self.req.raw_key_poses = key_poses
        self.req.raw_paths = [path_msg]
        self.req.smooth_method = "PP"

    def send_request(self):
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("Service call succeeded.")
            return future.result()
        else:
            self.get_logger().error("Service call failed: %r" % future.exception())
            return None

def plot_paths(raw_path: Path, opt_path: Path):
    def extract_xy(path):
        return [pose.pose.position.x for pose in path.poses], [pose.pose.position.y for pose in path.poses]

    raw_x, raw_y = extract_xy(raw_path)
    opt_x, opt_y = extract_xy(opt_path)

    plt.figure(figsize=(8,6))
    plt.plot(raw_x, raw_y, 'r--o', label='Raw Path')
    plt.plot(opt_x, opt_y, 'g-o', label='Optimized Path')
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Path Optimization Result")
    plt.legend()
    plt.axis("equal")
    plt.grid()
    plt.show()

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Usage: ros2 run <your_package> call_optimized_path_client.py <path_to_txt>")
        return

    txt_path = sys.argv[1]
    path_client = PathClient(txt_path)
    response = path_client.send_request()

    if response and response.success:
        raw_path = path_client.req.raw_paths[0]
        opt_path = response.opt_paths[0]
        plot_paths(raw_path, opt_path)
    else:
        print("Service failed:", response.error_msg if response else "Unknown error.")

    path_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
