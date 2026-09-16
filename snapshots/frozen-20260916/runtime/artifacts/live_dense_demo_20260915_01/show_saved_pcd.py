from pathlib import Path
import numpy as np,json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_msgs.msg import String
from live_capture import points_message
from sensor_msgs.msg import PointCloud2
r=Path(__file__).resolve().parent
rclpy.init();n=Node('wavego_saved_dense_pcd_view');q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL);pub=n.create_publisher(PointCloud2,'/wavego/live_dense/points',q)
a=np.load(r/'capture/go2w_live_dense_3cm.npz');pub.publish(points_message(a['points'],-.3,n.get_clock().now().to_msg()));n.get_logger().info(f"Displaying saved dense PCD: {len(a['points'])} points (recording finished)")
try:rclpy.spin(n)
except KeyboardInterrupt:pass
finally:n.destroy_node();rclpy.try_shutdown()
