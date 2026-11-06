# Add the timestamp at which this pose is predicted to occur
builtin_interfaces/Time stamp

# The 2D position (x, y) and yaw orientation of the agent
geometry_msgs/Pose pose

# The four corner points of the bounding box
geometry_msgs/Point[4] corners

# Add twist at this point (angular only includes yaw)
# Calculated manully by using two neighboring frames
geometry_msgs/Twist twist

# Add linear accelerations (only x and y)
geometry_msgs/Accel acceleration