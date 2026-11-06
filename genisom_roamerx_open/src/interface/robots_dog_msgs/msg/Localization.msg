std_msgs/Header header       # timestamp in the header is the time the measurement was taken
string type                  # message type
uint8 status                 # Localization status: 0=Initializing, 1=Relocalizing, 2=Relocalization Succeeded, 3=Normal, 4=Lost
uint8 coord_type             # Coordinate type: 0=Map coordinates, 1=Geodetic (latitude/longitude) coordinates
geometry_msgs/Vector3 pos    # Position (x, y, z) in map coordinates (m)
geometry_msgs/Vector3 rpy    # Orientation (roll, pitch, yaw) in radians
geometry_msgs/Vector3 vel    # Linear velocity (m/s)
geometry_msgs/Vector3 acc    # Linear acceleration (m/s^2)
geometry_msgs/Vector3 gyro   # angular velocity (rad/s)
float64 speed                # Speed (m/s)
