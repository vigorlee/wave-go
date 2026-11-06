std_msgs/Header header
uint8 state
uint8 function_id
geometry_msgs/Pose current_pose
geometry_msgs/Pose current_goal
builtin_interfaces/Duration navigation_time
builtin_interfaces/Duration estimated_time_remaining_to_current
builtin_interfaces/Duration estimated_time_remaining_to_final
float64 remaining_distance_to_current
float64 remaining_distance_to_final


# STATE Enum
uint8 STATE_STANDBY=0
uint8 STATE_INITIALIZING=1
uint8 STATE_ACTIVE=2
uint8 STATE_PAUSE=3
uint8 STATE_CANCELLED=4
uint8 STATE_SUCCEEDED=5
uint8 STATE_FAILED=6

# FUNCTION_ID Enum
uint8 FUNCTION_UNDEFINED = 0
uint8 FUNCTION_TRACKING = 1
uint8 FUNCTION_TRACKING_UWB = 2
uint8 FUNCTION_TRACKING_WITH_OBSTACLES = 3
uint8 FUNCTION_PATROL = 4
uint8 FUNCTION_EXPLORATION = 5
uint8 FUNCTION_MAPPING = 6