std_msgs/Header header
uint8 cmd
uint8 secondary_cmd
uint8 function_id
string map_path
string reference_path
int32 target_id
geometry_msgs/Pose[] goals
geometry_msgs/Vector3 speed
string extra_info

# CMD Enum
uint8 CMD_STANDBY=0
uint8 CMD_START=1
uint8 CMD_PAUSE=2
uint8 CMD_CONTINUE=3
uint8 CMD_STOP=4

# CMD Enum
uint8 SEC_CMD_NONE=0
uint8 SEC_CMD_RETURN=1

# FUNCTION_ID Enum
uint8 FUNCTION_UNDEFINED = 0
uint8 FUNCTION_TRACKING = 1
uint8 FUNCTION_SINGLE_POINT_PLAN = 2
uint8 FUNCTION_TRACKING_UWB = 3
uint8 FUNCTION_TRACKING_WITH_OBSTACLES = 4
uint8 FUNCTION_PATROL = 5
uint8 FUNCTION_EXPLORATION = 6
uint8 FUNCTION_MAPPING = 7