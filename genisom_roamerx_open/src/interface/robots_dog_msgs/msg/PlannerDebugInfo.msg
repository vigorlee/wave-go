# 基础信息
builtin_interfaces/Time stamp
uint32 planner_count
uint8 planner_result # 0=FAILED, 1=SUCCEED
uint16 error_code
string error_msg
string remark

# 输入输出
geometry_msgs/PoseStamped received_start_point
geometry_msgs/PoseStamped received_goal_point
geometry_msgs/PoseStamped planner_start_point
geometry_msgs/PoseStamped planner_goal_point

# 算法类型
uint8 frontend_searcher_name  # 0=KINO_HYBRID_ASTAR, 1=QUADTREE

# 耗时统计
float64 frontend_time
float64 corridor_time
float64 backend_time
float64 planner_total_time

# Error codes
# NONE=0
# UNKNOWN=100
# FRONTEND_FAILED_WITH_START_POINT_COLLISION=101
# FRONTEND_FAILED_WITH_END_POINT_COLLISION=102
# FRONTEND_FAILED_WITH_TIME_OUT=103
# FRONTEND_FAILED_WITH_OPENSET_EMPTY=104
# FRONTEND_FAILED_WITH_START_END_POINT_VERSION_ERROR=105
# FRONTEND_FAILED_WITH_ROAD_MAP_SEARCH_FAILED=106
# SAFE_DRIVE_CORRIDOR_FAILED=107
# BACKEND_FAILED=108

# Sample statistics
geometry_msgs/PoseStamped last_used_start_point
geometry_msgs/PoseStamped last_used_goal_point

uint16 total_sampled_starts       # 起点采样总数
uint16 total_sampled_goals       # 起点采样总数

uint16 total_search_attempts      # 尝试规划次数
uint16 failed_search_attempts     # 失败次数
uint16 successful_search_index    # 第几次尝试成功（从0开始），若没有成功为UINT16_MAX

