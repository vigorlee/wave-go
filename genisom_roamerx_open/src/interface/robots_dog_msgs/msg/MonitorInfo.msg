# Jszr Monitor msg format.

std_msgs/Header header    # ROS standard message header
ProcessInfo[]   module_process
CpuInfo[]       cpu_info
NetInfo[]       net_info
DiskInfo[]      disk_info
float64         global_cpu_usage
float64         global_cpu
float64         global_memory_usage
float64         global_memory