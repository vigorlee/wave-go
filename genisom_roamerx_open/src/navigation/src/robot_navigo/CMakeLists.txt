cmake_minimum_required(VERSION 3.8)

project(robot_navigo
        LANGUAGES CXX C
        VERSION 0.0.1
        DESCRIPTION "robot_navigo source code"
)

option(FOR_3588 OFF)
if(FOR_3588)
    add_definitions(-DFOR_3588)
endif()


if (BUILD_DEBUG_PACKAGES)
    message("Building in DEBUG mode for robot_navigo")
    SET(CMAKE_CXX_FLAGS "-g -O0")
    SET(CMAKE_BUILD_TYPE "Debug")
    SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
    SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
else ()
    message(STATUS "Building in RELEASE mode for robot_navigo")
endif ()

#############################################################################
## C++ standard and flags and some options
#############################################################################
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 99)

#############################################################################
# find dependencies
#############################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(navigation2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lcm REQUIRED)
find_package(navigo_behavior_tree REQUIRED)
find_package(navigo_bt_navigator REQUIRED)
find_package(navigo_core REQUIRED)
find_package(navigo_costmap_2d REQUIRED)
find_package(robots_dog_msgs REQUIRED)

find_package(tf2_geometry_msgs REQUIRED) 

#############################################################################
# lcm interface lib
#############################################################################
add_subdirectory(lcm-types/cpp)

#############################################################################
# udp interface lib
#############################################################################
add_subdirectory(udp) 

#############################################################################
# some exe lib
#############################################################################

#==========================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
add_executable(odom_to_tf_broadcaster
        src/tf_publisher.cpp
)
ament_target_dependencies(odom_to_tf_broadcaster
        rclcpp
        tf2_ros
        nav_msgs
)

#==========================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
add_executable(custom_odom_baselink_node
        src/custom_odom_baselink.cpp
)
ament_target_dependencies(custom_odom_baselink_node
        rclcpp
        tf2_ros
        nav_msgs
)

#==========================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
add_executable(odom_communication_node
        src/odom_communication_node.cpp
)
ament_target_dependencies(odom_communication_node
        rclcpp
        nav_msgs
        tf2_ros
)
target_link_libraries(odom_communication_node
        lcm_msg
        lcm
)

#==========================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>        
add_executable(odom_communication_node_udp
        src/odom_communication_node_udp.cpp
)
ament_target_dependencies(odom_communication_node_udp
        rclcpp
        nav_msgs
        tf2_ros
)
target_link_libraries(odom_communication_node_udp
        udp
)
target_include_directories(odom_communication_node_udp
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/udp/include  
)

#==========================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
add_executable(static_map_publisher
        src/static_map_publisher.cpp
)
ament_target_dependencies(static_map_publisher
        rclcpp
        tf2_ros
        nav_msgs
        OpenCV
        yaml-cpp
)
target_link_libraries(static_map_publisher
        yaml-cpp
)

#==========================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
add_executable(multiple_goal_nav
        src/mutiple_goal_nav.cpp
)
ament_target_dependencies(multiple_goal_nav
        geometry_msgs
        rclcpp
        sensor_msgs
        tf2_ros
        tf2
        nav2_msgs
        visualization_msgs
)

#==========================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
add_executable(vel_cmd_lcm_pub
        src/vel_cmd_lcm_publisher.cpp
)
ament_target_dependencies(vel_cmd_lcm_pub
        geometry_msgs
        rclcpp)
target_link_libraries(vel_cmd_lcm_pub
        lcm_msg
        lcm
)

#==========================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
add_executable(vel_cmd_udp_pub
        src/vel_cmd_udp_publisher.cpp
)
ament_target_dependencies(vel_cmd_udp_pub
        geometry_msgs
        tf2_ros
        tf2
        rclcpp)
target_link_libraries(vel_cmd_udp_pub
        udp
)
target_include_directories(vel_cmd_udp_pub
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/udp/include 
)

#==========================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
add_executable(mode_status_pub
        src/mode_status_publisher.cpp
)
ament_target_dependencies(mode_status_pub
        geometry_msgs
        rclcpp)

#==========================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
add_executable(vel_with_mc_trajectory_cmd_udp_pub
        src/vel_with_mc_trajectory_cmd_udp.cpp
)
ament_target_dependencies(vel_with_mc_trajectory_cmd_udp_pub
        robots_dog_msgs
        geometry_msgs
        rclcpp
        tf2_ros
        tf2)
target_link_libraries(vel_with_mc_trajectory_cmd_udp_pub
        udp
)
target_include_directories(vel_with_mc_trajectory_cmd_udp_pub
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/udp/include 
)

#==========================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
add_executable(costmap_listener
        src/costmap_listener.cpp
)
ament_target_dependencies(costmap_listener
        nav2_msgs
        rclcpp)

#==========================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
install(PROGRAMS
        scripts/emap_publisher.py
        DESTINATION lib/${PROJECT_NAME}
)

#==========================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 
# add_executable(detector_node 
#         src/obstacle_detector.cpp 
# )
# ament_target_dependencies(detector_node 
#         visualization_msgs 
#         rclcpp sensor_msgs 
#         tf2_geometry_msgs
#         pcl_conversions
#         geometry_msgs 
#         std_msgs 
#         tf2_ros 
#         PCL)

#############################################################################
# install rules
#############################################################################
install(TARGETS odom_to_tf_broadcaster
        DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS custom_odom_baselink_node
        DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS odom_communication_node
        DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS odom_communication_node_udp
        DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS static_map_publisher
        DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS multiple_goal_nav
        DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS vel_cmd_lcm_pub
        DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS vel_cmd_udp_pub
        DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS vel_with_mc_trajectory_cmd_udp_pub
        DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS costmap_listener
        DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS mode_status_pub
        DESTINATION lib/${PROJECT_NAME}
)
# install(TARGETS detector_node
#         DESTINATION lib/${PROJECT_NAME}
# )
install(DIRECTORY launch params map rviz
        DESTINATION share/${PROJECT_NAME}
        DIRECTORY_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
        GROUP_READ GROUP_WRITE GROUP_EXECUTE
        WORLD_READ WORLD_WRITE WORLD_EXECUTE
        FILES_MATCHING PATTERN "*"
        PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
        GROUP_READ GROUP_WRITE GROUP_EXECUTE
        WORLD_READ WORLD_WRITE WORLD_EXECUTE
)

ament_package()

#############################################################################
## enabel deb package
############################################################################
set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/ros/$ENV{ROS_DISTRO}")
set(CPACK_PACKAGE_NAME ${PROJECT_NAME})
set(CPACK_PACKAGE_FILE_NAME ${CPACK_PACKAGE_NAME}_${PROJECT_VERSION}_${CMAKE_HOST_SYSTEM_PROCESSOR}_$ENV{ROS_DISTRO}_${CMAKE_SYSTEM_NAME})
set(CPACK_GENERATOR "DEB")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "kanonwy")

include(CPack)
