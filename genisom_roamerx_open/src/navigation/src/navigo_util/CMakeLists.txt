cmake_minimum_required(VERSION 3.5)
project(navigo_util)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(bondcpp REQUIRED)
find_package(bond REQUIRED)
find_package(action_msgs REQUIRED)

set(dependencies
    nav2_msgs
    tf2_ros
    tf2
    tf2_geometry_msgs
    geometry_msgs
    nav_msgs
    rclcpp
    lifecycle_msgs
    rclcpp_action
    rclcpp_lifecycle
    bondcpp
    bond
    action_msgs
    rcl_interfaces
)

nav2_package()

include_directories(include)

set(library_name ${PROJECT_NAME}_core)
add_subdirectory(src)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
