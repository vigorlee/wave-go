cmake_minimum_required(VERSION 3.5)
project(navigo_velocity_optimizer)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(navigo_util REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

nav2_package()

include_directories(
  include
)

set(executable_name velocity_smoother)
set(library_name ${executable_name}_core)

set(dependencies
  rclcpp
  rclcpp_components
  geometry_msgs
  navigo_util
)

# Main library
add_library(${library_name} SHARED
  src/velocity_smoother.cpp
)
ament_target_dependencies(${library_name}
  ${dependencies}
)

# Main executable
add_executable(${executable_name}
  src/main.cpp
)
ament_target_dependencies(${executable_name}
  ${dependencies}
)
target_link_libraries(${executable_name} ${library_name})

rclcpp_components_register_nodes(${library_name} "navigo_velocity_optimizer::VelocityOptimizer")

install(
  TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
