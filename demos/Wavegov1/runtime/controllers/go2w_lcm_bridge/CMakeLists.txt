cmake_minimum_required(VERSION 3.16)
project(go2w_lcm_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_FIND_PACKAGE_PREFER_CONFIG ON)

find_package(eCAL 5.13.3 EXACT REQUIRED)
find_package(Protobuf 3.12.4 EXACT REQUIRED)
find_package(lcm 1.5.1 EXACT REQUIRED)
find_package(zsibot_common 0.5.9 EXACT REQUIRED)

add_executable(go2w_lcm_bridge src/main.cpp)
target_include_directories(go2w_lcm_bridge PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../genisom_roamerx_open/src/navigation/src/robot_navigo/lcm-types/cpp
)
target_link_libraries(go2w_lcm_bridge PRIVATE
  eCAL::core
  lcm
  protobuf::libprotobuf
  zsibot_common::robot_sdk
)

install(TARGETS go2w_lcm_bridge RUNTIME DESTINATION bin)
