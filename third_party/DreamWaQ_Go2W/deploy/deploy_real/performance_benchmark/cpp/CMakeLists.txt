# GO2W C++ Benchmark - 使用本地 Controller.cpp（含计时代码）
cmake_minimum_required(VERSION 3.16)
project(go2w_benchmark)

set(CMAKE_BUILD_TYPE Debug)

include_directories(/usr/local/include/ddscxx)
include_directories(/usr/local/include/iceoryx/v2.0.2)

set(CMAKE_CXX_STANDARD 17)

# 使用 cpp_go2w 目录下的 libtorch
set(CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../cpp_go2w/libtorch/share/cmake")
find_package(Torch REQUIRED)
include_directories(${TORCH_INCLUDE_DIRS})

# 使用本地的源文件（含计时代码）
set(SRC_LIST
    ${CMAKE_CURRENT_SOURCE_DIR}/Controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/benchmark_main.cpp
)
add_executable(${PROJECT_NAME} ${SRC_LIST})

find_package(yaml-cpp REQUIRED)
target_link_libraries(${PROJECT_NAME} unitree_sdk2 ddsc ddscxx ${TORCH_LIBRARIES} yaml-cpp pthread)
