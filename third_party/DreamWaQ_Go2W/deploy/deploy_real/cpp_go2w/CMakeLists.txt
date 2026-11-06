cmake_minimum_required(VERSION 3.16)
project(go2w_deploy_real)

# 1. 全局配置
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 17)

# 2. 查找所有依赖库
# 2.1 查找 LibTorch
set(CMAKE_PREFIX_PATH "${CMAKE_CURRENT_LIST_DIR}/libtorch/share/cmake")
find_package(Torch REQUIRED)
# 2.2 查找 yaml-cpp
find_package(yaml-cpp REQUIRED)

# 3. 包含头文件路径
include_directories(/usr/local/include/ddscxx)  
include_directories(/usr/local/include/iceoryx/v2.0.2) 
include_directories(${TORCH_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIRS}) 

# 4. 生成可执行文件
aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})

# 5. 链接库
target_link_libraries(${PROJECT_NAME} unitree_sdk2 ddsc ddscxx ${TORCH_LIBRARIES} yaml-cpp pthread)

# unitree_sdk2: 宇树科技机器人的 SDK
# ddsc, ddscxx: DDS 通信库 (C 和 C++ 版本)
# ${TORCH_LIBRARIES}: LibTorch 库
# yaml-cpp: YAML 解析库
# pthread: POSIX 线程库 (通常用于多线程支持)