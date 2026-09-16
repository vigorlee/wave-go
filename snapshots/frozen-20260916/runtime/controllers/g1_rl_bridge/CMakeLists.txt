cmake_minimum_required(VERSION 3.18)
project(g1_rl_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_FIND_PACKAGE_PREFER_CONFIG ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(eCAL 5.13.3 EXACT REQUIRED)
find_package(Protobuf 3.12.4 EXACT REQUIRED)
find_package(lcm 1.5.1 EXACT REQUIRED)
find_package(zsibot_common 0.5.9 EXACT REQUIRED)
find_package(Torch REQUIRED)
find_package(Threads REQUIRED)

get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(DEFAULT_DREAMWAQ_MODEL_PATH
    "${REPO_ROOT}/third_party/HumanoidDreamWaQ_G1/pre_train/g1/dreamwaq_level9.pt")
set(DEFAULT_OFFICIAL_MODEL_PATH
    "${REPO_ROOT}/third_party/unitree_rl_gym_g1/pre_train/g1/motion.pt")

add_executable(g1_rl_bridge src/main.cpp)
target_include_directories(g1_rl_bridge PRIVATE
  ${REPO_ROOT}/genisom_roamerx_open/src/navigation/src/robot_navigo/lcm-types/cpp
)
target_compile_definitions(g1_rl_bridge PRIVATE
  DEFAULT_DREAMWAQ_MODEL_PATH="${DEFAULT_DREAMWAQ_MODEL_PATH}"
  DEFAULT_OFFICIAL_MODEL_PATH="${DEFAULT_OFFICIAL_MODEL_PATH}"
)

separate_arguments(TORCH_CXX_FLAGS_LIST NATIVE_COMMAND "${TORCH_CXX_FLAGS}")
target_compile_options(g1_rl_bridge PRIVATE ${TORCH_CXX_FLAGS_LIST})
target_link_libraries(g1_rl_bridge PRIVATE
  eCAL::core
  lcm
  protobuf::libprotobuf
  zsibot_common::robot_sdk
  ${TORCH_LIBRARIES}
  Threads::Threads
)

if(DEFINED TORCH_INSTALL_PREFIX)
  set_target_properties(g1_rl_bridge PROPERTIES
    BUILD_RPATH "${TORCH_INSTALL_PREFIX}/lib"
    INSTALL_RPATH "${TORCH_INSTALL_PREFIX}/lib"
  )
endif()

install(TARGETS g1_rl_bridge RUNTIME DESTINATION bin)
