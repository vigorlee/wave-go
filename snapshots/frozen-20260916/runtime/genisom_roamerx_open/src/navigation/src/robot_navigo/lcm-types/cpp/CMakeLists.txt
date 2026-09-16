find_package(lcm REQUIRED)
add_library(lcm_msg INTERFACE)
target_include_directories(lcm_msg
        INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
)
