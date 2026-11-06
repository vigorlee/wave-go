cmake_minimum_required(VERSION 3.8)

add_library(udp STATIC
  src/highlevel_connector.cpp  
)

target_include_directories(udp PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include  
)

