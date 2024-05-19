cmake_minimum_required(VERSION 3.11)

project(robotParameters)

add_library(robotParameters INTERFACE)
target_link_libraries(robotParameters INTERFACE rbdl-static)
target_include_directories(robotParameters INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
