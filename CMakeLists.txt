cmake_minimum_required(VERSION 3.17)
cmake_policy(SET CMP0091 NEW)
project(Curan)

#Language definitions used throught the entire project
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}")
set(CMAKE_INSTALL_LIBDIR "${CMAKE_BINARY_DIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

#Externally provided content
add_subdirectory(third_party)

#Main targets built by this project
add_subdirectory(src)