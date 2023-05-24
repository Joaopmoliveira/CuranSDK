cmake_minimum_required(VERSION 3.17)
cmake_policy(SET CMP0091 NEW)
project(Curan)

#Language definitions used throught the entire project
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED YES)

set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}")
set(CMAKE_INSTALL_LIBDIR "${CMAKE_BINARY_DIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

#Find all required third parties (this should be moved elsewhere)
find_package(asio CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(Ceres CONFIG REQUIRED)
find_package(ITK CONFIG REQUIRED)
find_package(OpenIGTLink CONFIG REQUIRED)
find_package(unofficial-skia CONFIG REQUIRED)

#Externally provided targets
add_subdirectory(third_party)

#Add resources necessary by the display library
add_subdirectory(resources)

#Add internal Curan libraries to the list of known targets
add_subdirectory(libraries)

# Add tests for submodules of curan, i.e. utilities, communication, etc...
add_subdirectory(tests)

# Add medical applications
add_subdirectory(applications)
