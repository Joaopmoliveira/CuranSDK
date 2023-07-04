cmake_minimum_required(VERSION 3.17)
cmake_policy(SET CMP0091 NEW)
project(Curan)

#Language definitions used throught the entire project
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
if(WIN32)
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif(WIN32)
set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}")
set(CMAKE_INSTALL_LIBDIR "${CMAKE_BINARY_DIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

#Find all required third parties (this should be moved elsewhere)
find_package(asio CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(Ceres CONFIG REQUIRED)
find_package(unofficial-skia CONFIG REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(glfw3 3.3 REQUIRED)

#Externally provided targets
add_subdirectory(third_party)

cmake_path(SET original_resource_paths "${CMAKE_CURRENT_LIST_DIR}/resources")
cmake_path(SET post_build_resource_path "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources")

#Add resources necessary by the display library
add_custom_command(OUTPUT resources
               COMMAND ${CMAKE_COMMAND} -E copy_directory ${original_resource_paths} ${post_build_resource_path}
               DEPENDS "${original_resource_paths}"
               COMMENT "Currently copying the widget files to the correct directories"
               )

# create a target that includes some_file, this gives us a name that we can use later
add_custom_target(
  curan_resources
  DEPENDS resources
)

#Add internal Curan libraries to the list of known targets
add_subdirectory(libraries)

# Add tests for submodules of curan, i.e. utilities, communication, etc...
add_subdirectory(tests)

# Add medical applications
add_subdirectory(applications)

# Add medical applications
#add_subdirectory(tutorialsrc)

