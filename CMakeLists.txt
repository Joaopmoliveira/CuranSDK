cmake_minimum_required(VERSION 3.17)
cmake_policy(SET CMP0091 NEW)
cmake_policy(SET CMP0075 NEW)
project(Curan)

#Language definitions used throught the entire project
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
if(WIN32)
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif(WIN32)

set(OPERATING_SYSTEM_REQUIRMENTS "")

if(UNIX)
  string(CONCAT OPERATING_SYSTEM_REQUIRMENTS "atomic")
endif(UNIX)

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
find_package(Boost COMPONENTS filesystem)
find_package(vsg CONFIG REQUIRED)
find_package(vsgXchange CONFIG REQUIRED)
find_package(vsgImGui CONFIG REQUIRED)

#Externally provided targets
add_subdirectory(third_party)

# In Curan we have the Application laucher which is a executable that 
# lauches other executables. For us to know where these executables are
# we define their names here. Thus because we know the build directory 
# of all executables we can inject their path inside the Application laucher 

cmake_path(SET original_resource_paths "${CMAKE_CURRENT_LIST_DIR}/resources")
cmake_path(SET post_build_resource_path "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources")

set(WAS_PLUS_SPECIFIED OFF)
if(DEFINED CURAN_PLUS_EXECUTABLE_PATH)
  set(WAS_PLUS_SPECIFIED ON)
  message("Plus path was specified!")
  message("${CURAN_PLUS_EXECUTABLE_PATH}")
else(DEFINED CURAN_PLUS_EXECUTABLE_PATH)
  message("Plus path was NOT specified!")
endif(DEFINED CURAN_PLUS_EXECUTABLE_PATH)

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

# Add internal Curan libraries to the list of known targets
add_subdirectory(libraries)

# Add medical applications
add_subdirectory(applications)

# Most of the time there is no need to compile the tests and the tutorials
# folder so you can comment out these subfolders from the build directory

# Add tests for submodules of curan, i.e. utilities, communication, etc...
add_subdirectory(tests)

# Add medical applications
#add_subdirectory(tutorialsrc)

# https://www.fnac.pt/Monitor-Tatil-Portatil-Verbatim-PMT-15-FHD-15-6-Monitor-Monitor-LED/a10756603
# https://www.fnac.pt/Monitor-Portatil-Asus-ZenScreen-15-6-FHD-MB166C-60Hz-Monitor-Monitor-LED/a11231749#int=S:Suggestion|FA%3AMIC|10|11231749|BL3|L1