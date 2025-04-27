cmake_minimum_required(VERSION 3.24)
cmake_policy(SET CMP0091 NEW)
cmake_policy(SET CMP0075 NEW)

#Language definitions used throught the entire project
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED YES)

set(VCPKG_OVERLAY_TRIPLETS "${CMAKE_CURRENT_LIST_DIR}/vcpkg_custom_triplet/")

if(UNIX)
  set(VCPKG_TARGET_TRIPLET "x64-linux-mixed")
  set(VCPKG_HOST_TRIPLET "x64-linux-mixed")
endif(UNIX)

if(WIN32)
  set(VCPKG_TARGET_TRIPLET "x64-windows-mixed")
  set(VCPKG_HOST_TRIPLET "x64-windows-mixed")
endif(WIN32)

set(VCPKG_ROOT "${CMAKE_CURRENT_LIST_DIR}/vcpkg")
set(CMAKE_TOOLCHAIN_FILE "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")


project(Curan)

if(WIN32)
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif(WIN32)

enable_testing()

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
find_package(Vulkan REQUIRED)
find_package(glfw3 3.3 REQUIRED)
find_package(Boost COMPONENTS filesystem)
find_package(vsg CONFIG REQUIRED)
find_package(vsgXchange CONFIG REQUIRED)
find_package(vsgImGui CONFIG REQUIRED)
find_package(RBDL CONFIG REQUIRED)
find_package(gflags REQUIRED)
find_package(glog REQUIRED)


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
  message("Plus path executable not specified. If you wish\nto specify it please provive the variable CURAN_PLUS_EXECUTABLE_PATH with the\ncorrect plus server path.")
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

# Add medical applications
#add_subdirectory(unittests)

# Add tests for submodules of curan, i.e. utilities, communication, etc...
#add_subdirectory(tests)

# Add medical applications
#add_subdirectory(tutorials)