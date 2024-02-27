cmake_minimum_required(VERSION 3.7)

project(vsgparticlesystem
        VERSION 1.0.0
        DESCRIPTION "A VulkanSceneGrapg's Particle System demo, which includes the concept of Particle, Axis, Dynamics and a Pool of particles (and much more)."
        LANGUAGES CXX
)

# name of the library
set(LIBRARY_NAME vsgParticleSystem)

# adds a cmake folder with helping functions
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)

    set_property(GLOBAL PROPERTY USE_FOLDERS ON)

    include(CTest)
    enable_testing()

endif()

# build all examples into the bin directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

# Change the default build type to Release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif(NOT CMAKE_BUILD_TYPE)

# find the vsg
if (VULKAN_SDK)
    set(ENV{VULKAN_SDK} ${VULKAN_SDK})
endif()

# find_package(vsg 1.0.0 REQUIRED)

# find_package(nlohmann_json 3.11.2 REQUIRED)

# find_package(Eigen3 3.3 REQUIRED)

# find the optional vsgXchange that can be used for reading and range of image and 3d model formats and shader compilation
# find_package(vsgXchange 1.0.0 REQUIRED)

# set the use of C++17 globally as all examples require it
set(CMAKE_CXX_STANDARD 17)

# add clobber build target to clear all the non git registered files/directories
# add_custom_target(clobber
#     COMMAND git clean -d -f -x
# )

add_subdirectory(src)

add_subdirectory(app)

# Testing only available if this is the main app
# Emergency override MODERN_CMAKE_BUILD_TESTING provided as well
if((CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME OR MODERN_CMAKE_BUILD_TESTING)
   AND BUILD_TESTING)
  add_subdirectory(tests)
endif()
