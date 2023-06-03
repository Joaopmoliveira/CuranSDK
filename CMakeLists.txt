cmake_minimum_required(VERSION 3.14)

project(CuranSuperBuild)

include(FetchContent)
set(FETCHCONTENT_QUIET FALSE)

set(DOWNLOADS_DIR ${PROJECT_BINARY_DIR}/downloads)
set(COMPONENTS_DIR ${PROJECT_BINARY_DIR}/components)
set(FETCHCONTENT_BASE_DIR ${COMPONENTS_DIR})

# set compilation to use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads)

find_library(DL_LIBRARY dl)
if(NOT DL_LIBRARY)
    set(DL_LIBRARY "") # change from NOTFOUND to empty when passed to linker
endif()

##############################################################################################################
#
# VulkkanSDK
#
if (VULKAN_SDK)
    set(ENV{VULKAN_SDK} ${VULKAN_SDK})
endif()

find_package(Vulkan)

##############################################################################################################
#
# VulkaSceneGraph
#
OPTION(BUILD_VulkanSceneGraph "Build VulkanSceneGraph" ON)
if (${BUILD_VulkanSceneGraph})
    FetchContent_Declare(vsg
        GIT_REPOSITORY https://github.com/vsg-dev/VulkanSceneGraph.git
        GIT_TAG "master"
        GIT_PROGRESS TRUE
    )
    FetchContent_MakeAvailable(vsg ${glslang_DEPENDENCY})
    set(vsg_DEPENDENCY vsg)
endif()

##############################################################################################################
#
# assimp
#
OPTION(BUILD_assimp "Build assimp" ON)
if (${BUILD_assimp})
    FetchContent_Declare(assimp
        GIT_REPOSITORY "https://github.com/assimp/assimp.git"
        GIT_TAG "v5.2.5"
        GIT_PROGRESS TRUE
    )
    FetchContent_MakeAvailable(assimp)

    # add definitions to enable assimp locally built lib to be recognized by vsgXchange
    add_library(assimp::assimp ALIAS assimp)
    set(assimp_FOUND TRUE CACHE INTERNAL "assimp found.")
    set(CMAKE_DISABLE_FIND_PACKAGE_assimp TRUE CACHE INTERNAL "Disable find_package(assimp) as it's not necessary.")
    set(assimp_VERSION "5.2.5") # use the same version as pulled in the GIT_TAG above.

    set(assimp_DEPENDENCY assimp)
endif()

##############################################################################################################
#
# vsgXchange
#
OPTION(BUILD_vsgXchange "Build vsgXchange" ON)
if (${BUILD_vsgXchange})
    FetchContent_Declare(vsgxchange
        GIT_REPOSITORY https://github.com/vsg-dev/vsgXchange.git
        GIT_TAG master
        GIT_PROGRESS TRUE
    )
    FetchContent_MakeAvailable(vsgxchange ${vsg_DEPENDENCY} ${osg2vsg_DEPENDENCY} ${assimp_DEPENDENCY})
    set(vsgXchange_DEPENDENCY vsgXchange)
endif()

##############################################################################################################
#
# vsgImGui
#
OPTION(BUILD_vsgImGui "Build vsgImGui" ON)
if (${BUILD_vsgImGui})
    FetchContent_Declare(vsgimgui
        GIT_REPOSITORY "https://github.com/vsg-dev/vsgImGui.git"
        GIT_TAG "master"
        GIT_SUBMODULES "src/imgui"
        GIT_PROGRESS TRUE
    )
    FetchContent_MakeAvailable(vsgimgui ${vsg_DEPENDENCY} )
endif()

FetchContent_Declare(Curan
        SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/Curan
)
FetchContent_MakeAvailable(Curan ${vsg_DEPENDENCY} ${vsgXchange_DEPENDENCY} ${vsgImGui_DEPENDENCY})