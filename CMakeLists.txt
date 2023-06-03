cmake_minimum_required(VERSION 3.14)
cmake_policy(SET CMP0091 NEW)
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

if (VULKAN_SDK)
    set(ENV{VULKAN_SDK} ${VULKAN_SDK})
endif()

find_package(Vulkan)

FetchContent_Declare(vsg
    GIT_REPOSITORY https://github.com/vsg-dev/VulkanSceneGraph.git
    GIT_TAG "master"
    GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(vsg ${glslang_DEPENDENCY})
set(vsg_DEPENDENCY vsg)

#FetchContent_Declare(assimp
#    GIT_REPOSITORY "https://github.com/assimp/assimp.git"
#    GIT_TAG "v5.2.5"
#    GIT_PROGRESS TRUE
#)
#FetchContent_MakeAvailable(assimp)

# add definitions to enable assimp locally built lib to be recognized by vsgXchange
#add_library(assimp::assimp ALIAS assimp)
#set(assimp_FOUND TRUE CACHE INTERNAL "assimp found.")
#set(CMAKE_DISABLE_FIND_PACKAGE_assimp TRUE CACHE INTERNAL "Disable find_package(assimp) as it's not necessary.")
#set(assimp_VERSION "5.2.5") # use the same version as pulled in the GIT_TAG above.

#set(assimp_DEPENDENCY assimp)

FetchContent_Declare(vsgxchange
    GIT_REPOSITORY https://github.com/vsg-dev/vsgXchange.git
    GIT_TAG master
    GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(vsgxchange ${vsg_DEPENDENCY})
set(vsgXchange_DEPENDENCY vsgXchange)

#FetchContent_Declare(vsgimgui
#    GIT_REPOSITORY "https://github.com/vsg-dev/vsgImGui.git"
#    GIT_TAG "master"
#    GIT_SUBMODULES "src/imgui"
#    GIT_PROGRESS TRUE
#)
#FetchContent_MakeAvailable(vsgimgui ${vsg_DEPENDENCY} )


FetchContent_Declare(Curan
        SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/Curan
)
#FetchContent_MakeAvailable(Curan ${vsg_DEPENDENCY} ${vsgXchange_DEPENDENCY} ${vsgImGui_DEPENDENCY})
FetchContent_MakeAvailable(Curan ${vsg_DEPENDENCY} ${vsgXchange_DEPENDENCY})