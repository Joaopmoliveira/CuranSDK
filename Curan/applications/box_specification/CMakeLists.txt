find_package(ITK CONFIG REQUIRED)

include(${ITK_USE_FILE})

add_executable(RealTimeRobotBoxSpecifier
main.cpp
link_demo.cpp
SharedRobotState.cpp
)

target_compile_definitions(RealTimeRobotBoxSpecifier PRIVATE vsgXchange_FOUND)
target_compile_definitions(RealTimeRobotBoxSpecifier PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

if(WIN32) #FRILibrary pulls in macros which cause vsg to fail its compilation step thus we must define this macro when compiling for windows
    target_compile_definitions(RealTimeRobotBoxSpecifier PRIVATE NOMINMAX=1)
endif(WIN32)

target_link_libraries(RealTimeRobotBoxSpecifier
PUBLIC
${ITK_LIBRARIES}
communication
imageprocessing
KukaRobotLibrary 
FRILibrary
robotParameters
vsg::vsg
vsgXchange::vsgXchange
renderable
utils
)
