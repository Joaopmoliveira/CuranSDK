add_executable(RealTimeRobotVisualizer
MyLBRApp.cpp
MyLBRClient.cpp
link_demo.cpp
SharedRobotState.cpp
)

target_compile_definitions(RealTimeRobotVisualizer PRIVATE vsgXchange_FOUND)
target_compile_definitions(RealTimeRobotVisualizer PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

if(WIN32) #FRILibrary pulls in macros which cause vsg to compile thus we must define this macro when compiling for windows
    target_compile_definitions(RealTimeRobotVisualizer PRIVATE NOMINMAX=1)
endif(WIN32)

target_link_libraries(RealTimeRobotVisualizer
PUBLIC
KukaRobotLibrary 
FRILibrary
robotParameters
vsg::vsg
vsgXchange::vsgXchange
renderable
)