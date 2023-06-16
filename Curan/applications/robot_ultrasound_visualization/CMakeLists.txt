add_executable(RealTimeRobotVisualizer
src/MyLBRApp.cpp
src/MyLBRClient.cpp
src/link_demo.cpp
src/SharedRobotState.cpp
)
target_compile_definitions(RealTimeRobotVisualizer PRIVATE vsgXchange_FOUND)
target_compile_definitions(RealTimeRobotVisualizer PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(RealTimeRobotVisualizer
PUBLIC
KukaRobotLibrary 
FRILibrary
robotParameters
vsg::vsg
vsgXchange::vsgXchange
)