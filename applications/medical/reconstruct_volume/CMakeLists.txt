find_package(ITK CONFIG REQUIRED)

include(${ITK_USE_FILE})

add_executable(RealTimeRobotReconstructor
main.cpp
link_demo.cpp
SharedRobotState.cpp
)

target_compile_definitions(RealTimeRobotReconstructor PRIVATE vsgXchange_FOUND)
target_compile_definitions(RealTimeRobotReconstructor PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

if(WIN32)
    target_compile_definitions(RealTimeRobotReconstructor PRIVATE NOMINMAX=1)
endif(WIN32)

target_link_libraries(RealTimeRobotReconstructor
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
