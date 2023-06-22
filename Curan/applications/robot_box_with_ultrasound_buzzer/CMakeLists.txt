add_executable(RealTimeBoxWithBuzzer
MyLBRApp.cpp
MyLBRClient.cpp
link_demo.cpp
SharedRobotState.cpp
)

target_compile_definitions(RealTimeBoxWithBuzzer PRIVATE vsgXchange_FOUND)
target_compile_definitions(RealTimeBoxWithBuzzer PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

if(WIN32) #FRILibrary pulls in macros which cause vsg to fail its compilation step thus we must define this macro when compiling for windows
    target_compile_definitions(RealTimeBoxWithBuzzer PRIVATE NOMINMAX=1)
endif(WIN32)

target_link_libraries(RealTimeBoxWithBuzzer
PUBLIC
KukaRobotLibrary 
FRILibrary
robotParameters
vsg::vsg
vsgXchange::vsgXchange
renderable
asio
)

if(SERIAL_RECEIVE_ARGUMENT)
    if(WIN32)
        target_compile_definitions(RealTimeBoxWithBuzzer PRIVATE CURAN_SERIAL_PORT="COM6")
    else(WIN32)
        target_compile_definitions(RealTimeBoxWithBuzzer PRIVATE CURAN_SERIAL_PORT="/dev/ttyUSB0")
    endif(WIN32)
else(SERIAL_RECEIVE_ARGUMENT)
    target_compile_definitions(RealTimeBoxWithBuzzer PRIVATE CURAN_SERIAL_PORT="COM6")
endif(SERIAL_RECEIVE_ARGUMENT)
