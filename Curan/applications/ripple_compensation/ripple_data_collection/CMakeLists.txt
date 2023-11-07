add_executable(RippleCollection DataCollection.cpp)

target_compile_definitions(RippleCollection PRIVATE vsgXchange_FOUND)
target_compile_definitions(RippleCollection PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

if(WIN32) #FRILibrary pulls in macros which cause vsg to fail its compilation step thus we must define this macro when compiling for windows
    target_compile_definitions(RippleCollection PRIVATE NOMINMAX=1)
endif(WIN32)

target_link_libraries(RippleCollection PUBLIC
rbdl-static 
KukaRobotLibrary 
FRILibrary
tasksLib
robotParameters
)