add_executable(StateDerivativeController 
main.cpp
MyLBRClient.cpp
)

target_compile_definitions(StateDerivativeController PRIVATE vsgXchange_FOUND)
target_compile_definitions(StateDerivativeController PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

if(WIN32) #FRILibrary pulls in macros which cause vsg to fail its compilation step thus we must define this macro when compiling for windows
    target_compile_definitions(StateDerivativeController PRIVATE NOMINMAX=1)
endif(WIN32)


target_link_libraries(StateDerivativeController 
PUBLIC 
Eigen3::Eigen
rbdl-static 
KukaRobotLibrary 
FRILibrary
tasksLib
robotParameters
utils
communication
renderable
atomic
)