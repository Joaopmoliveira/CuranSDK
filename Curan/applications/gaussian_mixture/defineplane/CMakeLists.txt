add_executable(PlaneDefinition 
main.cpp
MyLBRClient.cpp
)

target_compile_definitions(PlaneDefinition PRIVATE vsgXchange_FOUND)
target_compile_definitions(PlaneDefinition PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

if(WIN32) #FRILibrary pulls in macros which cause vsg to fail its compilation step thus we must define this macro when compiling for windows
    target_compile_definitions(PlaneDefinition PRIVATE NOMINMAX=1)
endif(WIN32)


target_link_libraries(PlaneDefinition 
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
${OPERATING_SYSTEM_REQUIRMENTS}
)