add_executable(ForceControl 
main.cpp
MyLBRClient.cpp
MyLBRClient.h
LfD2LBR.h
LfD2LBR.cpp
)

target_compile_definitions(ForceControl PRIVATE vsgXchange_FOUND)
target_compile_definitions(ForceControl PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

if(WIN32) #FRILibrary pulls in macros which cause vsg to fail its compilation step thus we must define this macro when compiling for windows
    target_compile_definitions(ForceControl PRIVATE NOMINMAX=1)
endif(WIN32)


target_link_libraries(ForceControl 
PUBLIC 
vsgps
Eigen3::Eigen
rbdl-static 
KukaRobotLibrary 
FRILibrary
tasksLib
robotParameters
utils
communication
renderable
gaussianmixture
${OPERATING_SYSTEM_REQUIRMENTS}
)