add_executable(FRIPlusBridge 
main.cpp
MyLBRClient.cpp
)

target_link_libraries(FRIPlusBridge 
PUBLIC 
Eigen::Eigen 
rbdl-static 
KukaRobotLibrary 
FRILibrary
tasksLib
robotParameters
utils
communication
)