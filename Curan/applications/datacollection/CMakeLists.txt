add_executable(DataCollection 
main.cpp
MyLBRClient.cpp
)

target_link_libraries(DataCollection 
PUBLIC 
Eigen3::Eigen
rbdl-static 
KukaRobotLibrary 
FRILibrary
tasksLib
robotParameters
utils
communication
)