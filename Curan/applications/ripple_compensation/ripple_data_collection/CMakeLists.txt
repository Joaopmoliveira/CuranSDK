add_executable(RippleCollection DataCollection.cpp)

add_executable(FilterExecution FilterExecution.cpp)

target_link_libraries(RippleCollection PUBLIC
rbdl-static 
KukaRobotLibrary 
FRILibrary
tasksLib
robotParameters
)

target_link_libraries(FilterExecution PUBLIC
rbdl-static 
KukaRobotLibrary 
FRILibrary
tasksLib
robotParameters
)