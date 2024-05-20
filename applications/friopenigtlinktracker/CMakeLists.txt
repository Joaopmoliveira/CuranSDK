add_executable(FRIPlusBridge 
main.cpp
)

target_link_libraries(FRIPlusBridge 
PUBLIC 
Eigen3::Eigen
robotutils
utils
communication
)