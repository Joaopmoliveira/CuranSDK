add_executable(FakeFRIPlusBridge 
main.cpp
)

target_link_libraries(FakeFRIPlusBridge 
PUBLIC 
Eigen3::Eigen
robotutils
utils
communication
${OPERATING_SYSTEM_REQUIRMENTS}
)