add_executable(FRIPlusBridge 
main.cpp
)

target_compile_definitions(FRIPlusBridge PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(FRIPlusBridge 
PUBLIC 
Eigen3::Eigen
robotutils
utils
communication
)