add_executable(FakeFRIPlusBridge 
main.cpp
)

target_compile_definitions(FakeFRIPlusBridge PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(FakeFRIPlusBridge 
PUBLIC 
Eigen3::Eigen
robotutils
utils
communication
${OPERATING_SYSTEM_REQUIRMENTS}
)