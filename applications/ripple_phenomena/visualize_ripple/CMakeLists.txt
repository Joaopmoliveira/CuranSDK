add_executable(RippleVisualization 
main.cpp
)

target_compile_definitions(RippleVisualization PRIVATE vsgXchange_FOUND)
target_compile_definitions(RippleVisualization PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(RippleVisualization 
PUBLIC 
Eigen3::Eigen
robotutils
utils
renderable
${OPERATING_SYSTEM_REQUIRMENTS}
)