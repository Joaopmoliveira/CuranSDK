add_executable(DecreaseInertia 
main.cpp
)

target_compile_definitions(DecreaseInertia PRIVATE vsgXchange_FOUND)
target_compile_definitions(DecreaseInertia PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(DecreaseInertia 
PUBLIC 
Eigen3::Eigen
robotutils
utils
renderable
${OPERATING_SYSTEM_REQUIRMENTS}
)