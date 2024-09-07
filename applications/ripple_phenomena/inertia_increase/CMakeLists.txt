add_executable(IncreaseInertia 
main.cpp
)

target_compile_definitions(IncreaseInertia PRIVATE vsgXchange_FOUND)
target_compile_definitions(IncreaseInertia PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(IncreaseInertia 
PUBLIC 
Eigen3::Eigen
robotutils
utils
renderable
${OPERATING_SYSTEM_REQUIRMENTS}
)