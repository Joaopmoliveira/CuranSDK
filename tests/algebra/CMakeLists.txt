add_executable(test_centered_bounding_box 
test_centered_bounding_box.cpp 
)

target_compile_definitions(test_centered_bounding_box PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")


target_link_libraries(test_centered_bounding_box 
PUBLIC
Eigen3::Eigen
)


add_executable(test_clipping_of_geometries 
test_clipping_of_geometries.cpp 
)

target_compile_definitions(test_clipping_of_geometries PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")


target_link_libraries(test_clipping_of_geometries 
PUBLIC
geometry
)
