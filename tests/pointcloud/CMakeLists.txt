add_executable(test_point_cloud_experiments
main.cpp 
)

target_compile_definitions(test_point_cloud_experiments PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_point_cloud_experiments PRIVATE 
${PCL_LIBRARIES}
utils
)