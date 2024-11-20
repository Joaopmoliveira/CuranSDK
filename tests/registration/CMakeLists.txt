find_package(ITK CONFIG REQUIRED)

include(${ITK_USE_FILE})

add_executable(registration_volume main_volume.cpp )

target_compile_definitions(registration_volume PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(registration_volume PUBLIC
imageprocessing
${ITK_LIBRARIES}
)

add_executable(test_clustering_vs_matlab test_clustering_vs_matlab.cpp )

target_compile_definitions(test_clustering_vs_matlab PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_clustering_vs_matlab PUBLIC
imageprocessing
${ITK_LIBRARIES}
)